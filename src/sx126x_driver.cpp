#ifndef USE_RFM95
#include "sx126x_driver.h"
#include "hardware/spi.h"
#include "config.h"
#include "param.h"
#include "tools.h"
#include "gps.h"


// to do add setup of  reset pin (if required)

// At oXs side, the principle is to set Lora device in continuous receive mode
// In a loop, we look if a packet has been received (if so, it is one byte long with the Tx power for next transmit)
// When oXs get this byte, it transmits immediately RSSI, SNR, and GPS data (0 if no GPS) and GPS precision and then go back to receive mode
// receiving and sending are done on 2 different frequencies in order to reduce the channel occupation


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

uint8_t loraRxPacketRssi;
uint8_t loraRxPacketSnr;
uint8_t loraRxPacketType;
uint8_t loraRxPacketTxPower;
uint8_t loraState = 0;

bool locatorInstalled = false;

extern GPS gps;
extern uint32_t GPS_last_fix_millis; // time when last fix has been received (used by lora locator)
extern int32_t GPS_last_fix_lon;     // last lon when a fix has been received
extern int32_t GPS_last_fix_lat;     // last lat when a fix has been received
extern uint16_t GPS_hdop;
extern uint8_t forcedFields;
extern field fields[];

extern uint32_t debugFlags; // each bit says if a type of debug msg must be printed; list of bits is defined in an enum in param.h

extern CONFIG config;

#define SPI_PORT spi1

uint8_t xtalCap[2] = {0x12, 0x12};

// RF frequency setting
uint32_t rfFrequency = LOCATOR_FREQUENCY;

//TX power setting
uint8_t power = _power;       // use 0x16 for 22 db

// Define modulation parameters setting
uint8_t sf = _sf;                 // spreading factor 7; can be between 5 and 11 (higher = higher range)
uint8_t bw = SX126X_BW_250000;  // 125 kHz     ; can be 125000(4) 250000(5) 500000(6) (smaller = higher range; 
uint8_t cr = SX126X_CR_4_8;     // 4/5 code rate ; can be 4_5, 4_6, 4_7, 4_8
uint8_t ldro = SX126X_LDRO_ON; // low data rate optimize off, can be ON or OFF

// Define packet parameters setting
uint16_t preambleLength = _preambleLength;                // 12 bytes preamble
uint8_t headerType = SX126X_HEADER_IMPLICIT; // explicit packet header = variable length, can also be implicit (=fix length)
uint8_t payloadLength = 64;                  // 64 bytes payload
uint8_t crcType = SX126X_CRC_ON;             // cyclic redundancy check (CRC) on ; can also be OFF
uint8_t invertIq = SX126X_IQ_STANDARD;       // standard IQ setup

// SyncWord setting
uint8_t sw[2] = {0x34, 0x44};

void initSpi()
{   // configure the SPI
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(config.pinSpiCs);
    gpio_set_dir(config.pinSpiCs, GPIO_OUT);
    gpio_put(config.pinSpiCs, 1);

    spi_init(SPI_PORT, 2000 * 1000);
    gpio_set_function(config.pinSpiMiso, GPIO_FUNC_SPI);
    gpio_set_function(config.pinSpiSck, GPIO_FUNC_SPI);
    gpio_set_function(config.pinSpiMosi, GPIO_FUNC_SPI);

    // Busy pin is used to check if spi command can be sent
    gpio_init(config.pinE220Busy);
    gpio_set_dir(config.pinE220Busy, GPIO_IN);
}

void loraHandle()
{ // this function is called from main.cpp only when pinSpiCs is not equal to 255
    uint16_t loraIrqFlags;
    static uint32_t loraStateMillis;
    uint32_t currentMillis = millisRp();
    switch (loraState)
    {
    case LORA_TO_INIT:
        initSpi();   // init spi
        sleep_ms(500);
        locatorInstalled = loraSetup(); // init lora with some set up that need to be done only once
        if (locatorInstalled){
            loraState = LORA_IN_SLEEP;
            loraStateMillis = currentMillis + SLEEP_TIME;
        } else {
            loraState = LORA_IN_ERROR;
        }
        break;

    case LORA_IN_SLEEP:
        if (currentMillis > loraStateMillis)
        {
            if (debugFlags & (1 << DEBUG_LORA))
            {
                printf("End of sleep; listen for a request during 5 sec\n");
                // printf("Li ");
            }
            loraReceiveOn(2, 5000); // expect 2 char in max 5sec
            loraState = LORA_IN_RECEIVE;
            loraStateMillis = currentMillis + SHORT_RECEIVE_TIME;
        }
        break;

    case LORA_IN_RECEIVE:
        // check if a packet has been received a correct CRC of if a tieout occured
        sx126x_getIrqStatus(&loraIrqFlags);
        // loraIrqFlags = loraReadRegister(LORA_REG_IRQ_FLAGS);
        if (loraIrqFlags & SX126X_IRQ_RX_DONE)
        {
            if (loraIrqFlags & SX126X_IRQ_CRC_ERR)
            {
                printf("wrong crc received\n");
                loraReceiveOn(2, 5000); // expect again 2 char in max 5sec; stay in receiving state
            }
            else
            {
                loraReadPacket(); // read the data and decode it
                loraState = LORA_TO_TRANSMIT;
                if (debugFlags & (1 << DEBUG_LORA))
                {
                    printf("Request received\n");
                }
            }
        }
        else if (loraIrqFlags & SX126X_IRQ_TIMEOUT)
        { // back to sleep if we did not receive a packet within the expected time
            if (debugFlags & (1 << DEBUG_LORA))
            {
                printf("No request received; going to sleep for 5 sec\n");
            }
            loraState = LORA_IN_SLEEP;
            loraStateMillis = currentMillis + SLEEP_TIME;        }
        break;

    case LORA_TO_TRANSMIT:
        loraFillTxPacket();                    // fill buffer with data to be sent (6 bytes) and call fa unction to send it with a timeout
        loraState = LORA_WAIT_END_OF_TRANSMIT;
        if (debugFlags & (1 << DEBUG_LORA))
        {
            printf("Start sending a reply\n");
        }
        break;
    case LORA_WAIT_END_OF_TRANSMIT:
        // check if transmit is done or if timeout occurs
        // if transmitted, put lora in receive, change loraState to LORA_IN_RECEIVE , change loraStateMillis = currentMillis+LONG_RECEIVE
        // else, if timeOut, go in sleep for the SLEEP_TIME
        // check if a packet has been received a correct CRC of if a tieout occured
        sx126x_getIrqStatus(&loraIrqFlags);
        if (loraIrqFlags & SX126X_IRQ_TX_DONE)
        {
            loraReceiveOn(2, 60000); // expect 2 char in max 60sec
            loraState = LORA_IN_RECEIVE;
            //loraStateMillis = currentMillis + LONG_RECEIVE_TIME;
            if (debugFlags & (1 << DEBUG_LORA))
            {
                printf("Sending done; listen for a request during 60 sec\n");
            }
        }
        else if (loraIrqFlags & SX126X_IRQ_TIMEOUT)
        {
            printf("transmit timeout; go to standby (sleep) for 5sec\n") ;
            if (debugFlags & (1 << DEBUG_LORA))
            {
                printf("Sending time out; going to sleep during 5 sec\n");
            }
            loraState = LORA_IN_SLEEP;
            loraStateMillis = currentMillis + SLEEP_TIME;
        }
        break;
    case LORA_IN_ERROR:
        break; // nothing to do when lora init failed
    } // end of switch
}

bool loraSetup()
{ // making the setup; return false in case of error
    uint8_t mode;
    uint16_t error;
    // Set to standby mode
    sx126x_setStandby(SX126X_STANDBY_RC); // RC set in standby using 13Mhz rc; can also be SX126X_STANDBY_XOSC for 32Mz xtal
    sx126x_getStatus(&mode); // get the status
    printf("Status after setStandby= %X\n",mode);    
    if ((mode & 0x70)  != SX126X_STATUS_MODE_STDBY_RC)
    { // to compare with SX126X_STATUS_MODE_STDBY_RC when no xtal
        printf("Something wrong, can't set E220 to standby mode\n");
        return false;
    }
        // Set packet type to LoRa
    sx126x_setPacketType(SX126X_LORA_MODEM);
    sx126x_writeRegister(SX126X_REG_XTA_TRIM, xtalCap, 2);
    // configure DIO2 as RF switch control  (DIO2 has to be connected to TX pin)
    sx126x_setDio2AsRfSwitchCtrl(SX126X_DIO2_AS_RF_SWITCH);    
    printf("Set frequency to %i mHz\n", rfFrequency / 1000000);
    uint32_t rfFreq = ((uint64_t)rfFrequency * 33554432UL) / 32000000UL;
    sx126x_setRfFrequency(rfFreq); 

        // apply workarround given by semtech
    uint8_t data[5];
    sx126x_readRegister(SX126X_REG_TX_CLAMP_CONFIG , data, 1);
    data[0]= data[0] | 0X1E;
    sx126x_writeRegister(SX126X_REG_TX_CLAMP_CONFIG , data, 1);
  
    // PA and TX power setting
    uint8_t paDutyCycle;
    uint8_t hpMax;       
    if (power == 22){ 
        paDutyCycle=0X04; hpMax= 0X07;
    } else if (power >= 20){ 
        paDutyCycle=0X03; hpMax= 0X05;
    } else if (power >= 17){ 
        paDutyCycle=0X02; hpMax= 0X03;
    } else {
        paDutyCycle=0X02; hpMax= 0X02;
    }
    sx126x_setPaConfig(paDutyCycle, hpMax, 0x00, 0x01);
    sx126x_setTxParams(power, SX126X_PA_RAMP_200U); // ramping can go from 10us to 3.4ms
    // Configure modulation parameter with predefined spreading factor, bandwidth, coding rate, and low data rate optimize setting
    sx126x_setModulationParamsLoRa(sf, bw, cr, ldro);
    // Configure packet parameter with predefined preamble length, header mode type, payload length, crc type, and invert iq option
    sx126x_setPacketParamsLoRa(preambleLength, headerType, payloadLength, crcType, invertIq);
    // enable interruptmask 
    sx126x_setDioIrqParams(SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE |SX126X_IRQ_CRC_ERR | SX126X_IRQ_TIMEOUT, 0, 0, 0);
    // Set predefined syncronize word
    sx126x_writeRegister(SX126X_REG_LORA_SYNC_WORD_MSB, sw, 2);
    //  Set buffer base address
    sx126x_setBufferBaseAddress(0x00, 0x80); // first param is base adr of TX, second for Rx
    sx126x_setRxTxFallbackMode(SX126X_FALLBACK_STDBY_RC); // mode after a Tx or RX

    return true;                                            // here we consider that LORA is present
}


void loraTransmit(uint8_t *message, uint8_t length, uint32_t timeout)
{
// set base adress with SetBufferBaseAddress
// write the buffer with WriteBuffer
// set modulation param with SetModulationParams
// set frame format with SetPacketParams
// start the transmission with SetTx () with or without a timeout
// to finish, Wait for the IRQ TxDone; I presume it is the same to get the status and check the mode; it must fall back on standby when tx is done
// if irq flag is used, clear the irq
// to know the irq status, there is a command GetIrqStatus(); it provides 2 bytes; bit0= Tx done, bit1=Rx done bit 6= wrong crc received
// to clear the irq flag, use ClearIrqStatus() with 2 bytes (set bit =1 to clear an irq flag)

    // Write the message to buffer
    sx126x_writeBuffer(0x00, message , length);
    if ((loraRxPacketTxPower > 10) and (loraRxPacketTxPower <=22)){
        sx126x_setTxParams(loraRxPacketTxPower, SX126X_PA_RAMP_200U); // ramping can go from 10us to 3.4ms
    } else {
        sx126x_setTxParams(power, SX126X_PA_RAMP_200U); // ramping can go from 10us to 3.4ms
    }
    // Set payload length same as message length
    sx126x_setPacketParamsLoRa(preambleLength, headerType, length, crcType, invertIq);
    // Clear the interrupt status
    sx126x_clearIrqStatus(0XFFFF);
    // Calculate timeout (timeout duration = timeout * 15.625 us)
    uint32_t tOut = timeout * 64; // could be changed to timeout << 6
    // Set RF module to TX mode to transmit message
    sx126x_setTx(tOut);
}

void loraReceiveOn(uint8_t length, uint32_t timeout)
{
    // Set payload length same as message length //not sure it is required
    sx126x_setPacketParamsLoRa(preambleLength, headerType, length, crcType, invertIq);
    // Clear the interrupt status
    //uint16_t irqStat;
    //sx126x_getIrqStatus(&irqStat);
    sx126x_clearIrqStatus(0XFFFF);
    // Calculate timeout (timeout duration = timeout * 15.625 us)
    uint32_t tOut = timeout * 64; // could be changed to timeout << 6
    // Set RF module to RX mode to receive message
    sx126x_setRx(tOut);
}

void loraReadPacket() {           // read a packet with 2 bytes ; PacketType and PacketTxPower
    // read Rssi and snr
    uint8_t loraRxPacketRssiU8;
    uint8_t loraRxPacketSnrU8;
    uint8_t signalRssiPktU8;
    sx126x_getPacketStatus(&loraRxPacketRssiU8, &loraRxPacketSnrU8, &signalRssiPktU8);
    //loraRxPacketRssi = 0 - (((int)loraRxPacketRssiU8) >> 1); // rssi value = - U8/2
    loraRxPacketRssi = loraRxPacketRssiU8; // conversion will be done on locator receiver side.
    
    loraRxPacketSnr = ((float)loraRxPacketSnrU8) * 0.25;     // snr value = u8/4
    // get len and pointer
    uint8_t payloadLengthRx;
    uint8_t rxStartBufferPointer;
    sx126x_getRxBufferStatus(&payloadLengthRx, &rxStartBufferPointer);
    if (payloadLengthRx != 2) {
        printf("got a Lora packet with a wrong length= %i buffer= %i \n", payloadLengthRx , rxStartBufferPointer);
    }
    // Read message from buffer
    uint8_t loraRxBuffer[2];
    sx126x_readBuffer(rxStartBufferPointer, loraRxBuffer, payloadLengthRx);
    loraRxPacketType =  loraRxBuffer[0];
    loraRxPacketTxPower = loraRxBuffer[1];  
    //printf("Rssi=%i Snr=%i Type=%x  pow=%x\n", loraRxPacketRssi , loraRxPacketSnr, loraRxPacketType , loraRxPacketTxPower ) ;
}


void loraFillTxPacket() {
  // data to be sent are type of packet, Long, Lat, Gps accuracy, rssi, snr, nbr of min since last GPS
  // in order to stay with 6 bytes per packet, we will send 1 byte with
  //         Bit 7/ 0 = Long, 1=Lat ; this is type of packet
  //         Bit 6/3 = GPS accuracy 0= very good; 15 = bad (we discard décimal); it is gps_HDOP/128, if >15, then becomes 15
  //         Bit 2/0/= gps oldier than than x sec....1h; 0 = no GPS (yet)
  //         Then there is one byte with Rssi and 4 bytes for long/lat (or replaced by 00 if GPS is unavailable)
  //         Note: gps_last_fix_lon and lat are filled with last value when a fix was available and if a fix has never been available they are filled with 0
  uint8_t loraTxBuffer[6] ;
  static uint8_t packetType = 0 ;
  uint16_t temp16 = 0 ;
  uint8_t temp8  = 0 ;
  int32_t gpsPos = 0 ;
  packetType = (~packetType ) & 0x80 ;                  // reverse first bit to transmit once Long and once lat
  if (gps.gpsInstalled && gps.GPS_fix) {  
    temp16 = (gps.GPS_pdop >> 7) ;  // Divide by 128 instead of 100 to go faster and keep only 4 bytes
    if ( temp16 > 0x0F) temp16 = 0x0F ;
    packetType |= (temp16 << 3 ) ; // shift in pos 3 to 6
    if ( GPS_last_fix_millis > 0 ) { // if we receive at least one fix then we calculate the enlapsed time 
        temp16 = ( millisRp() - GPS_last_fix_millis) >> 16 ; // to save bytes, we divide millisec by 1024
        if ( temp16 > 3515 ) {
        temp8 = 6 ; // more than 1 h (3515 = 3600000/1024 
        } else if ( temp16 > 585 ) {
        temp8 = 5 ; // more than 10 min (585 = 600000/1024
        } else if ( temp16 > 59 ) {
        temp8 = 4 ; // more than 1 min (59 = 60000/1024
        } else if ( temp16 > 10 ) {
        temp8 = 3 ; // more than 10 sec (10 = 10000/1024
        } else if ( temp16 > 1 ) {
        temp8 = 2 ; // more than 1 sec (1 = 1000/1024
        } else {
        temp8 = 1 ; // less than 1 sec
        }
    }
  } else if (forcedFields == 1){ // just to debug with fvp or fvn withous gps
    temp16 = 0x0F ;
    packetType |= (temp16 << 3 ) ; // shift in pos 3 to 6
    temp8 = 3;
    if (packetType & 0x80) {
        gpsPos =  fields[LATITUDE].value;
    } else {
        gpsPos = fields[LONGITUDE].value ;
    }
  }   
  if (gps.gpsInstalled && gps.GPS_fix) {  
    if (packetType & 0x80) {
        gpsPos = GPS_last_fix_lat ;
    } else {
        gpsPos = GPS_last_fix_lon ;
    }
  }
  loraTxBuffer[0] = packetType | temp8 ;
  loraTxBuffer[1] = loraRxPacketRssi ; // RSSI value of last packet
  
  loraTxBuffer[2] = gpsPos >> 24 ;
  loraTxBuffer[3] = gpsPos >> 16 ;
  loraTxBuffer[4] = gpsPos >> 8 ;
  loraTxBuffer[5] = gpsPos ;

  //#define DEBUG_LORA_SEND
  #ifdef DEBUG_LORA_SEND
  printf("lora send: %x %x %x %x %x %x\n",loraTxBuffer[0],loraTxBuffer[1],loraTxBuffer[2],loraTxBuffer[3],loraTxBuffer[4],loraTxBuffer[5]);
  #endif
  loraTransmit(loraTxBuffer, 6, 400);  // wait max 400ms
}


//*****************  here the general functions to manage a sw126x or LLCC68

bool sx126x_busyCheck(uint32_t timeout)
{
    uint32_t t = millisRp();
    while (gpio_get(config.pinE220Busy) !=0)
    {
        if (millisRp() - t > timeout) {
            printf("busy time out expired after %i\n", timeout);
            return true;
        }    
    }
    return false;
}

//void sx126x_setSleep(uint8_t sleepConfig) {     sx126x_transfer(SX126X_SetSleep, &sleepConfig, 1); }

void sx126x_setStandby(uint8_t standbyConfig)
{
    sx126x_transfer(SX126X_SetStandby, &standbyConfig, 1);
}

//void sx126x_setFs() {  sx126x_transfer(SX126X_SetFs, NULL, 0); }

void sx126x_setTx(uint32_t timeout)
{
    uint8_t buf[3];
    buf[0] = timeout >> 16;
    buf[1] = timeout >> 8;
    buf[2] = timeout;
    sx126x_transfer(SX126X_SetTx, buf, 3);
}

void sx126x_setRx(uint32_t timeout)
{
    uint8_t buf[3];
    buf[0] = timeout >> 16;
    buf[1] = timeout >> 8;
    buf[2] = timeout;
    sx126x_transfer(SX126X_SetRx, buf, 3);
}

//void sx126x_stopTimerOnPreamble(uint8_t enable) {    sx126x_transfer(0x9F, &enable, 1);}
/*
void sx126x_setRxDutyCycle(uint32_t rxPeriod, uint32_t sleepPeriod)
{
    uint8_t buf[6];
    buf[0] = rxPeriod >> 16;
    buf[1] = rxPeriod >> 8;
    buf[2] = rxPeriod;
    buf[3] = sleepPeriod >> 16;
    buf[4] = sleepPeriod >> 8;
    buf[5] = sleepPeriod;
    sx126x_transfer(0x94, buf, 6);
}

void sx126x_setCad()
{
    sx126x_transfer(0xC5, NULL, 0);
}

void sx126x_setTxContinuousWave()
{
    sx126x_transfer(0xD1, NULL, 0);
}

void sx126x_setTxInfinitePreamble()
{
    sx126x_transfer(0xD2, NULL, 0);
}

void sx126x_setRegulatorMode(uint8_t modeParam)
{
    sx126x_transfer(0x96, &modeParam, 1);
}

void sx126x_calibrate(uint8_t calibParam)
{
    sx126x_transfer(0x89, &calibParam, 1);
}

void sx126x_calibrateImage(uint8_t freq1, uint8_t freq2)
{
    uint8_t buf[2];
    buf[0] = freq1;
    buf[1] = freq2;
    sx126x_transfer(0x98, buf, 2);
}
*/
void sx126x_setPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut)
{
    uint8_t buf[4];
    buf[0] = paDutyCycle;
    buf[1] = hpMax;
    buf[2] = deviceSel;
    buf[3] = paLut;
    sx126x_transfer(0x95, buf, 4);
}

void sx126x_setRxTxFallbackMode(uint8_t fallbackMode)
{
    sx126x_transfer(0x93, &fallbackMode, 1);
}

void sx126x_writeRegister(uint16_t address, uint8_t *data, uint8_t nData)
{
    uint8_t bufAdr[2] = { (uint8_t) (address >> 8), (uint8_t) address};
    sx126x_transfer(0x0D, data, nData, bufAdr, 2, false);
}

void sx126x_readRegister(uint16_t address, uint8_t *data, uint8_t nData)
{
    uint8_t bufAdr[3] = {(uint8_t) (address >> 8), (uint8_t )address, 0x00};
    sx126x_transfer(0x1D, data, nData, bufAdr, 3, true);
}

void sx126x_writeBuffer(uint8_t offset, uint8_t *data, uint8_t nData)
{
    uint8_t bufOfs[1] = {offset};
    sx126x_transfer(0x0E, data, nData, bufOfs, 1, false);
}

void sx126x_readBuffer(uint8_t offset, uint8_t *data, uint8_t nData)
{
    uint8_t bufOfs[2] = {offset, 0x00};
    sx126x_transfer(0x1E, data, nData, bufOfs, 2, true);
}

void sx126x_setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t buf[8];
    buf[0] = irqMask >> 8;
    buf[1] = irqMask;
    buf[2] = dio1Mask >> 8;
    buf[3] = dio1Mask;
    buf[4] = dio2Mask >> 8;
    buf[5] = dio2Mask;
    buf[6] = dio3Mask >> 8;
    buf[7] = dio3Mask;
    sx126x_transfer(0x08, buf, 8);
}

void sx126x_getIrqStatus(uint16_t *irqStatus)
{
    uint8_t buf[3];
    sx126x_transfer(0x12, buf, 3);
    *irqStatus = (buf[1] << 8) | buf[2];
}

void sx126x_clearIrqStatus(uint16_t clearIrqParam)
{
    uint8_t buf[2];
    buf[0] = clearIrqParam >> 8;
    buf[1] = clearIrqParam;
    sx126x_transfer(0x02, buf, 2);
}

void sx126x_setDio2AsRfSwitchCtrl(uint8_t enable)
{
    sx126x_transfer(0x9D, &enable, 1);
}
/*
void sx126x_setDio3AsTcxoCtrl(uint8_t tcxoVoltage, uint32_t delay)
{
    uint8_t buf[4];
    buf[0] = tcxoVoltage;
    buf[1] = delay >> 16;
    buf[2] = delay >> 8;
    buf[3] = delay;
    sx126x_transfer(0x97, buf, 4);
}
*/
void sx126x_setRfFrequency(uint32_t rfFreq)
{
    uint8_t buf[4];
    buf[0] = rfFreq >> 24;
    buf[1] = rfFreq >> 16;
    buf[2] = rfFreq >> 8;
    buf[3] = rfFreq;
    sx126x_transfer(0x86, buf, 4);
}

void sx126x_setPacketType(uint8_t packetType)
{
    sx126x_transfer(0x8A, &packetType, 1);
}

void sx126x_getPacketType(uint8_t *packetType)
{
    uint8_t buf[2]= {0};
    sx126x_transfer(0x11, buf, 2);
    *packetType = buf[1];
}

void sx126x_setTxParams(uint8_t power, uint8_t rampTime)
{
    uint8_t buf[2];
    buf[0] = power;
    buf[1] = rampTime;
    sx126x_transfer(0x8E, buf, 2);
}

void sx126x_setModulationParamsLoRa(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro)
{
    uint8_t buf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[0] = sf;
    buf[1] = bw;
    buf[2] = cr;
    buf[3] = ldro;
    sx126x_transfer(0x8B, buf, 8);
}
/*
void sx126x_setModulationParamsFSK(uint32_t br, uint8_t pulseShape, uint8_t bandwidth, uint32_t Fdev)
{
    uint8_t buf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[0] = br >> 16;
    buf[1] = br >> 8;
    buf[2] = br;
    buf[3] = pulseShape;
    buf[4] = bandwidth;
    buf[5] = Fdev >> 16;
    buf[6] = Fdev >> 8;
    buf[7] = Fdev;
    sx126x_transfer(0x8B, buf, 8);
}
*/
void sx126x_setPacketParamsLoRa(uint16_t preambleLength, uint8_t headerType, uint8_t payloadLength, uint8_t crcType, uint8_t invertIq)
{
    uint8_t buf[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[0] = preambleLength >> 8;
    buf[1] = preambleLength;
    buf[2] = headerType;
    buf[3] = payloadLength;
    buf[4] = crcType;
    buf[5] = invertIq;
    sx126x_transfer(0x8C, buf, 9);
}
/*
void sx126x_setPacketParamsFSK(uint16_t preambleLength, uint8_t preambleDetector, uint8_t syncWordLength, uint8_t addrComp, uint8_t packetType, uint8_t payloadLength, uint8_t crcType, uint8_t whitening)
{
    uint8_t buf[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[0] = preambleLength >> 8;
    buf[1] = preambleLength;
    buf[2] = preambleDetector;
    buf[3] = syncWordLength;
    buf[4] = addrComp;
    buf[5] = packetType;
    buf[6] = payloadLength;
    buf[7] = crcType;
    buf[8] = whitening;
    sx126x_transfer(0x8C, buf, 9);
}

void sx126x_setCadParams(uint8_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, uint8_t cadExitMode, uint32_t cadTimeout)
{
    uint8_t buf[7];
    buf[0] = cadSymbolNum;
    buf[1] = cadDetPeak;
    buf[2] = cadDetMin;
    buf[3] = cadExitMode;
    buf[4] = cadTimeout >> 16;
    buf[5] = cadTimeout >> 8;
    buf[6] = cadTimeout;
    sx126x_transfer(0x88, buf, 7);
}
*/
void sx126x_setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
    uint8_t buf[2];
    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;
    sx126x_transfer(0x8F, buf, 2);
}

void sx126x_setLoRaSymbNumTimeout(uint8_t symbnum)
{
    sx126x_transfer(0xA0, &symbnum, 1);
}

void sx126x_getStatus(uint8_t *status)
{
    uint8_t buf;
    sx126x_transfer(0xC0, &buf, 1);
    *status = buf;
}

void sx126x_getRxBufferStatus(uint8_t *payloadLengthRx, uint8_t *rxStartBufferPointer)
{
    uint8_t buf[3];
    sx126x_transfer(0x13, buf, 3);
    *payloadLengthRx = buf[1];
    *rxStartBufferPointer = buf[2];
}

void sx126x_getPacketStatus(uint8_t *rssiPkt, uint8_t *snrPkt, uint8_t *signalRssiPkt)
{
    uint8_t buf[4];
    sx126x_transfer(0x14, buf, 4);
    *rssiPkt = buf[1];
    *snrPkt = buf[2];
    *signalRssiPkt = buf[3];
}

void sx126x_getRssiInst(uint8_t *rssiInst)
{
    uint8_t buf[2];
    sx126x_transfer(0x15, buf, 2);
    *rssiInst = buf[1];
}

void sx126x_getStats(uint16_t *nbPktReceived, uint16_t *nbPktCrcError, uint16_t *nbPktHeaderErr)
{
    uint8_t buf[7];
    sx126x_transfer(0x10, buf, 7);
    *nbPktReceived = (buf[1] << 8) | buf[2];
    *nbPktCrcError = (buf[3] << 8) | buf[4];
    *nbPktHeaderErr = (buf[5] << 8) | buf[6];
}

void sx126x_resetStats()
{
    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    sx126x_transfer(0x00, buf, 6);
}

void sx126x_getDeviceErrors(uint16_t *opError)
{
    uint8_t buf[3];
    sx126x_transfer(0x17, buf, 3);
    *opError = buf[2];
}

void sx126x_clearDeviceErrors()
{
    uint8_t buf[2] = {0, 0};
    sx126x_transfer(0x07, buf, 2);
}

void sx126x_fixLoRaBw500(uint32_t bw)
{
    uint8_t packetType;
    sx126x_getPacketType(&packetType);
    uint8_t value;
    sx126x_readRegister(SX126X_REG_TX_MODULATION, &value, 1);
    if ((packetType == SX126X_LORA_MODEM) && (bw == 500000))
        value &= 0xFB;
    else
        value |= 0x04;
    sx126x_writeRegister(SX126X_REG_TX_MODULATION, &value, 1);
}

void sx126x_fixResistanceAntenna()
{
    uint8_t value;
    sx126x_readRegister(SX126X_REG_TX_CLAMP_CONFIG, &value, 1);
    value |= 0x1E;
    sx126x_writeRegister(SX126X_REG_TX_CLAMP_CONFIG, &value, 1);
}

void sx126x_fixRxTimeout()
{
    uint8_t value = 0x00;
    sx126x_writeRegister(SX126X_REG_RTC_CONTROL, &value, 1);
    sx126x_readRegister(SX126X_REG_EVENT_MASK, &value, 1);
    value = value | 0x02;
    sx126x_writeRegister(SX126X_REG_EVENT_MASK, &value, 1);
}

/*
void sx126x_fixInvertedIq(uint8_t invertIq)
{
    uint8_t value;
    sx126x_readRegister(SX126X_REG_IQ_POLARITY_SETUP, &value, 1);
    if (invertIq)
        value |= 0x04;
    else
        value &= 0xFB;
    sx126x_writeRegister(SX126X_REG_IQ_POLARITY_SETUP, &value, 1);
}
*/
void sx126x_transfer(uint8_t opCode, uint8_t *data, uint8_t nData)
{
    sx126x_transfer(opCode, data, nData, NULL, 0, true);
}

void sx126x_transfer(uint8_t opCode, uint8_t *data, uint8_t nData, uint8_t *address, uint8_t nAddress, bool read)
{
    if (sx126x_busyCheck(SX126X_BUSY_TIMEOUT)){
        return;
    }    
    gpio_put(config.pinSpiCs, 0);
    uint8_t op= opCode;
    spi_write_blocking(SPI_PORT, &op, 1); // sx126x_spi->transfer(opCode);
    if (nAddress > 0)
        spi_write_blocking(SPI_PORT, address, nAddress); // for (int8_t i=0; i<nAddress; i++) // sx126x_spi->transfer(address[i]);
    if (nData > 0)
    { //                                                 for (int8_t i=0; i<nData; i++) {
        if (read)
        {
            spi_write_read_blocking(SPI_PORT,data,data,nData);  //SPI,src,des, len
        }
        else
        {
            spi_write_blocking(SPI_PORT, data, nData); //       else sx126x_spi->transfer(data[i]
        }
    }
    gpio_put(config.pinSpiCs, 1);
}
#endif