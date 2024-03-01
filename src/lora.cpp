#include "lora.h"
#include "hardware/spi.h"
#include "tools.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "config.h"
#include "param.h"
#include "gps.h"

// At oXs side, the principle is to set Lora device in continuous receive mode
// In a loop, we look if a packet has been received (if so, it is one byte long with the Tx power for next transmit)
// When oXs get this byte, it transmits immediately RSSI, SNR, and GPS data (0 if no GPS) and GPS precision and then go back to receive mode 
// receiving and sending are done on 2 different frequencies in order to reduce the channel occupation 



// Process depends on loraState; it can be
#define LORA_TO_INIT 0           // device must be initialized
#define LORA_IN_SLEEP 1          // wait a delay and set lora in recieve continous mode
#define LORA_IN_RECEIVE 2             // wait that a package has been received or a max delay; if package has been received,Tx power changes, update Tx power, change mode to LORA_TO_TRANSMIT
#define LORA_TO_TRANSMIT 3           //fill lora with data to be send and ask for sending (but do not wait), change mode to LORA_WAIT_END_OF_TRANSMIT
#define LORA_WAIT_END_OF_TRANSMIT 4   //wait that pakage has been sent (or wait max x sec)



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

uint8_t loraRxPacketRssi ;
uint8_t loraRxPacketSnr ;
uint8_t loraRxPacketType ;
uint8_t loraRxPacketTxPower ;
uint8_t loraState = 0 ;

bool locatorInstalled = false;

extern GPS gps;
extern uint32_t GPS_last_fix_millis ; // time when last fix has been received (used by lora locator) 
extern int32_t GPS_last_fix_lon ;     // last lon when a fix has been received
extern int32_t GPS_last_fix_lat ;     // last lat when a fix has been received
extern uint16_t GPS_hdop ;

extern CONFIG config;
//#define PIN_SPI_CS   9
//#define PIN_SPI_SCK  10
//#define PIN_MOSI 11
//#define PIN_MISO 12
#define SPI_PORT spi1

void initSpi(){     // configure the SPI
    spi_init(SPI_PORT, 1000 * 1000);
    gpio_set_function(config.pinSpiMiso, GPIO_FUNC_SPI);
    gpio_set_function(config.pinSpiSck, GPIO_FUNC_SPI);
    gpio_set_function(config.pinSpiMosi, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(config.pinSpiCs);
    gpio_set_dir(config.pinSpiCs, GPIO_OUT);
    gpio_put(config.pinSpiCs, 1);
    
//  DDRB |= (1<<2)|(1<<3)|(1<<5);    // SCK, MOSI and SS as outputs on PORTB (PB2,PB3,PB5)
//  DDRB &= ~(1<<4);                 // MISO as input (PB4)
//  PORTB |= (1<<2)  ;                // Set chip select high (PB2)
  
//  SPCR =  (1<<MSTR) |               // Set as Master, send most significant bit first (because bit is left on 0)
//          (1<<SPR0) |               // divided clock by 16 (so 1mhz)
//          (1<<SPE);                 // Enable SPI, disable interrupt (bit7 on 0)
//  SPSR = 1 ;                        // Bit0 allows to multiply frequency by 2 (in master mode)

    // here we can read a register (like the version ) to check if the module is connected
    uint8_t loraVersion = loraReadRegister(LORA_REG_VERSION); 
    if( loraVersion != 0x12) {
        locatorInstalled = false;
    } else {
        locatorInstalled = true;
    }
}

//shifts out 8 bits of data
//  uint8_t data - the data to be shifted out
//  returns uint8_t - the data received during sending
uint8_t spiSend(uint8_t value){
  uint8_t response;
  spi_write_read_blocking(SPI_PORT, &value, &response, 1);
  return response;
  //uint8_t result;
  //SPDR = value; //shift the first byte of the value
  //while(!(SPSR & (1<<SPIF))); //wait for the SPI bus to finish
  //result = SPDR; //get the received data
  //return result;
}

//#define SPI_SELECT (PORTB &= ~(1<<2) ) // macro for selecting LORA device
//#define SPI_UNSELECT (PORTB |= (1<<2) ) // macro for unselecting LORA device  

#define SPI_SELECT (gpio_put(config.pinSpiCs, 0)) // macro for selecting LORA device
#define SPI_UNSELECT (gpio_put(config.pinSpiCs, 1)) // macro for unselecting LORA device  

uint8_t loraSingleTransfer(uint8_t reg, uint8_t value) {  // only for internal use; Write and read one LORA register
  uint8_t response;
  SPI_SELECT ;
  spi_write_blocking(SPI_PORT, &reg, 1);
  spi_write_read_blocking(SPI_PORT, &value, &response, 1);
  SPI_UNSELECT ;
  return response;
}

void loraWriteRegister(uint8_t reg, uint8_t value) {   // write a LORA register, discard the read value
  loraSingleTransfer(reg | 0x80, value);
}

uint8_t loraReadRegister(uint8_t reg) {                // Read a LORA register; send a dummy value because it is just a read
  return loraSingleTransfer(reg & 0x7f, 0x00);
}

void loraReadRegisterBurst( uint8_t reg, uint8_t* dataIn, uint8_t numBytes) {
  SPI_SELECT ;
  uint8_t readReg = reg  & 0x7f; 
  spi_write_blocking(SPI_PORT, &readReg, 1);
  spi_read_blocking (SPI_PORT, 0, dataIn, numBytes);
  //spiSend(reg & 0x7f);
  //for(size_t n = 0; n < numBytes; n++) {
  //      dataIn[n] = spiSend(0x00);
  //}
  SPI_UNSELECT ;
}

void loraWriteRegisterBurst( uint8_t reg, uint8_t* dataOut, uint8_t numBytes) {
  SPI_SELECT ;
  uint8_t writeReg = reg | 0x80; 
  spi_write_blocking(SPI_PORT, &writeReg, 1);
  spi_write_blocking(SPI_PORT, dataOut, numBytes); 
  //spiSend(reg | 0x80);
  //for(size_t n = 0; n < numBytes; n++) {
  //      spiSend(dataOut[n]);
  //}
  SPI_UNSELECT ;
}




void loraHandle(){   // this function is called from main.cpp only when pinSpiCs is not equal to 255
  uint8_t loraIrqFlags ;
  static uint32_t loraStateMillis ;
  //static uint32_t receiveMillis ;
  uint32_t currentMillis = millisRp() ;
  //printf("%i\n",loraState);
  //printf("%u\n",loraStateMillis ) ;
  switch (loraState) { 
    case  LORA_TO_INIT :
        initSpi();   // init spi
        loraSetup() ; // init lora with some set up that need to be done only once
        loraState = LORA_IN_SLEEP ;
        loraStateMillis = currentMillis + SLEEP_TIME ;
        //printf("End of init\n") ;
        break ;
    case  LORA_IN_SLEEP :
        if (currentMillis > loraStateMillis ){ 
          //printf("End of sleep; enter receive mode for 5 sec\n");
          //printf("Li ");
          loraRxOn(); 
          loraState = LORA_IN_RECEIVE ; 
          loraStateMillis = currentMillis + SHORT_RECEIVE_TIME ;
        }
        break;    
    case  LORA_IN_RECEIVE :
        // check if a packet has been received with a correct CRC
        loraIrqFlags = loraReadRegister(LORA_REG_IRQ_FLAGS);
        if ( loraIrqFlags & IRQ_RX_DONE_MASK  ) {
          if ( loraIrqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) {
            printf("wrong crc received\n") ;
            loraRxOn() ; // restart reading in continous mode if CRC is wrong (reset address and interrupt)
          } else {
              loraReadPacket() ; // read the data in fifo (type and TxPower) and Rssi+SNR 
              loraInSleep() ;
              loraState = LORA_TO_TRANSMIT;
              //printf("packet received\n") ;
                //printf("Re ");
          }
        } else if (currentMillis > loraStateMillis) {
           //printf("receive timeout; go to sleep\n") ;
            //printf("Sl \n");
           loraInSleep() ;
           loraState = LORA_IN_SLEEP;
           loraStateMillis = currentMillis + SLEEP_TIME ;
        }
        break;
    case  LORA_TO_TRANSMIT :
        loraFillTxPacket() ; // set mode to standby, fill fifo with data to be sent (6 bytes)  
        loraTxOn(loraRxPacketTxPower) ; // set TxOn  (adjust frequency, number of bytes, Txpower, start Tx)  // set lora in transmit mode
        loraStateMillis = currentMillis + 400 ; // start a timeout ; normally sending is done in less than 200msec
        loraState = LORA_WAIT_END_OF_TRANSMIT ;
        //printf("packet redy for sending\n") ;
        //printf("Se "); 
        break;
    case  LORA_WAIT_END_OF_TRANSMIT :
        // check if transmit is done or if timeout occurs
        // if transmitted, put lora in receive, change loraState to LORA_IN_RECEIVE , change loraStateMillis = currentMillis+LONG_RECEIVE
        // else, if timeOut, go in sleep for the SLEEP_TIME
        loraIrqFlags = loraReadRegister(LORA_REG_IRQ_FLAGS);
        if ( loraIrqFlags & IRQ_TX_DONE_MASK ) {
            //printf("packet has been sent; go to receive for 60sec\n") ;
            //printf("Es\n");
            loraRxOn();
            loraState = LORA_IN_RECEIVE ; 
            loraStateMillis = currentMillis + LONG_RECEIVE_TIME ;
        } else if ( currentMillis > loraStateMillis ) {
            //printf("transmit timeout; go to sleep for 55sec\n") ;
            loraInSleep() ;
            loraState = LORA_IN_SLEEP;
            loraStateMillis = currentMillis + SLEEP_TIME ;
        }
        
        break;

    } // end of switch
}

void loraSetup() {         // parameters that are set only once
  loraWriteRegister(LORA_REG_OP_MODE,  0x00);           //RegOpMode, need to set to sleep mode before configure for LoRa mode
  loraWriteRegister(LORA_REG_OP_MODE,  0x80);           // activate LORA mode and High frequency, stay in sleep mode
  loraWriteRegister(LORA_REG_PA_CONFIG, 0x8F) ;         // activate PA_Boost and max power = 17 dBm
  //loraWriteRegister(LORA_REG_PA_RAMP, 0x09)) ;         // keep default value
  loraWriteRegister(LORA_REG_OCP, 0b00111011) ;         // current protection On with max value of 240 mA
  loraWriteRegister(LORA_REG_LNA, 0x23) ;               // max gain in reception , boost on LNA
  loraWriteRegister(LORA_REG_MODEM_CONFIG_1,0b01110011) ;// BW=125khz,CR=4/5,No header
                                                          // bits 7-4 = BW; 0110 = 62.5Khz; 0111=125Khz
                                                          // bits 3-1 = CR ; 001 = 4/5; 100=4/8
                                                          //bit 0: 0=Explicit Header; 1=no Header
  loraWriteRegister(LORA_REG_MODEM_CONFIG_2,0b10100100) ; // SF=10,One Tx packet,CrcOn, timeOutMsb=0
                                                          // bits 7-4=SF ; from 6 up to 12
                                                          // bit 3=TxContinous mode;0=one packet only
                                                          // bit2=RxPayloadCrcON ; 1=Enable
                                                          // bits1-0= SymbTimeOut(9:8) = MSB
  loraWriteRegister(LORA_REG_SYMB_TIMEOUT_LSB, 0xFF) ;         // Receiver time out in single mode
  //loraWriteRegister(LORA_REG_PREAMBLE_MSB, 0x00) ;     // keep 00 default value
  loraWriteRegister(LORA_REG_PREAMBLE_LSB, 6) ;          // 6 preamble sysbols 
  loraWriteRegister(LORA_REG_MODEM_CONFIG_3, 0b00000100);  //
                                                          // bit3=LowDataRateOptimize; 1=Enabled mandated when symbol length exceeds 16ms
                                                          // bit2=AgcAutoOn; 1=LNA gain set by internal AGCloop instead of by register LnaGain
}

void loraTxOn(uint8_t txPower){                           // if TX power <= 15, do not use the boost
      loraWriteRegister(LORA_REG_OP_MODE,  0x80 | LORA_STANDBY);
    if ( txPower <= 15) {
        loraWriteRegister(LORA_REG_PA_CONFIG, 0x80 | txPower) ; // use PA_boost (power is from 2 up to 17dBm
    } else { 
        loraWriteRegister(LORA_REG_PA_CONFIG, 0x80 | 15) ; // use 15 as max
    }
    loraWriteRegister(LORA_REG_PA_DAC , 0x84 ) ;            // 0x84 = normal power (up to 17 dBm); 0x87= boost (20dBm)  
    loraWriteRegister(LORA_REG_FRF_MSB, TX_FRF_MSB);    //frequency (in steps of 61.035 Hz)
    loraWriteRegister(LORA_REG_FRF_MID, TX_FRF_MID);      
    loraWriteRegister(LORA_REG_FRF_LSB, TX_FRF_LSB);
    loraWriteRegister(LORA_REG_IRQ_FLAGS, 0xFF);       //reset interrupt flags
    loraWriteRegister(LORA_REG_PAYLOAD_LENGTH,6);      // set payload on 6 (because it is the same time on air as 5
    loraWriteRegister(LORA_REG_OP_MODE,  0x80 | LORA_TX);  
}


void loraRxOn(){
  loraWriteRegister(LORA_REG_OP_MODE,  0x80);
  loraWriteRegister(LORA_REG_FIFO_RX_BASE_ADDR, 0x00); // reset base Rx adress to 0
  loraWriteRegister(LORA_REG_FRF_MSB, RX_FRF_MSB);    //frequency (in steps of 61.035 Hz)
  loraWriteRegister(LORA_REG_FRF_MID, RX_FRF_MID);      
  loraWriteRegister(LORA_REG_FRF_LSB, RX_FRF_LSB);
  loraWriteRegister(LORA_REG_IRQ_FLAGS, 0xFF);       //reset interrupt flags
  loraWriteRegister(LORA_REG_PAYLOAD_LENGTH,2);      // set payload on 6 (because it is the same time on air as 5
  loraWriteRegister(LORA_REG_OP_MODE,  0x80 | LORA_RXCONTINUOUS); 
}

void loraInSleep(){
  loraWriteRegister(LORA_REG_OP_MODE,  0x80);
}  

void loraReadPacket() {           // read a packet with 2 bytes ; PacketType and PacketTxPower
  loraRxPacketRssi = loraReadRegister( LORA_REG_PKT_RSSI_VALUE ); // we should substract 157 or 137 ? but this is done on the receiver side
  loraRxPacketSnr = loraReadRegister( LORA_REG_PKT_SNR_VALUE );
  loraWriteRegister(LORA_REG_FIFO_ADDR_PTR, 0);        //set RX FIFO ptr
  SPI_SELECT ;       // start of read burst on fifo
  spiSend(LORA_REG_FIFO);                //address for fifo
  loraRxPacketType = spiSend(0);         // read the first byte of the request from receiver
  loraRxPacketTxPower = spiSend(0);      // read the second byte of the request from receiver
  SPI_UNSELECT ;
  //printf("Rssi=%i Snr=%i Type=%x  pow=%x\n", loraRxPacketRssi , loraRxPacketSnr, loraRxPacketType , loraRxPacketTxPower ) ;
  //printf("Rssi=%i \n", loraRxPacketRssi);
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
  packetType = (~packetType ) & 0x80 ;                  // reverse first bit
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
  }  
  loraTxBuffer[0] = packetType | temp8 ;
  loraTxBuffer[1] = loraRxPacketRssi ; // RSSI value of last packet
  if (gps.gpsInstalled && gps.GPS_fix) {  
    if (packetType & 0x80) {
        gpsPos = GPS_last_fix_lat ;
    } else {
        gpsPos = GPS_last_fix_lon ;
    }
  }
  loraTxBuffer[2] = gpsPos >> 24 ;
  loraTxBuffer[3] = gpsPos >> 16 ;
  loraTxBuffer[4] = gpsPos >> 8 ;
  loraTxBuffer[5] = gpsPos ;
   
  loraWriteRegister(LORA_REG_OP_MODE, 0x80 | LORA_STANDBY) ; //  set mode in standby ( to write FIFO)
  loraWriteRegister(LORA_REG_FIFO_ADDR_PTR, 0x80 );        // set FifoAddrPtr to 80 (base adress of byte to transmit)
  loraWriteRegisterBurst( LORA_REG_FIFO , loraTxBuffer, 6) ; // write the 6 bytes in lora fifo
}

//#endif
