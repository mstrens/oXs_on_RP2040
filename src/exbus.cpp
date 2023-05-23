
// exbus run on a pio uart at a baud rate of 115200 8N1 not inverted
// Rx sent a frame once every 10msec 
// it contains:  
// for a Rc channels frames: 0X3E 0X01/0X03 Len PackId 0X31 SubLen n*2bytes_per_channel 2bytes_crc ; 0X01 means oXs can reply
// for a tlm request:        0X3D 0X01      Len PackId 0X3A 0X00  2bytes_crc
// sensor replies (quite soon?) when byte 1 = 0X01
// in a reply  frame :       0X3B 0X01      Len PackId 0X3A SubLen ---ex tlm----  2bytes_crc
// ---ex tlm--- : 0Xnf Type+Len 2bytesDeviceID 2bytesSubId 0X00 followed by --txt--- or --data--- CRC ; Type= 2 bits (7..6) (00=txt, 01=data) ; Len=6 bits (5...0)
//  if type = txt : SensorIdx len1_len2 ----name----  ---unit---   ; len1= 5bits for name_length ; len2= 3bits for unit_length
//  if type = data : nX(SensorIdx_dataType 1/2/3/4bytes (lsb first) ) 
//                    last byte of each data has bit 7 = 1 if negative; bits 6...5 = number of decimals 
//                    SensorIdx = bits 7...4 ; dataType = code to say if it is 1/2/3/4 bytes or data/time/long/lat
// so there are 2 levels of length and 2 types of CRC (1 and 2 bytes)
// there is no need to transmit free text data
// there are also some frame to manage the menu on tx but this is not implemented


#include <stdio.h>
#include "pico/stdlib.h"
//#include "pico/multicore.h"
#include "hardware/pio.h"
//#include "hardware/uart.h"
#include "uart_exbus_tx_rx.pio.h"
#include "pico/util/queue.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include <string.h> // used by memcpy

#include "MS5611.h"
#include "SPL06.h"
#include "ms4525.h"
#include "sdp3x.h"

#include "exbus.h"
#include "tools.h"
#include "config.h"
#include "param.h"
#include "mpu.h"


// one pio and 2 state machines are used to manage the exbus in halfduplex
// one state machine (sm) handle the TX and the second the RX
// to receive data, the sm is initialised and use an IRQ handler when rx fifo is not empty
//    in irq, this byte is store in a Rx queue
//    This queue is read in main loop
//    When a byte is received after a timeout, we start storing the data up to the total length
//    when frame is complete we check it (type/crc) and store the channel and/or send a reply
//    If we have to reply, We fill a buffer with the data
//        one reply on 16, we send txt frame with dictionary
//        in others we put several data values (if possible) in the limit of max size of the buffer
//    We set up a dma to transfer the data to the TX fifo of the Tx state machine
//    We also set up a timestamp to stop after some msec the Tx state machine and start again the Rx one   

extern field fields[];  // list of all telemetry fields and parameters used by Sport
extern MS5611 baro1;
extern SPL06 baro2;
extern MS4525 ms4525;
extern SDP3X sdp3x; 
extern GPS gps;

extern CONFIG config;
extern uint8_t debugTlm;
queue_t exbusRxQueue ;

// one pio with 2 state machine is used to manage the inverted hal duplex uart for exbus
PIO exbusPio = pio0;
uint exbusSmTx = 0; // to send the telemetry to exbus
uint exbusSmRx = 1; // to get the request from exbus
uint exbusOffsetTx ; 
uint exbusOffsetRx ; 

// dma channel is used to send exbus telemetry without blocking
int exbus_dma_chan;
dma_channel_config exbusDmaConfig;

bool exbusIsBuffering = false;
uint8_t exbusRxBuffer[100];
uint16_t exbusRcChannels[24] ; // store the Rc values received 
uint8_t exbusRxBufferIdx = 0 ;
uint8_t exbusRxBufferLen ; // expected length of frame
uint8_t exbusChanCount ;   // says the number of channels (8,16 or 24) in Rc frame
uint8_t exbusPacketId =0;  // store the packetId received from RX and to use in the tlm frame

uint8_t exbusTxBuffer[60] = {      // all constant are prefilled
    // fix    fix   Len   pckId  fix   SuLen  fix  typ+len ID1   ID2   ID3   ID4   fix
    //  0     1     2     3      4     5      6     7     8     9     10    11     12
        0X3B, 0X01, 0X00, 0X00 , 0X3A, 0X00 , 0X9F, 0X00, 0x11, 0xA4, 0xAD, 0x04, 0x00
    };

uint32_t restoreExbusPioToReceiveMillis = 0; // when 0, the pio is normally in receive mode,
                                        // otherwise, it is the timestamp when pio transmit has to be restore to receive mode

extern field fields[];  // list of all telemetry fields that are measured

uint32_t exbusRxMicros;

extern MPU mpu;

extern bool exbusPriMissingFlag ;
extern bool exbusSecMissingFlag ;
extern bool exbusPriFailsafeFlag ;
extern bool exbusSecFailsafeFlag ;
extern uint32_t lastRcChannels ;
extern uint32_t lastPriChannelsMillis ;
extern uint32_t lastSecChannelsMillis; 
extern sbusFrame_s sbusFrame; // full frame including header and End bytes; To generate PWM , we use only the RcChannels part.
extern sbusFrame_s sbus2Frame; // full frame including header and End bytes; To generate PWM , we use only the RcChannels part.

uint8_t exbusFieldList[NUMBER_MAX_IDX+1];
uint8_t exbusMaxFields = 0;
//uint8_t exbusDataFieldIdx = 0;
//uint8_t exbusTxtFieldIdx = 0;

// sensor definition (max. 31 for DC/DS-16)
// name plus unit must be < 20 characters
// precision = 0 --> 0, precision = 1 --> 0.0, precision = 2 --> 0.00
JETISENSOR_CONST sensorsParam[] =
{
	// id         name          unit         data type      precision  length 
	{ 1      , "Gps Lat"     , "\xB0"     , EXBUS_TYPE_GPS ,       0 , 5},   //   LATITUDE   //  GPS special format
    { 2      , "Gps Long"    , "\xB0"     , EXBUS_TYPE_GPS ,       0 , 5},  //LONGITUDE,    //  GPS special format
    { 3      , "Gps speed"   , "km/h"     , EXBUS_TYPE_14  ,       1 , 3},  //GROUNDSPEED , //  GPS cm/s
    { 4      , "Gps Heading" , "\xB0"     , EXBUS_TYPE_14  ,       0 , 3},  //HEADING,      //  GPS 0.01 degree
    { 5      , "Gps Alt"     , "m"        , EXBUS_TYPE_14  ,       0 , 3},  //ALTITUDE ,    //  GPS cm
    { 6      , "Gps Sat"     , " "        , EXBUS_TYPE_14  ,       0 , 3},  //NUMSAT ,      //  5 GPS no unit   
    { 0xFF   , " "           , " "        , EXBUS_TYPE_NONE,       0 , 0},  //  GPS_DATE ,    // GPS special format AAMMJJFF
    { 0xFF   , " "           , " "        , EXBUS_TYPE_NONE,       0 , 0},  //  GPS_TIME ,    // GPS special format HHMMSS00
    { 0xFF   , " "           , " "        , EXBUS_TYPE_NONE,       0 , 0},  //  GPS_PDOP ,    // GPS no unit
    { 0xFF   , " "           , " "        , EXBUS_TYPE_NONE,       0 , 0},  //  GPS_HOME_BEARING, // GPS degree

    { 0xFF   , " "           , " "        , EXBUS_TYPE_NONE,       0 , 0},  //  GPS_HOME_DISTANCE, // 10 GPS  in m
    { 10     , "Volt 1"     , "V"         , EXBUS_TYPE_14 ,        2 , 3},  // MVOLT,        // volt1   in mVolt
    { 13     , "Current"    , "A"         , EXBUS_TYPE_22 ,        2 , 4},  // CURRENT,  // volt2 must be in seq for voltage.cpp in mA (mV)
    { 11     , "Volt 3"     , "V"         , EXBUS_TYPE_14 ,        2 , 3},  //RESERVE1, // volt3 must be in seq for voltage.cpp in mV
    { 12     , "Volt 4"     , "V"         , EXBUS_TYPE_14 ,        2 , 3},  //RESERVE2, // volt4 must be in seq for voltage.cpp in mV
    { 14     , "Consumption", "mAh"       , EXBUS_TYPE_22 ,        0 , 4},  //CAPACITY,    // based on current (volt2) in mAh
    { 15     , "Temp 1"     , "\xB0\x43"  , EXBUS_TYPE_14 ,        0 , 3},  //TEMP1,       // = Volt3 but saved as temp in degree
    { 16     , "Temp 2"     , "\xB0\x43"  , EXBUS_TYPE_14 ,        0 , 3},  //TEMP2,       // = Volt4 but saved as temp in degree
    { 17     , "Vspeed"     , "m/s"       , EXBUS_TYPE_14 ,        2 , 3},  //VSPEED,      // baro       in cm/s
    { 18     , "Alt"        , "m"         , EXBUS_TYPE_22 ,        1 , 4},  //RELATIVEALT , // baro      in cm
      
    { 19     , "Pitch"      , "\xB0"      , EXBUS_TYPE_14 ,        0 , 3},  //PITCH,       // 20 imu        in degree 
    { 20     , "Roll"       , "\xB0"      , EXBUS_TYPE_14 ,        0 , 3},  //ROLL,       // 20 imu        in degree 
    { 0xFF   , " "           , " "        , EXBUS_TYPE_NONE,       0 , 0},  //{ 21         , "Yaw"        , "\xB0"      , EXBUS_TYPE_14 ,        0 },  //YAW,       // 20 imu        in degree 
    { 22     , "Rpm"        , "t/min"     , EXBUS_TYPE_22 ,        0 , 4},  //RPM ,        // RPM sensor    in Herzt
    { 0xFF   , " "           , " "        , EXBUS_TYPE_NONE,       0 , 0},  //  ADS_1_1,      // Voltage provided by ads1115 nr 1 on pin 1
 
    { 0xFF   , " "           , " "        , EXBUS_TYPE_NONE,       0 , 0},  //  ADS_1_2,      // Voltage provided by ads1115 nr 1 on pin 2    25
    { 0xFF   , " "           , " "        , EXBUS_TYPE_NONE,       0 , 0},  //  ADS_1_3,      // Voltage provided by ads1115 nr 1 on pin 3
    { 0xFF   , " "           , " "        , EXBUS_TYPE_NONE,       0 , 0},  //  ADS_1_4,      // Voltage provided by ads1115 nr 1 on pin 4
    { 0xFF   , " "           , " "        , EXBUS_TYPE_NONE,       0 , 0},  //  ADS_2_1,      // Voltage provided by ads1115 nr 2 on pin 1
    { 0xFF   , " "           , " "        , EXBUS_TYPE_NONE,       0 , 0},  //  ADS_2_2,      // Voltage provided by ads1115 nr 2 on pin 2
      
    { 0xFF   , " "           , " "        , EXBUS_TYPE_NONE,       0 , 0},  //  ADS_2_3,      // Voltage provided by ads1115 nr 2 on pin 3    30
    { 0xFF   , " "           , " "        , EXBUS_TYPE_NONE,       0 , 0},  //  ADS_2_4,      // Voltage provided by ads1115 nr 2 on pin 4
    { 23     , "Airspeed"  , "Km/h"      , EXBUS_TYPE_14 ,         0 , 3},  //AIRSPEED,
    { 24     , "Comp Vspeed" , "m/s"     , EXBUS_TYPE_14 ,         2 , 3},  //AIRSPEED_COMPENSATED_VSPEED,
    { 0xFF   , " "           , " "        , EXBUS_TYPE_NONE,       0 , 0},  //  SBUS_HOLD_COUNTER,

    { 0xFF   , " "           , " "        , EXBUS_TYPE_NONE,       0 , 0},  //  SBUS_FAILSAFE_COUNTER,
    { 25    , "Gps cum. dist." , "km"        , EXBUS_TYPE_14,      1 , 3},  //  GPS cumulative distance,
    { 0      , "oXs"         , " "        , EXBUS_TYPE_DEVICE,     0 , 0},  // identify the name of the device      
};

void setupExbusList(bool activateAllFields){
    exbusFieldList[0] = NUMBER_MAX_IDX;  // index of the name "oXs" = last in the list
    exbusMaxFields = 1;
    if (( config.pinGpsTx != 255 ) ||  activateAllFields) {
        exbusFieldList[exbusMaxFields++] = LATITUDE ;  
        exbusFieldList[exbusMaxFields++] = LONGITUDE ;
        exbusFieldList[exbusMaxFields++] = GROUNDSPEED ;
        exbusFieldList[exbusMaxFields++] = HEADING ;
        exbusFieldList[exbusMaxFields++] = ALTITUDE ; 
        exbusFieldList[exbusMaxFields++] = NUMSAT ;
        exbusFieldList[exbusMaxFields++] = GPS_CUMUL_DIST ; 
    }
    if (( config.pinVolt[0] != 255)  ||  activateAllFields){
        exbusFieldList[exbusMaxFields++] = MVOLT ;
    }
    if (( config.pinVolt[1] != 255)  ||  activateAllFields){
        exbusFieldList[exbusMaxFields++] = CURRENT ;
        exbusFieldList[exbusMaxFields++] = CAPACITY ;
    }
    if ((( config.pinVolt[2] != 255)  && (config.temperature == 0 || config.temperature == 255)) ||  activateAllFields) {
        exbusFieldList[exbusMaxFields++] = RESERVE1 ;
    }
    if ((( config.pinVolt[3] != 255)  && ( config.temperature != 2))  ||  activateAllFields){
        exbusFieldList[exbusMaxFields++] = RESERVE2 ;
    }
    if ((( config.pinVolt[2] != 255)  && (config.temperature == 1 || config.temperature == 2) ) ||  activateAllFields) {
        exbusFieldList[exbusMaxFields++] = TEMP1 ;
    }
    if ((( config.pinVolt[3] != 255)  && (config.temperature == 2) ) ||  activateAllFields) {
        exbusFieldList[exbusMaxFields++] = TEMP2 ;
    }
    if (( baro1.baroInstalled || baro2.baroInstalled || baro1.baroInstalled)  ||  activateAllFields) {
        exbusFieldList[exbusMaxFields++] = RELATIVEALT ; 
        exbusFieldList[exbusMaxFields++] = VSPEED ;
    }
    if ((mpu.mpuInstalled)  ||  activateAllFields) {
        exbusFieldList[exbusMaxFields++] = PITCH ; 
        exbusFieldList[exbusMaxFields++] = ROLL ;
    }
    if (( config.pinRpm != 255 )  ||  activateAllFields) {
        exbusFieldList[exbusMaxFields++] = RPM ; 
    }
    if (( ms4525.airspeedInstalled || sdp3x.airspeedInstalled)  ||  activateAllFields) {
        exbusFieldList[exbusMaxFields++] = AIRSPEED ; 
    }
    if (( ( ms4525.airspeedInstalled || sdp3x.airspeedInstalled) && ( baro1.baroInstalled || baro2.baroInstalled || baro1.baroInstalled) )  ||  activateAllFields) {
        exbusFieldList[exbusMaxFields++] = AIRSPEED_COMPENSATED_VSPEED ; 
    }
    //exbusMaxFields-- ;
    uint16_t temp =  0x8000;
    for (uint8_t i = 0; i<24; i++) {
        exbusRcChannels[i] = temp; 
    }
    //#define EXBUS_PRINT_FIELDLIST
    #ifdef EXBUS_PRINT_FIELDLIST
    printf("exbusFieldList ");
    for (uint8_t i = 0; i< exbusMaxFields ; i++){
        printf(" %2X",exbusFieldList[i] );
    }
    printf("\n");
    #endif
}

void setupExbus() {
// configure some tables to manage priorities and exbus fields codes used by exbus    
    setupExbusList(false); 
// configure the queue to get the data from exbus in the irq handle (2 bytes because MSB says if there was a timeout; help to find start of frame)
    queue_init (&exbusRxQueue, sizeof(uint16_t), 250);

// set up the DMA but do not yet start it to send data to exbus
// Configure a channel to write the same byte (8 bits) repeatedly to PIO0
// SM0's TX FIFO, placed by the data request signal from that peripheral.
    exbus_dma_chan = dma_claim_unused_channel(true);
    exbusDmaConfig = dma_channel_get_default_config(exbus_dma_chan);
    channel_config_set_read_increment(&exbusDmaConfig, true);
    channel_config_set_write_increment(&exbusDmaConfig, false);
    channel_config_set_dreq(&exbusDmaConfig, DREQ_PIO0_TX0);  // use state machine 0 
    channel_config_set_transfer_data_size(&exbusDmaConfig, DMA_SIZE_8);
    dma_channel_configure(
        exbus_dma_chan,
        &exbusDmaConfig,
        &pio0_hw->txf[0], // Write address (only need to set this once)
        &exbusTxBuffer[0],   // we use always the same buffer             
        0 , // do not yet provide the number of bytes (DMA cycles)
        false             // Don't start yet
    );
// Set up the state machine for transmit but do not yet start it (it starts only when a request from receiver is received)
    exbusOffsetTx = pio_add_program(exbusPio, &exbus_uart_tx_program);
    exbus_uart_tx_program_init(exbusPio, exbusSmTx, exbusOffsetTx, config.pinPrimIn, 125000 , false); // we use the same pin and baud rate for tx and rx, true means thet UART is inverted 

// set an irq on pio to handle a received byte
    irq_set_exclusive_handler( PIO0_IRQ_0 , exbusPioRxHandlerIrq) ;
    irq_set_enabled (PIO0_IRQ_0 , true) ;

// Set up the state machine we're going to use to receive them.
    exbusOffsetRx = pio_add_program(exbusPio, &exbus_uart_rx_program);
    exbus_uart_rx_program_init(exbusPio, exbusSmRx, exbusOffsetRx, config.pinPrimIn, 125000 , false);  
}


#define EXBUS_RX_FREE_MICROS 200
void exbusPioRxHandlerIrq(){    // when a byte is received on the exbus, read the pio exbus fifo and push the data to a queue (to be processed in the main loop)
  // clear the irq flag
  irq_clear (PIO0_IRQ_0 );
  uint32_t nowMicros = microsRp();
  while (  ! pio_sm_is_rx_fifo_empty (exbusPio ,exbusSmRx)){ // when some data have been received
     uint16_t c = pio_sm_get (exbusPio , exbusSmRx) >> 24;         // read the data
     //when previous byte was received more than X usec after the previous, then add 1 in bit 15 
    if ( ( nowMicros - exbusRxMicros) > EXBUS_RX_FREE_MICROS ) c |= 0X8000 ;
    queue_try_add (&exbusRxQueue, &c);          // push to the queue
    exbusRxMicros = nowMicros;                    // save the timestamp.
  }
}

void handleExbusRxTx(void){   // main loop : restore receiving mode , wait for tlm request, prepare frame, start pio and dma to transmit it
    //static uint8_t previous = 0;
    //#define SIMULATE_RX_EXBUS
    #ifdef SIMULATE_RX_EXBUS
    static uint8_t exbusRcChannelsSimulation[] = {
        0x3E, 0x03, 0x28, 0x06, 0x31, 0x20, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F,
        0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x82, 0x1F, 0x4F, 0xE2
    };
    static uint8_t exbusTlmRequestSimulation[] = { 0x3D, 0x01, 0x08, 0x06, 0x3A, 0x00, 0x98, 0x81 };
    
    // use to check that CRC is OK; the example is copied from jeti doc
    //static uint8_t exbusTlmExample[] = {0x3B, 0x01, 0x20, 0x08, 0x3A, 0x18, 0x9F, 0x56, 0x00, 0xA4, 0x51, 0x55, 0xEE, 0x11, 0x30, 0x20, 0x21,
    //    0x00, 0x40, 0x34, 0xA3, 0x28, 0x00, 0x41, 0x00, 0x00, 0x51, 0x18, 0x00, 0x09, 0x91, 0xD6};
    //uint16_t crcCalc = 0;
	//for (int i = 0; i < ( exbusTlmExample[2]-2 ); i++) crcCalc = exbusCrc16Update(crcCalc,exbusTlmExample[i]);
	//printf("CRC=%x\n",crcCalc);

    static uint32_t exbusLastSimulationMs = 0;
    if ( (millisRp() - exbusLastSimulationMs) > 999 ) { // send a message once every 10 ms
        //printf("simulation fill queue\n");
        exbusLastSimulationMs = millisRp();
        uint16_t c = exbusRcChannelsSimulation[0] | 0X8000;
        queue_try_add (&exbusRxQueue, &c);          // push to the queue
        for (uint8_t i = 1; i < sizeof(exbusRcChannelsSimulation); i++){
            c = exbusRcChannelsSimulation[i] ;
            queue_try_add (&exbusRxQueue, &c);
        }
        
        //c = exbusTlmRequestSimulation[0] | 0X8000;
        c = exbusTlmRequestSimulation[0] ;
        
        queue_try_add (&exbusRxQueue, &c);          // push to the queue
        for (uint8_t i = 1; i < sizeof(exbusTlmRequestSimulation); i++){
            c = exbusTlmRequestSimulation[i] ;
            queue_try_add (&exbusRxQueue, &c);
        }
    }
    #endif // end of simulation
            
    uint16_t data;
    if (config.pinPrimIn == 255) return ; // skip when Tlm is not foreseen
    if ( restoreExbusPioToReceiveMillis) {            // put sm back in receive mode after some delay
        if (millisRp() > restoreExbusPioToReceiveMillis){
            exbus_uart_tx_program_stop(exbusPio, exbusSmTx, config.pinPrimIn );
            exbus_uart_rx_program_restart(exbusPio, exbusSmRx, config.pinPrimIn, false);  // true = inverted
            restoreExbusPioToReceiveMillis = 0 ;
        }
    } else {                             // when we are in receive mode
        while (! queue_is_empty(&exbusRxQueue)) {
            // we get the value in the queue
            queue_try_remove (&exbusRxQueue,&data);
            //printf("%x\n", (uint8_t) data);
            
            // if bit 15 = 1, it means that the value has been received after x usec from previous and so it must be a synchro 
            // so reset the buffer and process the byte
            if (data & 0X8000) {
                exbusRxBufferIdx = 0; 
                exbusIsBuffering = true; 
            } 
            if (exbusIsBuffering) {
                exbusProcessNextInputByte( (uint8_t) data); // process the incoming byte only when buffering
            }    
        } // end while
    }           
}

//#define MAX_CHANNELS 24

// CRC calculation
//////////////////
uint16_t exbusCrc16Update(uint16_t crc, uint8_t data)
{
	uint16_t ret_val;
	data ^= (uint8_t)(crc) & (uint8_t)(0xFF);
	data ^= data << 4;
	ret_val = ((((uint16_t)data << 8) | ((crc & 0xFF00) >> 8))
		       ^ (uint8_t)(data >> 4)
		       ^ ((uint16_t)data << 3));
	return ret_val;
}

bool exbusRxCrcCheck16(){
    // calc crc...
	uint16_t crcCalc = 0;
	for (int i = 0; i < exbusRxBufferLen-2; i++)
		crcCalc = exbusCrc16Update(crcCalc, exbusRxBuffer[i]);
	// ...and compare with crc from packet
	uint16_t crcPacket  = exbusRxBuffer[exbusRxBufferLen - 1] << 8;
	         crcPacket |= exbusRxBuffer[exbusRxBufferLen - 2];

	return (crcCalc == crcPacket);
}


bool exbusProcessNextInputByte( uint8_t c){
    // check if valid
    if ( (exbusRxBufferIdx == 0) && (c != 0x3D && c != 0x3E)) { // 3E = Rc channels, 3D = telemetry or box request
        exbusIsBuffering = false;
    }    
    if ( ( exbusRxBufferIdx == 1) &&  (c != 0x01 && c != 0x03) ) { // 01 = ok for a reply ; 03 = no reply allowed   
        exbusIsBuffering = false;
    }
    if ( exbusRxBufferIdx >= (sizeof(exbusRxBuffer) - 1 )) {     // reject if to many char
        exbusIsBuffering = false;
    }
    if ( exbusRxBufferIdx == 2) {                               // save expected number of char to receive
       exbusRxBufferLen = c;
    }  
    if ( exbusIsBuffering ) {
       exbusRxBuffer[exbusRxBufferIdx++] = c;                   // save the car   
        if ( exbusRxBufferIdx >= exbusRxBufferLen) {             // when buffer is full, check if CRC is valid
            //#define EXBUS_PRINT_RX_BUFFER
            #ifdef EXBUS_PRINT_RX_BUFFER
            printf("Rx= ");
            for (uint8_t i = 0; i< exbusRxBufferLen;i++){
                printf(" %2X", exbusRxBuffer[i]);
                if (( i & 0X07) == 7) printf("\n    "); 
            }
            printf("\n");
            #endif
            if (exbusRxCrcCheck16() == false) {
                printf("exbus rx buffer with wrong check digit\n");
                exbusIsBuffering = false;
            } else {
                exbusPacketId = exbusRxBuffer[3];              // save the packet Id (to be used in the reply)
                if ( (exbusRxBuffer[0] == 0X3E) &&  (exbusRxBuffer[4] == 0X31) ) {   // it is a frame with Rc data
						exbusDecodeRcChannels();
                        // here we do not set exbusIsBuffering on false, because, we can receive a second frame
                        exbusRxBufferIdx = 0; // we reset the buffer to zero
				} else if ( (exbusRxBuffer[0] == 0X3D) &&  (exbusRxBuffer[4] == 0X3A) ) { // packet is a telemetry request
						exbusCreateSendTelemetry();
                        exbusIsBuffering = false;  // we have to get pause in data transmission before continuing
				} else if ( (exbusRxBuffer[0] == 0X3D) &&  (exbusRxBuffer[4] == 0X3B) ) { // packet is a jetibox request
						exbusCreateSendJetiBox();
                        exbusIsBuffering = false;  // we have to get pause in data transmission before continuing
				}

					// packet is a Jetibox request
				//	else if (m_exBusBuffer[4] == 0x3b && m_bReleaseBus )
				//	{
				//		SendJetiBoxData();
				//		m_bBusReleased = true;
				//	}
            }
        }               
    }
    return true; // currently return value has no meaning
}


void exbusDecodeRcChannels(){             // channels values are coded on 2 bytes. last bit = 1/8 usec
	//printf("exbus decoding Rc channels\n");
    uint8_t sbus[23];
    uint8_t exbusNumRcChannels = exbusRxBuffer[5] >> 1 ; // number of channels = sub length / 2
    for (int i = 0; i < exbusNumRcChannels; i++) {
		exbusRcChannels[i] = ((exbusRxBuffer[6 + (i << 1)]) | (exbusRxBuffer[7 + (i << 1)] << 8 )) >> 5;   
    }
    sbus[0] = exbusRcChannels[0];
    sbus[1] = (exbusRcChannels[0] >> 8) | (exbusRcChannels[1] & 0x00FF)<<3;
    sbus[2] = exbusRcChannels[1]>>5|(exbusRcChannels[2]<<6);
    sbus[3] = (exbusRcChannels[2]>>2)& 0x00ff;
    sbus[4] = exbusRcChannels[2]>>10| (exbusRcChannels[3] & 0x00FF)<<1;
    sbus[5] = exbusRcChannels[3]>>7|  (exbusRcChannels[4] & 0x0FF )<<4;
    sbus[6] = exbusRcChannels[4]>>4| (exbusRcChannels[5] & 0xFF) <<7;
    sbus[7] = (exbusRcChannels[5]>>1)& 0x00ff;
    sbus[8] = exbusRcChannels[5]>>9| (exbusRcChannels[6] & 0xFF)<<2;
    sbus[9] = exbusRcChannels[6]>>6| (exbusRcChannels[7] & 0xFF)<<5;
    sbus[10] = (exbusRcChannels[7]>>3)& 0x00ff;//end
    sbus[11] = (exbusRcChannels[8] & 0XFF);
    sbus[12] = (exbusRcChannels[8]>> 8) | (exbusRcChannels[9] & 0xFF)<<3;
    sbus[13] = exbusRcChannels[9]>>5 | (exbusRcChannels[10]<<6);
    sbus[14] = (exbusRcChannels[10]>>2) & 0xff;
    sbus[15] = exbusRcChannels[10]>>10 | (exbusRcChannels[11] & 0XFF)<<1;
    sbus[16] = exbusRcChannels[11]>>7 | (exbusRcChannels[12] & 0XFF)<<4;
    sbus[17] = exbusRcChannels[12]>>4 | (exbusRcChannels[13] & 0XFF)<<7;
    sbus[18] = (exbusRcChannels[13]>>1)& 0xff;
    sbus[19] = exbusRcChannels[13]>>9 | (exbusRcChannels[14] & 0XFF)<<2;
    sbus[20] = exbusRcChannels[14]>>6 | (exbusRcChannels[15] & 0XFF)<<5;
    sbus[21] = (exbusRcChannels[15]>>3)& 0xff;

    sbus[22] = 0x00;
    //if ( channelOrFailsafe == SRXL2_RC_FAILSAFE) sbus[22] |= (1<<3);//FS activated   
    //    if(missingPackets >= 1) sbus[22] |= (1<<2);//frame lost
    memcpy( (uint8_t *) &sbusFrame.rcChannelsData, &sbus[0], 23) ; // copy the data to the Sbus buffer
    lastRcChannels = millisRp();
    lastPriChannelsMillis =  lastRcChannels;
} 
 
/*
    sbusPriMissingFlag = (exbusRxBuffer[24] >> 2) & 0X01;
    sbusPriFailsafeFlag = (exbusRxBuffer[24] >> 3) & 0X01;
    if ((( sbusPriMissingFlag == false) && (sbusPriFailsafeFlag == false)) || // copy when frame is OK   
        ( sbusSecFailsafeFlag)  ||                                            //   or previous SEC is failsafe
        ( ( millisRp() - lastSecChannelsMillis )  > 50 )) {                     //   or SEC do not exist                   
        memcpy(  (uint8_t *) &sbusFrame.rcChannelsData, &exbusRxBuffer[2], 22); // copy the 22 bytes of RC channels
    }
    lastRcChannels = millisRp();
    lastPriChannelsMillis =  lastRcChannels;
*/    

void exbusCreateSendTelemetry(){ // search for the next data to be sent
// exbusRxBuffer[] contains the frame with 
// in a reply  frame :       0X3B 0X01      Len PackId 0X3A SubLen ---ex tlm----  2bytes_crc
// ---ex tlm--- : 0Xnf Type+Len 2bytesDeviceID 2bytesSubId 0X00 followed by --txt--- or --data--- CRC ; Type= 2 bits (7..6) (00=txt, 01=data) ; Len=6 bits (5...0)
//  if type = txt : SensorIdx len1_len2 ----name----  ---unit---   ; len1= 5bits for name_length ; len2= 3bits for unit_length
//  if type = data : nX(SensorIdx_dataType 1/2/3/4bytes (lsb first) ) 
//                    last byte of each data has bit 7 = 1 if negative; bits 6...5 = number of decimals 
//                    SensorIdx = bits 7...4 ; dataType = code to say if it is 1/2/3/4 bytes or data/time/long/lat
// so there are 3 levels of length and 2 types of CRC (1 and 2 bytes)
    //printf("exbus creating tlm frame\n");
    
    waitUs(100); // wait a little before replying to a pooling
    if ( dma_channel_is_busy(exbus_dma_chan) ) {
        //printf("dma is busy\n");
        return ; // skip if the DMA is still sending data
    }
    exbusCreateTelemetry();  // create the frame in exbusTxBuffer[]
    //#define PRINT_EXBUS_TLM_FRAME
    #ifdef PRINT_EXBUS_TLM_FRAME
        printf("Frame=");
        for (uint8_t i = 13 ; i < (exbusTxBuffer[2]-3); i++){ // do not print the first 13 bytes nor the 3 CRC
            printf(" %2x ", exbusTxBuffer[i]);
        }
        printf("\n");
    #endif    
    // send the buffer 
    exbus_uart_rx_program_stop(exbusPio, exbusSmRx, config.pinPrimIn); // stop receiving
    exbus_uart_tx_program_start(exbusPio, exbusSmTx, config.pinPrimIn, false); // prepare to transmit ; true = invert
    // start the DMA channel with the data to transmit
    dma_channel_set_read_addr (exbus_dma_chan, &exbusTxBuffer[0], false);
    dma_channel_set_trans_count (exbus_dma_chan, exbusTxBuffer[2], true) ;
    // we need a way to set the pio back in receive mode when all bytes are sent 
    // this will be done in the main loop after some ms (here 2ms)
    restoreExbusPioToReceiveMillis = millisRp() + 4;  // to do  to check the delay for sending
}

void exbusCreateSendJetiBox(){
    //printf("exbus creating Jetibox frame\n");
    waitUs(100); // wait a little before replying to a pooling
    if ( dma_channel_is_busy(exbus_dma_chan) ) {
        //printf("dma is busy\n");
        return ; // skip if the DMA is still sending data
    }
    exbusCreateJetibox();  // create the frame in exbusTxBuffer[]
    // send the buffer    
    exbus_uart_rx_program_stop(exbusPio, exbusSmRx, config.pinPrimIn); // stop receiving
    exbus_uart_tx_program_start(exbusPio, exbusSmTx, config.pinPrimIn, false); // prepare to transmit ; true = invert
    // start the DMA channel with the data to transmit
    dma_channel_set_read_addr (exbus_dma_chan, &exbusTxBuffer[0], false);
    dma_channel_set_trans_count (exbus_dma_chan, exbusTxBuffer[2], true) ;
    // we need a way to set the pio back in receive mode when all bytes are sent 
    // this will be done in the main loop after some ms (here 2ms)
    restoreExbusPioToReceiveMillis = millisRp() + 4;  // to do  to check the delay for sending
}


uint8_t addOneValue(  uint8_t idx , uint8_t nextBufferWrite){
    uint8_t count = sensorsParam[idx].length; 
    int32_t value = fields[idx].value; 
    if ( sensorsParam[idx].id < 16) {
        exbusTxBuffer[nextBufferWrite++] = ( sensorsParam[idx].id << 4 ) | sensorsParam[idx].dataType ; 
    } else {
        exbusTxBuffer[nextBufferWrite++] =  sensorsParam[idx].dataType ;
        exbusTxBuffer[nextBufferWrite++] =  sensorsParam[idx].id ;
        count++;
    }
    switch (idx) {
        case RELATIVEALT :
            value = int_round(fields[idx].value , 10) ; // Altitude converted from cm to dm
            break ;
        case  MVOLT :
            value =  int_round(fields[idx].value  , 10) ; // converted from mV to 0.01V
            break;
        case  CURRENT :
            value =  int_round(fields[idx].value  , 10) ; // converted in A with 2 decimals
            break ;
        case HEADING :
            value = int_round(fields[idx].value  , 100000) ; // convert from degree * 100000 to degree
            break ;
        case GROUNDSPEED :
            value = ((uint32_t) fields[idx].value) * 36 /100 ;       // convert from cm/sec to 1/10 of km/h
            break ;
        case ALTITUDE : 
            value  = int_round(fields[idx].value , 100) ;                        // convert from cm to m 
            break ;
        case GPS_CUMUL_DIST : 
            value  = int_round(fields[idx].value , 10000) ;                        // convert from cm to 0.1km 
            break ;    
//      case GPS_DISTANCE :
//        if (GPS_no_fix ) return 0 ;
//        * fieldValue  = GPS_distance ;                             // keep in m
//        * dataType = JETI14_0D ;
//        break ;
        case LONGITUDE :      
            value =  exbusFormatGpsLongLat ( fields[idx].value , true ) ; // true says it is long
            break ;                          
        case LATITUDE :                                           
            value =  exbusFormatGpsLongLat (fields[idx].value , false ) ;
            //printf("lat orig=%x  jeti=%x\n", fields[idx].value , value);
            break ;
        case AIRSPEED :
            value = fields[idx].value   * 36 / 1000 ; // from cm/s to km/h
            break ;
        case RPM :
            value = fields[idx].value   * 60 ; // from Hz to RPM
            break ;  
    }
    // now value contains the formatted value in 4 bytes ; we still have to fill the buffer
    uint8_t codifJeti ;
    codifJeti = sensorsParam[idx].dataType & 0x0F ;
    switch (codifJeti) {
    case EXBUS_TYPE_30 :
        exbusTxBuffer[nextBufferWrite++] = value & 0xFF ;
        exbusTxBuffer[nextBufferWrite++] = ( value >> 8 ) & 0xFF ;
        exbusTxBuffer[nextBufferWrite++] = ( value >> 16 ) & 0xFF ;
        exbusTxBuffer[nextBufferWrite++] = ( ( value >> 24 ) & 0x1F ) | ((value < 0) ? 0x80 :0x00 )  ; 
        break ;
    case EXBUS_TYPE_22 :
        exbusTxBuffer[nextBufferWrite++] = value & 0xFF ;
        exbusTxBuffer[nextBufferWrite++] = ( value >> 8 ) & 0xFF ;
        exbusTxBuffer[nextBufferWrite++] = ( ( value >> 16 ) & 0x1F  ) | ((value < 0) ? 0x80 :0x00 )  ; 
        break ;
    case EXBUS_TYPE_14 :
        exbusTxBuffer[nextBufferWrite++] = value & 0xFF ;
        exbusTxBuffer[nextBufferWrite++] = ( ( value >> 8 ) & 0x1F ) | ((value < 0) ? 0x80 :0x00 )  ; 
        break ;
    case EXBUS_TYPE_GPS :
        exbusTxBuffer[nextBufferWrite++] = value & 0xFF ;
        exbusTxBuffer[nextBufferWrite++] = ( value >> 8 ) & 0xFF ;
        exbusTxBuffer[nextBufferWrite++] = ( value >> 16 ) & 0xFF ;
        exbusTxBuffer[nextBufferWrite++] = ( ( value >> 24 ) & 0xFF )   ; 
        break ;
    }
    if ( ( codifJeti == 8 ) || ( codifJeti <= 4 ) ) { // when it is a number, then add precision 
         exbusTxBuffer[nextBufferWrite-1] |= sensorsParam[idx].precision <<  5 ; // add the number of decimals in position 5 & 6 of the last byte
    }
    return count;
}

void exbusCreateTelemetry() {	
	static uint8_t dictIdx = 0; // index used to retrieve the TXT in exbusFieldList[]; start at 0 
    static uint8_t dataIdx = 1;  // index used to retrieve the parameter for data in exbusFieldList[]; start at 1
    static uint16_t frameCnt = 0;
    static uint32_t textFrameMask = 0X01;
    uint8_t sensorsParamIdx;
    uint8_t totalDataLen = 0;
    uint8_t nextBufferWrite;
    exbusTxBuffer[4]= 0X3A; // this byte says that it is a tlm frame and not a jetibox frame 
    //printf("creating tlm frame\n");
	// sensor name in frame 0
	if ((frameCnt & textFrameMask) == 0) { // once every N frame, send the field definition  // todo change to FF
	    sensorsParamIdx = exbusFieldList[dictIdx] ;            // retrieve the index in sensorParam[]
        exbusTxBuffer[13] = sensorsParam[sensorsParamIdx].id  ;       // index of field
        uint8_t i = 0 ;
        uint8_t k = 0 ;
        uint8_t nextBufferWrite=15;            // start writing the name at position 15 in Tx buffer   
        while( sensorsParam[sensorsParamIdx].name[i] != '\0'  ) {         // copy name and count 
            //printf(" %2X", sensorsParam[sensorsParamIdx].name[i]);
            exbusTxBuffer[nextBufferWrite++] = sensorsParam[sensorsParamIdx].name[i++];
        }
        //printf("\n");    
        while( sensorsParam[sensorsParamIdx].unit[k] != '\0'  ) {
            //printf(" %2X", sensorsParam[sensorsParamIdx].unit[k]);
            exbusTxBuffer[nextBufferWrite++] = sensorsParam[sensorsParamIdx].unit[k++];
        } 
        //printf("\ni %2X  k%2X\n", i , k);
        exbusTxBuffer[14] = (i << 3 ) | k ;           // contain the len of name and of units
        exbusTxBuffer[2]  = 18 + i + k;               // total legnth of the frame (including header and 2 bytes CRC)
        exbusTxBuffer[5]  = 10 + i + k;                // sub length
        exbusTxBuffer[7]  = 8 + i + k;                // bits 7...6 = 00 = txt ; bits 5..0 = number of folowing bytes including the 1 byte CRC 
        dictIdx++;                                    // next field being defined
        if (dictIdx >= exbusMaxFields) dictIdx = 0;
        // at this stage, we still have to calculate the 2 crc 
        if (frameCnt > (exbusMaxFields >> 2)) textFrameMask = 0x0F; // after a certain number of frames, sent text only once per 256 frame
	} else 	{   // send EX values in all other frames -------------------------------------------------
		nextBufferWrite = 13;
        uint8_t count = exbusMaxFields - 1;   // max number of fields (to avoid filling twice the same field)
        uint8_t countWritten = 0 ;
        uint8_t countWrittenTotal = 0 ;
        uint8_t lastDataIdx = 1;
        while (count--) { // we look max once into the list of fields 
            sensorsParamIdx = exbusFieldList[dataIdx] ;            // retrieve the index in sensorParam[]
            if ( fields[sensorsParamIdx].available ) {             // if the field is available, look if there is still space in the buffer
                if (( countWrittenTotal + sensorsParam[sensorsParamIdx].length) > 15 ) break; // exit if no space enough // todo check the real max value
                    // some doc says 26 bytes for the ex frame and there are 8 other bytes (7F + 2F Type, deviceID1...4, CRC)
                countWritten = addOneValue(sensorsParamIdx , nextBufferWrite) ;
                    //printf("dataIdx= %i paramIdx=%i len=%i count=%i cWritten=%i CWtot=%i\n", 
                    //(int) dataIdx, (int) sensorsParamIdx, (int) sensorsParam[sensorsParamIdx].length , (int) count, (int) countWritten,
                    //(int) countWrittenTotal );
                countWrittenTotal += countWritten ;
                nextBufferWrite += countWritten ;
                lastDataIdx = dataIdx;
                fields[sensorsParamIdx].available = false;
            } 
            dataIdx++;                                   // continue with next field
            if (dataIdx >= exbusMaxFields) dataIdx = 1;
        }
        if (countWrittenTotal) {  // when we write some value, we will start from the next one later on
            dataIdx = lastDataIdx + 1;
            if (dataIdx >= exbusMaxFields) dataIdx = 1;
        } 
        exbusTxBuffer[2] = 16 + countWrittenTotal ; // total len of the frame
        exbusTxBuffer[5] = 8 + countWrittenTotal ; // total len of the frame 
        exbusTxBuffer[7] = 0X46 + countWrittenTotal ;  // Sub length : bits 7...6 = 01 = data; bits 5...0 = length of folowing bytes including 1 byte CRC
        // at this stage, we still have to calculate the 2 crc 
    }
    exbusTxBuffer[3] = exbusPacketId ; // reuse the packet id received from RX   
    // fill one byte crc
    uint8_t crc = 0;
    for (uint8_t c = 7; c < (exbusTxBuffer[2] - 3) ; c++) crc = exbusUpdate8Crc(exbusTxBuffer[c], crc);
    exbusTxBuffer[exbusTxBuffer[2]-3 ] = crc;
    // fill 2 bytes crc
    uint16_t crcCalc = 0;
	for (int i = 0; i < ( exbusTxBuffer[2]-2 ); i++) crcCalc = exbusCrc16Update(crcCalc,exbusTxBuffer[i]);
	exbusTxBuffer[exbusTxBuffer[2]-2] = (uint8_t)(crcCalc & 0xFF);
	exbusTxBuffer[exbusTxBuffer[2]-1] = (uint8_t)(crcCalc >> 8);
    frameCnt++;
    //#define EXBUS_PRINT_TX_BUFFER
    #ifdef EXBUS_PRINT_TX_BUFFER
    printf("Tx= ");
    for (uint8_t i = 0; i< exbusTxBuffer[2];i++){
        printf(" %2X", exbusTxBuffer[i]);
        //if (( i & 0X0F) == 0X0F) printf("\n    "); 
    }
    printf("\n");
    #endif

}

void exbusCreateJetibox(){  // create the frame in exbusTxBuffer[]
    exbusTxBuffer[2] = 0X28;   // Length is always 0X28 bytes
    exbusTxBuffer[3] = exbusPacketId ;  // packet Id received in the request
    exbusTxBuffer[4] = 0X3B; // this byte says that it is a jetibox frame and not a tlm frame
    exbusTxBuffer[5] = 0X20 ;  // sub len is always 0X20 bytes
    for (uint8_t i = 6; i< (6+32); i++){
        exbusTxBuffer[i] = '+'; 
    }
    // fill 2 bytes crc
    uint16_t crcCalc = 0;
	for (int i = 0; i < ( exbusTxBuffer[2]-2 ); i++) crcCalc = exbusCrc16Update(crcCalc,exbusTxBuffer[i]);
	exbusTxBuffer[exbusTxBuffer[2]-2] = (uint8_t)(crcCalc & 0xFF);
	exbusTxBuffer[exbusTxBuffer[2]-1] = (uint8_t)(crcCalc >> 8);
    
}

uint32_t exbusFormatGpsLongLat (int32_t longLat, bool isLong ) { // return the long or latitude in Jeti format
  union   {
    uint32_t valueInteger;
    char    valueBytes[4];
  } gps;
  uint32_t degree ;
  uint16_t minX1000 ;
  uint32_t longLatE7P ;
  longLatE7P = (longLat >= 0 ? longLat : -longLat ) ;
  degree =  (uint16_t) (longLatE7P / 10000000) ; // remove 7 decimal from original value
  minX1000 =  (uint16_t ) ( (longLatE7P - (degree * 10000000)) * 6 / 1000 ) ; // keep 3 decimals to minutes
  gps.valueInteger = 0;
  gps.valueBytes[0]  = minX1000 & 0xFF;
  gps.valueBytes[1]  = ( minX1000 >> 8 ) & 0xFF;
  gps.valueBytes[2]  = degree & 0xFF;                      
  gps.valueBytes[3]  = ( degree >> 8 ) & 0x01;             
  gps.valueBytes[3] |= isLong  ? 0x20 : 0;
  gps.valueBytes[3] |= (longLat < 0) ? 0x40 : 0;
  return gps.valueInteger ;
}

// Published in "JETI Telemetry Protocol EN V1.06"
//* Jeti EX Protocol: Calculate 8-bit CRC polynomial X^8 + X^2 + X + 1
uint8_t exbusUpdate8Crc (uint8_t crc, uint8_t crc_seed)
{ 
  unsigned char crc_u;
  unsigned char i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for (i=0; i<8; i++)
    crc_u = ( crc_u & 0x80 ) ? 7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}