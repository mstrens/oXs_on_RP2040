
// fport run on a pio uart at a baud rate of 115200 8N1 inverted
// Rx sent a frame once every 9msec (with byte stuffing)
// it contains:  0X7E LEN(=0X19 if no stuff)  X000 + a sbus signal + a CRC (one byte) 7E
// it can be immediately followed by a downlink frame with 
//   0X7E LEN(=0X08 if no stuff)  0X01 PRI APP1 APP2 DATA1...DATA4 + CRC + 7E
// sensor reply within 3msec with
//   LEN(0X08 if no stuff)  0X81 PRI (= 0X10 id data or OX00 if there is no valid data) APP1 APP1 + DATA1...DATA4 + CRC
// Note :LEN does not include Header, LEN and CRC, TRAILER


//CRC byte : this byte is summed from Len field to the byte in front of CRC with carry should be 0xFF.
//Len byte : the value is the number of bytes in the frame, Head/Len/CRC bytes are not included

/* here fport2
FPort2 115200 8,n,1  (FBUS the same using 460800 baud, but may have 24 channels in it)
0x18 count after this to byte before CRC          ????? not clear if it is 0X19?????? when no stuff
0xFF Control type
0x02 SBUS, no 0x0F
0x07 0x1F 0x2B 0xA2 0x07 0x3E 0xF0 0x81 0xAF 0x7C 0xDC 0x03 0x1F 0xF8 0xC0 0x07 0x3E 0xF0 0x09 0x7C 0x15
0x00 SBUS flags
0x64 RSSI
0x38 CRC

0x08 count after this to byte before CRC
0x2F PhyID poll
0x10 prim (can be 0X00 = NULL, 0X10=data, 0X30= Read , 0X32= Write)
0x00 0x00 0x00 0x00 0x00 0x00 (these bytes may have data being sent to a sensor)
0xC0 CRC

Sensor response:
0x08 count after this to byte before CRC
0x2F PhyID
0x10 prim
0x00 0x00 Two bytes AppID
0x00 0x00 0x00 0x00 Four bytes Data
0xC0 CRC

FBUS runs at 460800 bauds
1 bytes takes 20 usec
The polling bytes follow immediately the RC channels bytes
There is 8msec between the begin of 2 RC channels frames

*/


#include <stdio.h>
#include "pico/stdlib.h"
//#include "pico/multicore.h"
#include "hardware/pio.h"
//#include "hardware/uart.h"
#include "uart_sport_tx_rx.pio.h"
#include "pico/util/queue.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "sport.h" // common for some #define
#include <string.h> // used by memcpy

#include "fport2.h"
#include "tools.h"
#include "config.h"
#include "param.h"



// one pio and 2 state machines are used to manage the fport in halfduplex
// one state machine (sm) handle the TX and the second the RX
// to receive data, the sm is initialised and use an IRQ handler when rx fifo is not empty
//    in irq, this byte is store in a Rx queue
//    This queue is read in main loop
//    When a byte is received after a 7E, then we stop the sm that receive (it means frsky made a polling)
//    We fill a buffer with the data
//    We set up a dma to transfer the data to the TX fifo of the Tx state machine
//    We also set up a timestamp to stop after some msec the Tx state machine and start again the Rx one   


#define FPORTSYNCREQUEST 0x7E
#define FPORTDEVICEID    0xE4




extern CONFIG config;
extern uint8_t debugTlm;
queue_t fportRxQueue ;

// one pio with 2 state machine is used to manage the inverted hal duplex uart for fport
PIO fportPio = pio0;
uint fportSmTx = 0; // to send the telemetry to fport
uint fportSmRx = 1; // to get the request from fport
uint fportOffsetTx ; 
uint fportOffsetRx ; 

// dma channel is used to send fport telemetry without blocking
int fport_dma_chan;
dma_channel_config fportDmaConfig;

uint8_t fportRxBuffer[50];
uint8_t fportRxBufferIdx = 0 ;
uint8_t fportLen = 0; // length of frame
uint8_t chanCount ;   // says the number of channels (8,16 or 24) in Rc frame

uint8_t fportTxBuffer[50];

uint32_t restoreFportPioToReceiveMillis = 0; // when 0, the pio is normally in receive mode,
                                        // otherwise, it is the timestamp when pio transmit has to be restore to receive mode

extern field fields[];  // list of all telemetry fields that are measured


uint16_t fportFieldId[NUMBER_MAX_IDX]; // contains the code to be used in fport to identify the field
uint8_t fportPriority[NUMBER_MAX_IDX]; // contains the list of fields in priority sequence (first = highest) 
uint8_t fportMaxPooling[NUMBER_MAX_IDX]; // contains the max number of polling allowed between 2 transmissions
uint8_t fportMinPooling[NUMBER_MAX_IDX]; // contains the min number of polling allowed between 2 transmissions
uint32_t fportLastPoolingNr[NUMBER_MAX_IDX] = {0}; // contains the last Pooling nr for each field
uint32_t fportPoolingNr= 0; // contains the current Pooling nr

uint32_t fportRxMicros;


extern bool sbusPriMissingFlag ;
extern bool sbusSecMissingFlag ;
extern bool sbusPriFailsafeFlag ;
extern bool sbusSecFailsafeFlag ;
extern uint32_t lastRcChannels ;
extern uint32_t lastPriChannelsMillis ;
extern uint32_t lastSecChannelsMillis; 
extern sbusFrame_s sbusFrame; // full frame including header and End bytes; To generate PWM , we use only the RcChannels part.
extern sbusFrame_s sbus2Frame; // full frame including header and End bytes; To generate PWM , we use only the RcChannels part.


void setupFportList(){     // table used by fport
    uint8_t temp[] = { // sequence of fields (first = highest priority to sent on fport)
        VSPEED,      // baro       in cm/s    5
        LATITUDE,  //  GPS special format     10
        LONGITUDE,    //  GPS special format  10
        GROUNDSPEED , //  GPS cm/s            20
        RELATIVEALT , // baro      in cm      20
        PITCH,       // 20 imu        in degree  20
        ROLL,        // imu           in degree  20 
        HEADING,      //  GPS 0.01 degree        50
        ALTITUDE ,    //  GPS cm                 50
        GPS_HOME_BEARING, // GPS degree          50
        GPS_HOME_DISTANCE, // 10 GPS  in m       50
        MVOLT,        // volt1   in mVolt        50
        CURRENT,  // volt2 must be in seq for voltage.cpp in mA (mV)  50
        RESERVE1, // volt3 must be in seq for voltage.cpp in mV       50
        RESERVE2, // volt4 must be in seq for voltage.cpp in mV       50
        TEMP1,       // = Volt3 but saved as temp in degree           50
        TEMP2,       // = Volt4 but saved as temp in degree           50
        RPM ,        // RPM sensor    in Herzt                        50
        YAW ,        // not used to save data  in degree              100
        NUMSAT ,      //  5 GPS no unit                               200 
        GPS_DATE ,    // GPS special format AAMMJJFF                  200
        GPS_TIME ,    // GPS special format HHMMSS00                  200
        GPS_PDOP ,    // GPS no unit                                  200
        CAPACITY,    // based on current (volt2) in mAh                200
        ADS_1_1,      // Voltage provided by ads1115 nr 1 on pin 2    200
        ADS_1_2,      // Voltage provided by ads1115 nr 1 on pin 2    200
        ADS_1_3,      // Voltage provided by ads1115 nr 1 on pin 3    200
        ADS_1_4,      // Voltage provided by ads1115 nr 1 on pin 4    200
        ADS_2_1,      // Voltage provided by ads1115 nr 2 on pin 1    200
        ADS_2_2,      // Voltage provided by ads1115 nr 2 on pin 2    200
        ADS_2_3,      // Voltage provided by ads1115 nr 2 on pin 3    200
        ADS_2_4     // Voltage provided by ads1115 nr 2 on pin 4      200
    };
    for (uint8_t i = 0; i < NUMBER_MAX_IDX ; i++){
        fportPriority[i]= temp[i];
    }
    
    // code used by fport to identify a field in a fport frame
    fportFieldId[LATITUDE] = GPS_LONG_LATI_FIRST_ID;
    fportFieldId[LONGITUDE] = GPS_LONG_LATI_FIRST_ID;
    fportFieldId[GROUNDSPEED] = GPS_SPEED_FIRST_ID;
    fportFieldId[HEADING] = GPS_COURS_FIRST_ID;
    fportFieldId[ALTITUDE] = GPS_ALT_FIRST_ID;
    fportFieldId[NUMSAT] = DIY_GPS_NUM_SAT;
    fportFieldId[GPS_DATE] = GPS_TIME_DATE_FIRST_ID;
    fportFieldId[GPS_TIME] = GPS_TIME_DATE_FIRST_ID;
    fportFieldId[GPS_PDOP] = DIY_GPS_PDOP;
    fportFieldId[GPS_HOME_DISTANCE] = DIY_GPS_HOME_DISTANCE;
    fportFieldId[GPS_HOME_BEARING] = DIY_GPS_HOME_BEARING;
    fportFieldId[MVOLT] = VFAS_FIRST_ID;
    fportFieldId[CURRENT] = CURR_FIRST_ID;
    fportFieldId[RESERVE1] = DIY_VOLT3;
    fportFieldId[RESERVE2] = DIY_VOLT4;
    fportFieldId[CAPACITY] = DIY_CAPACITY;
    fportFieldId[TEMP1] = T1_FIRST_ID; 
    fportFieldId[TEMP2] = T2_FIRST_ID ;
    fportFieldId[VSPEED] = VARIO_FIRST_ID;
    fportFieldId[RELATIVEALT] = ALT_FIRST_ID ;
    fportFieldId[PITCH] = DIY_PITCH;
    fportFieldId[ROLL] = DIY_ROLL;
    //fportFieldId[YAW] = DIY_YAW;
    fportFieldId[RPM] = RPM_FIRST_ID;
    fportFieldId[ADS_1_1] = DIY_ADS_1_1;
    fportFieldId[ADS_1_2] = DIY_ADS_1_2;
    fportFieldId[ADS_1_3] = DIY_ADS_1_3;
    fportFieldId[ADS_1_4] = DIY_ADS_1_4;
    fportFieldId[ADS_2_1] = DIY_ADS_2_1;
    fportFieldId[ADS_2_2] = DIY_ADS_2_2;
    fportFieldId[ADS_2_3] = DIY_ADS_2_3;
    fportFieldId[ADS_2_4] = DIY_ADS_2_4;
    
    fportMaxPooling[LATITUDE] = 10;
    fportMaxPooling[LONGITUDE] = 10;
    fportMaxPooling[GROUNDSPEED] = 20;
    fportMaxPooling[HEADING] = 50;
    fportMaxPooling[ALTITUDE] = 50;
    fportMaxPooling[NUMSAT] = 200;
    fportMaxPooling[GPS_DATE] = 200;
    fportMaxPooling[GPS_TIME] = 200;
    fportMaxPooling[GPS_PDOP] = 200;
    fportMaxPooling[GPS_HOME_BEARING] = 50;
    fportMaxPooling[GPS_HOME_DISTANCE] = 50;
    fportMaxPooling[MVOLT] = 50;
    fportMaxPooling[CURRENT] = 50;
    fportMaxPooling[RESERVE1] = 50;
    fportMaxPooling[RESERVE2] = 50;
    fportMaxPooling[CAPACITY] = 200;
    fportMaxPooling[TEMP1] = 50;
    fportMaxPooling[TEMP2] = 50;
    fportMaxPooling[VSPEED] = 5;
    fportMaxPooling[RELATIVEALT] = 20;
    fportMaxPooling[PITCH] = 20;
    fportMaxPooling[ROLL] = 20;
    fportMaxPooling[YAW] = 100;
    fportMaxPooling[RPM] = 50;
    fportMaxPooling[ADS_1_1] = 200;
    fportMaxPooling[ADS_1_2] = 200;
    fportMaxPooling[ADS_1_3] = 200;
    fportMaxPooling[ADS_1_4] = 200;
    fportMaxPooling[ADS_2_1] = 200;
    fportMaxPooling[ADS_2_2] = 200;
    fportMaxPooling[ADS_2_3] = 200;
    fportMaxPooling[ADS_2_4] = 200;

    fportMinPooling[LATITUDE] = 5;
    fportMinPooling[LONGITUDE] = 5;
    fportMinPooling[GROUNDSPEED] = 10;
    fportMinPooling[HEADING] = 30;
    fportMinPooling[ALTITUDE] = 30;
    fportMinPooling[NUMSAT] = 100;
    fportMinPooling[GPS_DATE] = 100;
    fportMinPooling[GPS_TIME] = 100;
    fportMinPooling[GPS_PDOP] = 100;
    fportMinPooling[GPS_HOME_BEARING] = 30;
    fportMinPooling[GPS_HOME_DISTANCE] = 30;
    fportMinPooling[MVOLT] = 30;
    fportMinPooling[CURRENT] = 30;
    fportMinPooling[RESERVE1] = 30;
    fportMinPooling[RESERVE2] = 30;
    fportMinPooling[CAPACITY] = 100;
    fportMinPooling[TEMP1] = 30;
    fportMinPooling[TEMP2] = 30;
    fportMinPooling[VSPEED] = 3;
    fportMinPooling[RELATIVEALT] = 10;
    fportMinPooling[PITCH] = 10;
    fportMinPooling[ROLL] = 10;
    fportMinPooling[YAW] = 50;
    fportMinPooling[RPM] = 30;
    fportMinPooling[ADS_1_1] = 50;
    fportMinPooling[ADS_1_2] = 50;
    fportMinPooling[ADS_1_3] = 50;
    fportMinPooling[ADS_1_4] = 50;
    fportMinPooling[ADS_2_1] = 50;
    fportMinPooling[ADS_2_2] = 50;
    fportMinPooling[ADS_2_3] = 50;
    fportMinPooling[ADS_2_4] = 50;
    
    //for (uint8_t i = 0 ;  i< NUMBER_MAX_IDX ; i++){
    //    printf("deviceId %d = %x\n", i , fields[i].fportDeviceId);
    //}

} 

void setupFport() {
// configure some table to manage priorities and fport fields codes used bu fport    
    setupFportList();
// configure the queue to get the data from fport in the irq handle
    queue_init (&fportRxQueue, sizeof(uint16_t), 250);

// set up the DMA but do not yet start it to send data to fport
// Configure a channel to write the same byte (8 bits) repeatedly to PIO0
// SM0's TX FIFO, placed by the data request signal from that peripheral.
    fport_dma_chan = dma_claim_unused_channel(true);
    fportDmaConfig = dma_channel_get_default_config(fport_dma_chan);
    channel_config_set_read_increment(&fportDmaConfig, true);
    channel_config_set_write_increment(&fportDmaConfig, false);
    channel_config_set_dreq(&fportDmaConfig, DREQ_PIO0_TX0);  // use state machine 0 
    channel_config_set_transfer_data_size(&fportDmaConfig, DMA_SIZE_8);
    dma_channel_configure(
        fport_dma_chan,
        &fportDmaConfig,
        &pio0_hw->txf[0], // Write address (only need to set this once)
        &fportTxBuffer[0],   // we use always the same buffer             
        0 , // do not yet provide the number of bytes (DMA cycles)
        false             // Don't start yet
    );
// Set up the state machine for transmit but do not yet start it (it starts only when a request from receiver is received)
    fportOffsetTx = pio_add_program(fportPio, &sport_uart_tx_program);
    sport_uart_tx_program_init(fportPio, fportSmTx, fportOffsetTx, config.pinPrimIn, 460800 , true); // we use the same pin and baud rate for tx and rx, true means thet UART is inverted 

// set an irq on pio to handle a received byte
    irq_set_exclusive_handler( PIO0_IRQ_0 , fportPioRxHandlerIrq) ;
    irq_set_enabled (PIO0_IRQ_0 , true) ;

// Set up the state machine we're going to use to receive them.
    fportOffsetRx = pio_add_program(fportPio, &sport_uart_rx_program);
    sport_uart_rx_program_init(fportPio, fportSmRx, fportOffsetRx, config.pinPrimIn, 460800 , true);  
}


#define FPORT_RX_FREE_MICROS 2000
void fportPioRxHandlerIrq(){    // when a byte is received on the fport, read the pio fport fifo and push the data to a queue (to be processed in the main loop)
  // clear the irq flag
  irq_clear (PIO0_IRQ_0 );
  uint32_t nowMicros = microsRp();
  while (  ! pio_sm_is_rx_fifo_empty (fportPio ,fportSmRx)){ // when some data have been received
     uint16_t c = pio_sm_get (fportPio , fportSmRx) >> 24;         // read the data
     //when previous byte was received more than X usec after the previous, then add 1 in bit 15 
    if ( ( nowMicros - fportRxMicros) > FPORT_RX_FREE_MICROS ) c |= 0X8000 ;
    queue_try_add (&fportRxQueue, &c);          // push to the queue
    fportRxMicros = nowMicros;                    // save the timestamp.
  }
}


//void processNextfportRxByte( uint8_t c){
//  static uint8_t previous = 0;
//  printf(" %X",c);
//  if ( ( previous ==  fportSYNCREQUEST)  && (c == fportDEVICEID) ) sendNextfportFrame();
//  previous = c;
//} 

void handleFportRxTx(void){   // main loop : restore receiving mode , wait for tlm request, prepare frame, start pio and dma to transmit it
    static uint8_t previous = 0;
    
    uint16_t data;
    if (config.pinPrimIn == 255) return ; // skip when Tlm is not foreseen
    if ( restoreFportPioToReceiveMillis) {            // put sm back in receive mode after some delay
        if (millisRp() > restoreFportPioToReceiveMillis){
            sport_uart_tx_program_stop(fportPio, fportSmTx, config.pinPrimIn );
            sport_uart_rx_program_restart(fportPio, fportSmRx, config.pinPrimIn, true);  // true = inverted
            restoreFportPioToReceiveMillis = 0 ;
        }
    } else {                             // when we are in receive mode
        while (! queue_is_empty(&fportRxQueue)) {
            // we get the value in the queue
            queue_try_remove (&fportRxQueue,&data);
            //printf("%x\n", (uint8_t) data);
            
            // if bit 15 = 1, it means that the value has been received after x usec from previous and so it must be a synchro 
            // so reset the buffer and process the byte
            if (data & 0X8000) {
                fportRxBufferIdx = 0; 
            } 
            processNextInputByte( (uint8_t) data); // process the incoming byte 
        } // end while
    }           
}



#define FRAME_LEN_16 0x18
#define FRAME_LEN_8  0x0D
#define FRAME_LEN_24 0x23
#define FRAME_LEN_DOWNLINK 0x08

#define FRAME_TYPE_CHANNEL 0xFF
#define FRAME_TYPE_FC 0x1B

#define MAX_CHANNELS 24


            
bool processNextInputByte( uint8_t c){
    static bool fportIsDownlink ; // buffer is currently being filled with a downlink frame
    if (fportRxBufferIdx == 0) {
        switch (c) {
        case FRAME_LEN_8:
            fportLen = 16;
            chanCount = 8;
            fportIsDownlink = false;
            break;
        case FRAME_LEN_16:
            fportLen = 27;
            chanCount = 16;
            fportIsDownlink = false;
            break;
        case FRAME_LEN_24:
            fportLen = 38;
            chanCount = 24;
            fportIsDownlink = false;
            break;
        case FRAME_LEN_DOWNLINK:
            fportLen = 10;
            fportIsDownlink = true;
            break;
        default:
            // definately not FPort2, missing header byte
            printf("fport2: first pos not a valid length frame\n");
            return false;
        }
    }

    if (fportRxBufferIdx == 1) {
        if ( (fportIsDownlink==false) && (c != FRAME_TYPE_CHANNEL)) { // for a Rc channel, byte must be FF, 
            // not channel data
            printf("fport2: second pos not = FF for channel frame\n");
            fportRxBufferIdx = 0;
            return false;
        }
    }

    fportRxBuffer[fportRxBufferIdx++] = c;  // save the received byte

    //const FPort2_Frame *frame = (const FPort2_Frame *)&byte_input.buf[0];

    if (fportLen > 2 && fportRxBufferIdx == fportLen) { //when all bytes have been received
        //for (uint8_t i=0; i<fportLen; i++ ){
        //    printf(" %x", fportRxBuffer[i]);
        //}
        //printf(" \n");
        if (! check_checksum()) {                   // check 
            fportRxBufferIdx = 0;
            printf("fport2 : frame with wrong checksum\n");
            return false;
        }
        if ( chanCount != 16) { // at this stage we manage only 16 channels
            fportRxBufferIdx = 0;
            printf("fport2 : oXs supports only 16 channels\n");
            return false;
        }
        if (fportIsDownlink) {
            if( (fportRxBuffer[1] == SPORT_DEVICEID) && // reply only the physical id = oXs device ID
                    (fportRxBuffer[2] == 0X10) ) {          // and type = 0X10 = polling for DATA          
                //printf("fport2: a pooling has been received\n");
                sendNextFportFrame() ;   // send telemetry
            }    
        } else {           
                //printf("fport2: RC received\n");
                fportDecodeRcChannels() ; // manage the RC channels frame                   
        }
        fportRxBufferIdx = 0; 
    }
    return true;
}

bool check_checksum(void){
    return crc_sum8(&fportRxBuffer[1], fportRxBufferIdx-1) == 0;
}

// simple 8 bit checksum used by FPort
uint8_t crc_sum8(const uint8_t *p, uint8_t len)
{
    uint16_t sum = 0;
    for (uint8_t i=0; i<len; i++) {
        sum += p[i];
        sum += sum >> 8;
        sum &= 0xFF;              
    }
    sum = 0xff - ((sum & 0xff) + (sum >> 8));
    return sum;
}

void fportDecodeRcChannels(){             // this code is similar to Sbus in
    sbusPriMissingFlag = (fportRxBuffer[23] >> 2) & 0X01;
    sbusPriFailsafeFlag = (fportRxBuffer[23] >> 3) & 0X01;
    if ((( sbusPriMissingFlag == false) && (sbusPriFailsafeFlag == false)) || // copy when frame is OK   
        ( sbusSecFailsafeFlag)  ||                                            //   or previous SEC is failsafe
        ( ( millisRp() - lastSecChannelsMillis )  > 50 )) {                     //   or SEC do not exist                   
        memcpy(  (uint8_t *) &sbusFrame.rcChannelsData, &fportRxBuffer[1], 22); // copy the 22 bytes of RC channels
    }
    lastRcChannels = millisRp();
    lastPriChannelsMillis =  lastRcChannels;
    
}


void sendNextFportFrame(){ // search for the next data to be sent
// fportRxBuffer[] contains the frame with 
//0x08 count after this to byte before CRC
//0x2F PhyID poll
//0x10 prim (can be 0X00 = NULL, 0X10=data, 0X30= Read , 0X32= Write)
//0x00 0x00 0x00 0x00 0x00 0x00 (these bytes may have data being sent to a sensor)
//0xC0 CRC

    // oXs search (in the sequence defined by priority) for a field that has not been sent since more or equal than max
    // if not found, it search for a field that has not been sent since more or equal than min
    // if not found, it sent a frame with all 0
    waitUs(250); // wait a little before replying to a pooling
    if ( dma_channel_is_busy(fport_dma_chan) ) {
        //printf("dma is busy\n");
        return ; // skip if the DMA is still sending data
    }
    //uint32_t _millis = millisRp();
    uint32_t currentPollingNr = ++fportPoolingNr;
    uint8_t _fieldId; 
    // first we search the first field 
    for (uint8_t i = 0 ; i< NUMBER_MAX_IDX ; i++ ){
         _fieldId = fportPriority[i]; // retrieve field ID to be checked
         if (fields[_fieldId].available) {
            if (currentPollingNr >= (fportLastPoolingNr[_fieldId] + fportMaxPooling[_fieldId])){
                sendOneFport(_fieldId);
                fields[_fieldId].available = false; // flag as sent
                fportLastPoolingNr[_fieldId] = currentPollingNr; // store pooling that has been used 
                return;
            }
         } 
    } // end for
    // repeat base on min 
    for (uint8_t i = 0 ; i< NUMBER_MAX_IDX ; i++ ){
         _fieldId = fportPriority[i]; // retrieve field ID to be checked
         if (fields[_fieldId].available) {
            if (currentPollingNr >= (fportLastPoolingNr[_fieldId] + fportMinPooling[_fieldId])){
                sendOneFport(_fieldId);
                fields[_fieldId].available = false; // flag as sent
                fportLastPoolingNr[_fieldId] = currentPollingNr; // store pooling that has been used 
                return;
            }
         } 
    } // end for
    // else send a frame with all zero.
    #define NO_SPORT_DATA 0xFF
    sendOneFport(NO_SPORT_DATA); // 0XFF identify the case where we should send a packet with all 0 (meaning no data available) 
}

void sendOneFport(uint8_t idx){  // fill one frame and send it
//    Sensor response:
//0x08 count after this to byte before CRC
//0x2F PhyID
//0x10 prim if data or 0X00 if no data available
//0x00 0x00 Two bytes AppID
//0x00 0x00 0x00 0x00 Four bytes Data
//0xC0 CRC
    
    //printf("%d\n", idx); // used to get the id and check the sequence in a xls
    // the frame contains 1 byte = type, 2 bytes = value id, 4 byte( or more) for value, 1 byte CRC
    uint8_t counter = 0;
    //uint8_t value[4] = { 0, 1, 2, 3};
    uint32_t uintValue = fields[idx].value ;
    int32_t intValue = fields[idx].value ;
    // change some formats just before sending e.g. for longitude and latitude
    switch (idx) {
    case LONGITUDE:
        uintValue = (( ((((uint32_t)( intValue < 0 ? -intValue : intValue)) /10 ) * 6 ) / 10 ) & 0x3FFFFFFF)  | 0x80000000;  
        if(intValue < 0) uintValue |= 0x40000000;
        break;
    case LATITUDE:
        uintValue = ((  ((((uint32_t)( intValue < 0 ? -intValue : intValue)) / 10 ) * 6 )/ 10 ) & 0x3FFFFFFF ) ;
        if(intValue < 0) uintValue |= 0x40000000;
        break;
    case HEADING:
        uintValue =  intValue / 1000 ; // convert from degree * 100000 to degree * 100
        break;
    case GROUNDSPEED:  // to do : test for the right value
        //uintValue =  ( ((uint32_t) uintValue) * 36 )  ; // convert cm/s in 1/100 of km/h (factor = 3.6)
        uintValue =  ( ((uint32_t) uintValue) * 700 ) / 36 ; // convert cm/s in 1/1000 of knots (factor = 19.44)
        break;
    case MVOLT:
        uintValue =  ( ((uint32_t) uintValue) /10 ) ;// voltage in mv is divided by 10 because FPORT expect it (volt * 100)
        break;    
    case CURRENT:
        uintValue =  ( ((uint32_t) uintValue) /100 ) ;// voltage in mv is divided by 100 because FPORT expect it (Amp * 10)
        break;    
    }
    fportTxBuffer[0] = 0X08 ; // fix frame length 
    fportTxBuffer[1] = SPORT_DEVICEID ;   
    if ( idx != NO_SPORT_DATA) {
        fportTxBuffer[2] = 0X10 ; // type of packet : data
        fportTxBuffer[3] = fportFieldId[idx]    ; // 0x0110 = Id for vario data
        fportTxBuffer[4] = fportFieldId[idx] >> 8 ; // 0x0110 = Id for vario data
        fportTxBuffer[5] = uintValue >> 0 ; // value 
        fportTxBuffer[6] = uintValue >> 8 ;  
        fportTxBuffer[7] = uintValue >> 16 ;  
        fportTxBuffer[8] = uintValue >> 24; // value
    } else {
        fportTxBuffer[2] = 0X10 ; 
        fportTxBuffer[3] = 0X10 ; 
        fportTxBuffer[4] = 0X02 ; 
        fportTxBuffer[5] = 0X3F ;  
        fportTxBuffer[6] = 0X03 ;  
        fportTxBuffer[7] = 0 ;  
        fportTxBuffer[8] = 0 ;
    }
    fportTxBuffer[9] = crc_sum8(&fportTxBuffer[1], 8);   
    // copy and convert bytes
    // Byte in frame has value 0x7E is changed into 2 bytes: 0x7D 0x5E
    // Byte in frame has value 0x7D is changed into 2 bytes: 0x7D 0x5D
        //printf("Fport2 :Tlm= ");
        //for (uint8_t j = 0 ; j < 10 ; j++ ){
        //    printf(" %02X" , fportTxBuffer[j]);
        //}
        //printf("\n");    
    
    //sleep_us(100) ;
    sport_uart_rx_program_stop(fportPio, fportSmRx, config.pinPrimIn); // stop receiving
    sport_uart_tx_program_start(fportPio, fportSmTx, config.pinPrimIn, true); // prepare to transmit
    // start the DMA channel with the data to transmit
    dma_channel_set_read_addr (fport_dma_chan, &fportTxBuffer[0], false);
    dma_channel_set_trans_count (fport_dma_chan, 9, true) ;
    // we need a way to set the pio back in receive mode when all bytes are sent 
    // this will be done in the main loop after some ms (here 2ms)
    restoreFportPioToReceiveMillis = millisRp() + 2;   
}

