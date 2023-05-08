
// fbus run on a pio uart at a baud rate of 115200 8N1 inverted
// Rx sent a frame once every 9msec (without byte stuffing)
// it contains:  0X7E LEN(=0X19)  X000 + a sbus signal + a CRC (one byte) 7E
// it can be immediately followed by a downlink frame with 
//   0X7E LEN(=0X08)  0X01 PRI APP1 APP2 DATA1...DATA4 + CRC + 7E
// sensor reply within 3msec with
//   LEN(0X08)  0X81 PRI (= 0X10 id data or OX00 if there is no valid data) APP1 APP2 + DATA1...DATA4 + CRC
// Note :LEN does not include Header, LEN and CRC, trailer


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

#include "fbus.h"
#include "tools.h"
#include "config.h"
#include "param.h"



// one pio and 2 state machines are used to manage the fbus in halfduplex
// one state machine (sm) handle the TX and the second the RX
// to receive data, the sm is initialised and use an IRQ handler when rx fifo is not empty
//    in irq, this byte is store in a Rx queue
//    This queue is read in main loop
//    When a byte is received after a 7E, then we stop the sm that receive (it means frsky made a polling)
//    We fill a buffer with the data
//    We set up a dma to transfer the data to the TX fifo of the Tx state machine
//    We also set up a timestamp to stop after some msec the Tx state machine and start again the Rx one   


#define FBUSSYNCREQUEST 0x7E
//#define FBUSDEVICEID    0xE4




extern CONFIG config;
extern uint8_t debugTlm;
queue_t fbusRxQueue ;

// one pio with 2 state machine is used to manage the inverted hal duplex uart for fbus
PIO fbusPio = pio0;
uint fbusSmTx = 0; // to send the telemetry to fbus
uint fbusSmRx = 1; // to get the request from fbus
uint fbusOffsetTx ; 
uint fbusOffsetRx ; 

// dma channel is used to send fbus telemetry without blocking
int fbus_dma_chan;
dma_channel_config fbusDmaConfig;

uint8_t fbusRxBuffer[50];
uint8_t fbusRxBufferIdx = 0 ;
uint8_t fbusLen = 0; // length of frame
uint8_t chanCount ;   // says the number of channels (8,16 or 24) in Rc frame

uint8_t fbusTxBuffer[50];

uint32_t restoreFbusPioToReceiveMillis = 0; // when 0, the pio is normally in receive mode,
                                        // otherwise, it is the timestamp when pio transmit has to be restore to receive mode

extern field fields[];  // list of all telemetry fields that are measured


extern uint16_t sportFieldId[NUMBER_MAX_IDX]; // contains the code to be used in fbus to identify the field
extern uint8_t sportPriority[NUMBER_MAX_IDX]; // contains the list of fields in priority sequence (first = highest) 
extern uint8_t sportMaxPooling[NUMBER_MAX_IDX]; // contains the max number of polling allowed between 2 transmissions
extern uint8_t sportMinPooling[NUMBER_MAX_IDX]; // contains the min number of polling allowed between 2 transmissions
uint32_t fbusLastPoolingNr[NUMBER_MAX_IDX] = {0}; // contains the last Pooling nr for each field
uint32_t fbusPoolingNr= 0; // contains the current Pooling nr
extern float sportMax ; // coeeficient to manage priorities of sport tlm fields (to ensure all fields are sent) 


uint32_t fbusRxMicros;


extern bool sbusPriMissingFlag ;
extern bool sbusSecMissingFlag ;
extern bool sbusPriFailsafeFlag ;
extern bool sbusSecFailsafeFlag ;
extern uint32_t lastRcChannels ;
extern uint32_t lastPriChannelsMillis ;
extern uint32_t lastSecChannelsMillis; 
extern sbusFrame_s sbusFrame; // full frame including header and End bytes; To generate PWM , we use only the RcChannels part.
extern sbusFrame_s sbus2Frame; // full frame including header and End bytes; To generate PWM , we use only the RcChannels part.


void setupFbus() {
// configure some table to manage priorities and fbus fields codes used by fbus    
    setupSportList(); // reuse the set up list from sport
// configure the queue to get the data from fbus in the irq handle
    queue_init (&fbusRxQueue, sizeof(uint16_t), 250);

// set up the DMA but do not yet start it to send data to fbus
// Configure a channel to write the same byte (8 bits) repeatedly to PIO0
// SM0's TX FIFO, placed by the data request signal from that peripheral.
    fbus_dma_chan = dma_claim_unused_channel(true);
    fbusDmaConfig = dma_channel_get_default_config(fbus_dma_chan);
    channel_config_set_read_increment(&fbusDmaConfig, true);
    channel_config_set_write_increment(&fbusDmaConfig, false);
    channel_config_set_dreq(&fbusDmaConfig, DREQ_PIO0_TX0);  // use state machine 0 
    channel_config_set_transfer_data_size(&fbusDmaConfig, DMA_SIZE_8);
    dma_channel_configure(
        fbus_dma_chan,
        &fbusDmaConfig,
        &pio0_hw->txf[0], // Write address (only need to set this once)
        &fbusTxBuffer[0],   // we use always the same buffer             
        0 , // do not yet provide the number of bytes (DMA cycles)
        false             // Don't start yet
    );
// Set up the state machine for transmit but do not yet start it (it starts only when a request from receiver is received)
    fbusOffsetTx = pio_add_program(fbusPio, &sport_uart_tx_program);
    sport_uart_tx_program_init(fbusPio, fbusSmTx, fbusOffsetTx, config.pinPrimIn, 460800 , true); // we use the same pin and baud rate for tx and rx, true means thet UART is inverted 

// set an irq on pio to handle a received byte
    irq_set_exclusive_handler( PIO0_IRQ_0 , fbusPioRxHandlerIrq) ;
    irq_set_enabled (PIO0_IRQ_0 , true) ;

// Set up the state machine we're going to use to receive them.
    fbusOffsetRx = pio_add_program(fbusPio, &sport_uart_rx_program);
    sport_uart_rx_program_init(fbusPio, fbusSmRx, fbusOffsetRx, config.pinPrimIn, 460800 , true);  
}


#define FBUS_RX_FREE_MICROS 2000
void fbusPioRxHandlerIrq(){    // when a byte is received on the fbus, read the pio fbus fifo and push the data to a queue (to be processed in the main loop)
  // clear the irq flag
  irq_clear (PIO0_IRQ_0 );
  uint32_t nowMicros = microsRp();
  while (  ! pio_sm_is_rx_fifo_empty (fbusPio ,fbusSmRx)){ // when some data have been received
     uint16_t c = pio_sm_get (fbusPio , fbusSmRx) >> 24;         // read the data
     //when previous byte was received more than X usec after the previous, then add 1 in bit 15 
    if ( ( nowMicros - fbusRxMicros) > FBUS_RX_FREE_MICROS ) c |= 0X8000 ;
    queue_try_add (&fbusRxQueue, &c);          // push to the queue
    fbusRxMicros = nowMicros;                    // save the timestamp.
  }
}


//void processNextFbusRxByte( uint8_t c){
//  static uint8_t previous = 0;
//  printf(" %X",c);
//  if ( ( previous ==  fbusSYNCREQUEST)  && (c == fbusDEVICEID) ) sendNextfbusFrame();
//  previous = c;
//} 

void handleFbusRxTx(void){   // main loop : restore receiving mode , wait for tlm request, prepare frame, start pio and dma to transmit it
    static uint8_t previous = 0;
    
    uint16_t data;
    if (config.pinPrimIn == 255) return ; // skip when Tlm is not foreseen
    if ( restoreFbusPioToReceiveMillis) {            // put sm back in receive mode after some delay
        if (millisRp() > restoreFbusPioToReceiveMillis){
            sport_uart_tx_program_stop(fbusPio, fbusSmTx, config.pinPrimIn );
            sport_uart_rx_program_restart(fbusPio, fbusSmRx, config.pinPrimIn, true);  // true = inverted
            restoreFbusPioToReceiveMillis = 0 ;
        }
    } else {                             // when we are in receive mode
        while (! queue_is_empty(&fbusRxQueue)) {
            // we get the value in the queue
            queue_try_remove (&fbusRxQueue,&data);
            //printf("%x\n", (uint8_t) data);
            
            // if bit 15 = 1, it means that the value has been received after x usec from previous and so it must be a synchro 
            // so reset the buffer and process the byte
            if (data & 0X8000) {
                fbusRxBufferIdx = 0; 
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
    static bool fbusIsDownlink ; // buffer is currently being filled with a downlink frame
    if (fbusRxBufferIdx == 0) {
        switch (c) {
        case FRAME_LEN_8:
            fbusLen = 16;
            chanCount = 8;
            fbusIsDownlink = false;
            break;
        case FRAME_LEN_16:
            fbusLen = 27;
            chanCount = 16;
            fbusIsDownlink = false;
            break;
        case FRAME_LEN_24:
            fbusLen = 38;
            chanCount = 24;
            fbusIsDownlink = false;
            break;
        case FRAME_LEN_DOWNLINK:
            fbusLen = 10;
            fbusIsDownlink = true;
            break;
        default:
            // definately not fbus, missing header byte
            printf("fbus: first pos not a valid length frame\n");
            return false;
        }
    }

    if (fbusRxBufferIdx == 1) {
        if ( (fbusIsDownlink==false) && (c != FRAME_TYPE_CHANNEL)) { // for a Rc channel, byte must be FF, 
            // not channel data
            printf("fbus: second pos not = FF for channel frame\n");
            fbusRxBufferIdx = 0;
            return false;
        }
    }

    fbusRxBuffer[fbusRxBufferIdx++] = c;  // save the received byte

    //const fbus_Frame *frame = (const fbus_Frame *)&byte_input.buf[0];

    if (fbusLen > 2 && fbusRxBufferIdx == fbusLen) { //when all bytes have been received
        //for (uint8_t i=0; i<fbusLen; i++ ){
        //    printf(" %x", fbusRxBuffer[i]);
        //}
        //printf(" \n");
        if (! check_checksum()) {                   // check 
            fbusRxBufferIdx = 0;
            printf("fbus : frame with wrong checksum\n");
            return false;
        }
        if ( chanCount != 16) { // at this stage we manage only 16 channels
            fbusRxBufferIdx = 0;
            printf("fbus : oXs supports only 16 channels\n");
            return false;
        }
        if (fbusIsDownlink) {
            if( (fbusRxBuffer[1] == SPORT_DEVICEID) && // reply only the physical id = oXs device ID
                    (fbusRxBuffer[2] == 0X10) ) {          // and type = 0X10 = polling for DATA          
                //printf("fbus: a pooling has been received\n");
                sendNextFbusFrame() ;   // send telemetry
            }    
        } else {           
                //printf("fbus: RC received\n");
                fbusDecodeRcChannels() ; // manage the RC channels frame                   
        }
        fbusRxBufferIdx = 0; 
    }
    return true;
}

bool check_checksum(void){
    return crc_sum8(&fbusRxBuffer[1], fbusRxBufferIdx-1) == 0;
}

// simple 8 bit checksum used by Fbus
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

void fbusDecodeRcChannels(){             // this code is similar to Sbus in
    sbusPriMissingFlag = (fbusRxBuffer[24] >> 2) & 0X01;
    sbusPriFailsafeFlag = (fbusRxBuffer[24] >> 3) & 0X01;
    if ((( sbusPriMissingFlag == false) && (sbusPriFailsafeFlag == false)) || // copy when frame is OK   
        ( sbusSecFailsafeFlag)  ||                                            //   or previous SEC is failsafe
        ( ( millisRp() - lastSecChannelsMillis )  > 50 )) {                     //   or SEC do not exist                   
        memcpy(  (uint8_t *) &sbusFrame.rcChannelsData, &fbusRxBuffer[2], 22); // copy the 22 bytes of RC channels
    }
    lastRcChannels = millisRp();
    lastPriChannelsMillis =  lastRcChannels;
    
}


void sendNextFbusFrame(){ // search for the next data to be sent
// fbusRxBuffer[] contains the frame with 
//0x08 count after this to byte before CRC
//0x2F PhyID poll
//0x10 prim (can be 0X00 = NULL, 0X10=data, 0X30= Read , 0X32= Write)
//0x00 0x00 0x00 0x00 0x00 0x00 (these bytes may have data being sent to a sensor)
//0xC0 CRC

    // oXs search (in the sequence defined by priority) for a field that has not been sent since more or equal than max
    // if not found, it search for a field that has not been sent since more or equal than min
    // if not found, it sent a frame with all 0
    waitUs(250); // wait a little before replying to a pooling
    if ( dma_channel_is_busy(fbus_dma_chan) ) {
        //printf("dma is busy\n");
        return ; // skip if the DMA is still sending data
    }
    //uint32_t _millis = millisRp();
    uint32_t currentPollingNr = ++fbusPoolingNr;
    uint8_t _fieldId; 
    // first we search the first field 
    for (uint8_t i = 0 ; i< NUMBER_MAX_IDX ; i++ ){
         _fieldId = sportPriority[i]; // retrieve field ID to be checked
        if ((fields[_fieldId].available) && (sportMaxPooling[_fieldId] > 0)) {
            if (currentPollingNr > (fbusLastPoolingNr[_fieldId] + sportMaxPooling[_fieldId])){
                sendOneFbus(_fieldId);
                fields[_fieldId].available = false; // flag as sent
                fbusLastPoolingNr[_fieldId] = currentPollingNr; // store pooling that has been used 
                return;
            }
         } 
    } // end for
    // repeat base on min 
    for (uint8_t i = 0 ; i< NUMBER_MAX_IDX ; i++ ){
         _fieldId = sportPriority[i]; // retrieve field ID to be checked
        if ((fields[_fieldId].available) && (sportMaxPooling[_fieldId] > 0)) {
            if (currentPollingNr > (fbusLastPoolingNr[_fieldId] + sportMinPooling[_fieldId])){
                sendOneFbus(_fieldId);
                fields[_fieldId].available = false; // flag as sent
                fbusLastPoolingNr[_fieldId] = currentPollingNr; // store pooling that has been used 
                return;
            }
         } 
    } // end for
    // else send a frame with all zero.
    #define NO_SPORT_DATA 0xFF
    sendOneFbus(NO_SPORT_DATA); // 0XFF identify the case where we should send a packet with all 0 (meaning no data available) 
}

void sendOneFbus(uint8_t idx){  // fill one frame and send it
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
        uintValue =  ( ((uint32_t) uintValue) /10 ) ;// voltage in mv is divided by 10 because Fbus expect it (volt * 100)
        break;    
    case CURRENT:
        uintValue =  ( ((uint32_t) uintValue) /100 ) ;// voltage in mv is divided by 100 because Fbus expect it (Amp * 10)
        break;    
    case AIRSPEED:
        uintValue =  (uint32_t)( ((float) intValue) * 0.194384 ) ;// from cm/s to 0.1kts/h
        if (intValue < 0) uintValue = 0; 
        break;   
    }
    fbusTxBuffer[0] = 0X08 ; // fix frame length 
    fbusTxBuffer[1] = SPORT_DEVICEID ;   
    if ( idx != NO_SPORT_DATA) {
        fbusTxBuffer[2] = 0X10 ; // type of packet : data
        fbusTxBuffer[3] = sportFieldId[idx]    ; // 0x0110 = Id for vario data
        fbusTxBuffer[4] = sportFieldId[idx] >> 8 ; // 0x0110 = Id for vario data
        fbusTxBuffer[5] = uintValue >> 0 ; // value 
        fbusTxBuffer[6] = uintValue >> 8 ;  
        fbusTxBuffer[7] = uintValue >> 16 ;  
        fbusTxBuffer[8] = uintValue >> 24; // value
    } else {
        fbusTxBuffer[2] = 0 ; 
        fbusTxBuffer[3] = 0 ; 
        fbusTxBuffer[4] = 0 ; 
        fbusTxBuffer[5] = 0 ;  
        fbusTxBuffer[6] = 0 ;  
        fbusTxBuffer[7] = 0 ;  
        fbusTxBuffer[8] = 0 ;
    }
    fbusTxBuffer[9] = crc_sum8(&fbusTxBuffer[1], 8);   
    // copy and convert bytes
    // Byte in frame has value 0x7E is changed into 2 bytes: 0x7D 0x5E
    // Byte in frame has value 0x7D is changed into 2 bytes: 0x7D 0x5D
        //printf("fbus :Tlm= ");
        //for (uint8_t j = 0 ; j < 10 ; j++ ){
        //    printf(" %02X" , fbusTxBuffer[j]);
        //}
        //printf("\n");    
    
    //sleep_us(100) ;
    sport_uart_rx_program_stop(fbusPio, fbusSmRx, config.pinPrimIn); // stop receiving
    sport_uart_tx_program_start(fbusPio, fbusSmTx, config.pinPrimIn, true); // prepare to transmit
    // start the DMA channel with the data to transmit
    dma_channel_set_read_addr (fbus_dma_chan, &fbusTxBuffer[0], false);
    dma_channel_set_trans_count (fbus_dma_chan, 10, true) ;
    // we need a way to set the pio back in receive mode when all bytes are sent 
    // this will be done in the main loop after some ms (here 2ms)
    restoreFbusPioToReceiveMillis = millisRp() + 2;   
}

