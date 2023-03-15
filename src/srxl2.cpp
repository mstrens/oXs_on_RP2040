
// srxl2 run on a pio uart at a baud rate of 115200 8N1 not inverted
// Rx sent a handshake at power on to different device ID
// when the handshake has the same device ID as oXs, oXs replies with a handshake
// After having received all handshake msg from all device, Rx sent one more handshake with destination = FF (broadcast)
// oXs handle this handshake to adapt the masterDeviceID and the baudrate (in theory)
// after this Rx send control data frame once every 11 or 22msec 
// This frame contains a byte that says what is the content of the payload (Rc channels, failsafe, VTX)
// It contains also the device id that can reply
// When this id match the Id of oXs, oXs send a telemetry frame (always a payload of 16 bytes)
// There are different formats of payload depending on the data to be sent
// Note : to identify that an incoming frame is complete, oXs has to detect that no byte are received during at least the time to get 2 char. 
// Normally the Rx does not initiate the handshake. If no one device on the bus initiates the handshake there would be no activity on the bus
// So if oXS does not see activity on the bus, it has to initiate the handshake.
// If oXs takes a quite long time to statup, there could already be some activity on the bus.
// In this case oXs has to detect the baudrate (115200 or 400000), wait for a control data frame with replyId =0 and then send a telemetry frame with the destination Id = 0
//    This will request the master to start again a handshake process.




//- setup :  comme pour fbus, initialise le pio pour TX & Rx but not inverted 
//- an ISR receives incoming char , put the char in a queue and save the timestamp (to allow main task to detect inactivity)
//- handleSrxl2TxRx (called by main loop):
//     store all char from the queue into the buffer (loose char if queue is full)
//     if there is more than x msec since last received char,
//        check the content of buffer (is it valid or not) and process it if valid
//        if it is a handshake or a control data, store the baudrate (it is at least valid)
//        if it is a handshake with same device id, send a handshake to reply
//        if it is a handshake with device FF, save the masterid and baudrate (normally should be 0)
//        if it is a control data with command = channels, save the channels
//        if it is a control data with command = failsafe, save the failsafes
//        if it is a control data (any type) and the deviceid = oXs and masterid is known,
//             then make a telemetry frame, send it,     
// When sending, switch to transmit, fill dma param and set an alarm to switch back to receive mode at the end of transmit

// handshake frame contains:
// 0XA6 + 0X21 + 14 (length) + SourceID + DestinationID + priority(=10) + Baudrate(0=115200) + info + UID (4 bytes) + CRC (2 bytes)
// Control data frame contains
// 0XA6 + 0XCD + lenght + command (0=channels, 1=failsafe) + ReplyId + payload + CRC (2 bytes)
//         payload for channels= RSSI(I8) + frameLosses(U16) + channel mask(U32) + n*channel(U16) (n depends on mask)
//         payload for Failsafe= RSSI(I8) + hold(U16) + channel mask(U32) + n*channel(U16) (n depends on mask) 
// telemetry frame contains
// 0XA6 + 0X80 + 22 (length) + destinationID + 16 bytes payload + CRC (2 bytes)
//      telemetry payload contains 1 byte with Type of info, 1 byte subtype + data
//      data are with MSB byte first; when no data, fill field with 7FFFFF when signed, FFFFFF when not signed 

#include <stdio.h>
#include "pico/stdlib.h"
//#include "pico/multicore.h"
#include "hardware/pio.h"
//#include "hardware/uart.h"
#include "uart_srxl2_tx_rx.pio.h"
#include "pico/util/queue.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include <string.h> // used by memcpy

#include "srxl2.h"
#include "tools.h"
#include "config.h"
#include "param.h"
#include "vario.h"
#include "ads1115.h"
#include "rpm.h"
#include "gps.h"
#include "tools.h"



// one pio and 2 state machines are used to manage the srxl2 in halfduplex
// one state machine (sm) handle the TX and the second the RX
// to receive data, the sm is initialised and use an IRQ handler when rx fifo is not empty
//    in irq, this byte is store in a Rx queue
//    This queue is read in main loop
//    When a frame is received and need a reply, then we stop the sm that receive
//    We fill a buffer with the data
//    We set up a dma to transfer the data to the TX fifo of the Tx state machine
//    We also set up a timestamp to stop after some msec the Tx state machine and start again the Rx one   


#define SXRL2_OXS_DEVICEID    0x00
#define SRXL2_BUFFER_LENGTH   80
extern CONFIG config;
queue_t srxl2RxQueue ;

// one pio with 2 state machine is used to manage the inverted hal duplex uart for fbus
PIO srxl2Pio = pio0;
uint srxl2SmTx = 0; // to send the telemetry to fbus
uint srxl2SmRx = 1; // to get the request from fbus
uint srxl2OffsetTx ; 
uint srxl2OffsetRx ; 

// dma channel is used to send fbus telemetry without blocking
int srxl2_dma_chan;
dma_channel_config srxl2DmaConfig;

uint8_t srxl2RxBuffer[SRXL2_BUFFER_LENGTH]; // 80
uint8_t srxl2RxBufferIdx = 0 ;
uint8_t srxl2Len = 0; // length of frame
//uint8_t chanCount ;   // says the number of channels (8,16 or 24) in Rc frame
bool srxl2BaudrateIsValid = false; // become true when a valid frame is received at least one with current baudrate
bool srxl2IsConnected = false; // become true when a handshake is received and replied; set false again when no frame is receive for 50 msec

uint8_t srxl2TxBuffer[SRXL2_BUFFER_LENGTH]; // 80

uint8_t srxl2MasterId = 0; // UID of the master of the bus ; must be the dest Id in a tlm frame
bool srxl2MasterIdIsValid = false; // when false we do not yet had a handshake with FF and so we do not have to reply to control data frame

srxl2Frames_t srxl2Frames;

uint32_t restoreSrxl2PioToReceiveMicros = 0; // when 0, the pio is normally in receive mode,
                                        // otherwise, it is the timestamp when pio transmit has to be restore to receive mode

extern field fields[];  // list of all telemetry fields that are measured

volatile uint32_t srxl2LastRxUs;

extern bool sbusPriMissingFlag ;
extern bool sbusSecMissingFlag ;
extern bool sbusPriFailsafeFlag ;
extern bool sbusSecFailsafeFlag ;
extern uint32_t lastRcChannels ;
extern uint32_t lastPriChannelsMillis ;
extern uint32_t lastSecChannelsMillis; 
extern sbusFrame_s sbusFrame; // full frame including header and End bytes; To generate PWM , we use only the RcChannels part.
extern sbusFrame_s sbus2Frame; // full frame including header and End bytes; To generate PWM , we use only the RcChannels part.

void setupSrxl2() {
// configure the queue to get the data from srxl2 in the irq handle
    queue_init (&srxl2RxQueue, sizeof(uint8_t), 250);  // one byte per item

// set up the DMA but do not yet start it to send data to srxl2
// Configure a channel to write the same byte (8 bits) repeatedly to PIO0
// SM0's TX FIFO, placed by the data request signal from that peripheral.
    srxl2_dma_chan = dma_claim_unused_channel(true);
    srxl2DmaConfig = dma_channel_get_default_config(srxl2_dma_chan);
    channel_config_set_read_increment(&srxl2DmaConfig, true);
    channel_config_set_write_increment(&srxl2DmaConfig, false);
    channel_config_set_dreq(&srxl2DmaConfig, DREQ_PIO0_TX0);  // use state machine 0 
    channel_config_set_transfer_data_size(&srxl2DmaConfig, DMA_SIZE_8);
    dma_channel_configure(
        srxl2_dma_chan,
        &srxl2DmaConfig,
        &pio0_hw->txf[0], // Write address (only need to set this once)
        &srxl2TxBuffer[0],   // we use always the same buffer             
        0 , // do not yet provide the number of bytes (DMA cycles)
        false             // Don't start yet
    );
// Set up the state machine for transmit but do not yet start it (it starts only when a request from receiver is received)
    srxl2OffsetTx = pio_add_program(srxl2Pio, &srxl2_uart_tx_program);
    srxl2_uart_tx_program_init(srxl2Pio, srxl2SmTx, srxl2OffsetTx, config.pinPrimIn, 115200 , false); // we use the same pin and baud rate for tx and rx, true means thet UART is inverted 

// set an irq on pio to handle a received byte
    irq_set_exclusive_handler( PIO0_IRQ_0 , srxl2PioRxHandlerIrq) ;
    irq_set_enabled (PIO0_IRQ_0 , true) ;

// Set up the state machine we're going to use to receive them.
    srxl2OffsetRx = pio_add_program(srxl2Pio, &srxl2_uart_rx_program);
    srxl2_uart_rx_program_init(srxl2Pio, srxl2SmRx, srxl2OffsetRx, config.pinPrimIn, 460800 , true);  
}


void srxl2PioRxHandlerIrq(){    // when a byte is received on the srxl2, read the pio srxl2 fifo and push the data to a queue (to be processed in the main loop)
  // clear the irq flag
  irq_clear (PIO0_IRQ_0 );
  uint32_t nowMicros = microsRp();
  while (  ! pio_sm_is_rx_fifo_empty (srxl2Pio ,srxl2SmRx)){ // when some data have been received
     uint16_t c = pio_sm_get (srxl2Pio , srxl2SmRx) >> 24;         // read the data
     //when previous byte was received more than X usec after the previous, then add 1 in bit 15     
    queue_try_add (&srxl2RxQueue, &c);          // push to the queue
    srxl2LastRxUs = nowMicros;                    // save the timestamp of last received byte.
  }
}

void handleSrxl2RxTx(void){   // main loop : restore receiving mode , wait for tlm request, prepare frame, start pio and dma to transmit it
    uint8_t data;
    if (config.pinPrimIn == 255) return ; // skip when Tlm is not foreseen
    if ( restoreSrxl2PioToReceiveMicros) {            // put sm back in receive mode after some delay
        if (microsRp() > restoreSrxl2PioToReceiveMicros){
            srxl2_uart_tx_program_stop(srxl2Pio, srxl2SmTx, config.pinPrimIn );
            srxl2_uart_rx_program_restart(srxl2Pio, srxl2SmRx, config.pinPrimIn, true);  // true = inverted
            restoreSrxl2PioToReceiveMicros = 0 ;
        } return;
    }
    // read the incoming char.
    #define SRXL2_DETECT_IDLE_RX 200 //10usec * 10 bits * 2 char    
    while (! queue_is_empty(&srxl2RxQueue)) {
            // we get the value from the queue and save it
            queue_try_remove (&srxl2RxQueue,&data);
            if (srxl2RxBufferIdx < SRXL2_BUFFER_LENGTH) {
                srxl2RxBuffer[srxl2RxBufferIdx] = data;
                srxl2RxBufferIdx++;
            } else {
                srxl2RxBufferIdx = 0 ; // loose the buffer if to long
            }
    } 
    if ((( microsRp() - srxl2LastRxUs) > SRXL2_DETECT_IDLE_RX ) && (srxl2RxBufferIdx > 0)){
        // expect a full frame has been received; check it and save it.
        if (srxl2FrameIsvalid() == false) {  // if frame, is valid, at least baudrateValid = true
            // discard invalid frame or frame that are not for oXS
            srxl2RxBufferIdx = 0; 
        } else {   // if the frame is valid, process it
            srxl2ProcessIncomingFrame();
            srxl2RxBufferIdx = 0;
        }
    }    
}

bool srxl2FrameIsvalid(){
    // srxl2RxBuffer[] contains the incomming buffer
    // srxl2RxBufferIdx contains the number of char in the buffer
    
    // check that first byte is 0xA6
    #define SRXL2_HEADER_BYTE1 0XA6
    if (srxl2RxBuffer[0] !=  SRXL2_HEADER_BYTE1 ) return false; 
    if ( srxl2RxBufferIdx < 5) return false; // frame is to short
    // check that length is correct
    if (srxl2RxBuffer[2] !=  srxl2RxBufferIdx ) return false;
    // check that type of frame is Handsake or control data
    #define SRXL2_HANDSHAKE_CODE 0X21
    #define SRXL2_CONTROL_DATA_CODE   0XCD
    if ((srxl2RxBuffer[1] !=  SRXL2_HANDSHAKE_CODE ) && (srxl2RxBuffer[1] !=  SRXL2_CONTROL_DATA_CODE )) return false;
    // check that CRC is OK
    uint16_t crc16Expected = srxl2CalculateCrc( &srxl2RxBuffer[0],srxl2RxBufferIdx - 2) ; // calculate CRC
    uint16_t crc16Received = ((uint16_t) srxl2RxBuffer[srxl2RxBufferIdx-2])  << 8;
    crc16Received |= srxl2RxBuffer[srxl2RxBufferIdx-1];
    if (crc16Expected != crc16Received) return false ; 
    // if crc is valid, at least the baud rate is valid
    srxl2BaudrateIsValid = true;     
    return true;  // frame is valid; it has still to be processed         
}         
    
#define SRXL2_BROADCAST_ID 0xFF
#define SRXL2_OXS_ID 0X00
#define SRXL2_NO_REPLY 0X00
#define SRXL2_REQUEST_HANDSHAKE_IDX 0XFF // dummy index to ask the function to fill a telemetry frame to request a (new) handshake

void srxl2ProcessIncomingFrame(){
    // we process only handshake and control data frames
    // for an handshake for oXs Id, we reply with an handshake (source and dest are reversed) 
    // 0XA6 + 0X21 + 14 (length) + SourceID + DestinationID + priority(=10) + Baudrate(0=115200) + info + UID (4 bytes) + CRC (2 bytes)
    #define SRXL2_HANDSHAKE_FRAME_LENGTH 14
    if  (srxl2RxBuffer[1] ==  SRXL2_HANDSHAKE_CODE ) {
        if ( srxl2RxBuffer[4] == SRXL2_OXS_ID) {   // reply to a handshake for our device ID
            srxl2TxBuffer[0] = SRXL2_HEADER_BYTE1;
            srxl2TxBuffer[1] = SRXL2_HANDSHAKE_CODE;
            srxl2TxBuffer[2] = srxl2RxBuffer[2]; // use original length 
            srxl2TxBuffer[3] = SRXL2_OXS_ID ;
            srxl2TxBuffer[4] = srxl2RxBuffer[3]; // fill dest with incoming source
            srxl2TxBuffer[5] = 10; // default priority
            srxl2TxBuffer[6] = 0; // low baudrate
            srxl2TxBuffer[7] = 0; // info
            srxl2TxBuffer[8] = 0XA1; // dummy ID
            srxl2TxBuffer[9] = 0XB5; 
            srxl2TxBuffer[10] = 0X56; 
            srxl2TxBuffer[11] = 0X92;
            uint16_t crc16 = srxl2CalculateCrc( &srxl2TxBuffer[0], 12) ; // calculate CRC
            srxl2TxBuffer[12] = crc16 >> 8;
            srxl2TxBuffer[13] = crc16;
            srxl2SendFrame(SRXL2_HANDSHAKE_FRAME_LENGTH); // send 14 bytes
            srxl2IsConnected = true;
        } else if ( srxl2RxBuffer[4] == SRXL2_BROADCAST_ID) {   // when destination = FF = broadcast, we do not have to reply
            srxl2MasterId = srxl2RxBuffer[3]; // we save the master ID
                                              // to do : change baudrate if required
            srxl2MasterIdIsValid = true; // from now we can reply to control data frame
        } 
        // we do not have to reply to other handshake frame
    } else {  // we got a control data frame
// 0XA6 + 0XCD + lenght + command (0=channels, 1=failsafe) + ReplyId + payload + CRC (2 bytes)
//         payload for channels= RSSI(I8) + frameLosses(U16) + channel mask(U32) + n*channel(U16) (n depends on mask)
//         payload for Failsafe= RSSI(I8) + hold(U16) + channel mask(U32) + n*channel(U16) (n depends on mask) 
        #define SRXL2_TELEMETRY_FRAME_LENGTH 22
        // when we get a control data for oXs, we reply with a telemetry frame (if data are available)
        if ( srxl2RxBuffer[4] == SRXL2_OXS_ID) {
            if ( srxl2FillTelemetryFrame() ) srxl2SendFrame(SRXL2_TELEMETRY_FRAME_LENGTH); // send 22 bytes if a buffer has been filled with telemetry
        } else if ((srxl2RxBuffer[4] == SRXL2_NO_REPLY) && (srxl2BaudrateIsValid) && (srxl2IsConnected == false) ){
            // when there is no reply needed and if we are not yet connected and if baudrate is known, then we request an handshake with a telemetry frame 
            srxl2FillTXBuffer(SRXL2_REQUEST_HANDSHAKE_IDX);
            srxl2SendFrame(SRXL2_TELEMETRY_FRAME_LENGTH);
        }
        // decode rc channels (but not other types of data)
        #define SRXL2_RC_CHANNELS 0
        if (srxl2RxBuffer[3] == SRXL2_RC_CHANNELS){
            srxl2DecodeRcChannels();
        }
    }
}    


void srxl2DecodeRcChannels(){  // todo : still to fill

}


/*
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
*/

// supported spektrum telemetry formats are:
// 0X26 GPS binary
// 0X40 Vspeed + alt
// 0x11 Airspeed
// 0X7E RPM
// User defined : several 0X50 + ...
// we can use the same logic as Sport based on a list where frame are in some sequence based on priority
// we use also a dummy value SRXL2_REQUEST_HANDSHAKE_IDX = TELE_DEVICE_RSV_06 to request a handshake
#define SRXL2_REQUEST_HANDSHAKE_IDX 0XFF
#define TELE_DEVICE_RSV_06

uint8_t srxl2PriorityList[] = { TELE_DEVICE_VARIO_S , TELE_DEVICE_GPS_BINARY , TELE_DEVICE_AIRSPEED , TELE_DEVICE_RPM,
                                 TELE_DEVICE_RX_MAH, TELE_DEVICE_TEMPERATURE };
uint8_t srxl2MaxPooling[]  = { 4, 8, 10, 10, 10, 10};
uint8_t srxl2MinPooling[]  = { 2, 4, 5, 5, 5, 5};                                
uint32_t srxl2LastPoolingNr[NUMBER_MAX_IDX] = {0}; // contains the last Pooling nr for each frame
uint32_t srxl2PoolingNr= 0; // contains the current Pooling nr


bool srxl2FillTelemetryFrame(){ 
    if ( dma_channel_is_busy(srxl2_dma_chan) ) {
        //printf("dma is busy\n");
        return false ; // skip if the DMA is still sending data
    }
    uint32_t currentPollingNr = ++srxl2PoolingNr;
    uint8_t numberOfFrames = sizeof(srxl2PriorityList)/sizeof(*srxl2PriorityList);
    bool dataAvailableForFrame[numberOfFrames] = {false}; 
    // first we search the first field 
    for (uint8_t i = 0 ; i< numberOfFrames ; i++ ){
        if (srxl2IsFrameDataAvailable(i)) {  // test if some data are available and if so, fill the frame in the structure
            if (currentPollingNr >= (srxl2LastPoolingNr[i] + srxl2MaxPooling[i])){
                srxl2FillTXBuffer(i);    // copy the payload into the TX buffer and fill header and crc
                srxl2LastPoolingNr[i] = currentPollingNr; // store pooling that has been used 
                return true;
            }
         } 
    } // end for
    // repeat base on min (knowing that the payload are already filled by srxl2IsFrameDataAvailable())
    for (uint8_t i = 0 ; i< numberOfFrames ; i++ ){
        if (dataAvailableForFrame[i]) {  // if data is available
            if (currentPollingNr >= (srxl2LastPoolingNr[i] + srxl2MinPooling[i])){
                srxl2FillTXBuffer(i);
                srxl2LastPoolingNr[i] = currentPollingNr; // store pooling that has been used 
                return true;
            }
        } 
    } // end for
    return false;  
}

// test if some data are available for sending for one frame type
// if so, fill the payload of the frame and return true when available
bool srxl2IsFrameDataAvailable(uint8_t frameIdx){
    switch (frameIdx) {
        case 0: //TELE_DEVICE_VARIO_S
            if (fields[VSPEED].available) {
                srxl2Frames.vario.identifier = TELE_DEVICE_VARIO_S ;
                srxl2Frames.vario.sID = 0;
                srxl2Frames.vario.altitude = (int16_t) int_round( fields[RELATIVEALT].value , 10); // 0.1m increments
                srxl2Frames.vario.delta_0250ms = (int16_t) int_round( fields[VSPEED].value , 10);	// change in altitude last 250ms, 0.1m/s increments 
                srxl2Frames.vario.delta_0500ms = 0x7FFF;
				srxl2Frames.vario.delta_1000ms = 0x7FFF;
				srxl2Frames.vario.delta_1500ms = 0x7FFF;			
				srxl2Frames.vario.delta_2000ms = 0x7FFF;	
				srxl2Frames.vario.delta_3000ms = 0x7FFF;
                fields[VSPEED].available = false;
                return true;
            } 
            break;
        case 1: //TELE_DEVICE_GPS_BINARY
            if (fields[NUMSAT].available) {
                srxl2Frames.gps.identifier = TELE_DEVICE_GPS_BINARY; //0X26
                srxl2Frames.gps.sID = 0; // Secondary ID
                if (fields[ALTITUDE].available) {
                    srxl2Frames.gps.altitude = (uint16_t) (int_round(fields[ALTITUDE].value + 100000, 100)); // from cm to m, 1000m offset
                } else {
                    srxl2Frames.gps.altitude = 0XFFFF;
                }
                if (fields[LATITUDE].available) {
                    srxl2Frames.gps.latitude = fields[LATITUDE].value; // degree / 10,000,000
                } else {
                    srxl2Frames.gps.altitude = 0X7FFF;
                }
                if (fields[LONGITUDE].available) {
                    srxl2Frames.gps.longitude = fields[LONGITUDE].value; // degree / 10,000,000
                } else {
                    srxl2Frames.gps.longitude = 0X7FFF;
                }
                if (fields[HEADING].available) {
                    srxl2Frames.gps.heading = (uint16_t) (int_round(fields[HEADING].value , 10)); // from 0.01 deg to 0.1 deg
                } else {
                    srxl2Frames.gps.heading = 0XFFFF;
                }
	            if (fields[GROUNDSPEED].available) {
                    srxl2Frames.gps.groundSpeed = (uint8_t) (int_round(fields[HEADING].value * 36, 100)); // cm/sec to Km/h
                } else {
                    srxl2Frames.gps.groundSpeed = 0XFF;
                }
                if (fields[NUMSAT].available) {
                    srxl2Frames.gps.numSats = (uint8_t) fields[NUMSAT].value; // count
                } else {
                    srxl2Frames.gps.numSats = 0XFF;
                }
                return true;
            }    
            break;
        case 2: //TELE_DEVICE_AIRSPEED
            return false;
            break;
        case 3: // TELE_DEVICE_RPM
            return false;
            break;
        case 4: //TELE_DEVICE_RX_MAH
            if (fields[MVOLT].available || fields[CURRENT].available){
                srxl2Frames.voltCurrentCap.identifier = TELE_DEVICE_RX_MAH ;  // 0X18
                srxl2Frames.voltCurrentCap.sID = 0; // Secondary ID
                if (fields[CURRENT].available) {
                    srxl2Frames.voltCurrentCap.current_A  = (int16_t) (int_round( fields[CURRENT].value ,  10));  //// Instantaneous current, 0.01A (0-328.7A)		7FFF-> no data  
                } else {
                    srxl2Frames.voltCurrentCap.current_A = 0x7FFF ; 
                }
                if (fields[CAPACITY].available) {
                    if ( fields[CAPACITY].value > 0 ){   
                        srxl2Frames.voltCurrentCap.chargeUsed_A  = (uint16_t) (int_round( fields[CAPACITY].value ,  1));  //// Integrated mAh used, 0.1mAh (0-3276.6mAh)  
                    } else {
                        srxl2Frames.voltCurrentCap.chargeUsed_A  = 0 ;
                    }
                } else {
                    srxl2Frames.voltCurrentCap.chargeUsed_A = 0xFFFF ; 
                }
                if (fields[MVOLT].available) {
                    if ( fields[MVOLT].value > 0 ){   
                        srxl2Frames.voltCurrentCap.volts_A  = (uint16_t) (int_round( fields[MVOLT].value ,  10));  // Volts, 0.01V increments (0-16.00V)
                    } else {
                        srxl2Frames.voltCurrentCap.volts_A  = 0 ;
                    }
                } else {
                    srxl2Frames.voltCurrentCap.volts_A = 0xFFFF ; 
                }
                srxl2Frames.voltCurrentCap.current_B = 0x7FFF ;
                srxl2Frames.voltCurrentCap.chargeUsed_B = 0xFFFF ;
                srxl2Frames.voltCurrentCap.volts_B = 0xFFFF ;
                srxl2Frames.voltCurrentCap.alerts = 0 ;// Bit mapped alert conditions (see below)
                srxl2Frames.voltCurrentCap.highCharge = 0;	//High nybble is extra bits for chargeUsed_B, Low is for chargeUsed_A
                return true;
            }
            break;
        case 5: //TELE_DEVICE_TEMPERATURE
            return false;
            break;
        default:
            return false;
            break;    
    };
    return false;    
}

#define SRXL2_TELEMETRY_CODE 0X80
void srxl2FillTXBuffer(uint8_t frameIdx){
    srxl2TxBuffer[0] = SRXL2_HEADER_BYTE1 ; // 0xA6
    srxl2TxBuffer[1] = SRXL2_TELEMETRY_CODE ; // 0x80
    srxl2TxBuffer[2] = 22 ; // length of telemetry frame
    srxl2TxBuffer[3] = srxl2MasterId ; // destination ID
    switch (frameIdx) {
        case 0: //TELE_DEVICE_VARIO_S
            memcpy(&srxl2TxBuffer[4], &srxl2Frames.vario.identifier, 16);
            break;
        case 1: //TELE_DEVICE_GPS_BINARY
            memcpy(&srxl2TxBuffer[4], &srxl2Frames.gps.identifier, 16);
            break;
        case 2: //TELE_DEVICE_AIRSPEED
            break;
        case 3: // TELE_DEVICE_RPM
            
            break;
        case 4: //TELE_DEVICE_RX_MAH
            memcpy(&srxl2TxBuffer[4], &srxl2Frames.voltCurrentCap.identifier, 16);
            break;
        case 5: //TELE_DEVICE_TEMPERATURE
            
            break;
        
        case 0XFF: //request new handshake (it is a dummy value I use)
            srxl2TxBuffer[3] = 0XFF ; // overwrite destination ID with FF to ask for a request
            for (uint8_t i=4; i< 22; i++){
                srxl2TxBuffer[i] = 0X00;
            }
            break;
        default:    
            break;    
    }
    uint16_t crc16 = srxl2CalculateCrc( &srxl2TxBuffer[0], 20) ; // calculate CRC
    srxl2TxBuffer[20] = crc16 >> 8;
    srxl2TxBuffer[21] = crc16;
}

// telemetry frame contains
// 0XA6 + 0X80 + 22 (length) + destinationID + 16 bytes payload + CRC (2 bytes)
//      telemetry payload contains 1 byte with Type of info, 1 byte subtype + data
//      data are with MSB byte first; when no data, fill field with 7FFFFF when signed, FFFFFF when not signed 


void srxl2SendFrame(uint8_t length){  // srxl2TxBuffer is already filled (including CRC)
    srxl2_uart_rx_program_stop(srxl2Pio, srxl2SmRx, config.pinPrimIn); // stop receiving
    srxl2_uart_tx_program_start(srxl2Pio, srxl2SmTx, config.pinPrimIn, true); // prepare to transmit
    // start the DMA channel with the data to transmit
    dma_channel_set_read_addr (srxl2_dma_chan, &srxl2TxBuffer[0], false);
    dma_channel_set_trans_count (srxl2_dma_chan, length , true) ;  // start with the right length
    // we need a way to set the pio back in receive mode when all bytes are sent 
    // this will be done in the main loop after some ms (here 2ms)
    restoreSrxl2PioToReceiveMicros = microsRp() + 2;   
}


const uint16_t srxlCRCTable[] =
{
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,

    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,

    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,

    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

uint16_t srxl2CalculateCrc( uint8_t * buffer , uint8_t length) {
    uint16_t crc = 0;                // Seed with 0
    for(uint8_t i = 0; i < length; ++i)
        {
            // Get indexed position in lookup table using XOR of current CRC hi byte
            uint8_t pos = (uint8_t)((crc >> 8) ^ buffer[i]);
            // Shift LSB up and XOR with the resulting lookup table entry
            crc = (uint16_t)((crc << 8) ^ (uint16_t)(srxlCRCTable[pos]));
        }
    return crc; 
}        
