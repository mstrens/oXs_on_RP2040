// todo move the device ID to config.h
// todo fill the function that decode the channels

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
#define SRXL2_PORT_BAUDRATE_HIGH 40000
#define SRXL2_PORT_BAUDRATE_DEFAULT 115200
#define SRXL2_RUNNING 1
#define SRXL2_LISTENING 2
#define SRXL2_HANDSHAKE_FRAME_LENGTH 14
#define SRXL2_BROADCAST_ID 0xFF
#define SRXL2_OXS_ID 0X61
#define SRXL2_NO_REPLY 0X00
#define SRXL2_USE_TLM_FOR_HANDSHAKE_REQUEST 0XFF // dummy index to ask the function to fill a telemetry frame to request a (new) handshake
#define SRXL2_TELEMETRY_CODE 0X80                        
#define SRXL2_DETECT_TIMEOUT_RX_US 50000 // us = 50 ms
#define SRXL2_DETECT_IDLE_RX_US 500 //10usec * 10 bits * 2 char ; with 200usec, some frames are splitted and considered as invalid    
#define SRXL2_NO_ACTIVITY_TO_US 50000
#define SRXL2_HEADER_BYTE1 0XA6
#define SRXL2_HANDSHAKE_CODE 0X21
#define SRXL2_CONTROL_DATA_CODE   0XCD
#define SRXL2_TELEMETRY_FRAME_LENGTH 22
#define SRXL2_RC_CHANNELS 0
#define SRXL2_RC_FAILSAFE 1
#define SRXL2_USE_TLM_FOR_HANDSHAKE_REQUEST 0XFF
              
#define SRXL2_ALARM_NUM_UART_IDLE 2 // number of the timer alarm used to detect uart idle. (alarm3 is sued by watchdog!)


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
uint8_t srxl2ProcessIn[SRXL2_BUFFER_LENGTH];
uint8_t srxl2ProcessInIdx = 0 ;

//uint8_t chanCount ;   // says the number of channels (8,16 or 24) in Rc frame
uint32_t srxl2ValidBaudrate = 0;  // filled with current baud rate when a valid frame is received
uint32_t srxl2CurrentBaudrate = 115200;
bool srxl2IsConnected = false; // become true when a handshake is received and replied; set false again when no frame is receive for 50 msec
uint8_t srxl2State = SRXL2_RUNNING;

uint8_t srxl2TxBuffer[SRXL2_BUFFER_LENGTH]; // 80

uint8_t srxl2MasterId = 0; // UID of the master of the bus ; must be the dest Id in a tlm frame
bool srxl2MasterIdIsValid = false; // when false we do not yet had a handshake with FF and so we do not have to reply to control data frame

srxl2Frames_t srxl2Frames; // frames with telemetry data per type of sensor

uint32_t restoreSrxl2PioToReceiveMicros = 0; // when 0, the pio is normally in receive mode,
                                        // otherwise, it is the timestamp when pio transmit has to be restore to receive mode

extern field fields[];  // list of all telemetry fields that are measured

volatile uint32_t srxl2LastRxUs;
uint32_t srxl2IdleDelay ; // delay in Us between last char and current proccess (in loop)
uint32_t srxl2LastIdleUs;
uint32_t srxl2LastValidFrameUs;
uint32_t srxl2LastRepliedFrameUs;

extern bool sbusPriMissingFlag ;
extern bool sbusSecMissingFlag ;
extern bool sbusPriFailsafeFlag ;
extern bool sbusSecFailsafeFlag ;
extern uint32_t lastRcChannels ;
extern uint32_t lastPriChannelsMillis ;
extern uint32_t lastSecChannelsMillis; 
extern sbusFrame_s sbusFrame; // full frame including header and End bytes; To generate PWM , we use only the RcChannels part.
extern sbusFrame_s sbus2Frame; // full frame including header and End bytes; To generate PWM , we use only the RcChannels part.

extern uint32_t sbusFailsafeCounter;
extern uint32_t sbusFrameCounter;
extern bool prevFailsafeFlag;



void setupSrxl2() {
// configure the queue to get the data from srxl2 in the irq handle
    queue_init (&srxl2RxQueue, sizeof(uint16_t), 250);  // one byte per item

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
    srxl2_uart_tx_program_init(srxl2Pio, srxl2SmTx, srxl2OffsetTx, config.pinPrimIn, SRXL2_PORT_BAUDRATE_DEFAULT , false); // we use the same pin and baud rate for tx and rx, true means thet UART is inverted 

// set an irq on pio to handle a received byte
    irq_set_exclusive_handler( PIO0_IRQ_0 , srxl2PioRxHandlerIrq) ;
    irq_set_enabled (PIO0_IRQ_0 , true) ;

// Set up the state machine we're going to use to receive them.
    srxl2OffsetRx = pio_add_program(srxl2Pio, &srxl2_uart_rx_program);
    srxl2_uart_rx_program_init(srxl2Pio, srxl2SmRx, srxl2OffsetRx, config.pinPrimIn, SRXL2_PORT_BAUDRATE_DEFAULT , false);  

// add an alarm callback to be used to detect idle in UART
hardware_alarm_set_callback(SRXL2_ALARM_NUM_UART_IDLE , srxl2AlarmUart_callback);

}

void srxl2AlarmUart_callback(uint alarmNum){
    uint16_t c = 0X8000;
    queue_try_add (&srxl2RxQueue, &c);  // add a code with MSB different from 0
}

void srxl2PioRxHandlerIrq(){    // when a byte is received on the srxl2, read the pio srxl2 fifo and push the data to a queue (to be processed in the main loop)
  // clear the irq flag
  irq_clear (PIO0_IRQ_0 );
  while (  ! pio_sm_is_rx_fifo_empty (srxl2Pio ,srxl2SmRx)){ // when some data have been received
    uint16_t c = pio_sm_get (srxl2Pio , srxl2SmRx) >> 24;         // read the data
    queue_try_add (&srxl2RxQueue, &c);          // push to the queue
    srxl2LastRxUs = microsRp();                    // save the timestamp of last received byte.
    hardware_alarm_set_target(SRXL2_ALARM_NUM_UART_IDLE , from_us_since_boot(time_us_64()+300));
  }
}

     
void handleSrxl2RxTx(void){   // main loop : restore receiving mode , wait for tlm request, prepare frame, start pio and dma to transmit it
    uint16_t data;
    bool fullFrameIsreceived = false;
    if (config.pinPrimIn == 255) return ; // skip when Tlm is not foreseen
    if ( restoreSrxl2PioToReceiveMicros) {            // put sm back in receive mode after some delay
        if (microsRp() > restoreSrxl2PioToReceiveMicros){
            srxl2_uart_tx_program_stop(srxl2Pio, srxl2SmTx, config.pinPrimIn );
            srxl2_uart_rx_program_restart(srxl2Pio, srxl2SmRx, config.pinPrimIn, false);  // true = inverted
            restoreSrxl2PioToReceiveMicros = 0 ;
        } return;
    }
    // read the incoming char.
    while (! queue_is_empty(&srxl2RxQueue)) {
            // we get the value from the queue and save it
            queue_try_remove (&srxl2RxQueue,&data);
            if ( data > 0x0100){
                fullFrameIsreceived = true;
                break; // exit the while because we got a byte that says that UART was Idle 
            }
            if (srxl2RxBufferIdx < SRXL2_BUFFER_LENGTH) {
                srxl2RxBuffer[srxl2RxBufferIdx] = data;
                srxl2RxBufferIdx++;
            } else {
                srxl2RxBufferIdx = 0 ; // loose the buffer if to long
            }
    }
    uint32_t irqStatus = save_and_disable_interrupts();
    uint32_t lastByteReadInIsrUs = srxl2LastRxUs;
    restore_interrupts(irqStatus); 
    srxl2IdleDelay = microsRp() - lastByteReadInIsrUs;
    //if (( srxl2IdleDelay > SRXL2_DETECT_IDLE_RX_US ) && (srxl2RxBufferIdx > 0)){
    if (fullFrameIsreceived){
    //if ((( microsRp() - srxl2LastRxUs) > SRXL2_DETECT_IDLE_RX_US ) && (srxl2RxBufferIdx > 0)){
        // expect a full frame has been received; save it in another buffer, check it and save it.
        
        srxl2LastIdleUs = microsRp(); 
        memcpy(srxl2ProcessIn, srxl2RxBuffer, srxl2RxBufferIdx);
        srxl2ProcessInIdx = srxl2RxBufferIdx ;
        srxl2RxBufferIdx = 0; // reset the buffer
        // check if frame is valid
        if (srxl2FrameIsvalid() ) {  // if frame, is valid, 
            //printf("Valid frame received\n");
            srxl2LastValidFrameUs = srxl2LastIdleUs;
            // at least baudrate is valid 
            srxl2ValidBaudrate = srxl2CurrentBaudrate;     
            // reply to a handshake of us or with dest = FF;
            // sent telemetry on control data with a replyId code = our device
            // sent a telemetry to request new handshake if we are not yet connected and lastValidFrame????????
            // decode the Rc channels
            // note: some frames are just discarded
            srxl2ProcessIncomingFrame();      
        } else {   
            printf("Invalid frame received\n");
            printf("with idleTime=%d\n", (int) srxl2IdleDelay );
            for (uint8_t i = 0; i< srxl2ProcessInIdx; i++ ){
                printf(" %x", srxl2ProcessIn[i] );
            }
            printf("\n");
            // discard invalid frame
        }
    }
    
    uint32_t nowUs =microsRp();
    static uint32_t beginListeningUs = 0;
    switch (srxl2State){
        case SRXL2_RUNNING:
            // go back to listening when no valid frame is received for 50 msec
            if ( (nowUs -  srxl2LastValidFrameUs) > SRXL2_NO_ACTIVITY_TO_US ){
                srxl2IsConnected = false; // we lose the connection
                srxl2State = SRXL2_LISTENING;
                srxl2LastIdleUs = 0;
                srxl2LastValidFrameUs = 0;
                beginListeningUs = nowUs ;
                //printf("Running : No frame => go to listening\n"); 
            }
            break;
        case SRXL2_LISTENING:
            // when we receive the end of a frame and we do not yet have baudrate; we alternate the baudrate
            // hoping to find the correct baudrate after a first delay to get the opportunity to get a valid frame.
            if ( (srxl2LastIdleUs > 0) && (srxl2ValidBaudrate == 0) && ((nowUs - beginListeningUs) > 50000 )) {
                // change current baudrate
                if(srxl2CurrentBaudrate == SRXL2_PORT_BAUDRATE_DEFAULT)
                    srxl2CurrentBaudrate = SRXL2_PORT_BAUDRATE_HIGH;
                else {
                    srxl2CurrentBaudrate = SRXL2_PORT_BAUDRATE_DEFAULT;
                }    
                changeBaudrate(srxl2Pio, srxl2SmTx, srxl2SmRx , srxl2CurrentBaudrate);
                //printf("Changing baudrate\n");
            }
            // when timeout expired, we did not processed a frame and are still waiting ; we can take the initiative for handshake
            if ( (nowUs - beginListeningUs) > 200000) {
                if (srxl2LastValidFrameUs == 0) { // if we did not received a valid frame, then we can send a handshake because there is no activity
                    //printf("Handshake request sent\n");
                    srxl2SendHandshake(); // send at validBaudrate if know, else to 115200;
                    srxl2State = SRXL2_RUNNING;
                } else { // we received a valid frame; so there is some activity on the bus but no one that we could process; 
                // we will send a telemetry to request a handshake ; this is done in the function that process valid incoming frames
                }
            }
        break;        
    }
}

bool srxl2FrameIsvalid(){
    // srxl2ProcessIn[] contains the incomming buffer
    // srxl2ProcessInIdx contains the number of char in the buffer
    
    // check that first byte is 0xA6
    if (srxl2ProcessIn[0] !=  SRXL2_HEADER_BYTE1 ) {
        if (srxl2IsConnected) printf("First char is not 0XA6\n");
        return false; 
    }
    if ( srxl2ProcessInIdx < 5){
        if (srxl2IsConnected) printf("Less than 5 char\n");
        return false; // frame is to short
    }
    // check that length is correct
    if (srxl2ProcessIn[2] !=  srxl2ProcessInIdx ) {
        if (srxl2IsConnected) printf("Incorrect length\n");
        return false;
    }    
    // check that type of frame is Handsake or control data
    if ((srxl2ProcessIn[1] !=  SRXL2_HANDSHAKE_CODE ) && (srxl2ProcessIn[1] !=  SRXL2_CONTROL_DATA_CODE )) {
        if (srxl2IsConnected) printf("Type is not handshake nor control data\n");
        return false;
    }    
    // check that CRC is OK
    uint16_t crc16Expected = srxl2CalculateCrc( &srxl2ProcessIn[0],srxl2ProcessInIdx - 2) ; // calculate CRC
    uint16_t crc16Received = ((uint16_t) srxl2ProcessIn[srxl2ProcessInIdx-2])  << 8;
    crc16Received |= srxl2ProcessIn[srxl2ProcessInIdx-1];
    if (crc16Expected != crc16Received) {
        if (srxl2IsConnected) printf("CRC is not correctn");
        return false ; 
    }    
    return true;  // frame is valid; it has still to be processed         
}         
    
void srxl2SendHandshake(){ // called when there is no activity on the bus and we take the initiative to send a handshake request 
    srxl2TxBuffer[0] = SRXL2_HEADER_BYTE1;
    srxl2TxBuffer[1] = SRXL2_HANDSHAKE_CODE;
    srxl2TxBuffer[2] = SRXL2_HANDSHAKE_FRAME_LENGTH ; // length of handframe 
    srxl2TxBuffer[3] = SRXL2_OXS_ID ;
    srxl2TxBuffer[4] = 0; // fill dest with 0 (= dummy destination)
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
    srxl2CurrentBaudrate = 115200;
    srxl2SendFrame(SRXL2_HANDSHAKE_FRAME_LENGTH); // send 14 bytes
    srxl2IsConnected = true;
    srxl2State = SRXL2_RUNNING;
}

void replyToHandshake(){
            srxl2TxBuffer[0] = SRXL2_HEADER_BYTE1;
            srxl2TxBuffer[1] = SRXL2_HANDSHAKE_CODE;
            srxl2TxBuffer[2] = srxl2ProcessIn[2]; // use original length 
            srxl2TxBuffer[3] = SRXL2_OXS_ID ;
            srxl2TxBuffer[4] = srxl2ProcessIn[3]; // fill dest with incoming source
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
}            


void srxl2ProcessIncomingFrame(){
    // we process only handshake and control data frames
    // for an handshake for oXs Id, we reply with an handshake (source and dest are reversed) 
    // 0XA6 + 0X21 + 14 (length) + SourceID + DestinationID + priority(=10) + Baudrate(0=115200) + info + UID (4 bytes) + CRC (2 bytes)
    static uint32_t srxl2LastHandshakeRequestMs = 0; // avoid to send to many telemetry frame the one after the other 
    if  (srxl2ProcessIn[1] ==  SRXL2_HANDSHAKE_CODE ) {
        //printf(("receiving a handshake\n"));
        if ( srxl2ProcessIn[4] == SRXL2_OXS_ID) {   // reply to a handshake for our device ID
            //printf("HS is for XS\n");
            replyToHandshake() ;
            srxl2IsConnected = true;
            srxl2State = SRXL2_RUNNING;
            //printf("Reply to HS has been sent\n");
        } else if ( srxl2ProcessIn[4] == SRXL2_BROADCAST_ID) {   // when destination = FF = broadcast, we do not have to reply
            //printf("HS received for brodcast\n");
            srxl2MasterId = srxl2ProcessIn[3]; // we save the master ID
            // change baudrate if required
            if ( srxl2ProcessIn[6] == 0) {  // 0 means that we will use load baudrate = 115200
                srxl2ValidBaudrate = SRXL2_PORT_BAUDRATE_DEFAULT ; 
            } else {  // else apply 400000 baud rate
                srxl2ValidBaudrate = SRXL2_PORT_BAUDRATE_HIGH ; 
            }
            if ( srxl2CurrentBaudrate != srxl2ValidBaudrate ) {
                changeBaudrate(srxl2Pio, srxl2SmTx, srxl2SmRx, srxl2ValidBaudrate);
                srxl2CurrentBaudrate = srxl2ValidBaudrate;
            }    
            srxl2MasterIdIsValid = true; // from now we can reply to control data frame
            srxl2IsConnected = true;
            srxl2State = SRXL2_RUNNING;
        } 
        // we do not have to reply to other handshake frame
    } else {  // we got a control data frame
// 0XA6 + 0XCD + lenght + command (0=channels, 1=failsafe) + ReplyId + payload + CRC (2 bytes)
//         payload for channels= RSSI(I8) + frameLosses(U16) + channel mask(U32) + n*channel(U16) (n depends on mask)
//         payload for Failsafe= RSSI(I8) + hold(U16) + channel mask(U32) + n*channel(U16) (n depends on mask) 
        // when we get a control data for oXs, we reply with a telemetry frame (if data are available)
        if ( srxl2ProcessIn[4] == SRXL2_OXS_ID) {
            //printf("Control data frame received for oXs at %d\n", (int) microsRp());
            if ( srxl2FillTelemetryFrame() ) {
                srxl2SendFrame(SRXL2_TELEMETRY_FRAME_LENGTH); // send 22 bytes if a buffer has been filled with telemetry
                srxl2IsConnected = true;
                srxl2State = SRXL2_RUNNING;
                //printf("Telemetry frame sent\n");
            }    
        } else if ((srxl2ProcessIn[4] == SRXL2_NO_REPLY) && ( srxl2State == SRXL2_LISTENING) &&
                 (srxl2IsConnected == false) && ( ( millisRp() - srxl2LastHandshakeRequestMs) > 200) ){
            // when there is no reply needed and if we are not yet connected then we request an handshake with a telemetry frame 
            //printf("Sending telemetry for handshaking\n");
            srxl2LastHandshakeRequestMs = millisRp(); 
            srxl2FillTXBuffer(SRXL2_USE_TLM_FOR_HANDSHAKE_REQUEST);
            srxl2SendFrame(SRXL2_TELEMETRY_FRAME_LENGTH);
            srxl2State = SRXL2_RUNNING;
        }
        // decode rc channels (but not other types of data)
        if ( (srxl2ProcessIn[3] == SRXL2_RC_CHANNELS) || (srxl2ProcessIn[3] == SRXL2_RC_FAILSAFE) ){
            srxl2DecodeRcChannels(srxl2ProcessIn[3]);
        }
    }
}    

uint16_t map2Sbus(uint16_t x) {
    return (uint16_t)(((int)x - 342) * (int)(1792 - 191) * 2 / (1706 - 342) +  191 * 2 + 1) / 2; 
}
// to do, limit to 16 channels; store in 16 fields, code the 16 into 22 bits
uint16_t srxl2RcChannels[16] = {0X0400}; // servo mid position coded on 11 bits

// format is:  0XA6 0XCD Length Command(0X0 for channels, 0X01 for failsafe, 0X02 for VTX) ReplyId Payload CRCA CRCB
// for channels, payload= RSSI(1 byte) Framelosses (U16) channelMask(U32) channels(U16)[n X depending on mask]
// channel value must be >> 5 to get usual 11 bits
// for failsafe, payload= RssiMin ( 1 byte) number of hold(u16) channel mask(U32) channels(U16)[n X]
void srxl2DecodeRcChannels(uint8_t channelOrFailsafe){  
    //uint32_t mask = srxl2ProcessIn[11]<<24 | srxl2ProcessIn[10]<<16 | srxl2ProcessIn[9]<<8 | srxl2ProcessIn[8];
    uint16_t mask = srxl2ProcessIn[9]<<8 | srxl2ProcessIn[8];
    uint8_t frameIdx = 12;
    uint8_t channelIdx = 0;
    uint8_t sbus[23];
    //If receiver is in a connected state, and a packet is missed, the channel mask will be 0.
    if (mask == 0) return;
    while (mask) {
        if ( mask & 0X1) {
            srxl2RcChannels[channelIdx] = map2Sbus((srxl2ProcessIn[frameIdx+1]<<8 | srxl2ProcessIn[frameIdx]) >> 5) ;
            //if( frameIdx == 12) {    // to debug Rc channels messages.
            //    printf("Frame begins with\n");
            //    for (uint8_t i = 0; i < 14 ; i++) {
            //        printf(" %X ", srxl2ProcessIn[i]);
            //    }
            //    printf("\n channel 1 = %X\n", srxl2RcChannels[channelIdx]);
            //}
            frameIdx += 2;
        }
        channelIdx++;
        mask = mask >> 1;
    }
    sbus[0] = srxl2RcChannels[0];
    sbus[1] = (srxl2RcChannels[0] >> 8) | (srxl2RcChannels[1] & 0x00FF)<<3;
    sbus[2] = srxl2RcChannels[1]>>5|(srxl2RcChannels[2]<<6);
    sbus[3] = (srxl2RcChannels[2]>>2)& 0x00ff;
    sbus[4] = srxl2RcChannels[2]>>10| (srxl2RcChannels[3] & 0x00FF)<<1;
    sbus[5] = srxl2RcChannels[3]>>7|  (srxl2RcChannels[4] & 0x0FF )<<4;
    sbus[6] = srxl2RcChannels[4]>>4| (srxl2RcChannels[5] & 0xFF) <<7;
    sbus[7] = (srxl2RcChannels[5]>>1)& 0x00ff;
    sbus[8] = srxl2RcChannels[5]>>9| (srxl2RcChannels[6] & 0xFF)<<2;
    sbus[9] = srxl2RcChannels[6]>>6| (srxl2RcChannels[7] & 0xFF)<<5;
    sbus[10] = (srxl2RcChannels[7]>>3)& 0x00ff;//end
    sbus[11] = (srxl2RcChannels[8] & 0XFF);
    sbus[12] = (srxl2RcChannels[8]>> 8) | (srxl2RcChannels[9] & 0xFF)<<3;
    sbus[13] = srxl2RcChannels[9]>>5 | (srxl2RcChannels[10]<<6);
    sbus[14] = (srxl2RcChannels[10]>>2) & 0xff;
    sbus[15] = srxl2RcChannels[10]>>10 | (srxl2RcChannels[11] & 0XFF)<<1;
    sbus[16] = srxl2RcChannels[11]>>7 | (srxl2RcChannels[12] & 0XFF)<<4;
    sbus[17] = srxl2RcChannels[12]>>4 | (srxl2RcChannels[13] & 0XFF)<<7;
    sbus[18] = (srxl2RcChannels[13]>>1)& 0xff;
    sbus[19] = srxl2RcChannels[13]>>9 | (srxl2RcChannels[14] & 0XFF)<<2;
    sbus[20] = srxl2RcChannels[14]>>6 | (srxl2RcChannels[15] & 0XFF)<<5;
    sbus[21] = (srxl2RcChannels[15]>>3)& 0xff;
    
    sbus[22] = 0x00;
    //printf("fs= %d\n", (int) channelOrFailsafe);
    sbusPriFailsafeFlag = false;
    if ( channelOrFailsafe == SRXL2_RC_FAILSAFE) sbusPriFailsafeFlag = true; // same action as sbus[22] |= (1<<3);//FS activated   
    //    if(missingPackets >= 1) sbus[22] |= (1<<2);//frame lost
    memcpy( (uint8_t *) &sbusFrame.rcChannelsData, &sbus[0], 22) ; // copy the data to the Sbus buffer
    sbusFrameCounter++;
    if ( sbusPriFailsafeFlag && (prevFailsafeFlag == false)) sbusFailsafeCounter++;
    prevFailsafeFlag = sbusPriFailsafeFlag;
    lastRcChannels = millisRp();
    lastPriChannelsMillis =  lastRcChannels;
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
// we use also a dummy value SRXL2_USE_TLM_FOR_HANDSHAKE_REQUEST = 0xFF to request a handshake

uint8_t srxl2PriorityList[] = { TELE_DEVICE_VARIO_S , TELE_DEVICE_GPS_BINARY , TELE_DEVICE_AIRSPEED , TELE_DEVICE_ESC,
                                 TELE_DEVICE_RX_MAH };
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
    uint16_t tempU16;
    int16_t tempI16;
    //uint32_t tempU32;
    int32_t tempI32;
    switch (frameIdx) {
        case 0: //TELE_DEVICE_VARIO_S
            if (fields[VSPEED].available) {
                srxl2Frames.vario.identifier = TELE_DEVICE_VARIO_S ; // 0x40
                srxl2Frames.vario.sID = 0;
                
                tempI16 = (int16_t) int_round( fields[RELATIVEALT].value , 10); // from cm to 0.1m increments
                srxl2Frames.vario.altitude = swapBinary(tempI16);
                
                tempI16 = (int16_t) int_round( fields[VSPEED].value , 10);	// change in altitude last 250ms, 0.1m/s increments 
                srxl2Frames.vario.delta_0250ms = swapBinary(tempI16);
                srxl2Frames.vario.delta_0500ms = 0x0010;
				srxl2Frames.vario.delta_1000ms = 0x0100;
				srxl2Frames.vario.delta_1500ms = 0x0020;			
				srxl2Frames.vario.delta_2000ms = 0x0200;	
				srxl2Frames.vario.delta_3000ms = 0x0100;
                

                //srxl2Frames.vario.delta_0500ms = 0xFF7F;
				//srxl2Frames.vario.delta_1000ms = 0xFF7F;
				//srxl2Frames.vario.delta_1500ms = 0xFF7F;			
				//srxl2Frames.vario.delta_2000ms = 0xFF7F;	
				//srxl2Frames.vario.delta_3000ms = 0xFF7F;
                fields[VSPEED].available = false;
                return true;
            } 
            break;
        case 1: //TELE_DEVICE_GPS_BINARY
            if (fields[NUMSAT].available) {
                srxl2Frames.gps.identifier = TELE_DEVICE_GPS_BINARY; //0X26
                srxl2Frames.gps.sID = 0; // Secondary ID
                if (fields[ALTITUDE].available) {
                    tempU16 = (uint16_t) (int_round(fields[ALTITUDE].value + 100000, 100)); // from cm to m, 1000m offset
                    srxl2Frames.gps.altitude = swapBinary(tempU16);
                } else {
                    srxl2Frames.gps.altitude = 0XFFFF;
                }
                
                if (fields[LATITUDE].available) {
                    tempI32 = fields[LATITUDE].value; // degree / 10,000,000
                    srxl2Frames.gps.latitude = swapBinary(tempI32);
                } else {
                    srxl2Frames.gps.latitude = 0XFF7F;
                }
                
                if (fields[LONGITUDE].available) {
                    tempI32 = fields[LONGITUDE].value; // degree / 10,000,000
                     srxl2Frames.gps.longitude = swapBinary(tempI32);
                } else {
                    srxl2Frames.gps.longitude = 0XFF7F;
                }
                
                if (fields[HEADING].available) {
                    tempU16 = (uint16_t) (int_round(fields[HEADING].value , 10)); // from 0.01 deg to 0.1 deg
                    srxl2Frames.gps.heading = swapBinary(tempU16);
                } else {
                    srxl2Frames.gps.heading = 0XFFFF;
                }
                
	            if (fields[GROUNDSPEED].available) {
                    srxl2Frames.gps.groundSpeed = (uint8_t) (int_round(fields[HEADING].value * 36, 1000)); // cm/sec to Km/h
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
            if (fields[AIRSPEED].available) {
                srxl2Frames.airspeed.identifier = TELE_DEVICE_AIRSPEED; //0X11
                srxl2Frames.airspeed.sID = 0; // Secondary ID
                if (fields[AIRSPEED].value >= 0) {
                    tempU16 = (uint16_t) int_round(fields[AIRSPEED].value * 36, 1000); //       from cm/sec to 1 km/h
                    srxl2Frames.airspeed.airspeed = swapBinary(tempU16);
                } else  { 
                    srxl2Frames.airspeed.airspeed = 0;
                }
            	srxl2Frames.airspeed.maxAirspeed =  0XFFFF;
                srxl2Frames.airspeed.reserve1 =  0;  // undefined part of the frame has to be filled with 0
                srxl2Frames.airspeed.reserve2 =  0;
                srxl2Frames.airspeed.reserve3 =  0;
                srxl2Frames.airspeed.reserve4 =  0;
                srxl2Frames.airspeed.reserve5 =  0;
                return true;
            }    
            break;
        case 3: // TELE_DEVICE_ESC
            if ((fields[RPM].available) || (fields[TEMP1].available) || (fields[TEMP2].available)) {
                srxl2Frames.esc.identifier = TELE_DEVICE_ESC; //0X20
                srxl2Frames.esc.sID = 0; // Secondary ID
                if (fields[RPM].available) {
                    tempU16 = (uint16_t) fields[RPM].value * 6  ; //from Hz to 10 tour/min
                    srxl2Frames.esc.RPM = swapBinary(tempU16);
                } else {
                    srxl2Frames.esc.RPM = 0XFFFF;
                }
                if (fields[MVOLT].available) {
                    tempU16 = (uint16_t) (int_round( fields[MVOLT].value ,  10));  // Volts, 0.01V increments
                    srxl2Frames.esc.voltsInput = swapBinary(tempU16);
                } else {
                    srxl2Frames.esc.voltsInput =  0XFFFF;
                }    
                if ((fields[TEMP1].available) && (fields[TEMP1].value > 0)) {
                    tempU16 =  (uint16_t) fields[TEMP1].value * 10; // from degree to 0.1 degree
                    srxl2Frames.esc.tempFET = swapBinary(tempU16);
                } else {
                    srxl2Frames.esc.tempFET =  0XFFFF;
                }
               if (fields[CURRENT].available) {
                    tempU16 = (uint16_t) (int_round( fields[CURRENT].value ,  10));  //// Instantaneous current, 0.01A (0-655.34A)		7FFF-> no data  
                    srxl2Frames.esc.currentMotor  = swapBinary(tempU16);
                } else {
                    srxl2Frames.esc.currentMotor = 0xFFFF ; 
                }
                if ((fields[TEMP2].available) && (fields[TEMP2].value > 0)) {
                    tempU16 =    fields[TEMP2].value * 10; // from degree to 0.1 degree
                    srxl2Frames.esc.tempBEC = swapBinary(tempU16);
                } else {
                    srxl2Frames.esc.tempBEC =  0XFFFF;
                }
                
                srxl2Frames.esc.currentBEC =  0XFF;
                srxl2Frames.esc.voltsBEC =  0XFF;
                srxl2Frames.esc.throttle =  0XFF; 
                srxl2Frames.esc.powerOut =  0XFF;
                return true ;
            }
            break;    
/*
    uint16_t		RPM;															// Electrical RPM, 10RPM (0-655340 RPM)  0xFFFF --> "No data"
	uint16_t		voltsInput;														// Volts, 0.01v (0-655.34V)       0xFFFF --> "No data"
	uint16_t		tempFET;														// Temperature, 0.1C (0-6553.4C)  0xFFFF --> "No data"
	uint16_t		currentMotor;													// Current, 10mA (0-655.34A)      0xFFFF --> "No data"
	uint16_t		tempBEC;														// Temperature, 0.1C (0-6553.4C)  0xFFFF --> "No data"
	uint8_t		currentBEC;														// BEC Current, 100mA (0-25.4A)   0xFF ----> "No data"
	uint8_t		voltsBEC;														// BEC Volts, 0.05V (0-12.70V)    0xFF ----> "No data"
	uint8_t		throttle;														// 0.5% (0-100%)                  0xFF ----> "No data"
	uint8_t		powerOut;														// Power Output, 0.5% (0-127%)    0xFF ----> "No data"
} STRU_TELE_ESC;
*/
/*
        case 4: //TELE_DEVICE_RX_MAH
            if (fields[MVOLT].available || fields[CURRENT].available){
                srxl2Frames.voltCurrentCap.identifier = TELE_DEVICE_RX_MAH ;  // 0X18
                srxl2Frames.voltCurrentCap.sID = 0; // Secondary ID
                if (fields[CURRENT].available) {
                    tempI16 = (int16_t) (int_round( fields[CURRENT].value ,  10));  //// Instantaneous current, 0.01A (0-328.7A)		7FFF-> no data  
                    srxl2Frames.voltCurrentCap.current_A  = swapBinary(tempI16);
                } else {
                    srxl2Frames.voltCurrentCap.current_A = 0xFF7F ; 
                }
                
                if (fields[CAPACITY].available) {
                    if ( fields[CAPACITY].value > 0 ){   
                        tempU16 =  (uint16_t) (int_round( fields[CAPACITY].value ,  1));  //// Integrated mAh used, 0.1mAh (0-3276.6mAh)  
                        srxl2Frames.voltCurrentCap.chargeUsed_A  = swapBinary(tempU16);
                    } else {
                        srxl2Frames.voltCurrentCap.chargeUsed_A  = 0 ;
                    }
                } else {
                    srxl2Frames.voltCurrentCap.chargeUsed_A = 0xFFFF ; 
                }
                
                if (fields[MVOLT].available) {
                    if ( fields[MVOLT].value > 0 ){   
                        tempU16  = (uint16_t) (int_round( fields[MVOLT].value ,  10));  // Volts, 0.01V increments (0-16.00V)
                        srxl2Frames.voltCurrentCap.volts_A  = swapBinary(tempU16);
                    } else {
                        srxl2Frames.voltCurrentCap.volts_A  = 0 ;
                    }
                } else {
                    srxl2Frames.voltCurrentCap.volts_A = 0xFFFF ; 
                }
                
                srxl2Frames.voltCurrentCap.current_B = 0xFF7F ;
                srxl2Frames.voltCurrentCap.chargeUsed_B = 0xFFFF ;
                srxl2Frames.voltCurrentCap.volts_B = 0xFFFF ;
                srxl2Frames.voltCurrentCap.alerts = 0 ;// Bit mapped alert conditions (see below)
                srxl2Frames.voltCurrentCap.highCharge = 0;	//High nybble is extra bits for chargeUsed_B, Low is for chargeUsed_A
                return true;
            }
            break;
*/
        default:
            return false;
            break;    
    };
    return false;    
}


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
            memcpy(&srxl2TxBuffer[4], &srxl2Frames.airspeed.identifier, 16);
            break;
        case 3: // TELE_DEVICE_ESC
            memcpy(&srxl2TxBuffer[4], &srxl2Frames.esc.identifier, 16);
            break;
        case 4: //TELE_DEVICE_RX_MAH
            memcpy(&srxl2TxBuffer[4], &srxl2Frames.voltCurrentCap.identifier, 16);
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
    //printf("sending : ");
    //for (uint8_t i= 0; i < length ; i++) printf(" %X ", srxl2TxBuffer[i]);
    //printf("\n");
    srxl2_uart_rx_program_stop(srxl2Pio, srxl2SmRx, config.pinPrimIn); // stop receiving
    srxl2_uart_tx_program_start(srxl2Pio, srxl2SmTx, config.pinPrimIn, false); // prepare to transmit; no invert
    // start the DMA channel with the data to transmit
    dma_channel_set_read_addr (srxl2_dma_chan, &srxl2TxBuffer[0], false);
    dma_channel_set_trans_count (srxl2_dma_chan, length , true) ;  // start with the right length
    // we need a way to set the pio back in receive mode when all bytes are sent 
    // this will be done in the main loop after some ms (here 2ms)
    restoreSrxl2PioToReceiveMicros = microsRp() + 3000; //(uint32_t) (1000000.0 * 10.0 * length / srxl2CurrentBaudrate);   
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
