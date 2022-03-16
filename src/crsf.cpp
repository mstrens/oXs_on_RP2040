#include "pico/stdlib.h"
#include <stdlib.h>
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "uart_tx.pio.h"
#include "uart_rx.pio.h"
#include "crsf.h"
#include "config.h"
#include "crc.h"
#include "pico/util/queue.h"
#include "MS5611.h"
#include "tools.h"
#include "stdio.h"
#include <string.h> // memcpy
#include "param.h"
#include <inttypes.h>


//// This is the same as the default UART baud rate on Pico
//const uint SERIAL_BAUD_CRSF = config.crsfBaudrate;


#define FRAME_TYPES_MAX 5
uint32_t crsfFrameNextMillis[FRAME_TYPES_MAX] = {0} ; 
uint8_t crsf_last_frame_idx = 0 ;  

voltageFrameStruct voltageFrame;
extern VOLTAGE voltage ;
  
varioFrameStruct varioFrame;
attitudeFrameStruct attitudeFrame;
extern MS5611 baro1; 
extern VARIO vario1 ;

gpsFrameStruct gpsFrame;
extern GPS gps ;

extern CONFIG config;

uint8_t CRSFBuffer[50]; // buffer that contains the frame to be sent (cvia dma)
uint8_t CRSFBufferLength;

const uint PIN_TX = 10;
const uint PIN_RX = 9;

queue_t crsfRxQueue ; // queue uses to push the data from the uart pio rx to the main loop

PIO pio = pio0; // we use pio 0; DMA is hardcoded to use it
uint smTx = 0;  // we use the state machine 0 for Tx; DMA is harcoded to use it (DREQ) 
uint smRx = 1;  // we use the state machine 1 for Rx; 

int dma_chan;
dma_channel_config c;



// Set up a PIO state machine to serialise our bits
void setup_DMA_PIO(){
    // setup the PIO for TX UART
    uint offsetTx = pio_add_program(pio, &uart_tx_program);
    uart_tx_program_init(pio, smTx, offsetTx, PIN_TX, config.crsfBaudrate);

    // Configure a channel to write the same word (32 bits) repeatedly to PIO0
    // SM0's TX FIFO, paced by the data request signal from that peripheral.
    dma_chan = dma_claim_unused_channel(true);
    c = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, DREQ_PIO0_TX0);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    dma_channel_configure(
        dma_chan,
        &c,
        &pio0_hw->txf[0], // Write address (only need to set this once)
        &CRSFBuffer[0],   // we use always the same buffer             
        0 , // do not yet provide the number of bytes (DMA cycles)
        false             // Don't start yet
    );
    // do not set interrupt on DMA. The main loop will check if DMA is busy before sending
}

GENERIC_CRC8 crsf_crc(CRSF_CRC_POLY);

void setupCRSF(){
}


void fillCRSFFrame(){
    static uint8_t crsf_last_frame_idx = 0 ;
    if ( dma_channel_is_busy(dma_chan) )return ; // skip if the DMA is still sending data
    uint32_t _millis = millis();
    for (uint8_t i = 0 ; i< FRAME_TYPES_MAX ; i++ ){
         crsf_last_frame_idx++;
         if (crsf_last_frame_idx >= FRAME_TYPES_MAX) crsf_last_frame_idx ;
         if ( (_millis >= crsfFrameNextMillis[crsf_last_frame_idx]) && (dataAvailable(crsf_last_frame_idx))) {
             fillOneFrame(crsf_last_frame_idx);
             continue;
         }
    }
}

bool dataAvailable(uint8_t idx) {
    switch (idx) {
        case  CRSF_FRAMEIDX_BATTERY_SENSOR : 
            return voltage.mVolt[0].available ;
        case CRSF_FRAMEIDX_VARIO :
            return vario1.climbRate.available ;    
        case CRSF_FRAMEIDX_ATTITUDE :
            return vario1.relativeAlt.available ;    // in this version, attitude frame is used to transmit altitude         
        case CRSF_FRAMEIDX_GPS :
            return gps.GPS_lonAvailable ;    
    }
    return false;
    // to continue with other frames/data
}

void fillFrameBattery(uint8_t idx){
    voltageFrame.device_addr = CRSF_ADDRESS_CRSF_RECEIVER;
    voltageFrame.frame_size = CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE + 2; // + 2 because we add type and crc byte 
    voltageFrame.type = CRSF_FRAMETYPE_BATTERY_SENSOR ;
    if ( voltage.mVolt[0].value > 0 ) {
        voltageFrame.mVolt = voltage.mVolt[0].value ;
    } else  voltageFrame.mVolt ;
    if ( voltage.mVolt[1].value > 0 ) {
        voltageFrame.current = voltage.mVolt[1].value ;
    } else voltageFrame.current = 0;
    if ( voltage.mVolt[2].value > 0 ) {
        voltageFrame.capacity = voltage.mVolt[2].value;
    } else voltageFrame.capacity = 0;
    if ( voltage.mVolt[3].value > 0 ) {
        voltageFrame.remain =  voltage.mVolt[3].value /10 ;  // it is only a uint8_t; to use for a voltage we divide by 10
    } else voltageFrame.remain = 0;
    voltageFrame.crc = crsf_crc.calc( ((uint8_t *) &voltageFrame) + 2 , CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE- 1)  ; // CRC skip 2 bytes( addr of message and frame size); length include type + 6 for payload  
    voltage.mVolt[0].available = false ;
    crsfFrameNextMillis[idx] = millis() + VOLTAGE_FRAME_INTERVAL;
    //printf("filling dma buffer with voltage\n");
    CRSFBufferLength = sizeof(voltageFrame);
    memcpy(&CRSFBuffer[0] , &voltageFrame , CRSFBufferLength);
    dma_channel_set_read_addr (dma_chan, &CRSFBuffer[0], false);
    dma_channel_set_trans_count (dma_chan, CRSFBufferLength, true) ;    
}

void fillFrameVario(uint8_t idx){
    if (! baro1.baroInstalled) return ; // skip when vario is not installed    
    varioFrame.device_addr = CRSF_ADDRESS_CRSF_RECEIVER;
    varioFrame.frame_size = CRSF_FRAME_VARIO_PAYLOAD_SIZE + 2; // + 2 because we add type and crc byte 
    varioFrame.type = CRSF_FRAMETYPE_VARIO ;
    varioFrame.vSpeed = vario1.climbRate.value ;
    varioFrame.crc = crsf_crc.calc( ((uint8_t *) &varioFrame) + 2 , CRSF_FRAME_VARIO_PAYLOAD_SIZE- 1)  ; // CRC skip 2 bytes( addr of message and frame size); length include type + 6 for payload  
    vario1.climbRate.available = false ;
    crsfFrameNextMillis[idx] = millis() + VARIO_FRAME_INTERVAL;
    //printf("filling dma buffer with vario data\n");
    CRSFBufferLength = sizeof(varioFrame);
    memcpy(&CRSFBuffer[0] , &varioFrame , CRSFBufferLength);
    dma_channel_set_read_addr (dma_chan, &CRSFBuffer[0], false);
    dma_channel_set_trans_count (dma_chan, CRSFBufferLength, true) ;    
}
void fillFrameAttitude(uint8_t idx){
    attitudeFrame.device_addr = CRSF_ADDRESS_CRSF_RECEIVER;
    attitudeFrame.frame_size = CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE + 2; // + 2 because we add type and crc byte 
    attitudeFrame.type = CRSF_FRAMETYPE_ATTITUDE;
    attitudeFrame.pitch = vario1.climbRate.value;
    attitudeFrame.roll = vario1.absoluteAlt.value / 100; //in m
    attitudeFrame.yaw = vario1.relativeAlt.value /100; //in m
    attitudeFrame.crc = crsf_crc.calc( ((uint8_t *) &attitudeFrame) + 2 , CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE- 1)  ; // CRC skip 2 bytes( addr of message and frame size); length include type + 6 for payload  
    vario1.relativeAlt.available = false ;
    crsfFrameNextMillis[idx] = millis() + ATTITUDE_FRAME_INTERVAL;
    //printf("filling dma buffer with vario and altitude data\n");
    //printf("alt abs %" PRIi16 "\n", attitudeFrame.roll);
    CRSFBufferLength = sizeof(attitudeFrame);
    memcpy(&CRSFBuffer[0] , &attitudeFrame , CRSFBufferLength);
    dma_channel_set_read_addr (dma_chan, &CRSFBuffer[0], false);
    dma_channel_set_trans_count (dma_chan, CRSFBufferLength, true) ;    
}

void fillFrameGps(uint8_t idx){
    gpsFrame.device_addr = CRSF_ADDRESS_CRSF_RECEIVER;
    gpsFrame.frame_size = CRSF_FRAME_GPS_PAYLOAD_SIZE + 2; // + 2 because we add type and crc byte 
    gpsFrame.type = CRSF_FRAMETYPE_GPS ;
    gpsFrame.latitude = gps.GPS_lat ;
    gpsFrame.longitude = gps.GPS_lon ;    // (degree / 10`000`000 )
    gpsFrame.groundspeed = gps.GPS_speed_3d ;  // ( km/h / 10 )
    gpsFrame.heading = gps.GPS_ground_course / 1000;      //( degree / 100  instead of 5 decimals)
    gpsFrame.altitude = gps.GPS_altitude /1000;     //( mm to m )
    gpsFrame.numSat = gps.GPS_numSat;       //( counter )
    gpsFrame.crc = crsf_crc.calc( ((uint8_t *) &gpsFrame) + 2 , CRSF_FRAME_GPS_PAYLOAD_SIZE- 1)  ; // CRC skip 2 bytes( addr of message and frame size); length include type + 6 for payload  
    gps.GPS_lonAvailable = false ;
    crsfFrameNextMillis[idx] = millis() + GPS_FRAME_INTERVAL;
    //printf("filling dma buffer for GPS height:%" PRIu16 "\n",gpsFrame.altitude);
    CRSFBufferLength = sizeof(gpsFrame);
    memcpy(&CRSFBuffer[0] , &gpsFrame , CRSFBufferLength);
    dma_channel_set_read_addr (dma_chan, &CRSFBuffer[0], false);
    dma_channel_set_trans_count (dma_chan, CRSFBufferLength, true) ;    
}

void fillOneFrame(uint8_t idx){
    switch (idx) {
        case  CRSF_FRAMEIDX_BATTERY_SENSOR : 
            fillFrameBattery(idx);
            return ;
        case CRSF_FRAMEIDX_VARIO :
            fillFrameVario(idx);
            return ;
        case CRSF_FRAMEIDX_ATTITUDE :
            fillFrameAttitude(idx);
            return ;
        case CRSF_FRAMEIDX_GPS :
            fillFrameGps(idx);
            return ;                
    } // end switch
}

// here the code to read the CRSF frames from the receiver (in order to get the RC channels data)


void pioRxHandlerIrq(){    // when a byte is received on the Sport, read the pio Sport fifo and push the data to a queue (to be processed in the main loop)
  // clear the irq flag
  irq_clear (PIO0_IRQ_0 );
  while (  ! pio_sm_is_rx_fifo_empty (pio ,smRx)){ // when some data have been received
     uint8_t c = pio_sm_get (pio , smRx) >> 24;         // read the data
     queue_try_add (&crsfRxQueue, &c);          // push to the queue
    //sportRxMillis = millis();                    // save the timestamp.
  }
}

void setupCrsfRxPio (void){
    // configure the queue to get the data from crsf in the irq handle
    queue_init (&crsfRxQueue, sizeof(uint8_t), 250);

    // set an irq on pio to handle a received byte
    irq_set_exclusive_handler( PIO0_IRQ_0 , pioRxHandlerIrq) ;
    irq_set_enabled (PIO0_IRQ_0 , true) ;

    uint offsetRx = pio_add_program(pio, &uart_rx_program);
    uart_rx_program_init(pio, smRx, offsetRx, PIN_RX, config.crsfBaudrate);
}


// a CRSF RC channel frame contains
// 1 byte with CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
// 1 byte with the payload length + 2 (type and crc) RC_PAYLOAD_LENGTH_PLUS2 = 24
// 1 byte with the type : CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
// 22 bytes = 16 channels (11 bits)
// 1 byte with the crc

#define CRSF_ADDRESS_FLIGHT_CONTROLLER    0xC8
#define RC_PAYLOAD_LENGTH                   22 // does not include the Type and CRC
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16

sbusFrame_s sbusFrame;

enum CRSFState{
    NO_FRAME = 0,
    WAIT_PAYLOAD_LENGTH,
    WAIT_FRAMETYPE_RC_CHANNELS_PACKED,
    RECEIVING_RC_CHANNELS,
    WAIT_CRC
} ;

uint32_t lastCrsfRcChannels = 0;
void handleCrsfRx(void){   // called by main loop : receive the CRSF frame
    static uint8_t crsfRxState = NO_FRAME;
    static uint8_t counter = 0;
    static uint8_t bufferRcChannels[RC_PAYLOAD_LENGTH];
    uint8_t data;
    uint8_t crc = 0; 
    while (! queue_is_empty(&crsfRxQueue)) {
        queue_try_remove (&crsfRxQueue,&data);
        switch ( crsfRxState ) {
            case NO_FRAME:
                if (data == CRSF_ADDRESS_FLIGHT_CONTROLLER) crsfRxState = WAIT_PAYLOAD_LENGTH;
            break;
            case  WAIT_PAYLOAD_LENGTH:
                if (data == (RC_PAYLOAD_LENGTH + 2)){
                    crsfRxState = WAIT_FRAMETYPE_RC_CHANNELS_PACKED;
                } else if (data == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
                    crsfRxState = WAIT_PAYLOAD_LENGTH;
                } else {
                    crsfRxState = NO_FRAME ;
                }
            break;
            case  WAIT_FRAMETYPE_RC_CHANNELS_PACKED:
                if (data == CRSF_FRAMETYPE_RC_CHANNELS_PACKED){
                    crsfRxState = RECEIVING_RC_CHANNELS;
                    counter = 0;
                } else if (data == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
                    crsfRxState = WAIT_PAYLOAD_LENGTH;
                } else {
                    crsfRxState = NO_FRAME ;
                }
            break;
            case  RECEIVING_RC_CHANNELS:
                bufferRcChannels[counter++] = data;
                if ( counter == RC_PAYLOAD_LENGTH ) {
                    crsfRxState = WAIT_CRC ;
                }                   
            break;
            case  WAIT_CRC:
                crc = crsf_crc.calc(CRSF_FRAMETYPE_RC_CHANNELS_PACKED); // CRC calculation includes the Type of message
                crc = crsf_crc.calc(&bufferRcChannels[0] ,  RC_PAYLOAD_LENGTH + 1, crc);
                if ( crc == data){
                    // we got a good frame; we can save for later use
                    memcpy(&sbusFrame.rcChannelsData, bufferRcChannels , RC_PAYLOAD_LENGTH) ;
                    lastCrsfRcChannels = millis();
                }
            break;
        }
    }        
}

void printAttitudeFrame(){
    printf("Attitude frame\n");
    printf("  device= 0x%X\n", attitudeFrame.device_addr);
    printf("  size  = 0x%X\n", attitudeFrame.frame_size);
    printf("  type  = 0x%X\n", attitudeFrame.type);
    printf("  pitch = %" PRIi16 "\n" , attitudeFrame.pitch);
    printf("  roll  = %" PRIi16 "\n" , attitudeFrame.roll);
    printf("  yaw   = %" PRIi16 "\n" , attitudeFrame.yaw);
    printf("  crc   = 0x%X\n", attitudeFrame.crc);
}

void printGpsFrame(){
    printf("GPS frame\n");
    printf("  device= 0x%X\n", gpsFrame.device_addr);
    printf("  size  = 0x%X\n", gpsFrame.frame_size);
    printf("  type  = 0x%X\n", gpsFrame.type);
    printf("  lat   = %" PRIi32 "\n" , gpsFrame.latitude);
    printf("  long  = %" PRIi32 "\n" , gpsFrame.longitude);
    printf("  speed = %" PRIu16 "\n" , (uint16_t) gpsFrame.groundspeed);
    printf("  head  = %" PRIu16 "\n" , (uint16_t) gpsFrame.heading);
    printf("  alt   = %" PRIu16 "\n" , (uint16_t) gpsFrame.altitude);
    printf("  numSat= 0x%X\n" , gpsFrame.numSat);
    printf("  crc   = 0x%X\n", gpsFrame.crc);
}

void printBatteryFrame(){
    printf("Battery frame\n");
    printf("  device= 0x%X\n", voltageFrame.device_addr);
    printf("  mvolt = %" PRIu16 "\n" , voltageFrame.mVolt);
    printf("  curr  = %" PRIu16 "\n" , voltageFrame.current);
    printf("  capac = %" PRIu32 "\n" , (uint32_t) voltageFrame.capacity);
    printf("  remain= 0x%X\n" , (uint8_t) voltageFrame.remain);
    printf("  crc   = 0x%X\n", voltageFrame.crc);
}