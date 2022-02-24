#include <Arduino.h>
#include "hardware/pio.h"
#include "hardware/dma.h"
//#include "hardware/irq.h"
#include "uart_tx.pio.h"

#include "crsf.h"
#include "config_basic.h"
#include "crc.h"

#define VOLTAGE_FRAME_INTERVAL 500 // msec
#define VARIO_FRAME_INTERVAL 100 
#define GPS_FRAME_INTERVAL 500
#define ATTITUDE_FRAME_INTERVAL 100

#define FRAME_TYPES_MAX 5
uint32_t crsfFrameNextMillis[FRAME_TYPES_MAX] = {0} ; 
uint8_t crsf_last_frame_idx = 0 ;  

#if defined(ARDUINO_MEASURES_VOLTAGES) && (ARDUINO_MEASURES_VOLTAGES == YES)
voltageFrameStruct voltageFrame;
extern VOLTAGE voltage ;
#endif  
#if defined( VARIO1) && (VARIO1 == MS5611)
varioFrameStruct varioFrame;
attitudeFrameStruct attitudeFrame;
extern VARIO vario1 ;
#endif  
#if defined(A_GPS_IS_CONNECTED) && (A_GPS_IS_CONNECTED == YES)
gpsFrameStruct gpsFrame;
extern GPS gps ;
#endif 



uint8_t CRSFBuffer[50]; // buffer that contains the frame to be sent (cvia dma)
uint8_t CRSFBufferLength;

const uint PIN_TX = 8;
// This is the same as the default UART baud rate on Pico
const uint SERIAL_BAUD_CRSF = 115200;

PIO pio = pio0; // we use pio 0; DMA is hardcoded to use it
uint sm = 0;  // we use the state machine 0 ; DMA is harcoded to use it (DREQ) 

int dma_chan;
dma_channel_config c;

// Set up a PIO state machine to serialise our bits
void setup_DMA_PIO(){
    // setup the PIO for TX UART
    uint offset = pio_add_program(pio, &uart_tx_program);
    uart_tx_program_init(pio, sm, offset, PIN_TX, SERIAL_BAUD_CRSF);

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
    #if defined(ARDUINO_MEASURES_VOLTAGES) && (ARDUINO_MEASURES_VOLTAGES == YES)
    #endif
    #if defined(A_GPS_IS_CONNECTED) && (A_GPS_IS_CONNECTED == YES)
    #endif
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
        #if defined(ARDUINO_MEASURES_VOLTAGES) && (ARDUINO_MEASURES_VOLTAGES == YES)  
        case  CRSF_FRAMEIDX_BATTERY_SENSOR : 
            return voltage.mVolt[0].available ;
        #endif
        #if defined( VARIO1) && (VARIO1 == MS5611)
        //case CRSF_FRAMEIDX_VARIO :
        //    return vario1.climbRate.available ;    
        case CRSF_FRAMEIDX_ATTITUDE :
            return vario1.climbRate.available ;    // in this version, attitude frame is used to transmit altitude and vspeed 
        #endif

        #if defined(A_GPS_IS_CONNECTED) && (A_GPS_IS_CONNECTED == YES)
        case CRSF_FRAMEIDX_GPS :
            return gps.GPS_lonAvailable ;    
        #endif
    }
    return false;
    // to continue with other frames/data
}


#if defined(ARDUINO_MEASURES_VOLTAGES) && (ARDUINO_MEASURES_VOLTAGES == YES)  
void fillFrameBattery(uint8_t idx){
    voltageFrame.device_addr = CRSF_ADDRESS_CRSF_RECEIVER;
    voltageFrame.frame_size = CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE + 2; // + 2 because we add type and crc byte 
    voltageFrame.type = CRSF_FRAMETYPE_BATTERY_SENSOR ;
    voltageFrame.mVolt = voltage.mVolt[0].value ;
    voltageFrame.current = 0 ;
    voltageFrame.capacity = 0;
    voltageFrame.remain = 0;
    voltageFrame.crc = crsf_crc.calc( ((uint8_t *) &voltageFrame) + 2 , CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE- 1)  ; // CRC skip 2 bytes( addr of message and frame size); length include type + 6 for payload  
    voltage.mVolt[0].available = false ;
    crsfFrameNextMillis[idx] = millis() + VOLTAGE_FRAME_INTERVAL;
    printf("filling dma buffer with voltage\n");
    CRSFBufferLength = sizeof(voltageFrame);
    memcpy(&CRSFBuffer[0] , &voltageFrame , CRSFBufferLength);
    dma_channel_set_read_addr (dma_chan, &CRSFBuffer[0], false);
    dma_channel_set_trans_count (dma_chan, CRSFBufferLength, true) ;    
}
#endif

#if defined( VARIO1) && (VARIO1 == MS5611)
void fillFrameVario(uint8_t idx){
    varioFrame.device_addr = CRSF_ADDRESS_CRSF_RECEIVER;
    varioFrame.frame_size = CRSF_FRAME_VARIO_PAYLOAD_SIZE + 2; // + 2 because we add type and crc byte 
    varioFrame.type = CRSF_FRAMETYPE_VARIO ;
    varioFrame.vSpeed = vario1.climbRate.value ;
    varioFrame.crc = crsf_crc.calc( ((uint8_t *) &varioFrame) + 2 , CRSF_FRAME_VARIO_PAYLOAD_SIZE- 1)  ; // CRC skip 2 bytes( addr of message and frame size); length include type + 6 for payload  
    vario1.climbRate.available = false ;
    crsfFrameNextMillis[idx] = millis() + VARIO_FRAME_INTERVAL;
    printf("filling dma buffer with vario data\n");
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
    attitudeFrame.pitch = vario1.absoluteAlt.value;
    attitudeFrame.pitch = vario1.relativeAlt.value;
    attitudeFrame.crc = crsf_crc.calc( ((uint8_t *) &attitudeFrame) + 2 , CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE- 1)  ; // CRC skip 2 bytes( addr of message and frame size); length include type + 6 for payload  
    vario1.climbRate.available = false ;
    crsfFrameNextMillis[idx] = millis() + ATTITUDE_FRAME_INTERVAL;
    printf("filling dma buffer with vario and altitude data\n");
    CRSFBufferLength = sizeof(attitudeFrame);
    memcpy(&CRSFBuffer[0] , &attitudeFrame , CRSFBufferLength);
    dma_channel_set_read_addr (dma_chan, &CRSFBuffer[0], false);
    dma_channel_set_trans_count (dma_chan, CRSFBufferLength, true) ;    
}
#endif

#if defined(A_GPS_IS_CONNECTED) && (A_GPS_IS_CONNECTED == YES)
void fillFrameGps(uint8_t idx){
    gpsFrame.device_addr = CRSF_ADDRESS_CRSF_RECEIVER;
    gpsFrame.frame_size = CRSF_FRAME_GPS_PAYLOAD_SIZE + 2; // + 2 because we add type and crc byte 
    gpsFrame.type = CRSF_FRAMETYPE_GPS ;
    gpsFrame.latitude = gps.GPS_lat ;
    gpsFrame.longitude = gps.GPS_lon ;    // (degree / 10`000`000 )
    gpsFrame.groundspeed = gps.GPS_speed_3d ;  // ( km/h / 10 )
    gpsFrame.heading = gps.GPS_ground_course / 1000;      //( degree / 100  instead of 5 decimals)
    gpsFrame.altitude = gps.GPS_altitude ;     //( meter ­1000m offset )
    gpsFrame.numSat = gps.GPS_numSat;       //( counter )
    gpsFrame.crc = crsf_crc.calc( ((uint8_t *) &gpsFrame) + 2 , CRSF_FRAME_GPS_PAYLOAD_SIZE- 1)  ; // CRC skip 2 bytes( addr of message and frame size); length include type + 6 for payload  
    gps.GPS_lonAvailable = false ;
    crsfFrameNextMillis[idx] = millis() + GPS_FRAME_INTERVAL;
    printf("filling dma buffer for GPS\n");
    CRSFBufferLength = sizeof(gpsFrame);
    memcpy(&CRSFBuffer[0] , &gpsFrame , CRSFBufferLength);
    dma_channel_set_read_addr (dma_chan, &CRSFBuffer[0], false);
    dma_channel_set_trans_count (dma_chan, CRSFBufferLength, true) ;    
}
#endif

void fillOneFrame(uint8_t idx){
    switch (idx) {
        #if defined(ARDUINO_MEASURES_VOLTAGES) && (ARDUINO_MEASURES_VOLTAGES == YES)  
        case  CRSF_FRAMEIDX_BATTERY_SENSOR : 
            fillFrameBattery(idx);
            return ;
        #endif
        #if defined( VARIO1) && (VARIO1 == MS5611)
        //case CRSF_FRAMEIDX_VARIO :
        //    fillFrameVario(idx);
        //    return ;
        case CRSF_FRAMEIDX_ATTITUDE :
            fillFrameAttitude(idx);
            return ;
        #endif
        #if defined(A_GPS_IS_CONNECTED) && (A_GPS_IS_CONNECTED == YES)
        case CRSF_FRAMEIDX_GPS :
            fillFrameGps(idx);
            return ;
        #endif
                
    } // end switch
}

