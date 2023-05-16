#include "pico/stdlib.h"
#include <stdlib.h>
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "uart_crsf_tx.pio.h"
#include "crsf_out.h"
#include "config.h"
#include "crc.h"
#include "pico/util/queue.h"
#include "MS5611.h"
#include "SPL06.h"
#include "tools.h"
#include "stdio.h"
#include <string.h> // memcpy
#include "param.h"
#include <inttypes.h>
#include "mpu.h"

#define FRAME_TYPES_MAX 5
uint32_t crsfFrameNextMillis[FRAME_TYPES_MAX] = {0} ; 
uint8_t crsf_last_frame_idx = 0 ;  

extern field fields[];  // list of all telemetry fields and parameters used by Sport

voltageFrameStruct voltageFrame;
  
varioFrameStruct varioFrame;
attitudeFrameStruct attitudeFrame;
baroAltitudeFrameStruct baroAltitudeFrame;

gpsFrameStruct gpsFrame;
extern GPS gps ;

extern CONFIG config;
extern uint8_t debugTlm;
extern MPU mpu;

uint8_t CRSFBuffer[50]; // buffer that contains the frame to be sent (via dma)
uint8_t CRSFBufferLength;


PIO crsfPio = pio0; // we use pio 0; DMA is hardcoded to use it
uint crsfSmTx = 0;  // we use the state machine 0 for Tx; DMA is harcoded to use it (DREQ) 

int crsf_dma_chan;
dma_channel_config c;

GENERIC_CRC8 crsf_crc_out(CRSF_CRC_POLY);

void setupCrsfOut(){
    if (config.pinTlm == 255) return ; // Skip when no telemetry is foreseen 
    // setup the PIO for TX UART
    uint crsfOffsetTx = pio_add_program(crsfPio, &uart_tx_program);
    uart_tx_program_init(crsfPio, crsfSmTx, crsfOffsetTx, config.pinTlm, config.crsfBaudrate);
    // Configure a channel to write the same word (32 bits) repeatedly to PIO0
    // SM0's TX FIFO, paced by the data request signal from that peripheral.
    crsf_dma_chan = dma_claim_unused_channel(true);
    c = dma_channel_get_default_config(crsf_dma_chan);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, DREQ_PIO0_TX0);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    dma_channel_configure(
        crsf_dma_chan,
        &c,
        &pio0_hw->txf[0], // Write address (only need to set this once)
        &CRSFBuffer[0],   // we use always the same buffer             
        0 , // do not yet provide the number of bytes (DMA cycles)
        false             // Don't start yet
    );
    // do not set interrupt on DMA. The main loop will check if DMA is busy before sending    
}


void fillCRSFFrame(){
    static uint8_t crsf_last_frame_idx = 0 ;
    if (config.pinTlm == 255) return ; // skip when Tlm is not foreseen
    if ( dma_channel_is_busy(crsf_dma_chan) )return ; // skip if the DMA is still sending data
    uint32_t _millis = millisRp();
    for (uint8_t i = 0 ; i< FRAME_TYPES_MAX ; i++ ){
         crsf_last_frame_idx++;
         if (crsf_last_frame_idx >= FRAME_TYPES_MAX) crsf_last_frame_idx = 0;
         if ( (_millis >= crsfFrameNextMillis[crsf_last_frame_idx]) && (dataAvailable(crsf_last_frame_idx))) {
            fillOneFrame(crsf_last_frame_idx) ; // if one frame has been filled
            break;  
         }
    }
}


bool dataAvailable(uint8_t idx) {
    switch (idx) {
        case  CRSF_FRAMEIDX_BATTERY_SENSOR : 
            return fields[MVOLT].available || fields[CURRENT].available || fields[CAPACITY].available  ;
        case CRSF_FRAMEIDX_VARIO :
            return fields[VSPEED].available ;    
        case CRSF_FRAMEIDX_ATTITUDE :
            return  fields[PITCH].available || fields[ROLL].available ;           
        case CRSF_FRAMEIDX_GPS :
            return gps.gpsInstalled ;   
            //return gps.gpsInstalled || baro1.baroInstalled ; //gps.GPS_lonAvailable ;  // gps.gpsInstalled || baro1.baroInstalled 
        case CRSF_FRAMEIDX_BARO_ALTITUDE :
            return fields[RELATIVEALT].available ;        
    }
    return false;
    // to be continue with other frames/data if ELRS support them.
}

void fillBufferU8( uint8_t val){
    CRSFBuffer[CRSFBufferLength++] = val;
}

void fillBufferU16( uint16_t val){
    CRSFBuffer[CRSFBufferLength++] = val >> 8 ;
    CRSFBuffer[CRSFBufferLength++] = val & 0xFF ;
}

void fillBufferI16( int16_t val){
    CRSFBuffer[CRSFBufferLength++] = val >> 8 ;
    CRSFBuffer[CRSFBufferLength++] = val & 0xFF ;
}

void fillBufferU24( uint32_t val){
    CRSFBuffer[CRSFBufferLength++] = (uint8_t) ((val >> 16 )  & 0xFF) ;
    CRSFBuffer[CRSFBufferLength++] = (uint8_t) ((val >> 8 )  & 0xFF) ;
    CRSFBuffer[CRSFBufferLength++] = (uint8_t) (val & 0xFF);
}

void fillBufferI24( int32_t val){
    CRSFBuffer[CRSFBufferLength++] = (uint8_t) ((val >> 16 )  & 0xFF) ;
    CRSFBuffer[CRSFBufferLength++] = (uint8_t) ((val >> 8 )  & 0xFF) ;
    CRSFBuffer[CRSFBufferLength++] = (uint8_t) (val & 0xFF);
}

void fillBufferI32( int32_t val){
    CRSFBuffer[CRSFBufferLength++] = (uint8_t) ((val >> 24 )  & 0xFF) ;
    CRSFBuffer[CRSFBufferLength++] = (uint8_t) ((val >> 16 )  & 0xFF) ;
    CRSFBuffer[CRSFBufferLength++] = (uint8_t) ((val >> 8 )  & 0xFF) ;
    CRSFBuffer[CRSFBufferLength++] = (uint8_t) (val & 0xFF);
}


void fillFrameBattery(uint8_t idx){
    CRSFBufferLength = 0;
    fillBufferU8(CRSF_ADDRESS_CRSF_RECEIVER) ;
    fillBufferU8(CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE + 2); // + 2 because we add type and crc byte 
    fillBufferU8(CRSF_FRAMETYPE_BATTERY_SENSOR) ;
    if ( fields[MVOLT].value > 0 ) {
        fillBufferU16( (uint16_t) int_round(fields[MVOLT].value , 100)) ; // convert from mV to 0.1V
    } else  fillBufferU16(0) ;
    if ( fields[CURRENT].value > 0 ) {
        fillBufferU16((uint16_t) int_round(fields[CURRENT].value, 100)) ; // convert from mA to 0.1A
    } else fillBufferU16(0);
    if ( fields[CAPACITY].value > 0 ) {
        fillBufferU24( (uint32_t) fields[CAPACITY].value);
    } else fillBufferU24(0);
    //if ( fields[REMAIN].value > 0 ) {
    //    fillBufferU8( (uint8_t) ((fields[REMAIN].value+5) /10 ));  // it is only a uint8_t; to use for a voltage we divide by 10
    //} else fillBufferU8(0);
    fillBufferU8(0); // in this version, field "remain" is not used.  
    //voltageFrame.mVolt = 0X0A01;   // !!!!!!!!!!!!!! to remove
    //voltageFrame.crc = crsf_crc.calc( ((uint8_t *) &voltageFrame) + 2 , CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE + 1)  ; // CRC skip 2 bytes( addr of message and frame size); length include type + 6 for payload  
    fillBufferU8(crsf_crc_out.calc( &CRSFBuffer[2] , CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE + 1) ) ; // CRC skip 2 bytes( addr of message and frame size); length include type + 6 for payload  
    
    fields[MVOLT].available = false ;
    crsfFrameNextMillis[idx] = millisRp() + VOLTAGE_FRAME_INTERVAL;
    if ( debugTlm == 'Y' ){
        printf("Bat: ");
        for (uint8_t i = 0; i< CRSFBufferLength ; i++) printf( " %02X ", CRSFBuffer[i]);
        printf("\n");
    }
    dma_channel_set_read_addr (crsf_dma_chan, &CRSFBuffer[0], false);
    dma_channel_set_trans_count (crsf_dma_chan, CRSFBufferLength, true) ;    
}

void fillFrameVario(uint8_t idx){
    //if (! baro1.baroInstalled) return ; // skip when vario is not installed ; not required because data will not be available and so fucntion should not be called    
    CRSFBufferLength = 0;
    fillBufferU8( CRSF_ADDRESS_CRSF_RECEIVER );
    fillBufferU8( CRSF_FRAME_VARIO_PAYLOAD_SIZE + 2 ); // + 2 because we add type and crc byte 
    fillBufferU8( CRSF_FRAMETYPE_VARIO ) ;
    fillBufferI16( (int16_t) fields[VSPEED].value ) ;  // cm/sec
    //printf("vspeed=%d\n", ((int16_t) fields[VSPEED].value) );
    fillBufferU8( crsf_crc_out.calc( &CRSFBuffer[2] , CRSF_FRAME_VARIO_PAYLOAD_SIZE + 1) ) ; // CRC skip 2 bytes( addr of message and frame size); length include type + 6 for payload  
    fields[VSPEED].available = false ;
    crsfFrameNextMillis[idx] = millisRp() + VARIO_FRAME_INTERVAL;
    if ( debugTlm == 'Y' ){
        printf("Vario: ");
        for (uint8_t i = 0; i< CRSFBufferLength ; i++) printf( " %02X ", CRSFBuffer[i]);
        printf("\n");
    }
    dma_channel_set_read_addr (crsf_dma_chan, &CRSFBuffer[0], false);
    dma_channel_set_trans_count (crsf_dma_chan, CRSFBufferLength, true) ;    
}

void fillFrameBaroAltitude(uint8_t idx){
    //if (! baro1.baroInstalled) return ; // skip when vario is not installed ; not required because data will not be available and so fucntion should not be called    
    CRSFBufferLength = 0;
    fillBufferU8( CRSF_ADDRESS_CRSF_RECEIVER );
    fillBufferU8( CRSF_FRAME_BARO_ALTITUDE_PAYLOAD_SIZE + 2 ); // + 2 because we add type and crc byte 
    fillBufferU8( CRSF_FRAMETYPE_BARO_ALTITUDE ) ;
    fillBufferI16( (int16_t) ( (fields[RELATIVEALT].value / 10 ) + 10000) ) ;  // from cm to dm and a 10000 dm offset
    //printf("alt=%f\n", (float) ( (int16_t) ( (fields[RELATIVEALT].value / 10 ) + 10000) )  );
    // in theory, when Alt in dm should be more than about 32000-10000, then value should be negative and in m instead of in dm (and no offset)
    // So when alt is more than 2000m; we can discard this situation
    fillBufferU8( crsf_crc_out.calc( &CRSFBuffer[2] , CRSF_FRAME_BARO_ALTITUDE_PAYLOAD_SIZE + 1) ) ; // CRC skip 2 bytes( addr of message and frame size); length include type + 6 for payload  
    fields[RELATIVEALT].available = false ;
    crsfFrameNextMillis[idx] = millisRp() + BARO_ALTITUDE_FRAME_INTERVAL;
    if ( debugTlm == 'Y' ){
        printf("Alt: ");
        for (uint8_t i = 0; i< CRSFBufferLength ; i++) printf( " %02X ", CRSFBuffer[i]);
        printf("\n");
    }
    dma_channel_set_read_addr (crsf_dma_chan, &CRSFBuffer[0], false);
    dma_channel_set_trans_count (crsf_dma_chan, CRSFBufferLength, true) ;    
}

void fillFrameAttitude(uint8_t idx){
    if (mpu.mpuInstalled) {
        CRSFBufferLength = 0;
        fillBufferU8( CRSF_ADDRESS_CRSF_RECEIVER );  
        fillBufferU8( CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE + 2 ); // + 2 because we add type and crc byte 
        fillBufferU8( CRSF_FRAMETYPE_ATTITUDE );
        //if ( fields[PITCH].available ) {
            fillBufferI16( (int16_t) (fields[PITCH].value * 175)) ; //pitch  (must be in 1/1000 of deci rad )
        //} else {
        //    fillBufferI16( (int16_t) 0);
        //}
        //if ( fields[ROLL].available ) {
            fillBufferI16( (int16_t) (fields[ROLL].value * 175)) ; //roll  
        //} else {
        //    fillBufferI16( (int16_t) 0);
        //}
        //if ( fields[RPM].available  ) {
        //    fillBufferI16( (int16_t) (fields[RPM].value  )); //= yaw : int16 allows values from -32000 up to +32000; apply SCALE4 if needed
        //} else {
            fillBufferI16( (int16_t) 0);
        //}
        fillBufferU8( crsf_crc_out.calc( &CRSFBuffer[2] , CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE+ 1))  ; // CRC skip 2 bytes( addr of message and frame size); length include type + 6 for payload  
        //fields[RPM].available = false ;
        //fields[PITCH].available = false ;
        //fields[ROLL].available = false ;
        crsfFrameNextMillis[idx] = millisRp() + ATTITUDE_FRAME_INTERVAL;
            //printf("Attitude: ");
            //for (uint8_t i = 0; i< CRSFBufferLength ; i++) printf( " %02X ", CRSFBuffer[i]);
            //printf("\n");
            //printf("p r= %d %d\n", (int16_t) (fields[PITCH].value) , (int16_t) (fields[ROLL].value));
        dma_channel_set_read_addr (crsf_dma_chan, &CRSFBuffer[0], false);
        dma_channel_set_trans_count (crsf_dma_chan, CRSFBufferLength, true) ;
    }
}

void fillFrameGps(uint8_t idx){
    CRSFBufferLength = 0;
    fillBufferU8( CRSF_ADDRESS_CRSF_RECEIVER );
    fillBufferU8( CRSF_FRAME_GPS_PAYLOAD_SIZE + 2 ); // + 2 because we add type and crc byte 
    fillBufferU8( CRSF_FRAMETYPE_GPS );
    //if (gps.gpsInstalled) {
        fillBufferI32( fields[LATITUDE].value ); // in degree with 7 decimals
        fillBufferI32( fields[LONGITUDE].value );    // in degree with 7 decimals // (degree / 10`000`000 )
        fillBufferU16( fields[GROUNDSPEED].value * 0.36);  // convert from cm/sec to ( km/h / 10 )
        if ( fields[HEADING].value < 0) {
            fillBufferU16( (uint16_t) (36000 - fields[HEADING].value) );      //( degree / 100  ; Ublox conversion from 5 to 2 decimals is done in GPS.cpp
        } else {
            fillBufferU16( (uint16_t) fields[HEADING].value );      //( degree / 100  ; Ublox conversion from 5 to 2 decimals is done in GPS.cpp
        }
        //if ( baro1.baroInstalled ){ // when a vario exist, priority for altitude is given to baro
        //    fillBufferU16( (uint16_t) (1000 + (fields[RELATIVEALT].value / 100) )) ;     
        //} else {
        //    fillBufferU16( (uint16_t) (1000 + fields[ALTITUDE].value / 100 )) ;     //( from mm to m and an offset of 1000 m )
        //}
        fillBufferU16( (uint16_t) (1000 + fields[ALTITUDE].value / 100 )) ;     //( converted to cm in gps.cpp and here from cm to m and an offset of 1000 m )
        fillBufferU8( (uint8_t) fields[NUMSAT].value );       //( counter including +100 when 3D fix )
        fields[NUMSAT].available = false;
    //} else {
    //    fillBufferI32( 0u );    // lat
    //    fillBufferI32( 0u );    // long
    //    fillBufferU16( (uint16_t) 0 );  // speed
    //    fillBufferU16( (uint16_t) 0 );      // ground course
    //    if ( baro1.baroInstalled ){ // when a vario exist, priority for altitude is given to baro
    //        fillBufferU16( (uint16_t) (1000 + fields[RELATIVEALT].value / 100 )) ;     //( from cm to m And an offset of 1000 )
    //    } else {
    //        fillBufferU16( (uint16_t) 1000 ) ;     //( m ; it is just the offset)
    //    }
    //    fillBufferU8( (uint8_t) 0 );       //( counter for numsat)
    //}
    fillBufferU8( crsf_crc_out.calc( &CRSFBuffer[2] , CRSF_FRAME_GPS_PAYLOAD_SIZE + 1) )  ; // CRC skip 2 bytes( addr of message and frame size); length include type + 6 for payload  

    crsfFrameNextMillis[idx] = millisRp() + GPS_FRAME_INTERVAL;
    //printf("GPS height:%" PRIu32 "\n",gps.GPS_altitude);
    //CRSFBufferLength = sizeof(gpsFrame);
    //
    //memcpy(&CRSFBuffer[0] , &gpsFrame , CRSFBufferLength);
    if ( debugTlm == 'Y' ){
        printf("Gps: ");
        for (uint8_t i = 0; i< CRSFBufferLength ; i++) printf( " %02X ", CRSFBuffer[i]);
        printf("\n");
    }    
    dma_channel_set_read_addr (crsf_dma_chan, &CRSFBuffer[0], false);
    dma_channel_set_trans_count (crsf_dma_chan, CRSFBufferLength, true) ;    
}

void fillOneFrame(uint8_t idx){
    //printf("frame: %X\n", idx);
    switch (idx) {
        case  CRSF_FRAMEIDX_BATTERY_SENSOR : 
            fillFrameBattery(idx);
            break ;
        case CRSF_FRAMEIDX_VARIO :
            fillFrameVario(idx);
            break ;
        case CRSF_FRAMEIDX_ATTITUDE :
            fillFrameAttitude(idx);
            break;
        case CRSF_FRAMEIDX_GPS :
            fillFrameGps(idx);
            break;
        case CRSF_FRAMEIDX_BARO_ALTITUDE :
            fillFrameBaroAltitude(idx);
            break ;
                           
    } // end switch
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
