#include "pico/stdlib.h"
#include <stdlib.h>
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "uart_frsky_hub_tx.pio.h"
#include "frsky_hub.h"
#include "config.h"
#include "pico/util/queue.h"
#include "MS5611.h"
#include "SPL06.h"
#include "tools.h"
#include "stdio.h"
#include <string.h> // memcpy
#include "param.h"
#include <inttypes.h>
#include "mpu.h"

#define DEBUGHUBPROTOCOL

extern field fields[];  // list of all telemetry fields and parameters used by Sport

extern GPS gps ;

extern CONFIG config;
//extern MPU mpu;

uint8_t frskyHubTxBuffer[70]; // buffer that contains the frame to be sent (via dma)
uint8_t frskyHubTxBufferLength; // number of char in the buffer


PIO frskyHubPio = pio0; // we use pio 0; DMA is hardcoded to use it
uint frskyHubSmTx = 0;  // we use the state machine 0 for Tx; DMA is harcoded to use it (DREQ) 

int frskyHub_dma_chan;
dma_channel_config frskHubDmaConfig;


void setupFrskyHub(){
    if (config.pinTlm == 255) return ; // Skip when no telemetry is foreseen 
    // setup the PIO for TX UART
    uint frskyHubOffsetTx = pio_add_program(frskyHubPio, &uart_frsky_hub_tx_program);
    uart_frsky_hub_tx_program_init(frskyHubPio, frskyHubSmTx, frskyHubOffsetTx, config.pinTlm, 9600, true) ; // true = inverted
    // Configure a channel to write the same word (32 bits) repeatedly to PIO0
    // SM0's TX FIFO, paced by the data request signal from that peripheral.
    frskyHub_dma_chan = dma_claim_unused_channel(true);
    frskHubDmaConfig = dma_channel_get_default_config(frskyHub_dma_chan);
    channel_config_set_read_increment(&frskHubDmaConfig, true);
    channel_config_set_write_increment(&frskHubDmaConfig, false);
    channel_config_set_dreq(&frskHubDmaConfig, DREQ_PIO0_TX0);
    channel_config_set_transfer_data_size(&frskHubDmaConfig, DMA_SIZE_8);
    dma_channel_configure(
        frskyHub_dma_chan,
        &frskHubDmaConfig,
        &pio0_hw->txf[0], // Write address (only need to set this once)
        &frskyHubTxBuffer[0],   // we use always the same buffer             
        0 , // do not yet provide the number of bytes (DMA cycles)
        false             // Don't start yet
    );
    // do not set interrupt on DMA. The main loop will check if DMA is busy before sending    
}

#define HUB_FRAME1_INTERVAL_MS 200  //  other data
#define HUB_FRAME2_INTERVAL_MS 1000 // GPS long,...
#define HUB_FRAME3_INTERVAL_MS 5000 // GPS date and time
  
void handleFrskyHubFrames(){
    //static uint8_t frskyHub_last_frame_idx = 0 ;
    if (config.pinTlm == 255) return ; // skip when Tlm is not foreseen
    if ( dma_channel_is_busy(frskyHub_dma_chan) )return ; // skip if the DMA is still sending data
    uint32_t _millis = millisRp();
    static uint32_t lastMsFrame1=0;
    static uint32_t lastMsFrame2=0;
    static uint32_t lastMsFrame3=0;
  
    static uint16_t lastFrameLength ;
    static uint32_t temp ;
    temp = millisRp() ;
    if (gps.gpsInstalled){
        if ( (temp - lastMsFrame3) > HUB_FRAME3_INTERVAL_MS ){
            lastMsFrame3 = temp;
            sendFrskyHubFrame3();
            return;
        }
        if ((temp - lastMsFrame2) > HUB_FRAME2_INTERVAL_MS){
            lastMsFrame2 = temp;
            sendFrskyHubFrame2();
            return;
        }
    }    
    if ((temp - lastMsFrame1) > HUB_FRAME1_INTERVAL_MS){    
        lastMsFrame1 = temp;
        sendFrskyHubFrame1();
        return;
    }
}

//list of all telemetry fields supported by Hub protocol (defined by Frsky) 
#define FRSKY_USERDATA_GPS_ALT_B    0x01 // Altitude m
#define FRSKY_USERDATA_TEMP1        0x02
#define FRSKY_USERDATA_RPM          0x03
#define FRSKY_USERDATA_FUEL         0x04
#define FRSKY_USERDATA_TEMP2        0x05
#define FRSKY_USERDATA_CELL_VOLT    0x06
#define FRSKY_USERDATA_GPS_ALT_A    0x09 // Altitude in centimeter
#define FRSKY_USERDATA_BARO_ALT_B   0x10
#define FRSKY_USERDATA_GPS_SPEED_B  0x11 // Speed knots
#define FRSKY_USERDATA_GPS_LONG_B   0x12 // Longitude (DDMM)
#define FRSKY_USERDATA_GPS_LAT_B    0x13 // Latitude (DDMM)
#define FRSKY_USERDATA_GPS_CURSE_B  0x14 // Course degrees
#define FRSKY_USERDATA_GPS_DM       0x15
#define FRSKY_USERDATA_GPS_YEAR     0x16
#define FRSKY_USERDATA_GPS_HM       0x17
#define FRSKY_USERDATA_GPS_SEC      0x18
#define FRSKY_USERDATA_GPS_SPEED_A  0x19 // Speed 2 decimals of knots
#define FRSKY_USERDATA_GPS_LONG_A   0x1A // Longitude (.MMMM)
#define FRSKY_USERDATA_GPS_LAT_A    0x1B // Latitude (.MMMM)
#define FRSKY_USERDATA_GPS_CURSE_A  0x1C // Course 2 decimals of degrees
#define FRSKY_USERDATA_BARO_ALT_A   0x21
#define FRSKY_USERDATA_GPS_LONG_EW  0x22 //(uint16_t)(flon < 0 ? 'W' : 'E')
#define FRSKY_USERDATA_GPS_LAT_EW   0x23 //(uint16_t)(lat < 0 ? 'S' : 'N')
#define FRSKY_USERDATA_ACC_X        0x24
#define FRSKY_USERDATA_ACC_Y        0x25
#define FRSKY_USERDATA_ACC_Z        0x26
#define FRSKY_USERDATA_CURRENT      0x28
#define FRSKY_USERDATA_VERT_SPEED   0x30 // open9x Vario Mode Only
#define FRSKY_USERDATA_ALT_MIN      0x31 // open9x Vario Mode Only
#define FRSKY_USERDATA_ALT_MAX      0x32 // open9x Vario Mode Only
#define FRSKY_USERDATA_RPM_MAX      0x33 // open9x Vario Mode Only
#define FRSKY_USERDATA_T1_MAX       0x34 // open9x Vario Mode Only
#define FRSKY_USERDATA_T2_MAX       0x35 // open9x Vario Mode Only
#define FRSKY_USERDATA_GPS_SPEED_MAX  0x36 // open9x Vario Mode Only
#define FRSKY_USERDATA_GPS_DIS_MAX  0x37 // open9x Vario Mode Only
#define FRSKY_USERDATA_VFAS_NEW     0x39 // Use this field in order to display the value on VFAS on Tx, Take care that for a voltage, the value must be in 1/10 of Volt and not in mVolt
#define FRSKY_USERDATA_VOLTAGE_B    0x3A // do not use this code to transmit a voltage. It requires a special formatting that is not implemented. Use VFAS_NEW instead
#define FRSKY_USERDATA_VOLTAGE_A    0x3B // do not use this code to transmit a voltage. It requires a special formatting that is not implemented. Use VFAS_NEW instead
#define FRSKY_USERDATA_GPS_DIST     0x3C
#define FRSKY_USERDATA_FUELPERCENT  0x3D
// Endof list of all telemetry fields supported by Hub protocol (defined by Frsky) 

void sendFrskyHubFrame1(){  // not gps data
  frskyHubTxBufferLength = 0 ; // reset of number of data to send
// Altitude
    if (fields[RELATIVEALT].onceAvailable ){
        uint16_t Centimeter =  uint16_t(abs(fields[RELATIVEALT].value)%100);
        int32_t Meter;
        if (fields[RELATIVEALT].value >0){
            Meter = (fields[RELATIVEALT].value - Centimeter);
        } else{
            Meter = -1*(abs(fields[RELATIVEALT].value) + Centimeter);
        }
        Meter=Meter/100;
        sendHubValue(FRSKY_USERDATA_BARO_ALT_B, (int16_t)Meter);
        sendHubValue(FRSKY_USERDATA_BARO_ALT_A, Centimeter);
    }
// VSpeed
    if (fields[VSPEED].onceAvailable ){ 
        sendHubValue( FRSKY_USERDATA_VERT_SPEED , (int16_t) fields[VSPEED].value);
    }
// vfas
    if (fields[MVOLT].onceAvailable ){
        sendHubValue( FRSKY_USERDATA_VFAS_NEW ,  (int16_t) (fields[MVOLT].value / 100) ) ; // convert mvolt in 1/10 of volt; in openTx 2.1.x, it is possible to get 1 more decimal using [VFAS_SOURCE - VOLT_1 ].value/10.)+2000);  
    }
// current
    if (fields[CURRENT].onceAvailable ){
        sendHubValue( FRSKY_USERDATA_CURRENT ,  (int16_t) ( fields[CURRENT].value / 100 ) ) ;
    }
// RPM
    if (fields[RPM].onceAvailable ){
        sendHubValue( FRSKY_USERDATA_RPM ,  (int16_t) ( fields[RPM].value ) ) ;
    }
// T1
    if (fields[TEMP1].onceAvailable ){
        sendHubValue( FRSKY_USERDATA_TEMP1 ,  (int16_t) ( fields[TEMP1].value ) ) ;
    }
// T2   
    if (fields[TEMP2].onceAvailable ){
        sendHubValue( FRSKY_USERDATA_TEMP2 ,  (int16_t) ( fields[TEMP2].value ) ) ;
    }
// airspeed                                        // not implemented in Hub protocol; 
// acc X Y Z
    if (fields[ACC_X].onceAvailable ){
        sendHubValue( FRSKY_USERDATA_ACC_X ,  (int16_t) ( fields[ACC_X].value ) ) ;
    }
    if (fields[ACC_Y].onceAvailable ){
        sendHubValue( FRSKY_USERDATA_ACC_Y ,  (int16_t) ( fields[ACC_Y].value ) ) ;
    }
    if (fields[ACC_Z].onceAvailable ){
        sendHubValue( FRSKY_USERDATA_ACC_Z ,  (int16_t) ( fields[ACC_Z].value ) ) ;
    }
// transmit
    if( frskyHubTxBufferLength > 0 ) {
        sendHubByte(0x5E) ; // End of Frame 1!
        dma_channel_set_read_addr (frskyHub_dma_chan, &frskyHubTxBuffer[0], false);
        dma_channel_set_trans_count (frskyHub_dma_chan, frskyHubTxBufferLength, true) ;    
    }  
#ifdef DEBUGHUBPROTOCOL
    printf("Frame 1 to send:");
    for (int cntPrint = 0 ; cntPrint < frskyHubTxBufferLength ; cntPrint++) {
        printf(" %X", frskyHubTxBuffer[cntPrint]);
    }
    printf("\n"); 
#endif  
}  // end send frame 1

void sendFrskyHubFrame2(){  // gps data
    uint32_t absLongLat ;
    uint32_t decimalPartOfDegree ;
    uint32_t minWith7Decimals  ;
    frskyHubTxBufferLength = 0 ; // reset of number of data to send
    // here we fill the buffer with all GPS data
    // GPS_lon             // longitude in degree with 7 decimals, (neg for S)
    // GPS_lat             // latitude   in degree with 7 decimals, (neg for ?)
    // GPS_altitude;       // altitude in mm
    // GPS_speed_3d;       // speed in cm/s
    // GPS_speed;          // speed in cm/s
    // GPS_ground_course ; // degrees with 5 decimals
    if ( fields[LATITUDE].onceAvailable ) {
        absLongLat = abs(fields[LATITUDE].value) ;
        decimalPartOfDegree = (absLongLat % 10000000 );
        minWith7Decimals = decimalPartOfDegree * 60 ;
        sendHubValue(FRSKY_USERDATA_GPS_LAT_B , (uint16_t) (((absLongLat / 10000000L) * 100 ) +  (minWith7Decimals / 10000000L )) ) ; // Latitude (DDMM)
        sendHubValue(FRSKY_USERDATA_GPS_LAT_A , (uint16_t) (( minWith7Decimals % 10000000L) / 1000 ) ) ;                              // Latitude (.MMMM)
        sendHubValue(FRSKY_USERDATA_GPS_LAT_EW , (uint16_t)(fields[LATITUDE].value < 0 ? 'S' : 'N')) ;
    } 
    if ( fields[LONGITUDE].onceAvailable ) {
        absLongLat = abs(fields[LONGITUDE].value) ;
        decimalPartOfDegree = (absLongLat % 10000000 );
        minWith7Decimals = decimalPartOfDegree * 60 ;
        sendHubValue(FRSKY_USERDATA_GPS_LONG_B , (uint16_t) (((absLongLat / 10000000L) * 100 ) +  (minWith7Decimals / 10000000L )) ) ; // Longitude (DDMM)
        sendHubValue(FRSKY_USERDATA_GPS_LONG_A , (uint16_t) (( minWith7Decimals % 10000000L) / 1000 ) ) ;                              // Longitude (.MMMM)
        sendHubValue(FRSKY_USERDATA_GPS_LONG_EW , (uint16_t)(fields[LONGITUDE].value < 0 ? 'W' : 'E')) ;
    }
    if ( fields[ALTITUDE].onceAvailable ) {
        sendHubValue(FRSKY_USERDATA_GPS_ALT_B ,  (int16_t) (fields[ALTITUDE].value / 1000) );                                                      // Altitude m
        sendHubValue(FRSKY_USERDATA_GPS_ALT_A , (uint16_t) ( ( (abs(fields[ALTITUDE].value) % 1000 ) / 10 ) ) )  ;                                    // Altitude centimeter
    }
    if ( fields[GROUNDSPEED].onceAvailable ) {
        uint32_t GPSSpeedKnot = fields[GROUNDSPEED].value * 1944L ;
        sendHubValue(FRSKY_USERDATA_GPS_SPEED_B , (uint16_t) ( GPSSpeedKnot / 100000) ) ;                                              // Speed knots
        sendHubValue(FRSKY_USERDATA_GPS_SPEED_A , (uint16_t) ( (GPSSpeedKnot % 100000 ) /1000) ) ;                                     // Speed 2 decimals of knots
    }
    if ( fields[HEADING].onceAvailable )  {
        sendHubValue(FRSKY_USERDATA_GPS_CURSE_B , (uint16_t) ( fields[HEADING].value / 100000 ) ) ;                                        // Course degrees
        sendHubValue(FRSKY_USERDATA_GPS_CURSE_A , (uint16_t) ( (fields[HEADING].value % 100000) / 1000 ) ) ;                               // Course 2 decimals of degrees
    }
// transmit
    if( frskyHubTxBufferLength > 0 ) {
        sendHubByte(0x5E) ; // End of Frame 1!
        dma_channel_set_read_addr (frskyHub_dma_chan, &frskyHubTxBuffer[0], false);
        dma_channel_set_trans_count (frskyHub_dma_chan, frskyHubTxBufferLength, true) ;    
    }  
#ifdef DEBUGHUBPROTOCOL
    printf("Frame 2 to send:");
    for (int cntPrint = 0 ; cntPrint < frskyHubTxBufferLength ; cntPrint++) {
        printf(" %X", frskyHubTxBuffer[cntPrint]);
    }
    printf("\n"); 
#endif  
}

void sendFrskyHubFrame3(){  // gps date & time
    frskyHubTxBufferLength = 0 ; // reset of number of data to send
    uint16_t value ;
    if (fields[GPS_DATE].onceAvailable )  {
        value= (fields[GPS_DATE].value >> 16) & 0XFF;// extract MM from original date format  YYMMDDXX
        value= value<<8;
        value+= (fields[GPS_DATE].value >> 8) & 0XFF;// extract DD from original date format YYMMDDXX
        sendHubValue(FRSKY_USERDATA_GPS_DM , value) ; // DATE (DDMM)
        value= (fields[GPS_DATE].value >> 24) & 0XFF;// extract YY from original date format  YYMMDDXX
        sendHubValue(FRSKY_USERDATA_GPS_YEAR , value) ; // YEAR (CC00)
    }
    if (fields[GPS_TIME].onceAvailable )  {
        value =(fields[GPS_TIME].value >> 16) & 0XFF;// extract MM from original time format  HHMMSS00;
        value = value <<8;
        value +=(fields[GPS_TIME].value >> 24) & 0XFF;// extract HH from original time format  HHMMSS00;
        sendHubValue(FRSKY_USERDATA_GPS_HM , value) ; // HOURMIN (hhmm)
        value=(fields[GPS_TIME].value >> 8) & 0XFF;// extract SS00 from original time format  HHMMSS00;
        sendHubValue(FRSKY_USERDATA_GPS_SEC , value) ; // seconds (ss00)
    }    
// transmit
    if( frskyHubTxBufferLength > 0 ) {
        sendHubByte(0x5E) ; // End of Frame 1!
        dma_channel_set_read_addr (frskyHub_dma_chan, &frskyHubTxBuffer[0], false);
        dma_channel_set_trans_count (frskyHub_dma_chan, frskyHubTxBufferLength, true) ;    
    }  
#ifdef DEBUGHUBPROTOCOL
    printf("Frame 2 to send:");
    for (int cntPrint = 0 ; cntPrint < frskyHubTxBufferLength ; cntPrint++) {
        printf(" %X", frskyHubTxBuffer[cntPrint]);
    }
    printf("\n"); 
#endif          
}



/*
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
    fillBufferU8( crsf_crc_out.calc( &CRSFBuffer[2] , CRSF_FRAME_GPS_PAYLOAD_SIZE + 1) )  ; // CRC skip 2 bytes( addr of message and frame size); length include type + 6 for payload  

    crsfFrameNextMillis[idx] = millisRp() + GPS_FRAME_INTERVAL;
    if ( debugTlm == 'Y' ){
        printf("Gps: ");
        for (uint8_t i = 0; i< CRSFBufferLength ; i++) printf( " %02X ", CRSFBuffer[i]);
        printf("\n");
    }    
    dma_channel_set_read_addr (crsf_dma_chan, &CRSFBuffer[0], false);
    dma_channel_set_trans_count (crsf_dma_chan, CRSFBufferLength, true) ;    
}

*/


// ******************************************************** //
// sendHubValue => send a value as frsky sensor hub data       //
// ******************************************************** //
void sendHubValue(uint8_t ID, uint16_t Value) {
  uint8_t tmp1 = Value & 0x00ff;
  uint8_t tmp2 = (Value & 0xff00)>>8;
  sendHubByte(0x5E) ;
  sendHubByte(ID);

  if ( (tmp1 == 0x5E) || (tmp1 == 0x5D) ){ 
	      tmp1 ^= 0x60 ;
        sendHubByte(0x5D);
  }
  sendHubByte(tmp1);  
  if ( (tmp2 == 0x5E) || (tmp2 == 0x5D) ){ 
	      tmp2 ^= 0x60 ;
        sendHubByte(0x5D);
  }
  sendHubByte(tmp2);
}

//***************************************************
// Put a byte in the buffer
//***************************************************
void sendHubByte( uint8_t byte )
{	
	frskyHubTxBuffer[frskyHubTxBufferLength] = byte ;
	frskyHubTxBufferLength ++ ;	
}

