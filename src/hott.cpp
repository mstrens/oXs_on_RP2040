// File for Hott
/*
In Hott protocol, the Rx send a polling to each of the possible sensors (there are 5 types of sensors).
Each sensor is in fact polled twice: one asking to reply in a text format and another in a binary format.
This version of oXs replies only to the request for a binary format sent to a GAM (general air module) or a GPS
It seems that for binary format, the polling is every 200 msec. 
Still this strange because the reply of only one sensor takes already about 100 msec (45 bytes * 2msec/byte).
When the sensor identifies that it must reply to a polling, it has to wait 5 msec before sending the first byte
Afterward it can send all the bytes (45 in total in binary GAM format) keeping 1 msec between each byte.
The format of the message is given in hott.h 
It is possible to send some info to reverse some fields on the display and to activate some alarms (but this version does not support it)
*/
#include <stdio.h>
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "uart_hott_tx_rx.pio.h"
#include "MS5611.h"
#include "SPL06.h"
#include "hott.h"
#include "tools.h"
#include "gps.h"
#include "param.h"
#include <inttypes.h> // used by PRIu32
#include "sport.h"
#include "string.h" // used for memset

#ifdef DEBUG
// ************************* here Several parameters to help debugging
  //#define DEBUGHOTT
#endif

extern field fields[];  // list of all telemetry fields and parameters used by Sport
//extern MS5611 baro1;
//extern SPL06 baro2;

extern GPS gps;
extern CONFIG config;
extern uint8_t debugTlm;

// one pio and 2 state machines are used to manage the hott bus in halfduplex
// one state machine (sm) handle the TX and the second the RX
// to receive data, the sm is initialised and use an IRQ handler when rx fifo is not empty
//    in irq, this byte is store in a Rx queue
//    This queue is read in main loop
//    When a combination of bytes are received that require a reply then we stop the sm that receive (it means hott made a polling)
//    We wait 5 msec and fill a buffer with the data
//    We set up a dma to transfer the data to the TX fifo of the Tx state machine
//    The pio that send each byte wait 1 msec between 2 bytes
//    We also set up a timestamp to stop after some msec the Tx state machine and start again the Rx one   

queue_t hottRxQueue ;  // get the request from the receiver
// one pio with 2 state machine is used to manage the inverted hal duplex uart for Sport
PIO hottPio = pio0;
uint hottSmTx = 0; // to send the telemetry to sport
uint hottSmRx = 1; // to get the request from sport
uint hottOffsetTx ; 
uint hottOffsetRx ; 

// dma channel is used to send hott telemetry without blocking
int hott_dma_chan;
dma_channel_config hottDmaConfig;

uint8_t hottTxBuffer[50];

enum HOTTSTATES {
    RECEIVING,
    WAIT_FOR_SENDING,
    SENDING,
    WAIT_END_OF_SENDING
};

uint32_t hottStartWaiting = 0; // timestamp (usec) when we start waiting 
HOTTSTATES hottState;

/*
extern OXS_MS5611 oXs_MS5611 ;
extern OXS_VOLTAGE oXs_Voltage ; 
extern OXS_CURRENT oXs_Current ;
extern OXS_4525 oXs_4525 ;
extern OXS_SDP3X oXs_sdp3x;


extern unsigned long microsRp( void ) ;
extern unsigned long millisRp( void ) ;
extern void delay(unsigned long ms) ;
*/

//extern int32_t GPS_lon;               // longitude in degree with 7 decimals, (neg for S)
//extern int32_t GPS_lat;               // latitude   in degree with 7 decimals, (neg for ?)
//extern bool    GPS_latAvailable;

 //extern int32_t GPS_altitude;              // altitude in mm
 //extern uint16_t GPS_speed_3d;              // speed in cm/s
 //extern uint16_t GPS_speed_2d;                 // speed in cm/s
//extern uint32_t GPS_ground_course ;     // degrees with 5 decimals
 //extern uint8_t GPS_numSat ;
 //extern uint16_t GPS_hdop;           // Compute GPS quality signal
 //extern bool GPS_fix ; // true if gps data are available.


 //extern uint8_t GPS_fix_type ;
 //extern int16_t GPS_distance ;
 //extern int16_t GPS_bearing ; 


//volatile uint8_t flagUpdateHottBuffer ; // When a polling for oXs is received, it says that main loop must update the buffer (within the 4mes)  
 

// Transmit buffer
static union {              // union is a easy way to access the data in several ways (name of field + index of byte)
    HOTT_GAM_MSG gamMsg ;   // structured general air module
    HOTT_GPS_MSG gpsMsg ;
    uint8_t txBuffer[TXHOTTDATA_BUFFERSIZE] ;
}  __attribute__((packed)) TxHottData ;

void setupHott() {                                                    
// configure the queue to get the data from hott in the irq handle
    queue_init (&hottRxQueue, sizeof(uint8_t), 250);

// set up the DMA but do not yet start it to send data to Hott
// Configure a channel to write the same byte (8 bits) repeatedly to PIO0
// SM0's TX FIFO, placed by the data request signal from that peripheral.
    hott_dma_chan = dma_claim_unused_channel(true);
    hottDmaConfig = dma_channel_get_default_config(hott_dma_chan);
    channel_config_set_read_increment(&hottDmaConfig, true);
    channel_config_set_write_increment(&hottDmaConfig, false);
    channel_config_set_dreq(&hottDmaConfig, DREQ_PIO0_TX0);  // use state machine 0 
    channel_config_set_transfer_data_size(&hottDmaConfig, DMA_SIZE_8);
    dma_channel_configure(
        hott_dma_chan,
        &hottDmaConfig,
        &pio0_hw->txf[0], // Write address (only need to set this once)
        &hottTxBuffer[0],   // we use always the same buffer             
        0 , // do not yet provide the number of bytes (DMA cycles)
        false             // Don't start yet
    );
// Set up the state machine for transmit but do not yet start it (it starts only when a request from receiver is received)
    hottOffsetTx = pio_add_program(hottPio, &hott_uart_tx_program);
    hott_uart_tx_program_init(hottPio, hottSmTx, hottOffsetTx, config.pinTlm, 19200 , false); // we use the same pin and baud rate for tx and rx, false means thet UART is not inverted 

// set an irq on pio to handle a received byte
    irq_set_exclusive_handler( PIO0_IRQ_0 , hottPioRxHandlerIrq) ;
    irq_set_enabled (PIO0_IRQ_0 , true) ;

// Set up the state machine we're going to use to receive them.
    hottOffsetRx = pio_add_program(hottPio, &hott_uart_rx_program);
    hott_uart_rx_program_init(hottPio, hottSmRx, hottOffsetRx, config.pinTlm, 19200 , false);
    hottState = RECEIVING; // consider we are receiving  
}


void hottPioRxHandlerIrq(){    // when a byte is received on the tlm pin, read the pio hott fifo and push the data to a queue (to be processed in the main loop)
  // clear the irq flag
  irq_clear (PIO0_IRQ_0 );
  while (  ! pio_sm_is_rx_fifo_empty (hottPio ,hottSmRx)){ // when some data have been received
     uint8_t c = pio_sm_get (hottPio , hottSmRx) >> 24;         // read the data
     //printf("%x", c);
     queue_try_add (&hottRxQueue, &c);          // push to the queue
    //hottRxMillis = millisRp();                    // save the timestamp.
  }
}

//#define DEBUG_HOTT_WITHOUT_RX
#ifdef DEBUG_HOTT_WITHOUT_RX
uint32_t lastHottRequest = 0;
#endif
void handleHottRxTx(void){   // main loop : restore receiving mode , wait for tlm request, prepare frame, start pio and dma to transmit it
    static uint8_t previous = 0;
    static uint8_t data;
    if (config.pinTlm == 255) return ; // skip when Tlm is not foreseen
    switch (hottState) {
        case RECEIVING :
            #ifdef DEBUG_HOTT_WITHOUT_RX  // simulate a RX sending a GAM polling every 200 msec 
            if ( (millisRp() - lastHottRequest) > 200)  { // to debug simulate a request once per 200msec
                hottState = WAIT_FOR_SENDING;
                hottStartWaiting = microsRp();
                data = HOTT_TELEMETRY_GAM_SENSOR_ID;
                lastHottRequest = millisRp();
            }
            break;
            #endif
            if (! queue_is_empty(&hottRxQueue)) {
                queue_try_remove (&hottRxQueue,&data);
                //printf("%X ", data);
                if ( ( previous ==  HOTT_BINARY_MODE_REQUEST_ID) &&
                     ( (data == HOTT_TELEMETRY_GAM_SENSOR_ID) || (data == HOTT_TELEMETRY_GPS_SENSOR_ID) ) ){
                    hottState = WAIT_FOR_SENDING;
                    hottStartWaiting = microsRp(); 
                    //printf("r\n");
                }
                previous = data;
            }        
            break;
        case WAIT_FOR_SENDING :
            if ( ( microsRp() - hottStartWaiting) > HOTTV4_REPLY_DELAY){
                if (sendHottFrame(data) ) { // data must be GAM or GPS; return true when a frame is really being sent
                    hottState = SENDING;
                    hottStartWaiting = microsRp();
                    //printf("s\n");
                } else { 
                    hottState = RECEIVING;
                }   
            }
            break;         
        
        case SENDING :   // wait that dma have sent all data (but the last bytes are not yet sent)
            if ( ! dma_channel_is_busy( hott_dma_chan)	){
                hottState = WAIT_END_OF_SENDING;
                hottStartWaiting = microsRp();
            }
            break;         
        
        case WAIT_END_OF_SENDING :
            if ( ( microsRp() - hottStartWaiting) > ( HOTTV4_END_SENDING_DELAY) ){
                hott_uart_tx_program_stop(hottPio, hottSmTx, config.pinTlm );
                hott_uart_rx_program_restart(hottPio, hottSmRx, config.pinTlm, false );  // false = not inverted
                hottState = RECEIVING ;
                //printf("e\n");
            }
            break;     
    }
     
}

bool sendHottFrame(uint8_t data) { // return true when a frame is being sent
    bool sendingRequired = false;
    switch (data) {
        case HOTT_TELEMETRY_GAM_SENSOR_ID:       
            sendingRequired = fillHottGamFrame();
            break;
        case HOTT_TELEMETRY_GPS_SENSOR_ID:
            sendingRequired = fillHottGpsFrame();
            break;
    }
    if (sendingRequired) { // when buffer is filled, it can be sent
        hott_uart_rx_program_stop(hottPio, hottSmRx, config.pinTlm); // stop receiving
        hott_uart_tx_program_start(hottPio, hottSmTx, config.pinTlm, false); // prepare to transmit (do not invert)
        // start the DMA channel with the data to transmit
        dma_channel_set_read_addr (hott_dma_chan, &TxHottData.txBuffer[0], false);
        dma_channel_set_trans_count (hott_dma_chan, TXHOTTDATA_BUFFERSIZE, true) ;
    }
    return sendingRequired;    
}       

bool fillHottGamFrame(){
    memset(&TxHottData.txBuffer[0], 0 , TXHOTTDATA_BUFFERSIZE) ;
    TxHottData.gamMsg.start_byte    = 0x7c ;
    TxHottData.gamMsg.gam_sensor_id = 0x8d ; //GENRAL AIR MODULE = HOTT_TELEMETRY_GAM_SENSOR_ID
    TxHottData.gamMsg.sensor_id     = 0xd0 ;
    TxHottData.gamMsg.stop_byte     = 0x7D ;
// in general air module data to fill are:
    // TxHottData.gamMsg.cell[0] =  voltageData->mVoltCell[0] /20 ; // Volt Cell 1 (in 2 mV increments, 210 == 4.20 V)
    // TxHottData.gamMsg.cell[1] =  voltageData->mVoltCell[1] /20 ; // Volt Cell 2 (in 2 mV increments, 210 == 4.20 V)
    // TxHottData.gamMsg.cell[2] =  voltageData->mVoltCell[2] /20 ; // Volt Cell 3 (in 2 mV increments, 210 == 4.20 V)
    // TxHottData.gamMsg.cell[3] =  voltageData->mVoltCell[3] /20 ; // Volt Cell 4 (in 2 mV increments, 210 == 4.20 V)
    // TxHottData.gamMsg.cell[4] =  voltageData->mVoltCell[4] /20 ; // Volt Cell 5 (in 2 mV increments, 210 == 4.20 V)
    // TxHottData.gamMsg.cell[5] =  voltageData->mVoltCell[5] /20 ; // Volt Cell 6 (in 2 mV increments, 210 == 4.20 V)
    if( fields[MVOLT].available ) {
              TxHottData.gamMsg.Battery1 = int_round(fields[MVOLT].value , 100);   //battery 1 voltage  0.1V steps. 55 = 5.5V only pos. voltages
              #ifdef DEBUG_HOTT_WITHOUT_RX
              if (TxHottData.gamMsg.Battery1 == 0) TxHottData.gamMsg.Battery1 = 0XFF;
              #endif
    }
    if( fields[RESERVE1].available ) {  // = volt3
              TxHottData.gamMsg.Battery2 = int_round( fields[RESERVE1].value ,  100) ; }    //battery 2 voltage  0.1V steps. 55 = 5.5V only pos. voltages
    if ( ( fields[TEMP1].available) && (fields[TEMP1].value > -20) ) {
        TxHottData.gamMsg.temperature1 = fields[TEMP1].value + 20 ; // Hott applies an offset of 20. A value of 20 = 0°C
    } else {
        TxHottData.gamMsg.temperature1 = 20 ; // Hott applies an offset of 20. A value of 20 = 0°C    
    }
    if ( ( fields[TEMP2].available) && (fields[TEMP2].value > -20) ) {
        TxHottData.gamMsg.temperature2 = fields[TEMP2].value + 20 ; // Hott applies an offset of 20. A value of 20 = 0°C
    } else {
        TxHottData.gamMsg.temperature2 = 20 ; // Hott applies an offset of 20. A value of 20 = 0°C    
    }
    if( fields[RPM].available )
        {TxHottData.gamMsg.rpm = int_round(fields[RPM].value , 10);}    //#22 RPM in 10 RPM steps. 300 = 3000rpm
    if( fields[RELATIVEALT].available ) {
        TxHottData.gamMsg.altitude = ( int_round(fields[RELATIVEALT].value , 100) ) +500 ;//altitude in meters. offset of 500, 500 = 0m
    } else { TxHottData.gamMsg.altitude =  500 ; }                             //altitude in meters. offset of 500, 500 = 0m
    if( fields[VSPEED].available ) {
        TxHottData.gamMsg.climbrate_L = ( fields[VSPEED].value ) +30000 ;//climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
        //printf("v= %d\n", TxHottData.gamMsg.climbrate_L);
    } else { TxHottData.gamMsg.climbrate_L =  30000 ; }           //climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
    TxHottData.gamMsg.climbrate3s = 120 ;                     //#28 climb rate in m/3sec. Value of 120 = 0m/3sec
    if( fields[CURRENT].available) {
        TxHottData.gamMsg.current =  int_round(fields[CURRENT].value , 100) ; }              //current in 0.1A steps 100 == 10,0A
    if( fields[RESERVE2].available ){ //Volt4
        TxHottData.gamMsg.main_voltage = int_round(fields[RESERVE2].value , 100);          //Main power voltage using 0.1V steps 100 == 10,0V] / 100
    }
    if( fields[CAPACITY].available ){
         TxHottData.gamMsg.batt_cap = int_round(fields[CAPACITY].value , 10) ;   // used battery capacity in 10mAh steps
    }
    // TxHottData.gamMsg.speed =  airSpeedData->airSpeed.value  ;                  //  Km/h 
    // TxHottData.gamMsg.min_cell_volt =  voltageData->mVoltCellMin /20 ; // minimum cell voltage in 2mV steps. 124 = 2,48V
    // TxHottData.gamMsg.warning_beeps = warning_beeps_Hott();   // Transmitter warning message

// field of msg not implemented
//  byte climbrate3s;                     //#28 climb rate in m/3sec. Value of 120 = 0m/3sec
//  byte min_cell_volt_num;               //#38 number of the cell with the lowest voltage
//  uint16_t rpm2;                        //#39 LSB 2nd RPM in 10 RPM steps. 100 == 1000rpm
//  byte general_error_number;            //#41 General Error Number (Voice Error == 12) TODO: more documentation
//  byte pressure;                        //#42 High pressure up to 16bar. 0,1bar scale. 20 == 2.0bar

    TxHottData.txBuffer[TXHOTTDATA_BUFFERSIZE-1] = 0 ;
    for(uint8_t i = 0; i < (TXHOTTDATA_BUFFERSIZE-1); i++){  // one byte less because the last byte is the checksum
        TxHottData.txBuffer[TXHOTTDATA_BUFFERSIZE-1] += TxHottData.txBuffer[i];
        //printf("%d %X\n ", i , TxHottData.txBuffer[i] );
    }  // end for
    return true;
}

static uint8_t convertGpsFix[5] = {0x2d , 0x2d , 0x32 , 0x33 , 0x44 } ; 

bool fillHottGpsFrame(){
    if ( ! gps.gpsInstalled) return false ;              // exit if not installed
    memset(&TxHottData.txBuffer[0], 0 , TXHOTTDATA_BUFFERSIZE) ;
    TxHottData.gpsMsg.startByte    = 0x7c ;
    TxHottData.gpsMsg.sensorID     = HOTT_TELEMETRY_GPS_SENSOR_ID ; //0x8A
    TxHottData.gpsMsg.sensorTextID     = HOTTV4_GPS_SENSOR_TEXT_ID ; // 0xA0
    TxHottData.gpsMsg.endByte     = 0x7D ;
    uint16_t altitudeHott = 500 ;                   // Hott uses an offset of 500 (m)
    if ( (gps.GPS_fix_type == 3 ) || (gps.GPS_fix_type == 4 ) ) {
//                    if( GPS_latAvailable ) {             // test if data are available (GPS fix) ; if available, fill the buffer
//                        GPS_latAvailable = false ;       // reset the flag     
        TxHottData.gpsMsg.flightDirection = fields[HEADING].value / 200 ; // convert from degre * 100 to 1/2 degree; Flightdir./dir. 1 = 2°; 0° (North), 90° (East), 180° (South), 270° (West)
        uint16_t speedHott ;
        speedHott = ((uint32_t) fields[GROUNDSPEED].value) * 36 /1000 ;       // convert from cm/sec to km/h
        TxHottData.gpsMsg.GPSSpeedLow = speedHott ;                     
        TxHottData.gpsMsg.GPSSpeedHigh = speedHott >> 8 ;               
        uint16_t degMin ;
        uint16_t decimalMin ;
        TxHottData.gpsMsg.LatitudeNS = (fields[LATITUDE].value < 0) ;                        // Byte 10: 000 = N = 48°39’0988 
        convertLonLat_Hott(fields[LATITUDE].value, & degMin , & decimalMin ) ;              // convert to 2 fields (one beging deg*100+min, the other being the decimal part of min with 4 decimals
        TxHottData.gpsMsg.LatitudeMinLow = degMin ;                           // Byte 11: 231 = 0xE7 <= 0x12E7 = 4839 
        TxHottData.gpsMsg.LatitudeMinHigh = degMin >> 8 ;                     // Byte 12: 018 = 0x12 <= 0x12E7 = 4839
        TxHottData.gpsMsg.LatitudeSecLow = decimalMin ;                           // Byte 13: 220 = 0xDC <= 0x03DC = 0988
        TxHottData.gpsMsg.LatitudeSecHigh = decimalMin >> 8 ;                     // Byte 14: 003 = 0x03 <= 0x03DC = 0988
        
        TxHottData.gpsMsg.longitudeEW = (fields[LONGITUDE].value < 0) ;                        // Byte 15: 000  = E= 9° 25’9360
        convertLonLat_Hott(fields[LONGITUDE].value, &degMin , &decimalMin ) ;              // convert to 2 fields (one beging deg*100+min, the other being the decimal part of min with 4 decimals
        TxHottData.gpsMsg.longitudeMinLow = degMin ;                           // Byte 16: 157 = 0x9D <= 0x039D = 0925
        TxHottData.gpsMsg.longitudeMinHigh = degMin >> 8 ;                     // Byte 17: 003 = 0x03 <= 0x039D = 0925
        TxHottData.gpsMsg.longitudeSecLow = decimalMin ;                           // Byte 18: 144 = 0x90 <= 0x2490 = 9360
        TxHottData.gpsMsg.longitudeSecHigh = decimalMin >> 8 ;                     // Byte 19: 036 = 0x24 <= 0x2490 = 9360
        
        TxHottData.gpsMsg.distanceLow = gps.GPS_distance ;                             // Byte 20: 027 123 = /distance low byte 6 = 6 m
        TxHottData.gpsMsg.distanceHigh = gps.GPS_distance >> 8 ;                       // Byte 21: 036 35 = /distance high byte
        TxHottData.gpsMsg.HomeDirection = gps.GPS_bearing / 2 ;                        //Byte 29: HomeDirection (direction from starting point to Model position) (1 byte) 2degree = 1
        altitudeHott += (fields[ALTITUDE].value / 100)  ;                                 // convert from cm to m (keep the ofsset of 500 m)
        TxHottData.gpsMsg.GPSNumSat = fields[NUMSAT].value;               // Byte 27: GPS.Satelites (number of satelites) (1 byte) 
    
        //printf("gpsAlt=%d\n", (int32_t) fields[ALTITUDE].value/100);
    }
                  
 /* not yet implemented
  uint8_t resolutionLow;           // Byte 24: 48 = Low Byte m/s resolution 0.01m 48 = 30000 = 0.00m/s (1=0.01m/s) 
  uint8_t resolutionHigh;          // Byte 25: 117 = High Byte m/s resolution 0.01m 
  uint8_t unknow1;                 // Byte 26: 120 = 0m/3s 
  uint8_t GPSNumSat;               // Byte 27: GPS.Satelites (number of satelites) (1 byte) 
  uint8_t GPSFixChar;              // Byte 28: GPS.FixChar. (GPS fix character. display, if DGPS, 2D oder 3D) (1 byte) 
  uint8_t HomeDirection;           // Byte 29: HomeDirection (direction from starting point to Model position) (1 byte) 
  uint8_t angleXdirection;         // Byte 30: angle x-direction (1 byte) 
  uint8_t angleYdirection;         // Byte 31: angle y-direction (1 byte) 
  uint8_t angleZdirection;         // Byte 32: angle z-direction (1 byte) 
  uint8_t gyroXLow;                // Byte 33: gyro x low byte (2 bytes) 
  uint8_t gyroXHigh;               // Byte 34: gyro x high byte 
  uint8_t gyroYLow;                // Byte 35: gyro y low byte (2 bytes) 
  uint8_t gyroYHigh;               // Byte 36: gyro y high byte 
  uint8_t gyroZLow;                // Byte 37: gyro z low byte (2 bytes) 
  uint8_t gyroZHigh;               // Byte 38: gyro z high byte 
  uint8_t vibration;               // Byte 39: vibration (1 bytes) 
  uint8_t Ascii4;                  // Byte 40: 00 ASCII Free Character [4] 
  uint8_t Ascii5;                  // Byte 41: 00 ASCII Free Character [5] 
  uint8_t GPS_fix;                 // Byte 42: 00 ASCII Free Character [6], we use it for GPS FIX 
  uint8_t version;                 // Byte 43: 00 version number 
  uint8_t endByte;                 // Byte 44: 0x7D Ende byte 
  uint8_t chksum;                  // Byte 45: Parity Byte 
*/            
    if (gps.GPS_fix_type > 4 ) { 
        TxHottData.gpsMsg.GPSFixChar =  convertGpsFix[0] ;
    } else {
        TxHottData.gpsMsg.GPSFixChar =  convertGpsFix[gps.GPS_fix_type] ;
    }    
    TxHottData.gpsMsg.GPS_fix = TxHottData.gpsMsg.GPSFixChar ;
    TxHottData.gpsMsg.GPSNumSat = fields[NUMSAT].value ;
    TxHottData.gpsMsg.altitudeLow = altitudeHott ; 
    TxHottData.gpsMsg.altitudeHigh = altitudeHott >> 8 ;
    uint16_t varioHott = 30000 ;
#ifdef VARIO
                varioHott += mainVspeed.value ;  // put vario vertical speed in GPS data
#endif                
    TxHottData.gpsMsg.resolutionLow = varioHott ;          //climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
    TxHottData.gpsMsg.resolutionHigh = varioHott >> 8;
    TxHottData.gpsMsg.unknow1 = 120 ;                                       // Byte 26: 120 = 0m/3s

    TxHottData.txBuffer[TXHOTTDATA_BUFFERSIZE-1] = 0 ;
    for(uint8_t i = 0; i < TXHOTTDATA_BUFFERSIZE-1; i++){  // one byte less because the last byte is the checksum
        TxHottData.txBuffer[TXHOTTDATA_BUFFERSIZE-1] += TxHottData.txBuffer[i];
    }  // end for

    return true;
}



#if defined(NUMBEROFCELLS) && (NUMBEROFCELLS > 0) && defined(CELL_UNDERVOLTAGE_WARNING)
 
 byte OXS_OUT::warning_beeps_Hott(void) {       // Determine whether a warning has to be sent to the transmitter. 
                                                // Define the state variables for the state machine controlling the warning generation.
                                                // W_IDLE is the IDLE state but that name cannot be used because IDLE is already defined in oXs_out_hott.h.
     static enum {W_IDLE = 0, WARNING, DEADTIME} state = W_IDLE;
     static unsigned long warning_start_time;
     static uint8_t current_warning;
      // In order not to flood the transmitter with warnings, we transmit our warnings only every 3 seconds.
      // If the dead time is over, fall back into the idle state.
     if (state == DEADTIME && millisRp() - warning_start_time > 3000)  state = W_IDLE;
      // State WARNING indicates that we just started to transmit a warning.
      // Repeat it for 500ms to make sure the transmitter can receive it.
      // After 500ms we stop transmitting the warning and disable new warnings for a while.
     if (state == WARNING && millisRp() - warning_start_time > 500) {
       state = DEADTIME;
       current_warning = 0;
     }
     // In the idle state, we are ready to accept new warnings.
     if (state == W_IDLE) {
        if (voltageData->mVoltCellMin < CELL_UNDERVOLTAGE_WARNING) {
            warning_start_time = millisRp();
            state = WARNING;
            current_warning = 17; /* Cell undervoltage warning */
        }
     }
     return current_warning;
 }
#endif

/*
void OXS_OUT::sendData() {
#ifdef DEBUGHOTT
//      printer->print(F("F= ")); printer->print(flagUpdateHottBuffer);
//      printer->print(F(" S=")); printer->print(state);
//      printer->print(F(" LR=")); printer->print(LastRx);
//      printer->print(F(" Tc=")); printer->println(TxCount);
#endif
    if ( flagUpdateHottBuffer ) {        // this flag is set to true when UART get a polling of the device. Then measurement must be filled in the buffer
#ifdef DEBUG_BLINK_UPLOAD_HOTT_DATA
      blinkLed(5) ; // blink every 500 ms at least
#endif 
        // fill the buffer with 0 
        for ( uint8_t i = 0 ; i < TXHOTTDATA_BUFFERSIZE ; i++ ) {       // first fill the buffer with 0 
           TxHottData.txBuffer[i] = 0 ;
        }
        if ( flagUpdateHottBuffer == HOTT_TELEMETRY_GAM_SENSOR_ID  ) {        // this flag is set to true when UART get a polling of the GAM device. Then measurement must be filled in the buffer
              TxHottData.gamMsg.start_byte    = 0x7c ;
              TxHottData.gamMsg.gam_sensor_id = 0x8d ; //GENRAL AIR MODULE = HOTT_TELEMETRY_GAM_SENSOR_ID
              TxHottData.gamMsg.sensor_id     = 0xd0 ;
              TxHottData.gamMsg.stop_byte     = 0x7D ;

// in general air module data to fill are:
#if defined(NUMBEROFCELLS) && (NUMBEROFCELLS >= 1) 
              TxHottData.gamMsg.cell[0] =  voltageData->mVoltCell[0] /20 ; // Volt Cell 1 (in 2 mV increments, 210 == 4.20 V)
#endif
#if  defined(NUMBEROFCELLS) && (NUMBEROFCELLS >= 2) 
              TxHottData.gamMsg.cell[1] =  voltageData->mVoltCell[1] /20 ; // Volt Cell 2 (in 2 mV increments, 210 == 4.20 V)
#endif
#if  defined(NUMBEROFCELLS) && (NUMBEROFCELLS >= 3) 
              TxHottData.gamMsg.cell[2] =  voltageData->mVoltCell[2] /20 ; // Volt Cell 3 (in 2 mV increments, 210 == 4.20 V)
#endif
#if  defined(NUMBEROFCELLS) && (NUMBEROFCELLS >= 4) 
             TxHottData.gamMsg.cell[3] =  voltageData->mVoltCell[3] /20 ; // Volt Cell 4 (in 2 mV increments, 210 == 4.20 V)
#endif
#if  defined(NUMBEROFCELLS) && (NUMBEROFCELLS >= 5) 
              TxHottData.gamMsg.cell[4] =  voltageData->mVoltCell[4] /20 ; // Volt Cell 5 (in 2 mV increments, 210 == 4.20 V)
#endif
#if  defined(NUMBEROFCELLS) && (NUMBEROFCELLS >= 6) 
              TxHottData.gamMsg.cell[5] =  voltageData->mVoltCell[5] /20 ; // Volt Cell 6 (in 2 mV increments, 210 == 4.20 V)
#endif
#if defined(BATTERY_1_SOURCE) && ( (BATTERY_1_SOURCE == VOLT_1) || (BATTERY_1_SOURCE == VOLT_2) || (BATTERY_1_SOURCE == VOLT_3) || (BATTERY_1_SOURCE == VOLT_4) || (BATTERY_1_SOURCE == VOLT_5) || (BATTERY_1_SOURCE == VOLT_6) ) && defined(ARDUINO_MEASURES_VOLTAGES) && (ARDUINO_MEASURES_VOLTAGES == YES)
              TxHottData.gamMsg.Battery1 = voltageData->mVolt[BATTERY_1_SOURCE - VOLT_1].value / 100;    //battery 1 voltage  0.1V steps. 55 = 5.5V only pos. voltages
#endif
#if defined(BATTERY_1_SOURCE) && ( (BATTERY_1_SOURCE == ADS_VOLT_1) || (BATTERY_1_SOURCE == ADS_VOLT_2) || (BATTERY_1_SOURCE == ADS_VOLT_3) || (BATTERY_1_SOURCE == ADS_VOLT_4) ) && defined(AN_ADS1115_IS_CONNECTED) && (AN_ADS1115_IS_CONNECTED == YES ) && defined(ADS_MEASURE)
              TxHottData.gamMsg.Battery1 = ads_Conv[BATTERY_1_SOURCE - ADS_VOLT_1].value / 100;    //battery 1 voltage  0.1V steps. 55 = 5.5V only pos. voltages
#endif
#if defined(BATTERY_2_SOURCE) && ( (BATTERY_2_SOURCE == VOLT_1) || (BATTERY_2_SOURCE == VOLT_2) || (BATTERY_2_SOURCE == VOLT_3) || (BATTERY_2_SOURCE == VOLT_4) || (BATTERY_2_SOURCE == VOLT_5) || (BATTERY_2_SOURCE == VOLT_6) ) && defined(ARDUINO_MEASURES_VOLTAGES) && (ARDUINO_MEASURES_VOLTAGES == YES)
              TxHottData.gamMsg.Battery2 = voltageData->mVolt[BATTERY_2_SOURCE - VOLT_1].value / 100;    //battery 1 voltage  0.1V steps. 55 = 5.5V only pos. voltages
#endif
#if defined(BATTERY_2_SOURCE) && ( (BATTERY_2_SOURCE == ADS_VOLT_1) || (BATTERY_2_SOURCE == ADS_VOLT_2) || (BATTERY_2_SOURCE == ADS_VOLT_3) || (BATTERY_2_SOURCE == ADS_VOLT_4) ) && defined(AN_ADS1115_IS_CONNECTED) && (AN_ADS1115_IS_CONNECTED == YES ) && defined(ADS_MEASURE)
              TxHottData.gamMsg.Battery1 = ads_Conv[BATTERY_2_SOURCE - ADS_VOLT_1].value / 100;    //battery 1 voltage  0.1V steps. 55 = 5.5V only pos. voltages
#endif

#if defined(TEMPERATURE_1_SOURCE) && (TEMPERATURE_1_SOURCE == TEST_1 )
              TxHottData.gamMsg.temperature1 = test1.value + 20 ; // Hott applies an offset of 20. A value of 20 = 0°C    
#elif defined(TEMPERATURE_1_SOURCE) && (TEMPERATURE_1_SOURCE == TEST_2 )
              TxHottData.gamMsg.temperature1 = test2.value + 20 ; // Hott applies an offset of 20. A value of 20 = 0°C    
#elif defined(TEMPERATURE_1_SOURCE) && (TEMPERATURE_1_SOURCE == TEST_3 )
              TxHottData.gamMsg.temperature1 = test3.value + 20 ; // Hott applies an offset of 20. A value of 20 = 0°C    
#elif defined(TEMPERATURE_1_SOURCE) && (TEMPERATURE_1_SOURCE == GLIDER_RATIO ) && defined(GLIDER_RATIO_CALCULATED_AFTER_X_SEC)
              TxHottData.gamMsg.temperature1 = gliderRatio.value + 20 ; // Hott applies an offset of 20. A value of 20 = 0°C    
#elif defined(TEMPERATURE_1_SOURCE) && (TEMPERATURE_1_SOURCE == SENSITIVITY ) && defined(VARIO)
              TxHottData.gamMsg.temperature1 = oXs_MS5611.varioData.sensitivity.value + 20 ; // Hott applies an offset of 20. A value of 20 = 0°C    
#elif defined(TEMPERATURE_1_SOURCE) && (TEMPERATURE_1_SOURCE == PPM ) && defined(PIN_PPM)
              TxHottData.gamMsg.temperature1 = ppm.value + 120 ; // Hott applies an offset of 20. A value of 20 = 0°C    
#elif defined(TEMPERATURE_1_SOURCE) && ( (TEMPERATURE_1_SOURCE == VOLT_1 ) || (TEMPERATURE_1_SOURCE == VOLT_2 ) || (TEMPERATURE_1_SOURCE == VOLT_3 ) || (TEMPERATURE_1_SOURCE == VOLT_4 ) || (TEMPERATURE_1_SOURCE == VOLT_5 ) || (TEMPERATURE_1_SOURCE == VOLT_6 ) )  && defined(ARDUINO_MEASURES_VOLTAGES) && (ARDUINO_MEASURES_VOLTAGES == YES)
              TxHottData.gamMsg.temperature1 = (voltageData->mVolt[TEMPERATURE_1_SOURCE - VOLT_1].value ) / 10 + 20 ; // Hott applies an offset of 20. A value of 20 = 0°C    
#else
              TxHottData.gamMsg.temperature1 = 20 ; // Hott applies an offset of 20. A value of 20 = 0°C    
#endif

#if defined(TEMPERATURE_2_SOURCE) && (TEMPERATURE_2_SOURCE == TEST_1 )
              TxHottData.gamMsg.temperature2 = test1.value + 20 ; // Hott applies an offset of 20. A value of 20 = 0°C    
#elif defined(TEMPERATURE_2_SOURCE) && (TEMPERATURE_2_SOURCE == TEST_2 )
              TxHottData.gamMsg.temperature2 = test2.value + 20 ; // Hott applies an offset of 20. A value of 20 = 0°C    
#elif defined(TEMPERATURE_2_SOURCE) && (TEMPERATURE_2_SOURCE == TEST_3 )
              TxHottData.gamMsg.temperature2 = test3.value + 20 ; // Hott applies an offset of 20. A value of 20 = 0°C    
#elif defined(TEMPERATURE_2_SOURCE) && (TEMPERATURE_2_SOURCE == GLIDER_RATIO ) && defined(GLIDER_RATIO_CALCULATED_AFTER_X_SEC)
              TxHottData.gamMsg.temperature2 = gliderRatio.value + 20 ; // Hott applies an offset of 20. A value of 20 = 0°C    
#elif defined(TEMPERATURE_2_SOURCE) && (TEMPERATURE_2_SOURCE == SENSITIVITY ) && defined(VARIO)
              TxHottData.gamMsg.temperature2 = oXs_MS5611.varioData.sensitivity.value + 20 ; // Hott applies an offset of 20. A value of 20 = 0°C    
#elif defined(TEMPERATURE_2_SOURCE) && (TEMPERATURE_2_SOURCE == PPM ) && defined(PIN_PPM)
              TxHottData.gamMsg.temperature2 = ppm.value + 120 ; // Hott applies an offset of 20. A value of 20 = 0°C    
#elif defined(TEMPERATURE_2_SOURCE) && ( (TEMPERATURE_2_SOURCE == VOLT_1 ) || (TEMPERATURE_2_SOURCE == VOLT_2 ) || (TEMPERATURE_2_SOURCE == VOLT_3 ) || (TEMPERATURE_2_SOURCE == VOLT_4 ) || (TEMPERATURE_2_SOURCE == VOLT_5 ) || (TEMPERATURE_2_SOURCE == VOLT_6 ) ) && defined(ARDUINO_MEASURES_VOLTAGES) && (ARDUINO_MEASURES_VOLTAGES == YES)
              TxHottData.gamMsg.temperature2 = (voltageData->mVolt[TEMPERATURE_2_SOURCE - VOLT_1].value ) /10  + 20 ; // Hott applies an offset of 20. A value of 20 = 0°C    
#else
              TxHottData.gamMsg.temperature2 = 20 ; // Hott applies an offset of 20. A value of 20 = 0°C    
#endif

              TxHottData.gamMsg.rpm++ ;
              if ( TxHottData.gamMsg.rpm > 1000) TxHottData.gamMsg.rpm = 1 ; 
#ifdef MEASURE_RPM 
              TxHottData.gamMsg.rpm  = RpmValue /10 ;                      //#22 RPM in 10 RPM steps. 300 = 3000rpm
#endif
#ifdef VARIO       
              TxHottData.gamMsg.altitude =  ((varioData->relativeAlt.value ) / 100 ) + 500 ;  //altitude in meters. offset of 500, 500 = 0m
              TxHottData.gamMsg.climbrate_L = mainVspeed.value + 30000 ;          //climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
#else
              TxHottData.gamMsg.altitude =  500 ;  //altitude in meters. offset of 500, 500 = 0m
              TxHottData.gamMsg.climbrate_L = 30000 ;          //climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
#endif
              TxHottData.gamMsg.climbrate3s = 120 ;                     //#28 climb rate in m/3sec. Value of 120 = 0m/3sec
#if defined(ARDUINO_MEASURES_A_CURRENT) && (ARDUINO_MEASURES_A_CURRENT == YES)
              TxHottData.gamMsg.current =  currentData->milliAmps.value /100;               //current in 0.1A steps 100 == 10,0A
#endif
#if defined(MAIN_BATTERY_SOURCE) && ( (MAIN_BATTERY_SOURCE == VOLT_1) || (MAIN_BATTERY_SOURCE == VOLT_2) || (MAIN_BATTERY_SOURCE == VOLT_3) || (MAIN_BATTERY_SOURCE == VOLT_4) || (MAIN_BATTERY_SOURCE == VOLT_5) || (MAIN_BATTERY_SOURCE == VOLT_6) ) && defined(ARDUINO_MEASURES_VOLTAGES) && (ARDUINO_MEASURES_VOLTAGES == YES)
              TxHottData.gamMsg.main_voltage = voltageData->mVolt[MAIN_BATTERY_SOURCE - VOLT_1].value / 100;          //Main power voltage using 0.1V steps 100 == 10,0V] / 100
#endif
#if defined(ARDUINO_MEASURES_A_CURRENT) && (ARDUINO_MEASURES_A_CURRENT == YES)
              TxHottData.gamMsg.batt_cap =  currentData->consumedMilliAmps.value / 10 ;   // used battery capacity in 10mAh steps
#endif
#ifdef AIRSPEED       
               TxHottData.gamMsg.speed =  airSpeedData->airSpeed.value  ;                  //  Km/h 
#endif
#if defined(NUMBEROFCELLS) && (NUMBEROFCELLS >= 0) 
              TxHottData.gamMsg.min_cell_volt =  voltageData->mVoltCellMin /20 ; // minimum cell voltage in 2mV steps. 124 = 2,48V
#endif
#if defined(NUMBEROFCELLS) && (NUMBEROFCELLS > 0) && defined(CELL_UNDERVOLTAGE_WARNING)
             TxHottData.gamMsg.warning_beeps = warning_beeps_Hott();   // Transmitter warning message
#endif

// field of msg not implemented
//  byte climbrate3s;                     //#28 climb rate in m/3sec. Value of 120 = 0m/3sec
//  byte min_cell_volt_num;               //#38 number of the cell with the lowest voltage
//  uint16_t rpm2;                        //#39 LSB 2nd RPM in 10 RPM steps. 100 == 1000rpm
//  byte general_error_number;            //#41 General Error Number (Voice Error == 12) TODO: more documentation
//  byte pressure;                        //#42 High pressure up to 16bar. 0,1bar scale. 20 == 2.0bar

            } 
#ifdef GPS_INSTALLED            
              else {
            // here the code for GPS
                    TxHottData.gpsMsg.startByte    = 0x7c ;
                    TxHottData.gpsMsg.sensorID     = HOTT_TELEMETRY_GPS_SENSOR_ID ; //0x8A
                    TxHottData.gpsMsg.sensorTextID     = HOTTV4_GPS_SENSOR_TEXT_ID ; // 0xA0
                    TxHottData.gpsMsg.endByte     = 0x7D ;
                    uint16_t altitudeHott = 500 ;                   // Hott uses an offset of 500 (m)
                    if ( (GPS_fix_type == 3 ) || (GPS_fix_type == 4 ) ) {
//                    if( GPS_latAvailable ) {             // test if data are available (GPS fix) ; if available, fill the buffer
//                        GPS_latAvailable = false ;       // reset the flag     
                        TxHottData.gpsMsg.flightDirection = GPS_ground_course / 200000 ; // convert from degre * 100000 to 1/2 degree; Flightdir./dir. 1 = 2°; 0° (North), 90° (East), 180° (South), 270° (West)
                        static uint16_t speedHott ;
#ifdef GPS_SPEED_3D
                        speedHott = ((uint32_t) GPS_speed_3d) * 36 /1000 ;       // convert from cm/sec to km/h
#else
                        speedHott = ((uint32_t) GPS_speed_2d) * 36 /1000 ;       // convert from cm/sec to km/h
#endif
                        TxHottData.gpsMsg.GPSSpeedLow = speedHott ;                     
                        TxHottData.gpsMsg.GPSSpeedHigh = speedHott >> 8 ;               
                        uint16_t degMin ;
                        uint16_t decimalMin ;
                        TxHottData.gpsMsg.LatitudeNS = (GPS_lat < 0) ;                        // Byte 10: 000 = N = 48°39’0988 
                        convertLonLat_Hott(GPS_lat, & degMin , & decimalMin ) ;              // convert to 2 fields (one beging deg*100+min, the other being the decimal part of min with 4 decimals
                        TxHottData.gpsMsg.LatitudeMinLow = degMin ;                           // Byte 11: 231 = 0xE7 <= 0x12E7 = 4839 
                        TxHottData.gpsMsg.LatitudeMinHigh = degMin >> 8 ;                     // Byte 12: 018 = 0x12 <= 0x12E7 = 4839
                        TxHottData.gpsMsg.LatitudeSecLow = decimalMin ;                           // Byte 13: 220 = 0xDC <= 0x03DC = 0988
                        TxHottData.gpsMsg.LatitudeSecHigh = decimalMin >> 8 ;                     // Byte 14: 003 = 0x03 <= 0x03DC = 0988
                        
                        TxHottData.gpsMsg.longitudeEW = (GPS_lon < 0) ;                        // Byte 15: 000  = E= 9° 25’9360
                        convertLonLat_Hott(GPS_lon, &degMin , &decimalMin ) ;              // convert to 2 fields (one beging deg*100+min, the other being the decimal part of min with 4 decimals
                        TxHottData.gpsMsg.longitudeMinLow = degMin ;                           // Byte 16: 157 = 0x9D <= 0x039D = 0925
                        TxHottData.gpsMsg.longitudeMinHigh = degMin >> 8 ;                     // Byte 17: 003 = 0x03 <= 0x039D = 0925
                        TxHottData.gpsMsg.longitudeSecLow = decimalMin ;                           // Byte 18: 144 = 0x90 <= 0x2490 = 9360
                        TxHottData.gpsMsg.longitudeSecHigh = decimalMin >> 8 ;                     // Byte 19: 036 = 0x24 <= 0x2490 = 9360
                        TxHottData.gpsMsg.distanceLow = GPS_distance ;                             // Byte 20: 027 123 = /distance low byte 6 = 6 m
                        TxHottData.gpsMsg.distanceHigh = GPS_distance >> 8 ;                       // Byte 21: 036 35 = /distance high byte
                        TxHottData.gpsMsg.HomeDirection = GPS_bearing / 2 ;                        //Byte 29: HomeDirection (direction from starting point to Model position) (1 byte) 2degree = 1
                        altitudeHott += (GPS_altitude / 1000)  ;                                 // convert from mm to m (keep the ofsset of 500 m)
                   } 
 */

 /* not yet implemented
  uint8_t resolutionLow;           // Byte 24: 48 = Low Byte m/s resolution 0.01m 48 = 30000 = 0.00m/s (1=0.01m/s) 
  uint8_t resolutionHigh;          // Byte 25: 117 = High Byte m/s resolution 0.01m 
  uint8_t unknow1;                 // Byte 26: 120 = 0m/3s 
  uint8_t GPSNumSat;               // Byte 27: GPS.Satelites (number of satelites) (1 byte) 
  uint8_t GPSFixChar;              // Byte 28: GPS.FixChar. (GPS fix character. display, if DGPS, 2D oder 3D) (1 byte) 
  uint8_t HomeDirection;           // Byte 29: HomeDirection (direction from starting point to Model position) (1 byte) 
  uint8_t angleXdirection;         // Byte 30: angle x-direction (1 byte) 
  uint8_t angleYdirection;         // Byte 31: angle y-direction (1 byte) 
  uint8_t angleZdirection;         // Byte 32: angle z-direction (1 byte) 
  uint8_t gyroXLow;                // Byte 33: gyro x low byte (2 bytes) 
  uint8_t gyroXHigh;               // Byte 34: gyro x high byte 
  uint8_t gyroYLow;                // Byte 35: gyro y low byte (2 bytes) 
  uint8_t gyroYHigh;               // Byte 36: gyro y high byte 
  uint8_t gyroZLow;                // Byte 37: gyro z low byte (2 bytes) 
  uint8_t gyroZHigh;               // Byte 38: gyro z high byte 
  uint8_t vibration;               // Byte 39: vibration (1 bytes) 
  uint8_t Ascii4;                  // Byte 40: 00 ASCII Free Character [4] 
  uint8_t Ascii5;                  // Byte 41: 00 ASCII Free Character [5] 
  uint8_t GPS_fix;                 // Byte 42: 00 ASCII Free Character [6], we use it for GPS FIX 
  uint8_t version;                 // Byte 43: 00 version number 
  uint8_t endByte;                 // Byte 44: 0x7D Ende byte 
  uint8_t chksum;                  // Byte 45: Parity Byte 
*/ 

/*
                if (GPS_fix_type > 4 ) GPS_fix_type = 0 ;
                TxHottData.gpsMsg.GPS_fix = TxHottData.gpsMsg.GPSFixChar =  convertGpsFix[GPS_fix_type] ;
                TxHottData.gpsMsg.GPSNumSat = GPS_numSat;
                TxHottData.gpsMsg.altitudeLow = altitudeHott ; 
                TxHottData.gpsMsg.altitudeHigh = altitudeHott >> 8 ;
                uint16_t varioHott = 30000 ;
#ifdef VARIO
                varioHott += mainVspeed.value ;  // put vario vertical speed in GPS data
#endif                
                TxHottData.gpsMsg.resolutionLow = varioHott ;          //climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
                TxHottData.gpsMsg.resolutionHigh = varioHott >> 8;
                TxHottData.gpsMsg.unknow1 = 120 ;                                       // Byte 26: 120 = 0m/3s
              
            }  // end else => flagUpdateHottBuffer == GPS
#endif         // end of GPS_Installed            
            // calculate the check sum on first bytes
            TxHottData.txBuffer[TXHOTTDATA_BUFFERSIZE-1] = 0 ;
            for(uint8_t i = 0; i < TXHOTTDATA_BUFFERSIZE-1; i++){  // one byte less because the last byte is the checksum
              TxHottData.txBuffer[TXHOTTDATA_BUFFERSIZE-1] += TxHottData.txBuffer[i];
            }  // end for
            flagUpdateHottBuffer = 0 ;       // reset the flag to say that all data have been updated and that UART can transmit the buffer            
#ifdef DEBUGHOTT
//            for(uint8_t i = 0; i < TXHOTTDATA_BUFFERSIZE; i++){  // include the last byte (checksum)
//                 printer->print(TxHottData.txBuffer[i], HEX); printer->print(F(" "));
//            } // end for    
//            printer->print(tempFlag , HEX) ;
//            printer->println(F(" "));
#endif
        
    }   // end ( flagUpdateHottBuffer )
}

*/
  
void convertLonLat_Hott( int32_t GPS_LatLon, uint16_t * degMin , uint16_t * decimalMin ) {
  static uint32_t GPS_LatLonAbs ;
  static uint16_t degre0decimals ;
  static uint32_t minute4decimals ;
  static uint16_t minute0decimals ;
  GPS_LatLonAbs = ( GPS_LatLon < 0 ? - GPS_LatLon : GPS_LatLon) / 100  ; // remove 2 decimals from original value which contains degre with 7 decimals (so next calculation are smaller)
  degre0decimals = GPS_LatLonAbs / 100000 ;                              // extract the degre without  decimal
  minute4decimals = ( GPS_LatLonAbs - ( ((uint32_t) degre0decimals) * 100000l ) ) * 6 ; // keep the decimal of degree and convert them in minutes (*60) and remove 1 decimal (/10) in order to keep 4 decimals 
  minute0decimals = minute4decimals / 10000 ;                                        // extract the minutes (without decimals)
  *degMin = degre0decimals * 100 + minute0decimals ;                                  // put degree and minutes toegether in a special format
  *decimalMin = minute4decimals - ( minute0decimals * 10000 ) ;                       // Extract the decimal part of the minutes (4 decimals) 
}


