#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico/util/queue.h"
#include "gps.h"
#include <math.h>
#include "param.h"
#include "tools.h"
#include <inttypes.h>

// scaling factor from 1e-7 degrees to meters at equater
// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
#define LOCATION_SCALING_FACTOR 0.011131884502145034f
#define DEG_TO_RAD_FOR_GPS 0.017453292519943295769236907684886f
#define GPS_UART_ID uart0

#define GPS_TX_PIN 12 
#define GPS_RX_PIN 13

extern queue_t gpsQueue ;
extern CONFIG config;
extern field fields[SPORT_TYPES_MAX];  // list of all telemetry fields and parameters used by Sport


// Receive buffer for Ublox
    union {
        ubx_nav_posllh posllh;
        ubx_nav_status status;
        ubx_nav_solution solution;
        ubx_nav_velned velned;
        ubx_nav_svinfo svinfo;
        uint8_t bytes[UBLOX_BUFFER_SIZE];
    } __attribute__((__packed__))  _buffer;

union {
        casic_nav_pv_info nav_pv;
        uint8_t bytes[sizeof(casic_nav_pv_info)];
    } __attribute__((__packed__)) _casicBuffer;


// RX interrupt handler
void on_uart_rx() {
    while (uart_is_readable(GPS_UART_ID)) {
        uint8_t ch = uart_getc(GPS_UART_ID);
        //int count = queue_get_level( &gpsQueue );
        //printf(" level = %i\n", count);
        //printf( "val = %X\n", ch);  // printf in interrupt generates error but can be tested for debugging if some char are received
        if (!queue_try_add ( &gpsQueue , &ch)) printf("queue try add error\n");
        //printf("%x\n", ch);
    }
}

GPS::GPS( void) {}

void GPS::setupGps(void){
    if ( config.gpsType == 'U') setupGpsUblox();
    if ( config.gpsType == 'C') setupGpsCasic();
}
void GPS::readGps(){
    if ( queue_is_empty (&gpsQueue)) return;
    if ( config.gpsType == 'U') readGpsUblox();
    if ( config.gpsType == 'C') readGpsCasic();
}

void GPS::setupGpsUblox(void){    // here the setup for a Ublox            
        uart_init(GPS_UART_ID, 9600);   // setup UART0 at 9600 baud
        uart_set_hw_flow(GPS_UART_ID, false, false);// Set UART flow control CTS/RTS, we don't want these, so turn them off
        uart_set_fifo_enabled(GPS_UART_ID, false);    // Turn off FIFO's - we want to do this character by character
        
        gpio_set_function(GPS_TX_PIN , GPIO_FUNC_UART); // Set the GPIO pin mux to the UART0
        gpio_set_function(GPS_RX_PIN , GPIO_FUNC_UART);

        const static uint8_t initGps1[] = { 
        // send command to GPS to change the setup
        // Here the code to activate galileo sat. (This has not yet been tested and is based on I-NAV code)
        /*
            0xB5,0x62,0x06,0x3E, 0x3C, 0x00, // GNSS + number of bytes= 60 dec = 003C in HEx
            0x00, 0x00, 0x20, 0x07,  // GNSS / min / max / enable
            0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, // GPS / 8 / 16 / Y
            0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, // SBAS / 1 / 3 / Y
            0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01, // Galileo / 4 / 8 / Y
            0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, // BeiDou / 8 / 16 / N
            0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, // IMES / 0 / 8 / N
            0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, // QZSS / 0 / 3 / N
            0x06, 0x08, 0x0e, 0x00, 0x01, 0x00, 0x01, 0x01, // GLONASS / 8 / 14 / Y
            0x30, 0xD8, // checksum

    // Here the code to activate SBAS for Europe (This has not yet been tested and is based on I-NAV code)
            0xB5,0x62,0x06,0x16, 0x08, 0x00, // SBAS + number of bytes = 8
            0x03, 0x03, 0x03, 0x00, // mode = test + enabled, usage=range+diffcorr, max =3, scanmode2=0
            0x00, 0x00, 0x08, 0x51, // scanmode1 120,124, 126, 131
            0x86, 0x2C, //checksum
        */
    // Here other code        
            0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, // activate NAV-POSLLH message
            0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x06,0x00,0x01,0x00,0x00,0x00,0x00,0x17,0xDA, //        NAV-SOL
            0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x12,0x00,0x01,0x00,0x00,0x00,0x00,0x23,0x2E, //        NAV-VELNED
            0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, // activate NAV-POSLLH message
            0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x06,0x00,0x01,0x00,0x00,0x00,0x00,0x17,0xDA, //        NAV-SOL
            0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x12,0x00,0x01,0x00,0x00,0x00,0x00,0x23,0x2E, //        NAV-VELNED
            
    #if defined(GPS_REFRESH_RATE) && (GPS_REFRESH_RATE == 1)
            0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39,  // NAV-RATE for 1 hz
    #elif defined(GPS_REFRESH_RATE) && (GPS_REFRESH_RATE == 10)
            0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, // NAV-RATE for 10 hz
    #else
            0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, // NAV-RATE for 5 hz
    #endif
            0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x96, //        CFG-PRT : Set port to output only UBX (so deactivate NMEA msg) and set baud = 38400.
                                0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x91,0x84  //                 rest of CFG_PRT command                            
        }  ;   
        busy_wait_us(750000) ; // wait to be sure that GPS has started (otherwise first bytes are lost)
        uint8_t initGpsIdx = 0 ;
        while (initGpsIdx < sizeof( initGps1)) {
    //    Serial.println( pgm_read_byte_near(initGps1 + initGpsIdx ), HEX) ;    
            if (initGps1[initGpsIdx] == 0XB5) busy_wait_us(5000) ;
            uart_putc_raw(GPS_UART_ID , initGps1[initGpsIdx++]);
        }
        busy_wait_us(10000);
                
        //printf("End of GPS setup\n"); sleep_ms(1000);
        // clear the input queue that is filled by the interrupt
        queue_init(&gpsQueue , sizeof(uint8_t) ,256) ;// queue for uart0 with 256 elements of 1
        
        // change the baudrate of UART0 to the new rate
        uart_set_baudrate(GPS_UART_ID , 38400);
        // And set up and enable the interrupt handlers
        irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
        irq_set_enabled(UART0_IRQ, true);
        // Now enable the UART to send interrupts - RX only
        uart_set_irq_enables(GPS_UART_ID, true, false);
        busy_wait_us(1000);
        uint8_t dummy;
        while (! queue_is_empty (&gpsQueue)) queue_try_remove ( &gpsQueue , &dummy ) ;
        _step = 0 ; 
}  // end setupGPSUblox;

    
void GPS::readGpsUblox(){
    uint8_t data;
    if (queue_try_remove ( &gpsQueue , &data ) ){
        //printf(" %X" , data);
        bool parsed = false;
        switch (_step) {
            case 0: // Sync char 1 (0xB5)
                if ( 0xB5 == data ) { // UBX sync char 1
                    _skip_packet = false;
                    _step++;
                }
                break;
            case 1: // Sync char 2 (0x62)
                if ( 0x62 != data) { // UBX sync char 1
                    _step = 0;
                    break;
                }
                _step++;
                break;
            case 2: // Class
                _step++;
                _class = data;  // normally we should check that the class is the expected (otherwise, frame should be skipped)
                _ck_b = _ck_a = data;   // reset the checksum accumulators
                if ( 0x01 != data ) { // we handle only message type = 0x01 = NAVigation message.
                    _skip_packet = true; // when skip packet = true, then the wholepacket will be read but discarded.
                }            
                break;
            case 3: // Id
                _step++;
                _ck_b += (_ck_a += data);       // checksum byte
                _msg_id = data;
                break;
            case 4: // Payload length (part 1)
                _step++;
                _ck_b += (_ck_a += data);       // checksum byte
                _payload_length = data; // payload length low byte
                break;
            case 5: // Payload length (part 2)
                _step++;
                _ck_b += (_ck_a += data);       // checksum byte
                _payload_length += (uint16_t)(data << 8);
                if (_payload_length > UBLOX_PAYLOAD_SIZE) {
                    _skip_packet = true; // when skip packet = true, then the wholepacket will be read but discarded.
                }
                _payload_counter = 0;   // prepare to receive payload
                if (_payload_length == 0) {
                    _step = 7;
                }
                break;
            case 6:
                _ck_b += (_ck_a += data);       // checksum byte
                if  (_payload_counter < UBLOX_BUFFER_SIZE)  {
                    _buffer.bytes[_payload_counter] = data; // save the content of the payload
    //                printer->print(data , HEX);
                }
                _payload_counter++ ;  
                if (_payload_counter >= _payload_length) {
                    _step++;
    //                printer->println(" ");
                }
                break;
            case 7:
                _step++;
                if (_ck_a != data) {
                    _skip_packet = true;          // bad checksum
                    gpsDataErrors++;
                }
                break;
            case 8:
                _step = 0;
                if (_ck_b != data) {
                    gpsDataErrors++;
                    break;              // bad checksum
                }

                GPS_packetCount++;
    //            printer->print("pac : ");  printer->print(GPS_packetCount); printer->print(",err: "); printer->print(gpsDataErrors); printer->print(",skip: "); printer->println(_skip_packet) ;

                if (_skip_packet) {
                    break;   // do not parse the packet to be skipped
                }
                            // if we arive here, it means that a valid frame has been received and that the gpsBuffer contains the data to be parsed
                if (parseGpsUblox() && (_class == 0x01) ) {
                    parsed = true; 
                }
        }  // end of case
    }    
}


bool GPS::parseGpsUblox(void) // move the data from buffer to the different fields
{
    uint8_t * pvelned =  (uint8_t *) &_buffer.velned; // only for debug
        
    // do we have new position information?
    new_position = false ;
    new_speed = false ;
    static bool next_fix = false ;
    //printf(" %X\n", _msg_id);
    switch (_msg_id) {
    case MSG_POSLLH:
        //i2c_dataset.time                = _buffer.posllh.time;
        gpsInstalled = true;
        fields[LONGITUDE].value = _buffer.posllh.longitude;           // in degree with 7 decimals
        fields[LATITUDE].value = _buffer.posllh.latitude;            // in degree with 7 decimals
        fields[ALTITUDE].value = _buffer.posllh.altitude_msl / 10;       //alt in mm in converted in cm (sport uses cm)
        if (next_fix) {                               // enable state if a position has been received after a positieve STATUS or SOL
            GPS_fix = true ;
            if ( GPS_home_lat == 0 ) { 
              GPS_home_lat = fields[LATITUDE].value ;  // save home position
              GPS_home_lon = fields[LONGITUDE].value ;
              GPS_scale = cosf(GPS_home_lat * 1.0e-7f * DEG_TO_RAD_FOR_GPS); // calculate scale factor based on latitude
            }
            // Calculate distance
            float dlat  = (float)(GPS_home_lat - fields[LATITUDE].value);
            float dlong  = ((float)(GPS_home_lon - fields[LONGITUDE].value)) * GPS_scale ;
            GPS_distance =  sqrtf( dlat * dlat + dlong * dlong  ) * LOCATION_SCALING_FACTOR;
            // calculate bearing
            int32_t off_x = fields[LONGITUDE].value - GPS_home_lon ;
            int32_t off_y = (fields[LATITUDE].value - GPS_home_lat) / GPS_scale ;
            GPS_bearing = 90 + atan2f(-off_y, off_x) * 57.2957795f;  // in degree
            if (GPS_bearing < 0) GPS_bearing += 360;
        } else {
            GPS_fix = false;
        }
        fields[LONGITUDE].available = GPS_fix;           // in degree with 7 decimals
        fields[LATITUDE].available = GPS_fix;
        fields[ALTITUDE].available = GPS_fix; 
        new_position = true;
        break;
//    case MSG_STATUS:                              // !!!!!!!!! I do not see real need of this message because same (and more) data are in SOL, so this message is not activated in init
//        next_fix = (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.status.fix_type == FIX_3D); // si valid position and speed or 3D fix
//        if (!next_fix)
//             GPS_fix = false;
//        break;
    case MSG_SOL:                                // !!!! here we could also use vertical speed which is I4 in cm/sec)
        next_fix = (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D);
        GPS_fix_type = _buffer.solution.fix_type;
        if (!next_fix)
             GPS_fix = false;
        fields[NUMSAT].value = _buffer.solution.satellites; 
        fields[NUMSAT].available = true;
        if ( _buffer.solution.fix_type == FIX_3D ) fields[NUMSAT].value += 100; // we add 100 when we have a 3d fix (for Ublox)
        GPS_hdop = _buffer.solution.position_DOP;
        //printf("nbr sat : %X \n", GPS_numSat) ; 
        break;
    case MSG_VELNED:   
        fields[GROUNDSPEED].value  = _buffer.velned.speed_3d;  // cm/s
        fields[GROUNDSPEED].available = GPS_fix ;
        GPS_speed_2d = _buffer.velned.speed_2d;    // cm/s
        GPS_speed_2dAvailable = GPS_fix ;
        fields[HEADING].value = _buffer.velned.heading_2d /1000;     // Heading 2D deg with 5 decimals is reduced to 2 d??cimals
        fields[HEADING].available = GPS_fix ;
        new_speed = true;
        //printf("spd= %f   Head= %f\n", _buffer.velned.speed_3d , _buffer.velned.heading_2d);
        //for (uint8_t i=0 ; i<36; i++){
        //    printf(" %X", _buffer.bytes[i]);
        //}
        //printf("  spd= %" PRIu32 "\n", _buffer.velned.heading_2d );
        //for (uint8_t i = 0 ; i < 36 ; i++){
        //    printf(".%X",    *(pvelned + i ) );
        //}    
        //printf("  fld= %" PRIi32 "\n" , fields[HEADING].value);
        break;
    default:
        return false;
    } // end of case

    // we only return true when we get new position and speed data
    // this ensures we don't use stale data
    if (new_position && new_speed) {
        new_speed = new_position = false;
        return true;
    }
    return false;
}

//  For casic GPS
void GPS::setupGpsCasic(void){    // for casic gps
    // clear the input queue that is filled by the interrupt
    queue_init(&gpsQueue , 1 ,256) ;// queue for uart0 with 256 elements of 1
    
    uart_init(GPS_UART_ID, 38400);   // setup UART0 at 38400 baud
    uart_set_hw_flow(GPS_UART_ID, false, false);// Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_fifo_enabled(GPS_UART_ID, false);    // Turn off FIFO's - we want to do this character by character
    
    gpio_set_function(GPS_TX_PIN , GPIO_FUNC_UART); // Set the GPIO pin mux to the UART 
    gpio_set_function(GPS_RX_PIN , GPIO_FUNC_UART);
    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);
    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(GPS_UART_ID, true, false);
    busy_wait_us(1000);
    uint8_t dummy;
    while (! queue_is_empty (&gpsQueue)) queue_try_remove ( &gpsQueue , &dummy ) ;
    _step = 0;
}  // end setupGPS
   

void GPS::readGpsCasic() { // read and process GPS data. do not send them.// for casic gps
    uint8_t data;
    static uint8_t _idx;
    if (queue_try_remove ( &gpsQueue , &data ) ) {
        //if (data == 0xBA) printf("\n"); // new line when sync byte is received
        //printf(" %x " , data );
        switch (_step) {
            case 0: // Sync char 1 (0xBA)
                if ( 0xBA == data ) _step++ ; // Casic sync char 1
                break;
            case 1: // Sync char 2 (0xCE)
                if ( 0xCE == data) _step++ ; else _step = 0 ;
                break;
            case 2: // Payload length (part 1)
                if ( 0x50 == data) _step++ ; else _step = 0 ;
                break;
            case 3: // Payload length (part 2)
                if ( 0x00 == data) _step++ ; else _step = 0 ;
                break;   
            case 4: // Class 
                if ( 0x01 == data) _step++ ; else _step = 0 ;
                break;   
            case 5: // Sub class
                if ( 0x03 == data) {
                    _step++ ;
                    _idx = 6 ;
                } else {
                    _step = 0 ;
                }    
                break;   
            case 6 : // payload and checksum    
                //printf(" size of casic_nav_pv_info %i", (int) sizeof(casic_nav_pv_info) );
                if  (_idx < sizeof(casic_nav_pv_info) ) {
                     _casicBuffer.bytes[_idx++] = data; // save the content of the payload
    //                printer->print(data , HEX);
                    //if (_idx == 11) printf("pos %x\n",_casicBuffer.bytes[_idx -1]);
                }
                if (_idx == sizeof(casic_nav_pv_info)) {
                    parseGpsCasic();
                    _step = 0;
    //                printer->println(" ");
                }
                break;
        
        }  // end of case
    }
}

bool GPS::parseGpsCasic(void) // move the data from buffer to the different fields
{
    //printf("sat : %x  posValid %x  height %f\n", _casicBuffer.nav_pv.numSV,  _casicBuffer.nav_pv.velValid , _casicBuffer.nav_pv.height);
    //printf("lon: %f   ", _casicBuffer.nav_pv.lon);
    //printf("lat: %f   ", _casicBuffer.nav_pv.lat);
    //printf("height %f \n",  _casicBuffer.nav_pv.height);
    //if ( _casicBuffer.nav_pv.posValid >= 7){
    //    printf("x");
    //} else {
    //    printf(".");
    //}
    gpsInstalled = true;
    fields[NUMSAT].value = _casicBuffer.nav_pv.numSV;
    fields[NUMSAT].available = true;
    if ( _casicBuffer.nav_pv.velValid >= 6) {
        GPS_speed_2d  = _casicBuffer.nav_pv.speed2D;
        GPS_speed_2dAvailable  = true;
    }    
    if ( _casicBuffer.nav_pv.velValid >= 7) {
        fields[NUMSAT].value += 100; // add 100 if 3d fix available
        fields[GROUNDSPEED].value  = _casicBuffer.nav_pv.speed3D * 100; // in ublox = cm/sec, in CASIC float M/sec
        fields[LONGITUDE].value = _casicBuffer.nav_pv.lon * 10000000;   // in Ublox = degree with 7 decimals, in CASIC float degree 
        fields[LATITUDE].value = _casicBuffer.nav_pv.lat* 10000000;   // in Ublox = degree with 7 decimals, in CASIC float degree
        if (_casicBuffer.nav_pv.height > 0) {
            fields[ALTITUDE].value = _casicBuffer.nav_pv.height * 100 ;  // in cm : in Ublox = mm , in CASIC float m
        } else {
            fields[ALTITUDE].value = 0 ;
        }
        fields[HEADING].value = _casicBuffer.nav_pv.heading * 100 ;    // in Ublox = deg with 5 decimals,  in CASIC = float degree
        fields[GROUNDSPEED].available  = true;
        fields[LONGITUDE].available = true;           
        fields[LATITUDE].available = true;
        fields[ALTITUDE].available = true; 
        fields[HEADING].available = true;
        if ( GPS_home_lat == 0 ) { 
              GPS_home_lat = fields[LATITUDE].value ;  // save home position
              GPS_home_lon = fields[LONGITUDE].value ;
              GPS_scale = cosf(GPS_home_lat * 1.0e-7f * DEG_TO_RAD_FOR_GPS); // calculate scale factor based on latitude
        }
        // Calculate distance
        float dlat  = (float)(GPS_home_lat - fields[LATITUDE].value);
        float dlong  = ((float)(GPS_home_lon - fields[LONGITUDE].value)) * GPS_scale ;
        GPS_distance =  sqrtf( dlat * dlat + dlong * dlong  ) * LOCATION_SCALING_FACTOR;
        // calculate bearing
        int32_t off_x = fields[LONGITUDE].value - GPS_home_lon ;
        int32_t off_y = (fields[LATITUDE].value - GPS_home_lat) / GPS_scale ;
        GPS_bearing = 90 + atan2f(-off_y, off_x) * 57.2957795f;  // in degree
        if (GPS_bearing < 0) GPS_bearing += 360;
        return true;
    } else {
        fields[GROUNDSPEED].value  = 0;        // in cm/sec
        fields[LONGITUDE].value = 0;           // in degree with 7 decimals
        fields[LATITUDE].value = 0;            // in degree with 7 decimals
        fields[ALTITUDE].value = 0 ;    // in mm
        fields[HEADING].value = 0 ;     // Heading 2D deg with 5 decimals
        fields[GROUNDSPEED].available  = false;
        fields[LONGITUDE].available = false;           
        fields[LATITUDE].available = false;
        fields[ALTITUDE].available = false; 
        fields[HEADING].available = false;
    }
    return false;
}
