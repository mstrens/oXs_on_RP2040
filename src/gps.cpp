#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico/util/queue.h"
#include "gps.h"
#include <math.h>

// scaling factor from 1e-7 degrees to meters at equater
// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
#define LOCATION_SCALING_FACTOR 0.011131884502145034f
#define DEG_TO_RAD_FOR_GPS 0.017453292519943295769236907684886f
#define GPS_UART_ID uart1


extern queue_t uart1Queue ;

// RX interrupt handler
void on_uart_rx() {
    while (uart_is_readable(GPS_UART_ID)) {
        uint8_t ch = uart_getc(GPS_UART_ID);
        queue_try_add ( &uart1Queue , &ch);
    }
}

GPS::GPS( void) {}

void GPS::setupGps(void){
    
    uart_init(GPS_UART_ID, 9600);   // setup UART1 at 9600 baud
    uart_set_hw_flow(GPS_UART_ID, false, false);// Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_fifo_enabled(GPS_UART_ID, false);    // Turn off FIFO's - we want to do this character by character
    
    gpio_set_function(4, GPIO_FUNC_UART); // Set the GPIO pin mux to the UART - 4 is TX, 5 is RX
    gpio_set_function(5, GPIO_FUNC_UART);

    const static uint8_t initGps1[] = { 
    // send command to GPS to change the setup
    // Here the code to activate galileo sat. (This has not yet been tested and is based on I-NAV code)
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

// Here other code        
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
    uint8_t initGpsIdx = 0 ;
     busy_wait_us(3000000) ; // wait to be sure that GPS has started
    while (initGpsIdx < sizeof( initGps1)) {
//    Serial.println( pgm_read_byte_near(initGps1 + initGpsIdx ), HEX) ;    
        uart_putc_raw(GPS_UART_ID , initGps1[initGpsIdx++]);
         busy_wait_us(1000) ;
    }
    busy_wait_us(100000) ; 
    printf("End of GPS setup");
    // change the baudrate of UART1 to the new rate
    uart_set_baudrate(GPS_UART_ID , 38400);
    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART1_IRQ, on_uart_rx);
    irq_set_enabled(UART1_IRQ, true);
    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(GPS_UART_ID, true, false);
    // clear the input queue that is filled by the interrupt
    queue_init(&uart1Queue , sizeof(uint8_t) ,256) ;// queue for uart1 with 256 elements of 1
     busy_wait_us(1000);
    uint8_t dummy;
    while (! queue_is_empty (&uart1Queue)) queue_try_remove ( &uart1Queue , &dummy ) ;
}  // end setupGPS

void GPS::readGps() { // read and process GPS data. do not send them.
    if ( queue_is_empty (&uart1Queue)) return;
    uint8_t data;
    queue_try_remove ( &uart1Queue , &data ) ;
    bool parsed = false;
//    printer->print(_step);  printer->print(" "); printer->println(data , HEX);
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
            if (UBLOX_parse_gps() && (_class == 0x01) ) {
                parsed = true; 
            }
    }  // end of case
}

bool GPS::UBLOX_parse_gps(void) // move the data from buffer to the different fields
{
    // do we have new position information?
    new_position = false ;
    new_speed = false ;
    static bool next_fix = false ;
  
    switch (_msg_id) {
    case MSG_POSLLH:
        //i2c_dataset.time                = _buffer.posllh.time;
        GPS_lon = _buffer.posllh.longitude;           // in degree with 7 decimals
        GPS_lat = _buffer.posllh.latitude;            // in degree with 7 decimals
        GPS_altitude = _buffer.posllh.altitude_msl ;  //alt in mm
        if (next_fix) {                               // enable state if a position has been received after a positieve STATUS or SOL
            GPS_fix = true ;
            if ( GPS_home_lat == 0 ) { 
              GPS_home_lat = _buffer.posllh.latitude ;  // save home position
              GPS_home_lon = _buffer.posllh.longitude ;
              GPS_scale = cosf(GPS_home_lat * 1.0e-7f * DEG_TO_RAD_FOR_GPS); // calculate scale factor based on latitude
            }
            // Calculate distance
            float dlat  = (float)(GPS_home_lat - GPS_lat);
            float dlong  = ((float)(GPS_home_lon - GPS_lon)) * GPS_scale ;
            GPS_distance =  sqrtf( dlat * dlat + dlong * dlong  ) * LOCATION_SCALING_FACTOR;
            // calculate bearing
            int32_t off_x = GPS_lon - GPS_home_lon ;
            int32_t off_y = (GPS_lat - GPS_home_lat) / GPS_scale ;
            GPS_bearing = 90 + atan2f(-off_y, off_x) * 57.2957795f;  // in degree
            if (GPS_bearing < 0) GPS_bearing += 360;
#if defined( A_LOCATOR_IS_CONNECTED)  && ( A_LOCATOR_IS_CONNECTED == YES)
            GPS_last_fix_millis = millis() ;  // used by lora locator
            GPS_last_fix_lon = GPS_lon ;      // used by lora locator
            GPS_last_fix_lat = GPS_lat ;      // used by lora locator
#endif
        } else {
            GPS_fix = false;
        }
        GPS_lonAvailable = GPS_latAvailable = GPS_altitudeAvailable = GPS_fix; 
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
        GPS_numSat = _buffer.solution.satellites;  
        GPS_hdop = _buffer.solution.position_DOP;  
        break;
    case MSG_VELNED:   
        GPS_speed_3d  = _buffer.velned.speed_3d;  // cm/s
        GPS_speed_3dAvailable = GPS_fix ;
        GPS_speed_2d = _buffer.velned.speed_2d;    // cm/s
        GPS_speed_2dAvailable = GPS_fix ;
        GPS_ground_course = _buffer.velned.heading_2d ;     // Heading 2D deg with 5 decimals
        GPS_ground_courseAvailable = GPS_fix ;
        new_speed = true;
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
