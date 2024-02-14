#include <stdio.h>
#include "pico/stdlib.h"
//#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico/util/queue.h"
#include "uart_gps_rx.pio.h"
#include "uart_gps_tx.pio.h"
#include "gps.h"
#include <math.h>
#include "param.h"
#include "tools.h"
#include <inttypes.h>

// scaling factor from 1e-7 degrees to centimeter meters at equater
// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
#define LOCATION_SCALING_FACTOR 1.1131884502145034f
#define DEG_TO_RAD_FOR_GPS 0.017453292519943295769236907684886f
//#define GPS_UART_ID uart0

//extern queue_t gpsQueue ;
extern CONFIG config;
extern field fields[];  // list of all telemetry fields and parameters used by Sport

queue_t gpsRxQueue ; // queue uses to push the data from the uart pio rx to the main loop


// Receive buffer for Ublox
    union {
        ubx_nav_posllh posllh;
        ubx_nav_status status;
        ubx_nav_solution solution;
        ubx_nav_pvt pvt;
        ubx_nav_velned velned;
        ubx_nav_svinfo svinfo;
        uint8_t bytes[UBLOX_BUFFER_SIZE];
    } __attribute__((__packed__))  _buffer;

union {
        casic_nav_pv_info nav_pv;
        uint8_t bytes[sizeof(casic_nav_pv_info)];
    } __attribute__((__packed__)) _casicBuffer;

uint8_t baudIdx = 0 ;

uint32_t prevRxChangeUs = 0;

PIO gpsPio = pio1; // we use pio 0; 
uint gpsSmTx = 0;  // we use the state machine 0 for Tx for a short time only (sending the config to a ublox);  
uint gpsSmRx = 0;  // we use the state machine 0 for Rx also (only when TX is removed); 

uint32_t GPS_last_fix_millis ; // time when last fix has been received (used by lora locator) 
int32_t GPS_last_fix_lon ;     // last lon when a fix has been received
int32_t GPS_last_fix_lat ;     // last lat when a fix has been received

#define CFG_RATE_MEAS 0x30210001 // U2 0.001 s Nominal time between GNSS measurements ; 100 = 10hz; 1000 = 1Hz
#define CFG_MSGOUT_UBX_NAV_POSLLH_UART1 0x2091002a // U1 - - Output rate of the UBX-NAV-POSLLH message on port UART1
#define CFG_MSGOUT_UBX_NAV_VELNED_UART1 0x20910043 // U1 - - Output rate of the UBX-NAV-VELNED message on port UART1
#define CFG_MSGOUT_UBX_NAV_PVT_UART1    0x20910007 // U1 - - Output rate of the UBX-NAV-PVT message on port UART1
#define CFG_UART1OUTPROT_NMEA 0x10740002 // L - - Flag to indicate if NMEA should be an output protocol on UART1
// Ublox and RP2040 are both little endian
#define CFG_RATE_NAV 0x30210002 // U2 - - Ratio of number of measurements to number of navigation solutions

const uint8_t initGpsM10[] = {
    0xB5,0x62,0x06,0x8A,   // config
        30, 0,  //length payload here after
        0x00,0x01,0x00,0x00,  // in ram
        0X01,0X00,0X21,0X30,   0X64 , 0X00, // key + Val in little endian for measurement rate (100 = 10Hz)
        0X2A,0X00,0X91,0X20,   0X01, // key + Val in little endian for POSLLH
        0X43,0X00,0X91,0X20,   0X01, // key + Val in little endian for VELNED
        0X07,0X00,0X91,0X20,   0X01, // key + Val in little endian for PVT
        0X02,0X00,0X74,0X10,   0X00, // L - - Flag to indicate if NMEA should be an output protocol on UART1
        0X75,0X46// checksum
};
    
const uint8_t initGpsM6Part1[] = {
    0xB5,0x62,0x06,0x00,
    0x14,0x00,
    0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x96, //        CFG-PRT : Set port to output only UBX (so deactivate NMEA msg) and set baud = 38400.
    0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00, //        and set baud = 38400.
    0x91,0x84  // Check sum                rest of CFG_PRT command                            
};

const uint8_t initGpsM6Part2[] = { 
        // send command to GPS to change the setup
    0xB5,0x62,0x06,0x00,
    0x14,0x00,
    0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x96, //        CFG-PRT : Set port to output only UBX (so deactivate NMEA msg) and set baud = 38400.
    0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00, //        and set baud = 38400.
    0x91,0x84,  // Check sum                rest of CFG_PRT command
    #if defined(GPS_REFRESH_RATE) && (GPS_REFRESH_RATE == 1)
            0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39,  // NAV-RATE for 1 hz
            0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39,  // NAV-RATE for 1 hz
    #elif defined(GPS_REFRESH_RATE) && (GPS_REFRESH_RATE == 10)
            0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, // NAV-RATE for 10 hz
            0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, // NAV-RATE for 10 hz
    #else
            0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, // NAV-RATE for 5 hz
            0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, // NAV-RATE for 5 hz
    #endif
    // Here other code        
            0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, // activate NAV-POSLLH message
            0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x06,0x00,0x01,0x00,0x00,0x00,0x00,0x17,0xDA, //        NAV-SOL
            0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x12,0x00,0x01,0x00,0x00,0x00,0x00,0x23,0x2E, //        NAV-VELNED
            0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //        NAV_PVT
            0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, // activate NAV-POSLLH message
            0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x06,0x00,0x01,0x00,0x00,0x00,0x00,0x17,0xDA, //        NAV-SOL
            0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x12,0x00,0x01,0x00,0x00,0x00,0x00,0x23,0x2E, //        NAV-VELNED
            0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //        NAV_PVT

            //0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x96, //        CFG-PRT : Set port to output only UBX (so deactivate NMEA msg) and set baud = 38400.
            //                    0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x91,0x84,  //                 rest of CFG_PRT command                            
            //0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x96, //        CFG-PRT : Set port to output only UBX (so deactivate NMEA msg) and set baud = 38400.
            //                    0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x91,0x84,  //                 rest of CFG_PRT command                            
            
            //0xB5,0x62,0x06,0x8A,
            //            13, 0,  //length 4 + payload here after
            //            0x00,0x01,0x00,0x00,  // in ram
            //            0x10,0x73,0x00,0x02,  // key  // no nema
            //            0x00 ,                // value
            //            0X23 , 0X7D           // checksum

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
            0x2F, 0xA1, // checksum

    // Here the code to activate SBAS for Europe (This has not yet been tested and is based on I-NAV code)
            0xB5,0x62,0x06,0x16, 0x08, 0x00, // SBAS + number of bytes = 8
            0x03, 0x03, 0x03, 0x00, // mode = test + enabled, usage=range+diffcorr, max =3, scanmode2=0
            0x00, 0x00, 0x08, 0x51, // scanmode1 120,124, 126, 131
            0x86, 0x2A, //checksum
*/        
        }  ;   

void uboxChecksum(){   // this function is used to calculate ublox checksum; It must be activated in a line of code below
    uint8_t buffer[]= {
        0xB5,0x62,0x06,0x00,
        0x14,0x00,
        0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x96, //        CFG-PRT : Set port to output only UBX (so deactivate NMEA msg) and set baud = 38400.
        0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,
        };
    uint8_t c1= 0;
    uint8_t c2 = 0;
    uint16_t len = sizeof(buffer);
    printf(" check sum for  ");
    for (uint16_t i= 2; i< sizeof(buffer);i++){
        c1 += buffer[i];
        c2 += c1;
        printf(" %x ",buffer[i]);
    }
    printf("\n is %X  %x\n", c1, c2);
    printf("lenght to specify is %d\n", (int) (len - 6) );
}


uint gpsOffsetTx; 

GPS::GPS( void) {}

void GPS::setupGps(void){
    if (config.pinGpsTx == 255) return; // skip if pin is not defined
    if  ( config.gpsType == 'U') {  // send gps config only for U blox with oXs setup (not for cadis nor when when ublox gps is configured externally 
    //if ( ( config.gpsType == 'U') || ( config.gpsType == 'E') ) {
        gpsOffsetTx = pio_add_program(gpsPio, &uart_tx_program); // upload the program
        uart_tx_program_init(gpsPio, gpsSmTx, gpsOffsetTx, config.pinGpsRx, 9600);
    } else {
        gpsInitRx(); // this part is common for both types of gps but can be done immediately for Cadis of when Ublox is configured externally
    }
}

void GPS::readGps(){
    if (config.pinGpsTx == 255) return; // skip if pin is not defined
    if ( ( config.gpsType == 'U')  || ( config.gpsType == 'E') ) handleGpsUblox();
    if ( config.gpsType == 'C') readGpsCasic();    
}

void gpsPioRxHandlerIrq(){    // when a byte is received on the PIO GPS, read the pio fifo and push the data to a queue (to be processed in the main loop)
  // clear the irq flag
  irq_clear (PIO1_IRQ_0 );
  while (  ! pio_sm_is_rx_fifo_empty (gpsPio ,gpsSmRx)){ // when some data have been received
     uint8_t c = pio_sm_get (gpsPio , gpsSmRx) >> 24;         // read the data
     queue_try_add (&gpsRxQueue, &c);          // push to the queue
  }
}

void GPS::gpsInitRx(){
    gpio_deinit	(config.pinGpsRx);
    gpio_init(config.pinGpsRx);  // configure TX signal on gpio as input/output
    gpio_set_dir(config.pinGpsRx, true); // set on output	
    gpio_put(config.pinGpsRx, true); // set level on high
    	                    
    // configure the queue to get the data from gps in the irq handle
    queue_init (&gpsRxQueue, sizeof(uint8_t), 500);

    // set an irq on pio to handle a received byte
    irq_set_exclusive_handler( PIO1_IRQ_0 , gpsPioRxHandlerIrq) ;
    irq_set_enabled (PIO1_IRQ_0 , true) ;
    if (pio_can_add_program(gpsPio, &uart_rx_program) == false) {
        printf("Error: can't add pio program for GPS Rx\n");
        sleep_ms(1000); // wait that message is displayed
        while (1) { ; }  // stop the program
    }
    uint gpsOffsetRx = pio_add_program(gpsPio, &uart_rx_program);
    //printf("uart rx program init will be performed\n");
    uart_rx_program_init(gpsPio, gpsSmRx, gpsOffsetRx, config.pinGpsTx, 38400);
    //printf("uart rx program init has been be performed\n");
    busy_wait_us(1000);
    uint8_t dummy;
    while (! queue_is_empty (&gpsRxQueue)) queue_try_remove ( &gpsRxQueue , &dummy ) ;
    _step = 0 ;
}

uint sendGpsConfig(const uint8_t buf[] , uint len , uint fromPos){
    uint idx = fromPos;
    while (idx < len) {
        if ( pio_sm_is_tx_fifo_empty( gpsPio, gpsSmTx )) {
            pio_sm_put (gpsPio, gpsSmTx, (uint32_t) buf[idx] );   
            //    Serial.println( pgm_read_byte_near(initGps1 + initGpsIdx ), HEX) ;    
            idx++;
            if (buf[idx] == 0XB5)  { // make a pause when there is a new command (0XB5 = begin )
            //    break; // quit the while loop
            }
        }
    } // end while
    return idx;
}

void GPS::handleGpsUblox(){
    static uint32_t lastActionUs = 0;
    if (config.pinGpsTx == 255) return;
    switch (gpsState){
        case GPS_CONFIGURED:
            readGpsUblox() ;   
            break;
        case GPS_WAIT_END_OF_RESET:
            if ( config.gpsType == 'E') {
                // gpsInitRx();                        // setup the reception of GPS char.
                gpsState = GPS_CONFIGURED;
            } else {
                if (lastActionUs == 0) {  // we make the init only once
                uart_tx_program_init(gpsPio, gpsSmTx, gpsOffsetTx, config.pinGpsRx, 9600);
                lastActionUs = microsRp();   
                }
                if ((microsRp() - lastActionUs ) > 5000000) { // wait at least  5 sec
                    //pio_sm_put (gpsPio, gpsSmTx, (uint32_t) 0 ); // send a dummy char to avoid glitch???
                    //sleep_ms(10); // wait to be sure the char is sent and line goes high again.
                    sendGpsConfig(&initGpsM6Part1[0] , sizeof(initGpsM6Part1), 0); // send in 9600 baud asking for 38400
                    
                        
                    //sleep_ms(2);
                    //sleep_ms(5);
                    //sendGpsConfig(&initGpsM6Part1[0] , sizeof(initGpsM6Part1), 0);
                    gpsState = GPS_M10_IN_RECONFIGURATION;
                    lastActionUs = microsRp();
                    //baudIdx = 0;       
                }
            }    
            break;
        case GPS_M10_IN_RECONFIGURATION:
            if ((microsRp() - lastActionUs ) > 5000) { // wait at least  4mse between baudrate change 
                uart_tx_program_init(gpsPio, gpsSmTx, gpsOffsetTx, config.pinGpsRx, 38400); 
                //sleep_ms(2); // to avoid perhaps a pulse due to change of baudrate
                //pio_sm_put (gpsPio, gpsSmTx, (uint32_t) 0 ); // send a dummy char to avoid glitch
                //sleep_ms(10); // wait to be sure the char is sent and line goes high again.
                initGpsIdx = 0; // reset on the first char of the first command to be sent
                while (initGpsIdx < sizeof( initGpsM10)) {
                    if ( pio_sm_is_tx_fifo_empty( gpsPio, gpsSmTx )) {
                        pio_sm_put (gpsPio, gpsSmTx, (uint32_t) initGpsM10[initGpsIdx] );   
                        initGpsIdx++;
                    }
                }
                gpsState = GPS_M6_IN_RECONFIGURATION;     
                lastActionUs = 0;
            }
            break; 
        case GPS_M6_IN_RECONFIGURATION:
            if (lastActionUs == 0) {  // last action = 0 means that baudrate has to be rconfigure
                //uart_tx_program_init(gpsPio, gpsSmTx, gpsOffsetTx, config.pinGpsRx, 9600);
                initGpsIdx = 0; // reset on the first char of the first command to be sent
                lastActionUs = microsRp();        
            }
            if ((microsRp() - lastActionUs ) > 40000) { // wait 40 ms between 2 commands
                if ( initGpsIdx >= sizeof( initGpsM6Part2)) { // when all bytes have been sent
                    baudIdx++;  // use next baudrate
                    if ( baudIdx >= 1){   // if text has been sent with all baudrate, we can continue
                        pio_sm_set_enabled(gpsPio, gpsSmTx, false);
                        pio_remove_program(gpsPio, &uart_tx_program, gpsOffsetTx); // remove the GPS TX program 		
                        gpsInitRx();                        // setup the reception of GPS char.
                        //printf("size of gps table=%d\n", sizeof(initGps1)); 
                        gpsState = GPS_CONFIGURED;
                        //uboxChecksum();
                    } else {
                        lastActionUs = 0;  // force setting again the baudrate (with next value)
                    }    
                }  else { // when not end of all commands
                    while (initGpsIdx < sizeof( initGpsM6Part2)) {
                        if ( pio_sm_is_tx_fifo_empty( gpsPio, gpsSmTx )) {
                            pio_sm_put (gpsPio, gpsSmTx, (uint32_t) initGpsM6Part2[initGpsIdx] );   
                            //    Serial.println( pgm_read_byte_near(initGps1 + initGpsIdx ), HEX) ;    
                            initGpsIdx++;
                            if (initGpsM6Part2[initGpsIdx] == 0XB5)  { // make a pause when there is a new command (0XB5 = begin )
                                //initGpsIdx++;
                                lastActionUs = microsRp();
                                break; // quit the while loop
                            } else {
                                //initGpsIdx++; // point to next char
                            }
                        }
                    } // end while
                }
            }
            break;
    } // end of switch
}

void GPS::readGpsUblox(){
    uint8_t data;
    if ( queue_is_empty (&gpsRxQueue)) return;
    while (!queue_is_empty(&gpsRxQueue)){
        if (queue_try_remove ( &gpsRxQueue , &data ) ){
            //printf(" %X" , data);
            //bool parsed = false;
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
                    if  (_class == 0x01) parseGpsUblox() ; // we handle only msg from class 01
                    //    parsed = true; 
                    //}
            }  // end of case
        } // end if
    } // end While     
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
        if (next_fix) {                               // enable state if a position has been received after a positieve STATUS or SOL
            GPS_fix = true ;
            sent2Core0(LONGITUDE, _buffer.posllh.longitude);           // in degree with 7 decimals
            sent2Core0(LATITUDE, _buffer.posllh.latitude);            // in degree with 7 decimals
            sent2Core0(ALTITUDE, _buffer.posllh.altitude_msl / 10);       //alt in mm in converted in cm (sport uses cm)
            //printf("POSLLH alt_msl  alt_ellipsoid = %d  %d\n", _buffer.posllh.altitude_msl / 10 , _buffer.posllh.altitude_ellipsoid /10);
            if ( GPS_home_lat == 0 ) { 
              GPS_home_lat = _buffer.posllh.latitude ;  // save home position
              GPS_home_lon = _buffer.posllh.longitude ;
              GPS_last_lat = _buffer.posllh.latitude ;  // use dto calculate the cumulative distance
              GPS_last_lon = _buffer.posllh.longitude ;
              GPS_cumulativeDistCm =  0; 
              GPS_scale = cosf(GPS_home_lat * 1.0e-7f * DEG_TO_RAD_FOR_GPS); // calculate scale factor based on latitude
            }
            // Calculate distance
            sent2Core0(GPS_HOME_DISTANCE, 0.01 * GpsDistanceCm(GPS_home_lat - _buffer.posllh.latitude , GPS_home_lon - _buffer.posllh.longitude ));
             // calculate bearing
            int32_t off_x = _buffer.posllh.longitude - GPS_home_lon ;
            int32_t off_y = (_buffer.posllh.latitude - GPS_home_lat) / GPS_scale ;
            GPS_bearing = 90 + atan2f(-off_y, off_x) * 57.2957795f;  // in degree
            if (GPS_bearing < 0) GPS_bearing += 360;
            sent2Core0(GPS_HOME_BEARING, (int32_t) GPS_bearing);
            // calculate cumulative flow
            int32_t deltaDistanceCm = GpsDistanceCm(GPS_last_lat - _buffer.posllh.latitude , GPS_last_lon - _buffer.posllh.longitude );
            if (deltaDistanceCm > 200 && deltaDistanceCm < 100000){
                GPS_cumulativeDistCm += deltaDistanceCm;
                GPS_last_lat = _buffer.posllh.latitude;
                GPS_last_lon = _buffer.posllh.longitude;
            }
            sent2Core0(GPS_CUMUL_DIST, GPS_cumulativeDistCm * 0.01) ;  // store in m
            // store data used for LORA
            GPS_last_fix_millis = millisRp() ;  // used by lora locator
            GPS_last_fix_lon = _buffer.posllh.longitude ;      // used by lora locator
            GPS_last_fix_lat = _buffer.posllh.latitude ;      // used by lora locator
        } else {
            GPS_fix = false;
        }
        //fields[LONGITUDE].available = GPS_fix;           // in degree with 7 decimals
        //fields[LATITUDE].available = GPS_fix;
        //fields[ALTITUDE].available = GPS_fix; 
        new_position = true;
        break;
//    case MSG_STATUS:                              // !!!!!!!!! I do not see real need of this message because same (and more) data are in SOL, so this message is not activated in init
//        next_fix = (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.status.fix_type == FIX_3D); // si valid position and speed or 3D fix
//        if (!next_fix)
//             GPS_fix = false;
//        break;
    case MSG_SOL:                                // !!!! here we could also use vertical speed which is I4 in cm/sec)
        next_fix = (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D);
        GPS_fix_type = _buffer.solution.fix_type; // use to send or not the data in Hott and ELRS
        if (!next_fix)
             GPS_fix = false;
        //fields[NUMSAT].available = true;
        if ( _buffer.solution.fix_type == FIX_3D ) _buffer.solution.satellites += 100; // we add 100 when we have a 3d fix (for Ublox)
        sent2Core0(NUMSAT, _buffer.solution.satellites); 
        if ( _buffer.solution.fix_status & NAV_STATUS_FIX_VALID) { // PDOP is valid only when bit 0 =1  
            GPS_pdop = _buffer.solution.position_DOP;
            sent2Core0(GPS_PDOP, _buffer.solution.position_DOP);
        }
        //printf("nbr sat : %X \n", GPS_numSat) ; 
        break;
    case MSG_PVT:                                // this message does not exist in ublox6 (but SOL does not exist in ublox10)
        next_fix = (_buffer.pvt.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.pvt.fix_type == FIX_3D);
        GPS_fix_type = _buffer.pvt.fix_type; // use to send or not the data in Hott and ELRS
        if (!next_fix)
             GPS_fix = false;
        //fields[NUMSAT].value = _buffer.pvt.satellites; 
        //fields[NUMSAT].available = true;
        if ( _buffer.pvt.fix_type == FIX_3D ) _buffer.pvt.satellites += 100; // we add 100 when we have a 3d fix (for Ublox)
        sent2Core0(NUMSAT, _buffer.pvt.satellites); 
        if ( _buffer.pvt.fix_status & NAV_STATUS_FIX_VALID) { // PDOP is valid only when bit 0 =1  
            GPS_pdop = _buffer.pvt.position_DOP;
            sent2Core0(GPS_PDOP, _buffer.pvt.position_DOP);
        }
        //if (_buffer.timeutc.flag & 0b111) {
        if (( _buffer.pvt.valid & 0x03) == 0X03 ){ // if date and time are valid (bit 0 and 1 = HIGH)
            //printf("nbr sat : %X \n", GPS_numSat) ;
            gpsDate =_buffer.pvt.year % 100;
            gpsDate <<= 8;
            gpsDate += _buffer.pvt.month;
            gpsDate <<= 8;
            gpsDate += _buffer.pvt.day;
            gpsDate <<= 8;
            gpsDate += 0xFF;
            gpsTime = _buffer.pvt.hour;
            gpsTime <<= 8;
            gpsTime += _buffer.pvt.min;
            gpsTime <<= 8;
            gpsTime += _buffer.pvt.sec;
            gpsTime <<= 8;
            if ( prevGpsTime != gpsTime) {
                prevGpsTime = gpsTime; 
                sent2Core0(GPS_DATE , gpsDate);
                sent2Core0(GPS_TIME , gpsTime);
            }
        }    
        break;
    case MSG_VELNED:   
        if( GPS_fix) sent2Core0(GROUNDSPEED , _buffer.velned.speed_3d ) ; 
        //fields[GROUNDSPEED].value  = _buffer.velned.speed_3d;  // cm/s
        //fields[GROUNDSPEED].available = GPS_fix ;
        //GPS_speed_2d = _buffer.velned.speed_2d;    // cm/s
        //GPS_speed_2dAvailable = GPS_fix ;
        if( GPS_fix) sent2Core0(HEADING , _buffer.velned.heading_2d /1000 ) ; 
        //fields[HEADING].value = _buffer.velned.heading_2d /1000;     // Heading 2D deg with 5 decimals is reduced to 2 décimals
        //fields[HEADING].available = GPS_fix ;
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

/*
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
*/   

void GPS::readGpsCasic() { // read and process GPS data. do not send them.// for casic gps
    uint8_t data;
    static uint8_t _idx;
    if ( queue_is_empty (&gpsRxQueue)) return;
    if (queue_try_remove ( &gpsRxQueue , &data ) ) {
    //    if (data == 0xBA) printf("\n"); // new line when sync byte is received
    //    printf(" %x " , data );
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
    sent2Core0(NUMSAT, _casicBuffer.nav_pv.numSV);
    //fields[NUMSAT].available = true;
    if ( _casicBuffer.nav_pv.velValid >= 6) {
        GPS_speed_2d  = _casicBuffer.nav_pv.speed2D;
        GPS_speed_2dAvailable  = true;
    }    
    if ( _casicBuffer.nav_pv.velValid >= 7) {
        sent2Core0(NUMSAT,  _casicBuffer.nav_pv.numSV + 100); // add 100 if 3d fix available
        sent2Core0(GROUNDSPEED, _casicBuffer.nav_pv.speed3D * 100); // in ublox = cm/sec, in CASIC float M/sec
        int32_t casicLat = (int32_t)( _casicBuffer.nav_pv.lat* 10000000);
        int32_t casicLon = (int32_t)( _casicBuffer.nav_pv.lon* 10000000);
        
        sent2Core0(LONGITUDE, casicLon);   // in Ublox = degree with 7 decimals, in CASIC float degree 
        sent2Core0(LATITUDE, casicLat);   // in Ublox = degree with 7 decimals, in CASIC float degree
        if (_casicBuffer.nav_pv.height > 0) {
            sent2Core0(ALTITUDE,  _casicBuffer.nav_pv.height * 100) ;  // in cm : in Ublox = mm , in CASIC float m
        } else {
            sent2Core0(ALTITUDE, 0) ;
        }
        sent2Core0(HEADING, _casicBuffer.nav_pv.heading * 100) ;    // in Ublox = deg with 5 decimals,  in CASIC = float degree
        //fields[GROUNDSPEED].available  = true;
        //fields[LONGITUDE].available = true;           
        //fields[LATITUDE].available = true;
        //fields[ALTITUDE].available = true; 
        //fields[HEADING].available = true;
        if ( GPS_home_lat == 0 ) { 
              GPS_home_lat = casicLat ;  // save home position
              GPS_home_lon = casicLon ;
              GPS_last_lat = GPS_home_lat;
              GPS_last_lon = GPS_home_lon;
              GPS_cumulativeDistCm =  0;
              GPS_scale = cosf(GPS_home_lat * 1.0e-7f * DEG_TO_RAD_FOR_GPS); // calculate scale factor based on latitude
        }
        // Calculate distance
         
        float dlat  = (float)(GPS_home_lat -  casicLat);
        float dlong  = ((float)(GPS_home_lon - casicLon)) * GPS_scale ;
        GPS_distance =  sqrtf( dlat * dlat + dlong * dlong  ) * LOCATION_SCALING_FACTOR;
        // calculate bearing
        int32_t off_x = (int32_t) (_casicBuffer.nav_pv.lon * 10000000) - GPS_home_lon ;
        int32_t off_y = ((int32_t)( _casicBuffer.nav_pv.lat* 10000000) - GPS_home_lat) / GPS_scale ;
        GPS_bearing = 90 + atan2f(-off_y, off_x) * 57.2957795f;  // in degree
        if (GPS_bearing < 0) GPS_bearing += 360;
                    // calculate cumulative flow
        int32_t deltaDistanceCm = GpsDistanceCm(GPS_last_lat - casicLat , GPS_last_lon - casicLon );
        if (deltaDistanceCm > 200 && deltaDistanceCm < 100000){
            GPS_cumulativeDistCm += deltaDistanceCm;
            GPS_last_lat = casicLat ;
            GPS_last_lon = casicLon ;
        }
        sent2Core0(GPS_CUMUL_DIST, GPS_cumulativeDistCm * 0.01) ;  // store in m
        return true;
    } else {
        //fields[GROUNDSPEED].value  = 0;        // in cm/sec
        //fields[LONGITUDE].value = 0;           // in degree with 7 decimals
        //fields[LATITUDE].value = 0;            // in degree with 7 decimals
        //fields[ALTITUDE].value = 0 ;    // in mm
        //fields[HEADING].value = 0 ;     // Heading 2D deg with 5 decimals
        //fields[GROUNDSPEED].available  = false;
        //fields[LONGITUDE].available = false;           
        //fields[LATITUDE].available = false;
        //fields[ALTITUDE].available = false; 
        //fields[HEADING].available = false;
    }
    return false;
}

int32_t GPS::GpsDistanceCm(int32_t deltaLat , int32_t deltaLon){
                float deltaLatFloat = (float) deltaLat;
                float deltaLonFloat = (float) deltaLon * GPS_scale ;
                return (int32_t)  (sqrtf( deltaLatFloat * deltaLatFloat + deltaLonFloat * deltaLonFloat  ) * LOCATION_SCALING_FACTOR) ; // in cm   
}
           