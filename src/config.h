#pragma once

#include <stdint.h>
#define VERSION "0.0.1"
// ------- General ------------------
// This project is foreseen to generate telemetry data to a ELRS receiver when a flight controller is not useD
// It can provide
//    - up to 4 analog voltages
//    - the altitude and the vertical speed when connected to a pressure sensor (optional)
//    - GPS data (longitude, latitude, speed, altitude,...) (optional)


// It can also provide 8 PWM RC channels (channels 1...4 and 6...9) and a SBUS signal with all ELRS channels  
// -------  Hardware -----------------
// This project requires a board with a RP2040 processor (like the rapsberry pi pico).
// A better alternative is the RP2040-Zero (same processor but smaller board)
// This board can be connected to:
//    - a pressure sensor (GY63 or GY86 board based on MS5611) to get altitude and vertical speed
//    - a GPS from UBlox (like the beitian bn220) or one that support CASIC messages
//       note : a CASIC gps has to be configured before use in order to generate only NAV-PV messages at 38400 bauds
//             this can be done using a FTDI and the program GnssToolkit3.exe (to download from internet)
//    - some voltage dividers (=2 resistors) when the voltages to measure exceed 3V
//    - Note : a voltage can be used to measure a current 
// ----------Wiring --------------------
// ELRS receiver, MS5611 and GPS must share the same Gnd
// Connect a 5V source to the Vcc pin of RP2040 board
// Connect pin 9 from RP2040 (= PIO RX signal) to the TX pin from ELRS receiver (this wire transmit the RC channels)
// Connect pin 10 from RP2040 (= PIO TX signal) to the RX pin from ELRS receiver (this wire transmits the telemetry data)
//
// SBus signal is available on pin 0
// PWM signals (channels 1...4 and 6...9) are avaialble on pins 1...8 ()
//
// Voltages 1...4 are measured on pins 26...29 
//       Take care to use a voltage divider (2 resistances) in order to limit the voltage on those pins to 3V max 
//
// When a MS5611 (baro sensor) is used:
//       Connect the 3V pin from RP2040 board to the 5V pin of GY63/GY86 
//            Note: do not connect 5V pin of GY63/GY86 to a 5V source because the SDA and SCL would then be at 5V level and would damage the RP2040          
//       Connect SCL to pin 15 (I2C1)
//       Connect SDA to pin 14 (I2C1)
//
// When a GPS is used:

//    Connect the 3V pin from RP2040 board to the Vin/5V pin from GPS
//    Connect the RX pin from GPS to pin 12 (UART0-TX) 
//    Connect the TX pin from GPS to pin 13 (UART0-RX)
//        
// --------- software -------------------
// The RP2040 send the telemetry data to the ELRS receiver at some speed.
// This speed (=baud rate) must be the same as the baudrate defined on the receiver
// Usually ELRS receiver uses a baudrate of 420000 to transmit the CRSF channels signal to the flight controller and to get the telemetry data.
// Still, ELRS receivers can be configured to use another baud rate. In this case, change the baudrate in parameters accordingly
// E.g. when ELRS receiver generates a Sbus signal instead of a CRSF, then the baud rate is (normally) 1000000
//
// The number of data that ELRS can send back per second to the transmitter is quite limitted
// There are 4 types of frame being generated (voltage, gps, vario and attitude)
// Following #define allow to set up the interval between 2 frames of the same type
// This allows e.g. to transmit vertical speed (in 'vario' frame) more often than GPS data
// The values are in milli seconds
#define VOLTAGE_FRAME_INTERVAL 500 // This version transmit only one voltage; it could be change in the future
#define VARIO_FRAME_INTERVAL 100   // This frame contains only Vertical speed
#define GPS_FRAME_INTERVAL 500     // This frame contains longitude, latitude, altitude, ground speed, heading and number of satellites
#define ATTITUDE_FRAME_INTERVAL 300 // This should normally contains pitch, roll and yaw. Still it is reused to transmit absolute altitude and relative altitude
 
// -------- Parameters for the vario -----
#define SENSITIVITY_MIN 50
#define SENSITIVITY_MAX 300
#define SENSITIVITY_MIN_AT 100
#define SENSITIVITY_MAX_AT 1000
#define VARIOHYSTERESIS 5

// --------- Parameters for GPS ---------------
#define GPS_REFRESH_RATE 5 // For Ublox GPS, it is possible to select a refresh rate of 1Hz, 5Hz (defeult) or 10Hz 
//                        note :a casic gps has to be configured before use in order to generate only NAV-PV messages at 38400 bauds
//                        this can be done using a FTDI and program GnssToolkit3.exe (to download from internet)

// --------- 10 - Reserved for developer. DEBUG must be activated here when we want to debug one or several functions in some other files. ---------

typedef struct {
  uint8_t available ;
  int32_t value ;
} oneMeasurement_t;



#define YES 1
#define NO  0

#define DEBUG


