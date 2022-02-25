#pragma once
// ------- General ------------------
// This project is foreseen to generate telemetry data to a ELRS receiver when a flight controller is not use
// It can provide
//    - up to 4 analog voltages
//    - the altitude and the vertical speed when connected to a pressure sensor (optional)
//    - GPS data (longitude, latitude, speed, altitude,...) (optional)
// -------  Hardware -----------------
// This project requires a board with a RP2040 processor (like the rapsberry pi pico).
// A better alternative is the RP2040-Zero (same processor but smaller board)
// This board can be connected to:
//    - a pressure sensor (GY63 or GY86 board based on MS5611) to get altitude and vertical speed
//    - a GPS from UBlox (like the beitian bn220)
//    - some voltage dividers (=2 resistors) when the voltages to measure exceed 3V
// ----------Wiring --------------------
// ELRS receiver, MS5611 and GPS must share the same Gnd
// Connect a 5V source to the Vcc pin of RP2040 board
// Connect gpio 8 from RP2040 (= TX signal) to the RX pin from ELRS receiver (this wires transmit the telemetry data)
//
// When a board like GY63 or GY86 is used as pressure sensor for altitude and vertical speed:
//    Connect the 3V pin from RP2040 board to the 5V pin of GY63/GY86 
//       Note: do not connect 5V pin of GY63/GY86 to a 5V source because the SDA and SCL would then be at 5V level and would damage the RP2040
//    Connect gpio 2 from RP2040 to SDA from MS5611
//    Connect gpio 3 from RP2040 to SCL from MS5611
//
// When a GPS is used:
//    Connect the 3V pin from RP2040 board to the Vin/5V pin from GPS
//    Connect gpio 4 (UART1-TX) from RP2040 board to the RX pin from GPS
//    Connect gpio 5 (UART1-RX) from RP2040 board to the TX pin from GPS
//
// To measure voltages, connect GPIO 26, 27, 28 and/or 29 to the sources to be measured
// Take care to use a voltage divider (2 resistances) in order to limit the voltage on those pins to 3V max 
//
//
// --------- software -------------------
// The RP2040 send the telemetry data to the ELRS receiver at some speed.
// This speed (=baud rate) must be the same as the baudrate defined on the receiver
// Usually ELRS receiver uses a baudrate of 420000 to transmit the CRSF channels signal to the flight controller and to get the telemetry data.
// Still, ELRS receivers can be configured to use another baud rate.
// When ELRS receiver generates a Sbus signal instead of a CRSF, then the baud rate is (normally) 1000000
#define SERIAL_BAUD_ELRS 115200
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
 
// --------- 4 - Vario settings ---------

// ***** 4.1 - Connecting 1 or 2 MS5611 barometric sensor *****
#define FIRST_BARO_SENSOR_USE   MS5611 // set as comment if there is no vario; in a future version, it could be an BMP280 sensor
                                      //  in a future version, it could be that we support 2 sensors simultanously                      

// ***** 4.2 - Sensitivity predefined by program *****
#define SENSITIVITY_MIN 50
#define SENSITIVITY_MAX 300
#define SENSITIVITY_MIN_AT 100
#define SENSITIVITY_MAX_AT 1000

// ***** 4.4 - Hysteresis parameter *****
#define VARIOHYSTERESIS 5

// --------- 6 - Voltages & Current sensor settings ---------
#define ARDUINO_MEASURES_VOLTAGES YES

// ***** 6.2 - Voltage parameters *****
// Each of following lines contains 4 parameters, the first value is for VOLT_1, the second for VOLT_2,  
#define PIN_VOLTAGE        26  , 0     , 0   , 0 //  Fill all 4 values; set to 26 up to 29 ; set the value to 0 for the voltage(s) not to be measured.
#define RESISTOR_TO_GROUND  0 , 10    , 10  , 10                // set value to 0 when no divider is used for a voltage; can contains decimals 
#define RESISTOR_TO_VOLTAGE 0 , 8.7 , 22 , 27                // set value to 0 when no divider is used for a voltage; can contains decimals 
#define SCALE_VOLTAGE       1.00 , 1.0   , 1.0  , 1.0               // optionnal, can be negative, can have decimals

// --------- 9 - GPS ---------------                                               see oXs_config_advanced.h for additionnal parameters (normally no need to change them)
#define A_GPS_IS_CONNECTED      YES                   // select between YES , NO
#define GPS_REFRESH_RATE 5 // it is possible to select a refresh rate of 1Hz, 5Hz (defeult) or 10Hz 

// --------- 10 - Reserved for developer. DEBUG must be activated here when we want to debug one or several functions in some other files. ---------

typedef struct {
  uint8_t available ;
  int32_t value ;
} oneMeasurement_t;


#if defined(FIRST_BARO_SENSOR_USE) && (FIRST_BARO_SENSOR_USE == MS5611)
#define VARIO1 MS5611// set as comment if there is no vario; in a future version, it could be an BMP280 sensor
#endif

#define YES 1
#define NO  0

#define DEBUG

