#pragma once

#include <stdint.h>
#define VERSION "0.4.0"
// ------- General ------------------
// This project can be interfaced with one or 2 ELRS, JETI or FRSKY receiver(s) (protocol has to be selected accordingly)
// 
// This project is foreseen to generate telemetry data (e.g. when a flight controller is not used) , PWM and/or Sbus signals
// For telemetry, it can provide
//    - up to 4 analog voltages measurement (with scaling and offset) (optional)
//    - one RPM measurement; a scaling (SCALE4) can be used to take care e.g. of number of blades (optional)
//    - the altitude and the vertical speed when connected to a pressure sensor (optional)
//    - GPS data (longitude, latitude, speed, altitude,...) (optional)
//
// It can also provide up to 16 PWM RC channels from a CRSF/ELRS or from a Sbus signal (e.g Frsky or Jeti).
// It can also provide SBUS signal. 
//
// When connected to 2 receivers, the generated PWM and Sbus signals will be issued from the last received Rc channels (= diversity)
//
// Each functions (telemetry/PWM/SBUS) can be used alone or combined with the others.
// -------  Hardware -----------------
// This project requires a board with a RP2040 processor (like the rapsberry pi pico).
// A better alternative is the RP2040-Zero (same processor but smaller board)
// This board can be connected to:
//    - a pressure sensor (GY63 or GY86 board based on MS5611, SPL06 or BMP280) to get altitude and vertical speed
//    - a GPS from UBlox (like the beitian bn220) or one that support CASIC messages
//       note : a Ublox GPS has to use the default standard config. It will be automatically reconfigure by this firmware
//              a CASIC gps has to be configured before use in order to generate only NAV-PV messages at 38400 bauds
//             this can be done using a FTDI and the program GnssToolkit3.exe (to download from internet)
//    - some voltage dividers (=2 resistors) when the voltages to measure exceed 3V
//       note : a voltage can be used to measure e.g. a current when some external devices are use to generate an analog voltage 
//
// ----------Wiring --------------------
// FRSKY/ELRS/JETI receiver, MS5611 and GPS must share the same Gnd
// Connect a 5V source to the Vcc pin of RP2040 board
// There is no default affectation of the RP2040 pins so user has to specify it with some parameters after flashing the firmware (see below)
// When used with a ELRS receiver:
//    - Connect PRIMARY/SECONDARY RC Channel pin(s) to the TX pin from ELRS receiver (this wire transmit the RC channels)
//    - Connect TLM pin to the Rx pin from ELRS receiver that is supposed to transmit telemetry data (this wire transmits the telemetry data)
// When used with a FRSKY/JETI receiver:
//    - Connect PRIMARY/SECONDARY RC Channel pin(s) to the Sbus pin from Frsky/Jeti receiver (this wire transmit the RC channels)
//    - Connect TLM pin via a 1k resistor to the Sport pin from Frsky receiver or EX pin from Jeti receiver (this wire transmits the telemetry data)
//
// Up to 16 PWM signals can be generated on pin gpio 0...15 (to select in setup parameters). 
//
// Voltages 1, 2, 3, 4 can be measured on gpio 26...29 
//       Take care to use a voltage divider (2 resistances) in order to limit the voltage on those pins to 3V max 
//
// One RPM (Hz) can be measured
//       Take care to limit the voltage to the range 0-3V; so if you use capacitor coupling, add diodes and resistor to limit the voltage
//       All pulsed are counted (no debouncing); so use a hardware low pass filter (resistor/capitor) to avoid dummy pulses reading
//
// When a MS5611/SPL06/BMP280 (baro sensor) is used:
//       Connect the 3V pin from RP2040 board to the 5V pin of GY63/GY86 or the Vcc from other sensor 
//            Note: do not connect 5V pin of GY63/GY86 to a 5V source because the SDA and SCL would then be at 5V level and would damage the RP2040          
//       Connect SCL from baro sensor to the pin selected as SCL in parameter for RP2040
//       Connect SDA from baro sensor to the pin selected as SDA in parameter for RP2040
//
// When a GPS is used:
//    Connect the 3V pin from RP2040 board to the Vin/5V pin from GPS
//    Connect the RX pin from GPS to the RX pin selected in parameter for RP2040  
//    Connect the TX pin from GPS to the TX pin selected in parameter for RP2040
//    So take care that here TX is NOT connected to RX (as usual in Serial connection)  
//       
// The affectation of the pins has to be defined by the user.
// Here are the command codes and the pins that can be used are:
// Note: pin 16 is reserved for an internal LED on RP2040-zero as so should not be used.
// C1 = 0/15  ... C16 = 0/15     (for PWM output)
// GPS_TX = 0/29                 (for GPS)
// GPS_RX = 0/29                 (for GPS)
// PRI = 5 ,9, 21 ,25            (for primary RC channel input)
// SEC = 1, 13 , 17 ,29          (for secondary RC channel input)
// SBUS_OUT = 0/29               (for Sbus output)
// TLM = 0/29                    (for telemetry data)
// VOLT1= 26/29 ... VOLT4 = 26/29  (for voltage measurements)
// SDA = 2, 6, 10, 14, 18, 22, 26  (for baro)
// SCL = 3, 7, 11, 15, 19, 23, 27  (for baro)
// RPM = 0/29                      (for RPM)
// LED = 16                        (internal led of RP2040-zero)
//
// --------- software -------------------
//    This software has been developped using the RP2040 SDK provided by Rapsberry.
//    It uses as IDE platformio and the WIZIO extension (to be found on internet here : https://github.com/Wiz-IO/wizio-pico )
//    Developers can compile and flash this software with those tools.
//    Still if you just want to use it, there is no need to install/use those tools.
//    On github, in uf2 folder, there is already a compiled version of this software that can be directly uploaded and configured afterwards
//    To upload this compiled version, the process is the folowing:
//        - download the file in folder uf2 on your pc
//        - insert the USB cable in the RP2040 board
//        - press on the "boot" button on the RP2040 board while you insert the USB cable in your PC.
//        - this will enter a special bootloader mode and your pc should show a new drive named RPI-RP2
//        - copy and paste the uf2 file to this new drive
//        - the file should be automatically picked up by the RP2040 bootloader and flashed
//        - the RPI_RP2 drive should disapear from the PC and the PC shoud now have a new serial port (COMx on windows)
//        - you can now use a serial terminal (like putty , the one from arduino IDE, ...) and set it up for 115200 baud 8N1
//        - while the RP2040 is connected to the pc with the USB cable, connect this serial terminal to the serial port from the RP2040
//        - when the RP2040 start (or pressing the reset button), press Enter and it will display the current configuration and the commands to change it.
//        - if you want to change some parameters, fill in the command (code=value) and press the enter.
//        - the RP2040 should then display the new (saved) config.  
//
// Notes:
// The RP2040 send the telemetry data to the ELRS receiver at some speed.
// This speed (=baud rate) must be the same as the baudrate defined on the receiver
// Usually ELRS receiver uses a baudrate of 420000 to transmit the CRSF channels signal to the flight controller and to get the telemetry data.
// Still, ELRS receivers can be configured to use another baud rate. In this case, change the baudrate in parameters accordingly
//
// The number of data that ELRS can send back per second to the transmitter is quite limitted (and depends on your ELRS setup)
// There are 5 types of frame being generated (voltage, gps, vario, attitude and baro_altitude)
// Following #define allow to set up the interval between 2 frames of the same group.
// This allows e.g. to transmit vertical speed (in 'vario' frame) more often than GPS data
// The values are in milli seconds




#define VOLTAGE_FRAME_INTERVAL 500 // This version transmit only one voltage; it could be change in the future
#define VARIO_FRAME_INTERVAL 50   // This frame contains only Vertical speed
#define GPS_FRAME_INTERVAL 500     // This frame contains longitude, latitude, altitude, ground speed, heading and number of satellites
#define ATTITUDE_FRAME_INTERVAL 500 // This should normally contains pitch, roll and yaw. It is currently not used in this project.
#define BARO_ALTITUDE_FRAME_INTERVAL 500 // This frame contains only barometric relative altitude

#define RPM_COUNTER_INTERVAL_USEC 100000 // 100 msec
// Note: ELRS has no field to transmit RPM; so RPM is sent in "attitude" frame as pitch.  
//       This field has a max value of about 32000 and some digits are considered as decimals and lost by openTx.
//       So you probably have to apply a rpm multiplier to get a value in a range that fits your need and has enough signicant digits displayed

// Here some additional parameters that can't be changed via the serial terminal 
// -------- Parameters for the vario -----
#define SENSITIVITY_MIN 100
#define SENSITIVITY_MAX 300
#define SENSITIVITY_MIN_AT 100
#define SENSITIVITY_MAX_AT 1000
#define VARIOHYSTERESIS 5

// --------- Parameters for GPS ---------------
#define GPS_REFRESH_RATE 10 // For Ublox GPS, it is possible to select a refresh rate of 1Hz, 5Hz (defeult) or 10Hz 
//                        note :a casic gps has to be configured before use in order to generate only NAV-PV messages at 38400 bauds
//                        this can be done using a FTDI and program GnssToolkit3.exe (to download from internet)

// --------- 10 - Reserved for developer. DEBUG must be activated here when we want to debug one or several functions in some other files. ---------

typedef struct {
  uint8_t available ;
  int32_t value ;
} oneMeasurement_t;

// I activate this when I buid a device for Sport with voltage measured on VOLT2 instead of VOLT1 (because it is easier to solder the resistor to Grnd)
//#define SKIP_VOLT1_3_4


