// OpenXsensor https://code.google.com/p/openxsensor/
// started by Rainer Schloßhan

//***********************************************************************************************************************
// Another file in this project (see oXs_config_description.h) provides detailed explanations on how to set up this file.
//***********************************************************************************************************************
#pragma once
#ifndef OXS_CONFIG_h
#define OXS_CONFIG_h

// --------- 4 - Vario settings ---------

// ***** 4.1 - Connecting 1 or 2 MS5611 barometric sensor *****
#define VARIO1 MS5611// set as comment if there is no vario
//#define VARIO2 // set as comment if there is no second vario

// ***** 4.2 - Sensitivity predefined by program *****
#define SENSITIVITY_MIN 50
#define SENSITIVITY_MAX 300
#define SENSITIVITY_MIN_AT 100
#define SENSITIVITY_MAX_AT 1000

// ***** 4.3 - Sensitivity adjusted from the TX *****
#define SENSITIVITY_MIN_AT_PPM 10    // sensitivity will be changed by OXS only when PPM signal is between the specified range enlarged by -5 / +5
#define SENSITIVITY_MAX_AT_PPM 40
#define SENSITIVITY_PPM_MIN  20      // common value for vario is 20
#define SENSITIVITY_PPM_MAX 100      // common value for vario is 100

// ***** 4.4 - Hysteresis parameter *****
#define VARIOHYSTERESIS 5

// ***** 4.5 - Vertical speeds calculations *****
#define VARIO_PRIMARY       2        // 0 means first ms5611, 1 means second ms5611 , 2 means vario based on vario 1 + compensation from airspeed
#define VARIO_SECONDARY     0        // 0 means first ms5611, 1 means second ms5611 , 2 means vario based on vario 1 + compensation from airspeed
#define SWITCH_VARIO_MIN_AT_PPM 10
#define SWITCH_VARIO_MAX_AT_PPM 90

// ***** 4.6 - Analog vertical speed *****
//#define PIN_ANALOG_VSPEED 3
#define ANALOG_VSPEED_MIN -3
#define ANALOG_VSPEED_MAX  3


// --------- 6 - Voltages & Current sensor settings ---------
#define ARDUINO_MEASURES_VOLTAGES YES
// ***** 6.1 - Voltage Reference selection (VCC or 1.1V internal) *****
//#define USE_INTERNAL_REFERENCE

// ***** 6.2 - Voltage parameters *****
// Each of following lines contains 6 parameters, the first value is for VOLT_1, the second for VOLT_2, ... up to the sixth for VOLT_6 
#define PIN_VOLTAGE        26  , 27     , 28   , 29                   //  Fill all 6 values; set to 0 up to 7 for analog pins A0 up to A7 ; set the value to 8 for the voltage(s) not to be measured.
#define RESISTOR_TO_GROUND  2.95 , 10    , 10  , 10                // set value to 0 when no divider is used for a voltage; can contains decimals 
#define RESISTOR_TO_VOLTAGE 46.9 , 8.7 , 22 , 27                // set value to 0 when no divider is used for a voltage; can contains decimals 
//#define OFFSET_VOLTAGE      0   , 0     , 0    , 0                   // optionnal, can be negative, must be integer, in principe in mv
#define SCALE_VOLTAGE       1.00 , 1.0   , 1.0  , 1.0               // optionnal, can be negative, can have decimals

// ***** 6.3 - Max number of Lipo cells to measure (and transmit to Tx) *****      Is defined only in oXs_config_basic.h file

// ***** 6.3 - Voltage measurements calibration parameters *****
#define OFFSET_1             0
#define MVOLT_PER_STEP_1       4.89
#define OFFSET_2             0
#define MVOLT_PER_STEP_2       10.1
//#define OFFSET_3             0
//#define MVOLT_PER_STEP_3       1
//#define OFFSET_4             0
//#define MVOLT_PER_STEP_4       1
//#define OFFSET_5             0
//#define MVOLT_PER_STEP_5       1
//#define OFFSET_6             0
//#define MVOLT_PER_STEP_6       1


// --------- 8 - Persistent memory settings ---------
//#define SAVE_TO_EEPROM
#define PIN_PUSHBUTTON    4   // allow to control the volume
#define PIN_BT_STATE      8
#define PIN_BT_RX         7
#define PIN_BT_TX         6
//#define PIN_BT_KEY        5
//#define USE_USB

// --------- 9 - GPS ---------------                                               see oXs_config_advanced.h for additionnal parameters (normally no need to change them)
#define A_GPS_IS_CONNECTED      NO                   // select between YES , NO


// --------- 10 - Reserved for developer. DEBUG must be activated here when we want to debug one or several functions in some other files. ---------
//#define DEBUG

//#ifdef DEBUG
//#include "HardwareSerial.h"
//#endif

typedef struct {
  uint8_t available ;
  int32_t value ;
} oneMeasurement_t;


#define YES 1
#define NO  0
#define FIRST_BARO_SENSOR_USE   MS5611
#define USE_VARIO1

#define DEBUG


#endif// End define OXS_CONFIG_h

