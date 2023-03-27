#pragma once

#include <stdint.h>
#define VERSION "1.9.12"

#define DEBUG  // force the MCU to wait for some time for the USB connection; still continue if not connected

// Here some additional parameters that can't be changed via the serial terminal 

// -----------  for Sport protocol -------------------------------
#define SPORT_DEVICEID    DATA_ID_FAS  // this line defines the physical ID used by sport. 

// default SPORT_SENSOR_ID use by some original frsky sensors
#define DATA_ID_VARIO  0x00  // = sensor 0 used for Vspeed, GPS long and lat as P1
#define DATA_ID_FLVSS  0xA1  //          1 used as P2
#define DATA_ID_FAS    0x22  //          2
#define DATA_ID_GPS    0x83  //          3 used as P3
#define DATA_ID_RPM    0xE4  //          4
#define DATA_ID_ACC    0x67  //          7
//list of all possible 28 device ID codes (in sequence)
// 0x00,0xA1,0x22,0x83,0xE4,0x45,0xC6,0x67,0x48,0xE9,0x6A,0xCB,0xAC,0x0D,0x8E,0x2F, 
// 0xD0,0x71,0xF2,0x53,0x34,0x95,0x16,0xB7,0x98,0x39,0xBA,0x1B



// -------------- for ELRS protocol  ------------------------------
#define VOLTAGE_FRAME_INTERVAL 500 // This version transmit only one voltage; it could be change in the future
#define VARIO_FRAME_INTERVAL 50   // This frame contains only Vertical speed
#define GPS_FRAME_INTERVAL 500     // This frame contains longitude, latitude, altitude, ground speed, heading and number of satellites
#define ATTITUDE_FRAME_INTERVAL 500 // This should normally contains pitch, roll and yaw.
#define BARO_ALTITUDE_FRAME_INTERVAL 500 // This frame contains only barometric relative altitude


// -------------- for Futaba Sbus2 protocol -----------------------
// For temperature, oXs emulates a SBS/01T
// For RPM, oXs emulates a SBS/01RO or 01RM
// For vario, oXs emulates a F1672 or F1712
// For Volt, current,capacity, oXs emulates SBS/01C ; seems to the same as F-1678
// For GPS, oXs emulates a SBS/01G
// V3 and V4 can be get using F125 or F1713 

// here you can define the slot being used by different sensors
#define SBUS2_SLOT_GPS_8  8   // this must be 8,16 or 24 because GPS requires 8 slots
#define SBUS2_SLOT_VARIO_2 16 // Vario requires 2 lots (Alt and Vspeed)
#define SBUS2_SLOT_BATTERY_3 18  // Battery requires 3 slots (volt, current, capacity)
#define SBUS2_SLOT_TEMP1_1 21    // Temp1 requires 1 slot
#define SBUS2_SLOT_TEMP2_1 22    // Temp2 requires 1 slot
#define SBUS2_SLOT_RESERVE1_1 23    // V3 requires 1 slot, emulate F1713
#define SBUS2_SLOT_RESERVE2_1 24    // V4 requires 1 slot, emulate F1713
#define SBUS2_SLOT_RPM_1 25    // Rpm requires 1 slot
#define SBUS2_SLOT_AIRSPEED_1 26    // Airspeed requires ? slot

#define SBUS2_SLOT_HOLD_COUNTER_1 28    // Count the number of hold frames ; require 1 slot; emulate F1713
#define SBUS2_SLOT_FAILSAFE_COUNTER_1 29    // Count the number of failsafeframes ; require 1 slot; emulate F1713

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

// --------- Parameter for RPM -------------------
#define RPM_COUNTER_INTERVAL_USEC 100000 // 100 msec

// --------- Parameters for Ads1115 ----------------
#define I2C_ADS_Add1 0x48 // I2C address of ads1115 when addr pin is connected to ground
#define I2C_ADS_Add2 0x49 // I2C address of ads1115 when addr pin is connected to vdd

#define ADS1_MEASURE A0_TO_GND ,  A1_TO_GND , A2_TO_GND , A3_TO_GND // select 4 values between A0_TO_A1, A0_TO_A3, A1_TO_A3, A2_TO_A3, A0_TO_GND, A1_TO_GND, A2_TO_GND, A3_TO_GND, ADS_OFF
#define ADS1_FULL_SCALE_VOLT  MV4096, MV4096, MV4096, MV4096 //  select between MV6144 MV4096 MV2048 MV1024 MV512 MV256
#define ADS1_OFFSET 0.0, 0.0 , 0.0 , 0.0 // can be a float (positive or negative)
#define ADS1_SCALE 1.0, 1.0, 1.0, 1.0 // can be a float
#define ADS1_RATE  MS5 , MS5, MS5 , MS5 // select between MS137, MS69, MS35, MS18, MS9, MS5, MS3 , MS2
#define ADS1_AVERAGING_ON 10 , 10, 10, 10 // number of values used for averaging (must be between 1 and 254) 

#define ADS2_MEASURE A0_TO_GND ,  A1_TO_GND , A2_TO_GND , A3_TO_GND // select 4 values between A0_TO_A1, A0_TO_A3, A1_TO_A3, A2_TO_A3, A0_TO_GND, A1_TO_GND, A2_TO_GND, A3_TO_GND, ADS_OFF
#define ADS2_FULL_SCALE_VOLT  MV4096, MV4096, MV4096, MV4096 //  select between MV6144 MV4096 MV2048 MV1024 MV512 MV256
#define ADS2_OFFSET 0.0, 0.0 , 0.0 , 0.0 // can be a float (positive or negative)
#define ADS2_SCALE 1.0, 1.0, 1.0, 1.0 // can be a float
#define ADS2_RATE  MS5 , MS5, MS5 , MS5 // select between MS137, MS69, MS35, MS18, MS9, MS5, MS3 , MS2
#define ADS2_AVERAGING_ON 10 , 10, 10, 10 // number of values used for averaging (must be between 1 and 254) 

// --------- Parameters for MS4525 ----------------

#define MS4525_ADDRESS 0X28 // 0x28 is the default I2C adress of a 4525DO sensor ; 0X46 is an alternative

// --------- Parameters for SDP3x ----------------

#define SDPXX_ADDRESS 0X21 // 0x21 is the default I2C adress of a SDP3X sensor
//#define SDPXX_ADDRESS   0x25 // 0x25 is the I2C adress of a SDP810 sensor

// --------- Reserve for developer. ---------

typedef struct {
  int32_t value ;
  uint8_t available ;
} oneMeasurement_t;

//Note:
// I activate this when I buid a device for Sport with voltage measured on VOLT2 instead of VOLT1 (because it is easier to solder the resistor to Grnd)
//#define SKIP_VOLT1_3_4


