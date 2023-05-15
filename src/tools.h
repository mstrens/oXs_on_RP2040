#pragma once

#include "pico/stdlib.h"
//#include "Button2.h" moved to Button2.cpp
//#include "stdio.h"


struct field {
    int32_t value;
    bool available;
    bool onceAvailable;
} ;



enum fieldIdx {     // Internal Id for the measurements stored in oXs and that can be sent (some in a different format/field)
      LATITUDE =0,  //  GPS special format
      LONGITUDE,    //  GPS special format
      GROUNDSPEED , //  GPS cm/s
      HEADING,      //  GPS 0.01 degree
      ALTITUDE ,    //  GPS cm

      NUMSAT ,      //  5 GPS no unit   
      GPS_DATE ,    // GPS special format AAMMJJFF
      GPS_TIME ,    // GPS special format HHMMSS00
      GPS_PDOP ,    // GPS no unit
      GPS_HOME_BEARING, // GPS degree

      GPS_HOME_DISTANCE, // 10 GPS  in m
      MVOLT,        // volt1   in mVolt
      CURRENT,  // volt2 must be in seq for voltage.cpp in mA (mV)
      RESERVE1, // volt3 must be in seq for voltage.cpp in mV
      RESERVE2, // volt4 must be in seq for voltage.cpp in mV
      
      CAPACITY,    // based on current (volt2) in mAh
      TEMP1,       // = Volt3 but saved as temp in degree
      TEMP2,       // = Volt4 but saved as temp in degree
      VSPEED,      // baro       in cm/s
      RELATIVEALT , // baro      in cm
      
      PITCH,       // 20 imu        in degree 
      ROLL,        // imu           in degree
      YAW ,        // not used to save data  in degree
      RPM ,        // RPM sensor    in Herzt
      ADS_1_1,      // Voltage provided by ads1115 nr 1 on pin 1

      ADS_1_2,      // Voltage provided by ads1115 nr 1 on pin 2    25
      ADS_1_3,      // Voltage provided by ads1115 nr 1 on pin 3
      ADS_1_4,      // Voltage provided by ads1115 nr 1 on pin 4
      ADS_2_1,      // Voltage provided by ads1115 nr 2 on pin 1
      ADS_2_2,      // Voltage provided by ads1115 nr 2 on pin 2
      
      ADS_2_3,      // Voltage provided by ads1115 nr 2 on pin 3    30
      ADS_2_4,      // Voltage provided by ads1115 nr 2 on pin 4
      AIRSPEED,
      AIRSPEED_COMPENSATED_VSPEED,
      SBUS_HOLD_COUNTER,

      SBUS_FAILSAFE_COUNTER,                                        // 35        
      GPS_CUMUL_DIST,   
      NUMBER_MAX_IDX, // used to count the number of entries       
};
// note : when a new field is added to this list we have to change also:
//    - in tools.cpp to add the positive and the negative values for FVP and FVN commands
//    - in param.cpp to add the text (printf) for FV command (adding case...) 
//    - in sport.cpp in setupSportList() there are several tables
//    - in ibus.cpp there is a list to complete
//    - in sbus2_tlm.cpp add code (for a slot)
//    - in config.h , add slot for sbus2
//    - in exbus.cpp , adapt one table and program (name and unit)
//    - For mpx there is also some check to be done (???)

#define SAVE_CONFIG_ID 0XFF
#define CAMERA_PITCH_ID 0XFE
#define CAMERA_ROLL_ID 0XFD


int32_t int_round(int32_t n, uint32_t d);

uint32_t millisRp() ;

uint32_t microsRp();

void waitUs(uint32_t delayUs);

bool __no_inline_not_in_flash_func(get_bootsel_button)();

typedef struct
{
    uint8_t type;
    int32_t data;
} queue_entry_t;

void sent2Core0( uint8_t fieldType, int32_t value);

void enlapsedTime(uint8_t idx);
void startTimerUs(uint8_t idx); 
void getTimerUs(uint8_t idx);

void calculateAirspeed();

void fillFields( uint8_t forcedFields);

uint16_t swapBinary(uint16_t value) ;

int16_t swapBinary(int16_t value) ;

uint32_t swapBinary(uint32_t value) ;

int32_t swapBinary(int32_t value) ;


#define DO_DEBUG
extern int debug ; // set from conf file at runtime
#ifdef DO_DEBUG
  #define debugP(fmt, ...)  do { if(debug == 1){printf((fmt), ##__VA_ARGS__);} } while (0)
  #define debugAX(txt , a, n)  do { if(debug == 1){printf((txt)); for(uint8_t i = 0; i < n; i++){\
    printf(" %2X",a[i]);} printf("\n");} } while (0)
  
#else
   #define debugP(fmt, ...){}
//   #define debugAX(fmt , uint8_t * a[], uint8_t n) {}
#define debugAX(txt , a, n) {} 
#endif
//  #define debug_print(fmt, ...)  do { if(debug == 1){plog(__FILE__, ___FUNCTION__, __LINE__, ((fmt)), ##__VA_ARGS__);} } while (0)
//#define debug_print(fmt, ...) \
//            do { if (DEBUG) fprintf(stderr, fmt, ##__VA_ARGS__); } while (0)


