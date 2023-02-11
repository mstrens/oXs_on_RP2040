#pragma once

#include "pico/stdlib.h"
//#include "Button2.h" moved to Button2.cpp
//#include "stdio.h"

//#define SPORT_TYPES_MAX 24 // = NUMBER_MAX_IDX

struct field {
    int32_t value;
    bool available;
    uint32_t nextMillis;
    uint16_t sportInterval; // msec
    uint8_t sportDeviceId;
    uint16_t sportFieldId;
} ;



enum fieldIdx {     // Internal Id for the measurements stored in oXs and that can be sent (some in a different format/field)
      LATITUDE =0,  //  GPS
      LONGITUDE,    //  GPS
      GROUNDSPEED , //  GPS
      HEADING,      //  GPS
      ALTITUDE ,    //  GPS

      NUMSAT ,      //  GPS   5
      GPS_DATE ,    // GPS
      GPS_TIME ,    // GPS
      GPS_PDOP ,    // GPS
      GPS_HOME_BEARING, // GPS

      GPS_HOME_DISTANCE, // GPS  10
      MVOLT,        // volt1  
      CURRENT,  // volt2 must be in seq for voltage.cpp
      RESERVE1, // volt3 must be in seq for voltage.cpp
      RESERVE2, // volt4 must be in seq for voltage.cpp
      
      CAPACITY,    // based on current (volt2)
      TEMP1,       // = Volt3 but saved as temp
      TEMP2,       // = Volt4 but saved as temp
      VSPEED,      // baro       
      RELATIVEALT , // baro      
      
      PITCH,       // imu        20 
      ROLL,        // imu        
      YAW ,        // not used to save data
      RPM ,        // RPM sensor  
      NUMBER_MAX_IDX, // used to count the number of entries  24
};


uint32_t millis() ;

uint32_t micros();

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