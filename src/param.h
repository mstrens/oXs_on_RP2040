#pragma once
#include <ctype.h>
#include "pico/stdlib.h"
#include "crsf.h"

struct CONFIG{
    uint8_t version = 1;
    uint32_t crsfBaudrate = 420000;
    float scaleVolt1 = 1.0;
    float scaleVolt2 = 1.0;
    float scaleVolt3 = 1.0;
    float scaleVolt4 = 1.0;
    float offset1 = 0.0;
    float offset2 = 0.0;
    float offset3= 0.0;
    float offset4 = 0.0;
    uint8_t gpsType = 0 ;
    uint8_t failsafeType = 'H';
    crsf_channels_s failsafeChannels ;
};

void handleUSBCmd(void);
void processCmd(void);


char * skipWhiteSpace(char * str);
void removeTrailingWhiteSpace( char * str);
void findKeyAndValue( char * &buffer, char * &key, char * &cvalue);
void upperStr( char *p);
void setFailsafe();
void saveConfig();
void setupConfig();
void printConfig();
 //uint8_t * find(uint8_t * search, uint8_t in , uint16_t max); // search for first occurence of search string in "in" buffer  
