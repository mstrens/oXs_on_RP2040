#pragma once
#include <ctype.h>
#include "pico/stdlib.h"
#include "crsf_frames.h"

#define CONFIG_VERSION 6
struct CONFIG{
    uint8_t version = CONFIG_VERSION;
    uint8_t pinChannels[16] = {0XFF};
    uint8_t pinGpsTx = 0xFF;
    uint8_t pinGpsRx = 0xFF;
    uint8_t pinPrimIn = 0xFF;
    uint8_t pinSecIn = 0xFF; 
    uint8_t pinSbusOut = 0xFF;
    uint8_t pinTlm = 0xFF;
    uint8_t pinVolt[4] = {0xFF};
    uint8_t pinSda = 0xFF;
    uint8_t pinScl = 0xFF;
    uint8_t pinRpm = 0xFF;
    uint8_t pinLed = 16;
    uint8_t protocol = 'S' ; // S = Sport, C = crossfire, J = Jeti
    uint32_t crsfBaudrate = 420000;
    float scaleVolt1 = 1.0;
    float scaleVolt2 = 1.0;
    float scaleVolt3 = 1.0;
    float scaleVolt4 = 1.0;
    float offset1 = 0.0;
    float offset2 = 0.0;
    float offset3= 0.0;
    float offset4 = 0.0;
    uint8_t gpsType = 'U' ;
    float rpmMultiplicator = 1;
    //uint8_t gpio0 = 0; // 0 mean SBUS, 1 up to 16  = a RC channel
    //uint8_t gpio1 = 1;
    //uint8_t gpio5 = 6;
    //uint8_t gpio11 = 11;
    uint8_t failsafeType = 'H';
    crsf_channels_s failsafeChannels ;
    int16_t accOffsetX;
    int16_t accOffsetY;
    int16_t accOffsetZ;
    int16_t gyroOffsetX;
    int16_t gyroOffsetY;
    int16_t gyroOffsetZ;
    uint8_t temperature; 
    uint8_t VspeedCompChannel;
    uint8_t ledInverted;
};

void handleUSBCmd(void);
void processCmd(void);

char * skipWhiteSpace(char * str);
void removeTrailingWhiteSpace( char * str);
void findKeyAndValue( char * &buffer, char * &key, char * &cvalue);
void upperStr( char *p);
void setFailsafe();
void saveConfig();                 // save the config
void cpyChannelsAndSaveConfig();   // copy the channels values and save them into the config.
void addPinToCount(uint8_t pinId);
void checkConfig();
void setupConfig();
void printConfig();
void requestMpuCalibration();
void printConfigOffsets();
void printFieldValues();
void printPwmValues();

 //uint8_t * find(uint8_t * search, uint8_t in , uint16_t max); // search for first occurence of search string in "in" buffer  
