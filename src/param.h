#pragma once
#include <ctype.h>
#include "pico/stdlib.h"
#include "crsf_frames.h"

#define CONFIG_VERSION 6
struct CONFIG{
    uint8_t version = CONFIG_VERSION;
    uint8_t pinChannels[16] ;
    uint8_t pinGpsTx ;
    uint8_t pinGpsRx ;
    uint8_t pinPrimIn ;
    uint8_t pinSecIn ; 
    uint8_t pinSbusOut ;
    uint8_t pinTlm ;
    uint8_t pinVolt[4] ;
    uint8_t pinSda  ;
    uint8_t pinScl ;
    uint8_t pinRpm ;
    uint8_t pinLed ;
    uint8_t protocol  ; // S = Sport, C = crossfire, J = Jeti
    uint32_t crsfBaudrate ;
    float scaleVolt1 ;
    float scaleVolt2 ;
    float scaleVolt3 ;
    float scaleVolt4 ;
    float offset1 ;
    float offset2 ;
    float offset3 ;
    float offset4 ;
    uint8_t gpsType  ;
    float rpmMultiplicator ;
    //uint8_t gpio0 = 0; // 0 mean SBUS, 1 up to 16  = a RC channel
    //uint8_t gpio1 = 1;
    //uint8_t gpio5 = 6;
    //uint8_t gpio11 = 11;
    uint8_t failsafeType ;
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
    uint8_t pinLogger ;
    uint32_t loggerBaudrate ;
    uint8_t pinEsc;
    uint8_t escType;
    uint16_t pwmHz ; 
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
void checkConfigAndSequencers();
void setupConfig();
void printConfigAndSequencers();
void requestMpuCalibration();
void printConfigOffsets();
void printFieldValues();
void printPwmValues();


// for sequencer
#define SEQUENCER_VERSION 1
enum SEQ_OUTPUT_TYPE : uint8_t {
    SERVO = 0,
    ANALOG = 1
};


struct SEQ_DEF {
    uint8_t pin {}; // pin
    SEQ_OUTPUT_TYPE type {}; // type 0= SERVO or 1 = ANALOG
    uint32_t clockMs {};  // unit in msec
    uint8_t channel {};   // channel used to control the sequencer
    int8_t defValue {} ;  // default value (when no channel is yet received)
    int8_t minValue ;
    int8_t maxValue ;
} ;

enum CH_RANGE : int8_t {
    m100 = -100, m75 = -75, m50 = -50, m25 = -25, m0 = 0,
    p0 = 0, p25 = 25 , p50 = 50, p75 = 75, p100 = 100 , dummy = 127
};
struct SEQ_STEP {
    CH_RANGE chRange {}; // channel value (converted in range) to activate this seq
    uint8_t smooth {};
    int8_t value {} ;
    uint8_t keep {} ;
};


#define SEQUENCER_MAX_NUMBER_OF_STEPS 256 // to be modified
struct SEQUENCER{
    uint8_t version = SEQUENCER_VERSION;
    uint8_t defsMax = 0;
    SEQ_DEF defs[16] ;
    uint16_t stepsMax = 0;
    SEQ_STEP steps[SEQUENCER_MAX_NUMBER_OF_STEPS];  
};    

 //uint8_t * find(uint8_t * search, uint8_t in , uint16_t max); // search for first occurence of search string in "in" buffer  
bool getSetOfInt(uint8_t itemsMax); // fill a table with n integers ; format is e.g. { 1 2 3 5}
void printSequencers();
void setupSequencers();
bool getSequencers();
bool getStepsSequencers();
void checkSequencers();
void saveSequencers(); // save pin sequencers and step definitions

#define HW4 4
#define HW3 3
#define KONTRONIK 2