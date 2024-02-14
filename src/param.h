#pragma once

#include <ctype.h>
#include "pico/stdlib.h"
#include "crsf_frames.h"
#include "gyro.h"

#define CONFIG_VERSION 8

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
    //                for gyro
    uint8_t gyroChanControl ; // Rc channel used to say if gyro is implemented or not and to select the mode and the general gain. Value must be in range 1/16 or 255 (no gyro)
    uint8_t gyroChan[3] ;    // Rc channel used to transmit original Ail, Elv, Rud stick position ; Value must be in range 1/16 when gyroControlChannel is not 255
    
    struct _pid_param pid_param_rate; // each structure store the Kp, Ki, Kd parameters for each of the 3 axis; here for normal mode (= rate)
    struct _pid_param pid_param_hold; // idem for hold mode
    struct _pid_param pid_param_stab;  //each structure store the Kp, Ki, Kd parameters for each of the 3 axis; here for stabilize mode (= rate)
    
    int8_t vr_gain[3];          // store the gain per axis (to combine with global gain provided by gyroChanControl)
    enum STICK_GAIN_THROW stick_gain_throw;  // this parameter allows to limit the compensation on a part of the stick travel (gain decreases more or less rapidly with stick offset)
    enum MAX_ROTATE max_rotate;
    enum RATE_MODE_STICK_ROTATE rate_mode_stick_rotate;
    bool gyroAutolevel;           // true means that stabilize mode replies the Hold mode (on switch position)
    uint8_t mpuOrientationH;       // define the orientation of the mpu when plane is horizontal;
    uint8_t mpuOrientationV;       // define the orientation of the mpu when plane is vertical (nose up);
    // for Lora locator
    uint8_t pinSpiCs;
    uint8_t pinSpiSck;
    uint8_t pinSpiMosi;
    uint8_t pinSpiMiso;
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
#define SEQUENCER_VERSION 4
enum SEQ_OUTPUT_TYPE : uint8_t {
    SERVO = 0,
    ANALOG = 1
};


struct SEQ_DEF {             // one set of parameters per sequencer
    uint8_t pin {}; // pin
    SEQ_OUTPUT_TYPE type {}; // type 0= SERVO or 1 = ANALOG
    uint32_t clockMs {};  // unit in msec
    uint8_t channel {};   // channel used to control the sequencer
    int8_t defValue {} ;  // default value (when no channel is yet received)
    int8_t minValue ;
    int8_t maxValue ;
} ;

//enum CH_RANGE : int8_t {
//    m100 = -100, m75 = -75, m50 = -50, m25 = -25, m0 = 0,
//    p0 = 0, p25 = 25 , p50 = 50, p75 = 75, p100 = 100 , dummy = 127
//};

enum CH_RANGE : int8_t {
    m100=-100, m95=-95, m90=-90, m85=-85 ,m80=-80, m75=-75, m70=-70, m65=-65, m60=-60, m55=-55, m50=-50,
         m45=-45, m40=-40, m35=-35, m30=-30, m25=-25, m20=-20, m15=-15, m10=-10, m5=-5, m0 = 0,
    p0=0, p5=5, p10=10, p15=15, p20=20, p25=25, p30=30, p35=35, p40=40, p45=45, p50=50, p55=55,
     p60=60, p65=65, p70=70, p75=75, p80=80, p85=85, p90=90, p95=95, p100=100 , dummy = 127
};


struct SEQ_STEP {           // one set of parameters per sequence and per step
    CH_RANGE chRange {}; // channel value (converted in range) to activate this seq
    uint8_t smooth {};
    int8_t value {} ;
    uint8_t keep {} ;
    uint8_t nextSequencerBegin:1; // 1 means that this is the first step of a new sequencer
    uint8_t nextSequenceBegin:1; // 1 means that this is the first step of a new sequence
    uint8_t toRepeat:1; // 1 means that this is the sequence must be repeated when reaching the end
    uint8_t neverInterrupted:1 ;// 1 means that this sequence may never be interrupted by another one
    uint8_t priorityInterruptOnly:1 ; // 1 means that this sequence may only be interrupted by a sequence marqued as priority
    uint8_t isPriority:1 ;         // 1 means that this sequence must always interrupt a sequence that may be interrupted
    uint8_t reserver1:1;           // not used currently
    uint8_t reserver2:1;           // not used currently
    
    //bool nextSequencerBegin {} ;
};


#define SEQUENCER_MAX_NUMBER_OF_STEPS 256 // to be modified
struct SEQUENCER{            // one set of parameters for all sequencers (strored in "seq")
    uint8_t version = SEQUENCER_VERSION;
    uint8_t defsMax = 0;
    SEQ_DEF defs[16] ;
    uint16_t sequencesMax = 0;
    uint16_t stepsMax = 0;
    SEQ_STEP steps[SEQUENCER_MAX_NUMBER_OF_STEPS];  
};    

 //uint8_t * find(uint8_t * search, uint8_t in , uint16_t max); // search for first occurence of search string in "in" buffer  
//bool parseOneSequencer(); // fill a table with 7 integers for a sequencer; format is e.g. { 1 2 3 5 6 7}
bool getAllSequencers();    // get all sequencers, sequences and steps (call parseOneSequencer)
bool parseOneSequencer(); // get one sequencer and all his sequences and steps in seqdefsTemp[] and stepsTemp.
bool parseOneSequence(); // get one sequence and his steps in stepsTemp.
bool parseOneStep();      // parse one step and save parameter in stepsTemp[]
//bool parseOneStep();      // fill a table with sequence parameter and with step parameter.
void printSequencers();
void setupSequencers();

//bool getSequencers();
//bool getStepsSequencers();
void checkSequencers();
void saveSequencers(); // save all sequencers definitions


void setupGyroMixer();
void saveGyroMixer();  // save the gyro mixer collected during the learning process (mixer calibration)
void printGyro(); 
void printGyroMixer();

bool getPid(uint8_t mode);  // get all pid parameters for one mode; return true if valid; config is then updated


#define HW4 4
#define HW3 3
#define KONTRONIK 2
#define ZTW1 1
#define BLH 5

#define REQUEST_HORIZONTAL_MPU_CALIB 0X01
#define REQUEST_VERTICAL_MPU_CALIB 0X02