
#pragma once


#include "config.h"
#include "MS5611.h"

//typedef struct {
//  uint8_t available ;
//  int32_t value ;
//} oneMeasurement_t;


class VARIO
{
public:
    bool firstCalc = true; 
    //int32_t altitude ;   // in cm * 100
    float altitudeSum = 0; 
    float rawOffsetAltitudeCm ;
    
    float rawRelAltitudeCm = 0;
    
    float prev_baroAltitudeCm;
    
    float altitudeLowPassCm ;
    float altitudeHighPassCm;
    float intervalSmoothUs;  
    float smoothRelAltitudeCm ;
    
    //int32_t intervalSmooth ; // we expect an interval of 20msec between 2 conversions
    float climbRateFloat ;
    float prevClimbRateFloat; 
    int32_t compensatedVpseed;
    float climbRate2AltFloat;
    float abs_deltaClimbRate;
    uint32_t altMillis ;
    uint32_t lastAltMillis = 0;
    uint32_t nextAverageAltMillis;
    int sensitivityMin = SENSITIVITY_MIN ; // set the min smoothing to the default value
    uint8_t firstCalcCounter = 100;

    //float    altitude; // in cm *100
    uint32_t altIntervalMicros = 0; // enlapstime between 2 calculations of altitude
    explicit VARIO(void); 
    // common for several varios
    //bool sensitivityAvailable ;  //used to decide if sensivityPpm can be sent or not
    int sensitivityPpm ;      // sensivity to apply when PPM is used. Value has to be divided by 1000 in order to calculate the smoothing parameter
    int sensitivity ; 
    //oneMeasurement_t absoluteAlt;     // in cm  
//    bool altitudeAvailableForDte   ;  // use to say to readsensors() that an altitude is available and that dte can be calculated.
//    int32_t relativeAlt;     // in cm  
    bool switchClimbRateAvailable ; // use to say to the readsensors loop that that a climbrate is available (to select the one being send)  
    bool newClimbRateAvailableForMpu = false;
    bool newClimbRateAvailableForCompensation = false; 

    
    float compensatedClimbRateCmS = 0;

    void calculateAltVspeed(float baroAltitudeCm , int32_t baro_altIntervalMicros);
    void calculateVspeedDte();
    uint16_t findVspeedCompensation();
private:
    int32_t prevAlt[20] ;   // table contains the 20 latest altitude
    uint8_t idxPrevAlt ;       // index of last entry in table
};
