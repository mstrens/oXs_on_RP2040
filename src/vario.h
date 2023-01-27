
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
  int32_t altitudeLowPass ;
  int32_t altitudeHighPass;
  int32_t altitude ;
  int32_t intervalSmooth ; // we expect an interval of 20msec between 2 conversions
  float   climbRateFloat ;
  float prevClimbRateFloat; 
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
  bool sensitivityAvailable ;  //used to decide if sensivityPpm can be sent or not
  int sensitivityPpm ;      // sensivity to apply when PPM is used. Value has to be divided by 1000 in order to calculate the smoothing parameter
  int sensitivity ; 
  oneMeasurement_t absoluteAlt;     // in cm  
  bool altitudeAvailableForDte   ;  // use to say to readsensors() that an altitude is available and that dte can be calculated.
  //oneMeasurement_t relativeAlt;     // in cm  
  oneMeasurement_t relativeAltMax;     // in cm  
  int32_t altOffset ;
  oneMeasurement_t vSpeed10Sec; // Altitude gain/loose between 10 sec (is calculated and send every 500 msec)
  
  //oneMeasurement_t climbRate;       // in cm /sec = vertical speed
  
  bool switchClimbRateAvailable ; // use to say to the readsensors loop that that a climbrate is available (to select the one being send)  
  
  void calculateAltVspeed(int32_t baro_altitude , int32_t baro_altIntervalMicros);

private:
  
  
  int32_t prevAlt[20] ;   // table contains the 20 latest altitude
  uint8_t idxPrevAlt ;       // index of last entry in table
};
