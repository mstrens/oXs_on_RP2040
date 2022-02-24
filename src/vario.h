
#pragma once

#include "Arduino.h"
#include "config_basic.h"
#include "MS5611.h"

//typedef struct {
//  uint8_t available ;
//  int32_t value ;
//} oneMeasurement_t;


class VARIO
{
public:
  float    altitude;
  uint32_t altIntervalMicros; // enlapstime between 2 calculations of altitude
  explicit VARIO(void); 
  // common for several varios
  bool sensitivityAvailable ;  //used to decide if sensivityPpm can be sent or not
  int sensitivityPpm ;      // sensivity to apply when PPM is used. Value has to be divided by 1000 in order to calculate the smoothing parameter
  int sensitivity ; 
  oneMeasurement_t absoluteAlt;     // in cm  * 100
  bool altitudeAvailableForDte   ;  // use to say to readsensors() that an altitude is available and that dte can be calculated.
  
  oneMeasurement_t vSpeed10Sec; // Altitude gain/loose between 10 sec (is calculated and send every 500 msec)
  
  oneMeasurement_t climbRate;       // in cm /sec = vertical speed
  
  bool switchClimbRateAvailable ; // use to say to the readsensors loop that that a climbrate is available (to select the one being send)  
  
  void calculateAltVspeed(MS5611 * baro);

private:
  int32_t prevAlt[20] ;   // table contains the 20 latest altitude
  byte idxPrevAlt ;       // index of last entry in table
};
