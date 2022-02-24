#include "vario.h"

VARIO::VARIO(){}

void VARIO::calculateAltVspeed(MS5611  *baro){
  static bool firstCalc = true; 
  static int32_t altitudeLowPass = 0;
  static int32_t altitudeHighPass = 0;
  static int32_t altitude = 0;
  static int32_t intervalSmooth = 20000; // we expect an interval of 20msec between 2 conversions
  static float   climbRateFloat = 0; 
  float climbRate2AltFloat;
  float abs_deltaClimbRate;
  uint32_t altMillis ;
  uint32_t lastAltMillis = 0;
  uint32_t nextAverageAltMillis;
  int sensitivityMin = SENSITIVITY_MIN ; // set the min smoothing to the default value
  
    // smooth altitude
  if (firstCalc) {
    firstCalc = false;
    altitudeLowPass = altitudeHighPass = altitude = (int32_t) baro->altitude ;
    intervalSmooth = 20000 ; // perhaps not required
  }
  altitude += 0.04 * (baro->altitude - altitude) ;
  absoluteAlt.value = altitude ;
  absoluteAlt.available = true ;
  altitudeAvailableForDte = true; // it was altitudeAt20MsecAvailable = true ; // inform openxsens.ino that calculation of dTE can be performed

  altitudeLowPass += 0.085 * ( baro->altitude - altitudeLowPass) ;
  altitudeHighPass += 0.1 * ( baro->altitude - altitudeHighPass) ;
  intervalSmooth += 0.1 * (baro->altIntervalMicros - intervalSmooth) ; //delay between 2 measures  only if there is no overflow of pressureMicos
  climbRate2AltFloat = ((altitudeHighPass - altitudeLowPass )  * 5666.685 ) / intervalSmooth; 

  abs_deltaClimbRate =  abs(climbRate2AltFloat - climbRateFloat) ;
  if ( sensitivityPpm  > 0) sensitivityMin =   sensitivityPpm ; 
  if ( (abs_deltaClimbRate <= SENSITIVITY_MIN_AT) || (sensitivityMin >= SENSITIVITY_MAX) ) {
      sensitivity = sensitivityMin ;  
  } else if (abs_deltaClimbRate >= SENSITIVITY_MAX_AT)  {
      sensitivity = SENSITIVITY_MAX ; 
  } else {
      sensitivity = sensitivityMin + ( SENSITIVITY_MAX - sensitivityMin ) * (abs_deltaClimbRate - SENSITIVITY_MIN_AT) / (SENSITIVITY_MAX_AT - SENSITIVITY_MIN_AT) ;
  }
  climbRateFloat += sensitivity * (climbRate2AltFloat - climbRateFloat)  * 0.001 ; // sensitivity is an integer and must be divided by 1000
  
  // update climbRate only if the difference is big enough
  if ( abs((int32_t)  climbRateFloat - climbRate.value) > VARIOHYSTERESIS ) {
      climbRate.value = (int32_t)  climbRateFloat  ;
  }    
  climbRate.available=true; // allows SPORT protocol to transmit the value
  switchClimbRateAvailable = true ; // inform readsensors() that a switchable vspeed is available
  
  // AltitudeAvailable is set to true only once every 100 msec in order to give priority to climb rate on SPORT
  altMillis = millis() ;
  if ( (altMillis - lastAltMillis) > 100){
    lastAltMillis = altMillis;
    absoluteAlt.value = altitude / 100; // altitude is in m *10000 and AbsoluteAlt must be in m * 100
    absoluteAlt.available=true;  // Altitude is considered as available only after several loop in order to reduce number of transmission on Sport.
    sensitivityAvailable = true ;
    if ( altMillis > nextAverageAltMillis ){ // calculation of the difference of altitude (in m) between the 10 last sec
        nextAverageAltMillis = altMillis + 500 ; // calculate only once every 500 msec
        vSpeed10Sec.value = (absoluteAlt.value - prevAlt[idxPrevAlt]) /100 ;
        prevAlt[idxPrevAlt] = absoluteAlt.value ;
        idxPrevAlt++ ;
        if ( idxPrevAlt >= 20 ) idxPrevAlt = 0 ;
        if ( altMillis > 15000) {  // make the data avalaible only after 15 sec)
            vSpeed10Sec.available = true ;
        }  
    }  
  } // end If (altMillis - lastAltMillis) > 100
}
