#include "pico/stdlib.h"  // this generates lot of warnings
//#include <stdlib.h>
#include "vario.h"
#include "MS5611.h"
#include "stdio.h"
#include <inttypes.h>
#include "tools.h"
#include "mpu.h"
//#include <stdlib.h>     /* abs */

extern field fields[];  // list of all telemetry fields and parameters used by Sport
extern MPU mpu;

uint32_t abs1(int32_t value){
  if (value > 0) return value;
  return -value;
}

VARIO::VARIO(){}

void VARIO::calculateAltVspeed(int32_t baro_altitude , int32_t baro_altIntervalMicros){
  static int32_t prev_baro_altitude;
  
  if (firstCalc) {
    //if (firstCalcCounter == 100) printf("start alt at millis %" PRIu32 "\n", millisRp() );
    if (firstCalcCounter > 10){  // skip the first reading in order to get a better value as first Altitude
        firstCalcCounter--;
        return;
    } else if (firstCalcCounter--) {
      altitude += baro_altitude;    // at power on, we will calculate the average
      return; 
    } else {
        //printf("first alt at millis %" PRIu32 "\n", millisRp() );
        firstCalc = false;
        prev_baro_altitude = baro_altitude;
        altitudeLowPass = altitudeHighPass = altitude =  altitude / 10 ; // all in cm *100 and in int32
        rawOffsetAltitudeCm = altitude * 0.01 ;  // in cm
        intervalSmooth = 20000 ; // perhaps not required
    }
  }
  #define DIFFERENCE_ALTITUDE_MAX 200000 // in cm * 100
  // check that the new value is quite similar to the previous one (avoid glitch)
  
  if ( abs(prev_baro_altitude - baro_altitude) > DIFFERENCE_ALTITUDE_MAX) {
    prev_baro_altitude = baro_altitude;
    return;
  }
  prev_baro_altitude = baro_altitude;
  rawRelAltitudeCm = baro_altitude * 0.01 - rawOffsetAltitudeCm;  // raw relative altitude
  // smooth altitude
  altitude += 0.04 * ( baro_altitude - altitude) ;
  
  altitudeLowPass += 0.085 * ( baro_altitude - altitudeLowPass) ;
  altitudeHighPass += 0.1 * ( baro_altitude - altitudeHighPass) ;
  intervalSmooth += 0.1 * (baro_altIntervalMicros - intervalSmooth) ; //delay between 2 measures  only if there is no overflow of pressureMicos
  //printf("inter= %" PRIu32 "\n", baro_altIntervalMicros);
  climbRate2AltFloat = ( (float) (altitudeHighPass - altitudeLowPass )  * 5666.685 ) / (float) intervalSmooth; // climbRate is in cm/sec 
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
  newClimbRateAvailable = true; // this flag is used in mpu.cpp to say that a new vspeed may be calculated by kalman filter.
  //printf("altitude %f   lowpass %f  highpass %f  dif %f   climbRateFloat %f  \n",
  //   (float)  altitude , (float) altitudeLowPass , (float)  altitudeHighPass, (float) altitudeLowPass -  (float)  altitudeHighPass,   (float) climbRateFloat);
  // update climbRate only if the difference is big enough
  if ( abs1(((int32_t)  (climbRateFloat - prevClimbRateFloat) )) > (int32_t) VARIOHYSTERESIS  ) {
      prevClimbRateFloat = climbRateFloat  ;
  }    
  if ( !mpu.mpuInstalled) {   // do not sent when a mp6050 is installed (value will be sent by mpu)
    sent2Core0( VSPEED , (int32_t) prevClimbRateFloat) ; 
  }
  // AltitudeAvailable is set to true only once every 100 msec in order to give priority to climb rate on SPORT
  //if (altOffset == 0) altOffset = altitude* 0.01 ; // altitude is in 1/100 of cm 
  relativeAlt =   altitude* 0.01 -  rawOffsetAltitudeCm; 
  altMillis = millisRp() ;
  if ( (altMillis - lastAltMillis) > 100){
    lastAltMillis = altMillis;
    sensitivityAvailable = true ;
    sent2Core0(RELATIVEALT, relativeAlt) ;    
    /*
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
    */ 
  } // end If (altMillis - lastAltMillis) > 100
}
