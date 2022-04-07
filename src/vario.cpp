#include "pico/stdlib.h"  // this generates lot of warnings
//#include <stdlib.h>
#include "vario.h"
#include "MS5611.h"
#include "stdio.h"
#include <inttypes.h>
#include "tools.h"
//#include <stdlib.h>     /* abs */

extern field fields[SPORT_TYPES_MAX];  // list of all telemetry fields and parameters used by Sport


uint32_t abs(int32_t value){
  if (value > 0) return value;
  return -value;
}

VARIO::VARIO(){}

void VARIO::calculateAltVspeed(MS5611  *baro){
  if ( !baro->baroInstalled) return ; // skip when baro is not installed
    // smooth altitude
  if (firstCalc) {
    if (firstCalcCounter--){  // skip the first reading in order to get a better value as first Altitude
        return;
    } else {
        firstCalc = false;
        altitudeLowPass = altitudeHighPass = altitude =  baro->altitude ; // all in cm *100 and in int32
        intervalSmooth = 20000 ; // perhaps not required
    }
  }
  // check the delay between 2 calculations
  //static uint32_t prevMillis = 0;
  //printf( "time %" PRIu32 "\n", millis()-prevMillis);
  //prevMillis = millis();
  
  altitude += 0.04 * (baro->altitude - altitude) ;
  absoluteAlt.value = altitude ;
  absoluteAlt.available = true ;
  altitudeAvailableForDte = true; // it was altitudeAt20MsecAvailable = true ; // inform openxsens.ino that calculation of dTE can be performed

  altitudeLowPass += 0.085 * ( baro->altitude - altitudeLowPass) ;
  altitudeHighPass += 0.1 * ( baro->altitude - altitudeHighPass) ;
  intervalSmooth += 0.1 * (baro->altIntervalMicros - intervalSmooth) ; //delay between 2 measures  only if there is no overflow of pressureMicos
  //printf("inter= %" PRIu32 "\n", baro->altIntervalMicros);
  climbRate2AltFloat = ( (float) (altitudeHighPass - altitudeLowPass )  * 5666.685 ) / (float) intervalSmooth; // climbRate is in cm/sec 
  //printf("altDif= %" PRIi32 "  interval=%" PRIu32 "  climb= %f\n", (altitudeHighPass - altitudeLowPass ) , intervalSmooth , climbRate2AltFloat);
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
  if ( abs((int32_t)  climbRateFloat - fields[VSPEED].value) > VARIOHYSTERESIS ) {
      fields[VSPEED].value = (int32_t)  climbRateFloat  ;
  }    
  fields[VSPEED].available=true; // allows SPORT protocol to transmit the value
  switchClimbRateAvailable = true ; // inform readsensors() that a switchable vspeed is available
  
  // AltitudeAvailable is set to true only once every 100 msec in order to give priority to climb rate on SPORT
  altMillis = millis() ;
  if ( (altMillis - lastAltMillis) > 100){
    lastAltMillis = altMillis;
    absoluteAlt.value = altitude / 100; // altitude is in m *10000 and AbsoluteAlt must be in m * 100
    absoluteAlt.available=true;  // Altitude is considered as available only after several loop in order to reduce number of transmission on Sport.
    //printf("abs alt= %" PRIu32 "\n", absoluteAlt.value );
    sensitivityAvailable = true ;
    if (altOffset == 0) altOffset = absoluteAlt.value ;
    fields[RELATIVEALT].value = absoluteAlt.value - altOffset ;
    //printf("relAlt=%f  vpseed =%f  interval=%f\n", fields[RELATIVEALT].value / 100.0 , climbRateFloat , baro->altIntervalMicros*1.0);
    fields[RELATIVEALT].available = true ;
    if ( fields[RELATIVEALT].value > relativeAltMax.value ) relativeAltMax.value = fields[RELATIVEALT].value ;
    relativeAltMax.available = true ;
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
