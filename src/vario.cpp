#include "pico/stdlib.h"  // this generates lot of warnings
//#include <stdlib.h>
#include "vario.h"
#include "MS5611.h"
#include "ms4525.h"
#include "stdio.h"
#include <inttypes.h>
#include "tools.h"
#include "mpu.h"
//#include <stdlib.h>     /* abs */

extern field fields[];  // list of all telemetry fields and parameters used by Sport
extern MPU mpu;
extern MS4525 ms4525;
extern float actualPressurePa; // used to calculate airspeed


uint32_t abs1(int32_t value){
  if (value > 0) return value;
  return -value;
}

VARIO::VARIO(){}

void VARIO::calculateAltVspeed(float baroAltitudeCm , int32_t baro_altIntervalMicros){
    
    if (firstCalc) {
        //if (firstCalcCounter == 100) printf("start alt at millis %" PRIu32 "\n", millisRp() );
        if (firstCalcCounter > 10){  // skip the first reading in order to get a better value as first Altitude
            firstCalcCounter--;
            return;
        } else if (firstCalcCounter--) {
            altitudeSum += baroAltitudeCm;    // at power on, we will calculate the average
            return; 
        } else {
            //printf("first alt at millis %" PRIu32 "\n", millisRp() );
            firstCalc = false;
            rawOffsetAltitudeCm =  altitudeSum * 0.1 ; // all in cm and in float // average of 10 values
            prev_baroAltitudeCm = altitudeLowPass = altitudeHighPass =  rawOffsetAltitudeCm ;
            intervalSmoothUs = 20000 ; // perhaps not required
        }
    }
    #define DIFFERENCE_ALTITUDE_MAX 2000 // in cm 
    // check that the new value is quite similar to the previous one (avoid glitch)
    if ( abs(prev_baroAltitudeCm - baroAltitudeCm) > DIFFERENCE_ALTITUDE_MAX) {
        prev_baroAltitudeCm = baroAltitudeCm;
        return;                // skip if the value is abnormal
    }
    prev_baroAltitudeCm = baroAltitudeCm;
    rawRelAltitudeCm = baroAltitudeCm - rawOffsetAltitudeCm;  // raw relative altitude used for mpu6050
    // smooth altitude
    smoothRelAltitudeCm += 0.04 * ( rawRelAltitudeCm - smoothRelAltitudeCm) ;    
    
    altitudeLowPass += 0.085 * ( rawRelAltitudeCm - altitudeLowPass) ;
    altitudeHighPass += 0.1 * ( rawRelAltitudeCm - altitudeHighPass) ;
    intervalSmoothUs += 0.1 * (baro_altIntervalMicros - intervalSmoothUs) ; //delay between 2 measures  only if there is no overflow of pressureMicos
    //printf("inter= %" PRIu32 "\n", baro_altIntervalMicros);
    climbRate2AltFloat = ( (float) (altitudeHighPass - altitudeLowPass )  * 566668.5 ) / (float) intervalSmoothUs; // climbRate is in cm/sec 
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
    newClimbRateAvailableForMpu = true; // this flag is used in mpu.cpp to say that a new vspeed may be calculated by kalman filter.
    newClimbRateAvailableForCompensation = true; // this flag is used in compensation to say that a new vspeed may be calculated by kalman filter.
    
    //printf("altitude %f   lowpass %f  highpass %f  dif %f   climbRateFloat %f  \n",
    //   (float)  altitude , (float) altitudeLowPass , (float)  altitudeHighPass, (float) altitudeLowPass -  (float)  altitudeHighPass,   (float) climbRateFloat);
    // update climbRate only if the difference is big enough
    if ( abs1(((int32_t)  (climbRateFloat - prevClimbRateFloat) )) > (int32_t) VARIOHYSTERESIS  ) {
        prevClimbRateFloat = climbRateFloat  ;
    }    
    if ( !mpu.mpuInstalled) {   // do not sent when a mp6050 is installed (value will be sent by mpu)
        sent2Core0( VSPEED , (int32_t) prevClimbRateFloat) ; 
    }
    // AltitudeAvailable is set to true only once every 200 msec in order to give priority to climb rate on SPORT
    altMillis = millisRp() ;
    if ( (altMillis - lastAltMillis) > 200){
        lastAltMillis = altMillis;
        sent2Core0(RELATIVEALT, smoothRelAltitudeCm) ;    
    } 
}


#define SMOOTHING_DTE_MIN SENSITIVITY_MIN
#define SMOOTHING_DTE_MAX SENSITIVITY_MAX
#define SMOOTHING_DTE_MIN_AT SENSITIVITY_MIN_AT
#define SMOOTHING_DTE_MAX_AT SENSITIVITY_MAX_AT
#define DTE_COMPENSATION_FACTOR 1.15

void VARIO::calculateVspeedDte () {  // is calculated about every 2O ms each time that an altitude is available
    
    float difPressureAdc_zero;
    float rawCompensation ;
    float rawTotalEnergy ;
    static float totalEnergyLowPass ;
    static float totalEnergyHighPass ;
    //static float intervalSmoothDteUs = 20000;
    
    static float rawCompensatedClimbRate;
    static float abs_deltaCompensatedClimbRate ;
    static float smoothingDteMin =  SMOOTHING_DTE_MIN ;
    static float expoSmoothDte_auto ;
    static float smoothCompensatedClimbRate ;
    float compensatedClimbRate = 0; 

    // for 4525:
        //  difPressure (in PSI) = difPressureAdc * 0.0001525972 because 1 PSI = (8192 - 1638) = 6554 steps
        //  difPressure (Pa) =  difPressure (in PSI) * 6894.757f  = difPressureAdc * 6894.757 *  0.0001525972 = difPressureAdc * 1.0520
        // airspeed = sqr(2 * differential_pressure / air_density) ; air density = pressure  pa / (287.05 * (Temp celcius + 273.15))
        // so airspeed m/sec =sqr( 2 * 287.05 * differential_pressure pa * (temperature Celsius + 273.15) / pressure pa )
        // total energy = (m * g * altitude) + (m * airspeed * airspeed / 2) => m * 9.81 * (altitude + airspeed * airspeed / 2 /9.81)
        // compensation (m/sec) = airspeed * airspeed / 2 / 9.81 =
        //                      = 2 * 287.05 * difPressureAdc * 1.0520  * (temperature Celsius + 273.15) / pressure pa /2 /9.81 (m/sec) = 30.78252803 * difPressureAdc * Temp(kelv) / Press (Pa)
        // compensation (cm/sec) =  3078.252803 * difPressureAdc * Temp(kelv) / Press (Pa)
    if (ms4525.airspeedInstalled == false) return; // skip when no MS4525 is installed
    if (newClimbRateAvailableForCompensation == false) return; // skip when no new Vspeed is available
    // calculate average diff of pressure because MS4525 is read more ofen than Vspeed
    difPressureAdc_zero = ms4525.difPressureAdc_0SumValue / ms4525.difPressureAdc_0SumCount ;
    ms4525.difPressureAdc_0SumCount = 0;
    ms4525.difPressureAdc_0SumValue = 0; // reset the values used for averaging  
    rawCompensation = 307825 * difPressureAdc_zero * ms4525.temperatureKelvin /  actualPressurePa    ; // 3078.25 = comp = 2 * 287.05 / 2 / 9.81 * 1.0520 * 100 * Temp / Pressure  
    rawTotalEnergy =  rawRelAltitudeCm + rawCompensation * DTE_COMPENSATION_FACTOR ; // 1 means 100% compensation but we add 15% because it seems that it is 15% undercompensated. 
    if (totalEnergyLowPass == 0) { // initiaise smoothing 
        totalEnergyLowPass = totalEnergyHighPass = rawTotalEnergy ; 
    }
    totalEnergyLowPass += 0.085 * ( rawTotalEnergy - totalEnergyLowPass) ;
    totalEnergyHighPass += 0.1 * ( rawTotalEnergy - totalEnergyHighPass) ;
    //intervalSmoothDteUs += 0.1 * (baro_altIntervalMicros - intervalSmoothDteUs) ; //delay between 2 measures  only if there is no overflow of pressureMicos
  //printf("inter= %" PRIu32 "\n", baro_altIntervalMicros);
    rawCompensatedClimbRate = ((totalEnergyHighPass - totalEnergyLowPass )  * 566667.0 ) / intervalSmoothUs; // 0.566667 is the parameter to be used for 0.085 and 0.1 filtering if delay is in sec
    abs_deltaCompensatedClimbRate =  abs(rawCompensatedClimbRate - smoothCompensatedClimbRate) ;
    if ( sensitivityPpm  > 0) smoothingDteMin =   sensitivityPpm  ; // a value of sensitivityPpmMapped = 50 becomes a smoothing factor 0.1
        if ( (abs_deltaCompensatedClimbRate <= SMOOTHING_DTE_MIN_AT) || (smoothingDteMin >= SMOOTHING_DTE_MAX ) ){
        expoSmoothDte_auto = smoothingDteMin ;  
    } else if (abs_deltaCompensatedClimbRate >= SMOOTHING_DTE_MAX_AT)  {
        expoSmoothDte_auto = SMOOTHING_DTE_MAX ; 
    } else {
        expoSmoothDte_auto = smoothingDteMin + ( SMOOTHING_DTE_MAX - smoothingDteMin ) * (abs_deltaCompensatedClimbRate - SMOOTHING_DTE_MIN_AT) / (SMOOTHING_DTE_MAX_AT - SMOOTHING_DTE_MIN_AT) ;
    }
    smoothCompensatedClimbRate += expoSmoothDte_auto * (  rawCompensatedClimbRate -  smoothCompensatedClimbRate ) * 0.001; 
    if ( abs( ((int32_t)  smoothCompensatedClimbRate) - compensatedClimbRate) > VARIOHYSTERESIS ) {
        compensatedClimbRate = smoothCompensatedClimbRate  ;
    } 
    sent2Core0( AIRSPEED_COMPENSATED_VSPEED , (int32_t) compensatedClimbRate) ; 
} // end calculateVspeedDte