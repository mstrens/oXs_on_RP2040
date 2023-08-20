#include "pico/stdlib.h"  // this generates lot of warnings
//#include <stdlib.h>
#include "vario.h"
#include "MS5611.h"
#include "ms4525.h"
#include "sdp3x.h"
#include "stdio.h"
#include <inttypes.h>
#include "tools.h"
#include "mpu.h"
#include "math.h"
#include "param.h"
#include "crsf_in.h"
//#include <stdlib.h>     /* abs */

extern CONFIG config;
extern uint32_t lastRcChannels; // used here for dte compensation factor
extern sbusFrame_s sbusFrame; // used here for dte compensation factor

extern field fields[];  // list of all telemetry fields and parameters used by Sport
extern MPU mpu;
extern MS4525 ms4525;
extern SDP3X sdp3x;
extern float actualPressurePa; // used to calculate airspeed

extern float difPressureCompVspeedSumPa  ; // calculate a moving average on x values
extern uint32_t difPressureCompVspeedCount ;
extern float temperatureKelvin;
uint32_t prevCompVspeedAvailableMs;
float dteCompensationFactor = DTE_DEFAULT_COMPENSATION_FACTOR;


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
            prev_baroAltitudeCm = altitudeLowPassCm = altitudeHighPassCm =  rawOffsetAltitudeCm ;
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
    
    altitudeLowPassCm += 0.085 * ( rawRelAltitudeCm - altitudeLowPassCm) ;
    altitudeHighPassCm += 0.1 * ( rawRelAltitudeCm - altitudeHighPassCm) ;
    intervalSmoothUs += 0.1 * (baro_altIntervalMicros - intervalSmoothUs) ; //delay between 2 measures  only if there is no overflow of pressureMicos
    //printf("inter= %" PRIu32 "\n", baro_altIntervalMicros);
    climbRate2AltFloat = ( (float) (altitudeHighPassCm - altitudeLowPassCm )  * 566668.5 ) / (float) intervalSmoothUs; // climbRate is in cm/sec 
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
        compensatedVpseed =  (int32_t) prevClimbRateFloat ; // we save it here first, so we can reuse this field for compensated Vspeed when it is disabled 
        sent2Core0( VSPEED , compensatedVpseed) ;
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


void VARIO::calculateVspeedDte () {  // is calculated about every 2O ms each time that an altitude is available
    
    static float totalEnergyLowPassCm =0 ;
    static float totalEnergyHighPassCm =0 ;
    static float smoothCompensatedClimbRateCmS = 0;
    static float difPressureAvgPa = 0; 

    // for 4525:
        // airspeed = sqr(2 * differential_pressure / air_density) ; air density = pressure  pa / (287.05 * (Temp celcius + 273.15))
        // so airspeed m/sec =sqrt( 2 * 287.05 * differential_pressure pa * (temperature Celsius + 273.15) / pressure pa )
        // total energy = (m * g * altitude) + (m * airspeed * airspeed / 2) 
        //    energy => m * 9.81 * (altitude + airspeed * airspeed / 2 /9.81)
        // compensation (m) = airspeed * airspeed / 2 / 9.81 =
        //                  = 2 * 287.05 * difPressurePa * (temperature Celsius + 273.15) / pressure pa /2 /9.81 (m/sec) = 29.26 * difPressureAdc * Temp(kelv) / Press (Pa)
        // compensation (cm) =  2926.0 * difPressureAdc * Temp(kelv) / Press (Pa)
    if (ms4525.airspeedInstalled == false && sdp3x.airspeedInstalled == false) return; // skip when no MS4525/sdp3x is installed
    if (newClimbRateAvailableForCompensation == false) return; // skip when no new Vspeed is available
    newClimbRateAvailableForCompensation = false; // reset the flag
    // calculate average diff of pressure because MS4525 is read more ofen than Vspeed
    if (difPressureCompVspeedCount > 0) difPressureAvgPa = difPressureCompVspeedSumPa / (float) difPressureCompVspeedCount ;
    difPressureCompVspeedSumPa = 0;
    difPressureCompVspeedCount = 0; // reset the values used for averaging  
    // calculate the dteCompensationFactor
    // there are 3 cases:
    // when channel is more than positive(in %), channel value is used to calculate the factor and compensated Vspeed is send
    // when channel is around at center, we use default value to calculate the factor and compensated Vspeed is send
    // when channel is more than negative (in %), compensated Vspeed is calculated with previous factor but not sent (we send normal Vspeed) 
    uint16_t dteChannelValue = 0X400;  // default value = send compensation
    if (( config.VspeedCompChannel != 255) && lastRcChannels) { // when a channel is used and has been received
        dteChannelValue =  findVspeedCompensation();
        
        #define DTE_MIN_CHANNEL_COMP_VALUE 0X0500
        #define DTE_MAX_CHANNEL_COMP_VALUE 0X0700
        #define DTE_NO_CHANNEL_COMP_VALUE  0X0200
        if ( dteChannelValue > DTE_MIN_CHANNEL_COMP_VALUE) {
            if (dteChannelValue > DTE_MAX_CHANNEL_COMP_VALUE) dteChannelValue = DTE_MAX_CHANNEL_COMP_VALUE;
            dteCompensationFactor = 0.9 + ((float) (dteChannelValue - DTE_MIN_CHANNEL_COMP_VALUE)
                 * 0.5 / ((float) (DTE_MAX_CHANNEL_COMP_VALUE - DTE_MIN_CHANNEL_COMP_VALUE)) );
        } else if ( dteChannelValue >= DTE_NO_CHANNEL_COMP_VALUE) {
            dteCompensationFactor = DTE_DEFAULT_COMPENSATION_FACTOR;
        } // when dteChannelValue < DTE_NO_CHANNEL_COMP_VALUE, we calculate with the previous dteCompensationFactor
    }
    float rawCompensationCm = 2926.0 * difPressureAvgPa * temperatureKelvin /  actualPressurePa    ; 
    float rawTotalEnergyCm =  rawRelAltitudeCm + (rawCompensationCm * dteCompensationFactor) ; // 1 means 100% compensation but we add 15% because it seems that it is 15% undercompensated. 
    if (totalEnergyLowPassCm == 0) { // initialize smoothing 
        totalEnergyLowPassCm = totalEnergyHighPassCm = rawTotalEnergyCm ; 
    }
    totalEnergyLowPassCm += 0.085 * ( rawTotalEnergyCm - totalEnergyLowPassCm) ;
    totalEnergyHighPassCm += 0.1 * ( rawTotalEnergyCm - totalEnergyHighPassCm) ;
    //intervalSmoothDteUs += 0.1 * (baro_altIntervalMicros - intervalSmoothDteUs) ; //delay between 2 measures  only if there is no overflow of pressureMicos
    //printf("inter= %" PRIu32 "\n", baro_altIntervalMicros);
    float rawCompensatedClimbRateCmS = ((totalEnergyHighPassCm - totalEnergyLowPassCm )  * 566667.0 ) / intervalSmoothUs; // 0.566667 is the parameter to be used for 0.085 and 0.1 filtering if delay is in sec
    float abs_deltaCompensatedClimbRateCmS =  abs(rawCompensatedClimbRateCmS - smoothCompensatedClimbRateCmS) ;
    float smoothingDteMin =  SMOOTHING_DTE_MIN ;
    float expoSmoothDte_auto ;
    if ( sensitivityPpm  > 0) smoothingDteMin =   sensitivityPpm  ; // a value of sensitivityPpmMapped = 50 becomes a smoothing factor 0.1
        if ( (abs_deltaCompensatedClimbRateCmS <= SMOOTHING_DTE_MIN_AT) || (smoothingDteMin >= SMOOTHING_DTE_MAX ) ){
        expoSmoothDte_auto = smoothingDteMin ;  
    } else if (abs_deltaCompensatedClimbRateCmS >= SMOOTHING_DTE_MAX_AT)  {
        expoSmoothDte_auto = SMOOTHING_DTE_MAX ; 
    } else {
        expoSmoothDte_auto = smoothingDteMin + ( SMOOTHING_DTE_MAX - smoothingDteMin ) * (abs_deltaCompensatedClimbRateCmS - SMOOTHING_DTE_MIN_AT) / (SMOOTHING_DTE_MAX_AT - SMOOTHING_DTE_MIN_AT) ;
    }
    smoothCompensatedClimbRateCmS += expoSmoothDte_auto * (  rawCompensatedClimbRateCmS -  smoothCompensatedClimbRateCmS ) * 0.001; 
    if ( abs( ((int32_t)  smoothCompensatedClimbRateCmS) - compensatedClimbRateCmS) > VARIOHYSTERESIS ) {
        compensatedClimbRateCmS = smoothCompensatedClimbRateCmS  ;
    } 
    if (( config.VspeedCompChannel != 255) && lastRcChannels && ( dteChannelValue < DTE_NO_CHANNEL_COMP_VALUE) ){
        sent2Core0( AIRSPEED_COMPENSATED_VSPEED , compensatedVpseed) ; // used normal Vspeed saved value in vario or mpu 
    } else {
        sent2Core0( AIRSPEED_COMPENSATED_VSPEED , (int32_t) compensatedClimbRateCmS) ;
    }     
        
    
     
} // end calculateVspeedDte

uint16_t VARIO::findVspeedCompensation(){
    //printf("channel= %i\n", config.VspeedCompChannel);
    switch (config.VspeedCompChannel) {
        case 1: return (uint16_t) sbusFrame.rcChannelsData.ch0 ;
        case 2: return (uint16_t) sbusFrame.rcChannelsData.ch1 ;
        case 3: return (uint16_t) sbusFrame.rcChannelsData.ch2 ;
        case 4: return (uint16_t) sbusFrame.rcChannelsData.ch3 ;
        case 5: return (uint16_t) sbusFrame.rcChannelsData.ch4 ;
        case 6: return (uint16_t) sbusFrame.rcChannelsData.ch5 ;
        case 7: return (uint16_t) sbusFrame.rcChannelsData.ch6 ;
        case 8: return (uint16_t) sbusFrame.rcChannelsData.ch7 ;
        case 9: return (uint16_t) sbusFrame.rcChannelsData.ch8 ;
        case 10: return (uint16_t) sbusFrame.rcChannelsData.ch9 ;
        case 11: return (uint16_t) sbusFrame.rcChannelsData.ch10 ;
        case 12: return (uint16_t) sbusFrame.rcChannelsData.ch11 ;
        case 13: return (uint16_t) sbusFrame.rcChannelsData.ch12 ;
        case 14: return (uint16_t) sbusFrame.rcChannelsData.ch13 ;
        case 15: return (uint16_t) sbusFrame.rcChannelsData.ch14 ;
        case 16: return (uint16_t) sbusFrame.rcChannelsData.ch15 ;
    }
    return 0X400; // mid value for 11 bits
}