#pragma once


#ifndef MS4525_ADDRESS
#define MS4525_ADDRESS 0X28 // 0x28 is the default I2C adress of a 4525DO sensor
#endif

#include "stdint.h"


//#define CALCULATEINTEGER

extern float actualPressure ;

class MS4525 {
public:
    MS4525(uint8_t deviceAddress);
    bool  airspeedInstalled = false;
    float temperatureKelvin;     // in Kelvin , used when compensation is calculated
    int32_t airSpeedKmH ;        // in km/h (no decimal)
    bool airspeedReset ;
    float smoothAirSpeedCmS ;    //cm/sec ; use in glider ratio
    float difPressureAdc_zero ; 
    float difPressureAdc_0SumValue;
    uint32_t difPressureAdc_0SumCount;
    

    void begin();
    void getAirspeed();
private:
    uint8_t  _address;    
    uint8_t readBuffer[4];
    uint32_t prevReadUs ;
    uint32_t prevAirspeedMsAvailable ;     
    
    bool calibrated4525 ;
    int calibrateCount4525 ;
    int32_t difPressureSum ;
   
   int32_t difPressureAdc;          // in steps ADC 
   int32_t temperatureAdc ;   // in steps ADC
   
   float offset4525 ; 
   float difPressureAdc_0 ;
   float abs_deltaDifPressureAdc ;
   float smoothDifPressureAdc ;  // in steps ADC/

   float expoSmooth4525_adc_auto ;
//   float smoothAirSpeed ;    //cm/sec
//  float rawAirSpeed ;       // cm/sec

   unsigned long  airSpeedMillis ; //save time when airspeed is made available
   unsigned long  nextAirSpeedMillis ; //next time that airspeed must be available
  
}; // end class OXS_4525

