#pragma once


#ifndef MS4525_ADDRESS
#define MS4525_ADDRESS  0X28 // 0x28 is the default I2C adress of a 4525DO sensor
#endif

#include "stdint.h"

extern float difPressureAirspeedSumPa; // calculate a moving average on x values
extern uint32_t difPressureAirspeedCount;
extern float difPressureCompVspeedSumPa; // calculate a moving average on x values
extern uint32_t difPressureCompVspeedCount;
extern float temperatureKelvin;     // in Kelvin , used when compensation is calculated


class MS4525 {
public:
    MS4525(uint8_t deviceAddress);
    bool  airspeedInstalled = false;
    
    void begin();
    void getDifPressure();
private:
    uint8_t  _address;    
    uint8_t readBuffer[4]; // read the I2C
    
    bool calibrated4525 = false;
    int32_t difPressureCalSum = 0;  // use to calibrate the sensor at reset
    int calibrateCount = 0;
    
    uint32_t prevReadUs ;
    float offset ;
    float difPressurePa; 
    
}; // end class OXS_4525

    //    uint32_t prevAirspeedMsAvailable ;      
   //float difPressureAdc_0 ;
   //float abs_deltaDifPressureAdc ;
   //float smoothDifPressureAdc ;  // in steps ADC/
   //float expoSmoothFactor ;  // smoothing factor

   //unsigned long  nextAirSpeedMillis ; //next time that airspeed must be available

    //int32_t airSpeedKmH ;        // in km/h (no decimal)
    //bool airspeedReset = true;   // force a reset (recalibration)
    //float smoothAirSpeedCmS ;    //cm/sec ; use in glider ratio
    //float difPressureAdc_zero ; 
    //float difPressureAdc_0SumValue;
    //uint32_t difPressureAdc_0SumCount;
    
