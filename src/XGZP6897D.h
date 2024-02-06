#pragma once



#define XGZP_ADDRESS  0X6D // 0x6D is the only one I2C adress of a XGZP sensor

#include "stdint.h"

extern float difPressureAirspeedSumPa; // calculate a moving average on x values
extern uint32_t difPressureAirspeedCount;
extern float difPressureCompVspeedSumPa; // calculate a moving average on x values
extern uint32_t difPressureCompVspeedCount;
extern float temperatureKelvin;     // in Kelvin , used when compensation is calculated


class XGZP {
public:
    XGZP(uint8_t deviceAddress);
    bool  airspeedInstalled = false;
    
    void begin();
    void getDifPressure();
private:
    uint8_t  _address;    
    uint8_t readBuffer[6]; // read the I2C
    
    bool calibrated = false;
    int32_t difPressureCalSum = 0;  // use to calibrate the sensor at reset
    int calibrateCount = 0;
    
    uint32_t prevReadUs ;
    float offset ;
    float difPressurePa; 
    
}; // end class XGZP

