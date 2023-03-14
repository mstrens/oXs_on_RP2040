#pragma once


#ifndef SDPXX_ADDRESS
#define SDPXX_ADDRESS 0X21 // 0x21 is the default I2C adress of a SDP3X sensor (see config.h for sdp810)
#endif

#include "stdint.h"


class SDP3X {
public:
    SDP3X(uint8_t deviceAddress) ;
    bool  airspeedInstalled = false;
    float temperatureCelsius = 0;

    void begin();
    void getDifPressure();
    
private:
   uint8_t _address; // I2c adress
   
   float dpScaleSdp3x;      // differential pressure scale factor
   uint8_t readBuffer[9];                 // get 2 bytes returned by SDP3X
   uint32_t prevReadUs ;
   float difPressurePa; 
    
}; // end class OXS_SDP3X

