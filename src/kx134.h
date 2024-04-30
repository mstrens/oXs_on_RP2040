#pragma once


#include "stdint.h"

class KX134
{
public:
    explicit KX134(int a);  
    bool kx134Installed = false; 
    uint32_t lastKx134Us = 0;
    void begin();
    void getAcc();
private:
    int32_t sumAx  = 0;
    int32_t sumAy  = 0;
    int32_t sumAz  = 0;
    int32_t countSumAcc = 0;
    uint32_t lastSendAccXYZMs = 0;
};


// -- END OF FILE --

