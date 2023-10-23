#pragma once

#include <ctype.h>
#include "pico/stdlib.h"
#include "stdio.h"


#define GYRO_VERSION 0

struct gyroMixer_t{
    uint8_t version = GYRO_VERSION;
    bool used ; //true means that this Rc channel is impacted by gyro compensation

    uint16_t neutralUs    ; //PWM (us) value for this Rc channel when ail, elv and rud are all in neutral position 
    uint16_t minUs        ; //min PWM value when ail, elv and rud sticks are moved in all directions
    uint16_t maxUs        ; //max PWM value when ail, elv and rud sticks are moved in all directions

    int16_t rateRollLeftUs ; //difference of PWM value between neutral and 100% ail stick on left side
    int16_t rateRollRightUs; //difference of PWM value between neutral and 100% ail stick on right side
    int16_t ratePitchUpUs ;  //difference of PWM value between neutral and 100% elv stick on up side
    int16_t ratePitchDownUs ; //difference of PWM value between neutral and 100% elv stick on down side
    int16_t rateYawLeftUs   ; //difference of PWM value betwwen neutral and 100% rud stick on left side
    int16_t rateYawRightUs  ; //difference of PWM value betwwen neutral and 100% rud stick on right side
};

