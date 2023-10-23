#pragma once

#include <ctype.h>
#include "pico/stdlib.h"
#include "stdio.h"


#define GYRO_VERSION 0

enum STICK_GAIN_THROW {STICK_GAIN_THROW_FULL=1, STICK_GAIN_THROW_HALF=2, STICK_GAIN_THROW_QUARTER=3}; // values are important
enum MAX_ROTATE {MAX_ROTATE_VLOW=1, MAX_ROTATE_LOW=2, MAX_ROTATE_MED=3, MAX_ROTATE_HIGH=4}; // values are important
enum RATE_MODE_STICK_ROTATE {RATE_MODE_STICK_ROTATE_DISABLE=1, RATE_MODE_STICK_ROTATE_ENABLE};


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

