#pragma once

#include <ctype.h>
#include "pico/stdlib.h"
#include "stdio.h"


#define GYROMIXER_VERSION 0

// stabilization mode
enum STAB_MODE {STAB_RATE, STAB_HOLD};
enum LEARNING_STATE {LEARNING_OFF, LEARNING_INIT, LEARNING_WAIT, LEARNING_MIXERS, LEARNING_MIXERS_DONE, LEARNING_LIMITS};
enum STICK_GAIN_THROW {STICK_GAIN_THROW_FULL=1, STICK_GAIN_THROW_HALF=2, STICK_GAIN_THROW_QUARTER=3}; // values are important
enum MAX_ROTATE {MAX_ROTATE_VLOW=1, MAX_ROTATE_LOW=2, MAX_ROTATE_MED=3, MAX_ROTATE_HIGH=4}; // values are important
enum RATE_MODE_STICK_ROTATE {RATE_MODE_STICK_ROTATE_DISABLE=1, RATE_MODE_STICK_ROTATE_ENABLE};

// codification of the orientation
// first word gives the direction of the base of the mpu (normally down), front means that the base point to the nose of the plane, left to the left wing....
// second word gives the direction of the Y axis (normally front)
// there are 6X4 possible orientations
enum MPU_ORIENTATION {DOWN_FRONT = 0, DOWN_RIGHT, DOWN_BACK, DOWN_LEFT ,\
                      UP_FRONT, UP_RIGHT, UP_BACK, UP_LEFT ,\
                      LEFT_FRONT, LEFT_UP, LEFT_BACK, LEFT_DOWN ,\
                      RIGHT_FRONT, RIGHT_UP, RIGHT_BACK, RIGHT_DOWN ,\
                      FRONT_UP, FRONT_RIGHT , FRONT_DOWN , FRONT_LEFT,\
                      BACK_UP, BACK_RIGHT , BACK_DOWN , BACK_LEFT   };
// roll increases when right wing goes down, 
// pitch increases when nose does up
// yaw increases when the plane turn to the right
// Normally acc Y point to the nose of the plan, acc X point to the right and Z to up (so acc Z is normally -1g)
// Then if gyro Y is positive==> Roll increase, gyro X positive=> pitch increase, gyroZ positive=>yaw increase




struct gyroMixer_t{
    uint8_t version = GYROMIXER_VERSION;
    bool isCalibrated = false;
    bool used[16] ; //true means that this Rc channel is impacted by gyro compensation

    uint16_t neutralUs[16]    ; //PWM (us) value for this Rc channel when ail, elv and rud are all in neutral position 
    uint16_t minUs[16]        ; //min PWM value when ail, elv and rud sticks are moved in all directions
    uint16_t maxUs[16]        ; //max PWM value when ail, elv and rud sticks are moved in all directions

    int16_t rateRollLeftUs[16] ; //difference of PWM value between neutral and 100% ail stick on left side
    int16_t rateRollRightUs[16]; //difference of PWM value between neutral and 100% ail stick on right side
    int16_t ratePitchUpUs[16] ;  //difference of PWM value between neutral and 100% elv stick on up side
    int16_t ratePitchDownUs[16] ; //difference of PWM value between neutral and 100% elv stick on down side
    int16_t rateYawLeftUs[16]   ; //difference of PWM value betwwen neutral and 100% rud stick on left side
    int16_t rateYawRightUs[16]  ; //difference of PWM value betwwen neutral and 100% rud stick on right side
};

struct _pid_param {
  int16_t kp[3]; // [0, 1000] 11b signed    // 3 values because one per axis
  int16_t ki[3];
  int16_t kd[3];
  int8_t output_shift ;
};

void compute_pid(struct _pid_state *ppid_state, struct _pid_param *ppid_param);
void applyGyroCorrections(); // apply gyro corrections taking care of the gyro mixers

void calculateCorrectionsToApply(); // calculate gyro corrections on 3 axis but without taking care of the mixer (so do not apply them)

void calibrateGyroMixers();
int pc(int16_t val);        // convert a Rc value (-512/512) in %
