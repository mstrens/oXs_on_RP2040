#pragma once

//#include "crsf.h"
enum LEDState{
    STATE_OK= 0,
    STATE_PARTLY_OK,
    STATE_FAILSAFE,
    STATE_NO_SIGNAL,              // is also used in case of error during the learning process
    STATE_GYRO_CAL_MIXER_NOT_DONE,
    STATE_GYRO_CAL_MIXER_DONE,
    STATE_GYRO_CAL_LIMIT, 
};


// functions used for the 8 PWM signals generated without the pio.
void setupSbusOutPio();
void setupSbusInPio();

void setRcChannels();

void fillSbusFrame();
void setupPwm();
void updatePWM();
void handleSbusRx();

uint16_t  fmap(uint16_t x); // Convert sbus to pwm usec
uint16_t fmapMinMax(int x);
uint16_t  fmapExtended(uint16_t x); // convert Sbus value to Pwm value using extended range


// functions used by the PIO for 2 additionnal PWM 
// this part is not used anymore (pio pwm)
//void pio_pwm_set_period(PIO pio, uint sm, uint32_t period);
//void setupPioPwm();
//void updatePioPwm();

void applyPwmValue( uint8_t pin , uint16_t pwmValue); // apply PWM value on a pin

void setLedState();                // set the color of the led