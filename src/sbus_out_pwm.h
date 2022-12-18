#pragma once

//#include "crsf.h"
enum LEDState{
    STATE_OK= 0,
    STATE_PARTLY_OK,
    STATE_FAILSAFE,
    STATE_NO_SIGNAL
};


// functions used for the 8 PWM signals generated without the pio.
void setupSbusOutPio();
void setupSbusInPio();
void fillSbusFrame();
void setupPwm();
void updatePWM();
void handleSbusRx();

uint16_t  fmap(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);

// functions used by the PIO for 2 additionnal PWM 
void pio_pwm_set_period(PIO pio, uint sm, uint32_t period);
void setupPioPwm();
void updatePioPwm();