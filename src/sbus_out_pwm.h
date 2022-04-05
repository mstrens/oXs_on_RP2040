#pragma once

#include "crsf.h"


void setupSbusOutPio();
void setupSbusInPio();
void fillSbusFrame();
void setupPwm();
void updatePWM();
void handleSbusRx();

uint16_t  fmap(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);

