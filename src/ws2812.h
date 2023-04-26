#pragma once


void setupLed();
void setRgbColorOn(uint8_t red , uint8_t green , uint8_t blue);
void setRgbColor(uint8_t red , uint8_t green , uint8_t blue);

void setRgbOn();
void setRgbOff();
void toggleRgb();
void blinkRgb(uint8_t red , uint8_t green , uint8_t blue, uint32_t period , uint32_t count);
void checkLedColors();
