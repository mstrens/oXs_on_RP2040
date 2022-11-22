#pragma once

#include "crsf_frames.h"



void fillCRSFFrame();
void setupCrsfOut();
bool dataAvailable(uint8_t idx);
void fillFrameBattery(uint8_t idx);
void fillFrameVario(uint8_t idx);
void fillFrameGps(uint8_t idx);
void fillFrameAttitude(uint8_t idx);
void fillFrameBaroAltitude(uint8_t idx);
void fillOneFrame(uint8_t idx);

void printAttitudeFrame();
void printGpsFrame();
void printBatteryFrame();