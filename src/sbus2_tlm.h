#pragma once
#include <stdio.h>
#include "pico/stdlib.h"


void setupSbus2Tlm();

void enableSbus2Transmit();
void disableSbus2Transmit();

void fill8Sbus2Slots (uint8_t slotGroup);

void fillVario(uint8_t slot8);
void fillBattery(uint8_t slot8);
void fillTemp1(uint8_t slot8);
void fillTemp2(uint8_t slot8);
void fillRpm(uint8_t slot8);
void fillGps(uint8_t slot8);

int64_t sendNextSlot_callback(alarm_id_t id, void *parameters);