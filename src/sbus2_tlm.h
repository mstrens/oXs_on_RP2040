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
void fillReserve1(uint8_t slot8);
void fillReserve2(uint8_t slot8);
void fillSbusHoldCounter(uint8_t slot8);
void fillSbusHoldCounterTotal(uint8_t slot8);
void fillSbusFailsafeCounter(uint8_t slot8);
void fillRpm(uint8_t slot8);
void fillAirspeed(uint8_t slot);
void fillGps(uint8_t slot8);

int64_t sendNextSlot_callback(alarm_id_t id, void *parameters);

// for simulation
//#define SIMULATE_SBUS2_ON_PIN 0 // put as comment to avoid simulation futaba sendet on uart0
                                // can't be used with SEC pin because SEC uses uart0 too
#define SIMU_SBUS2_UART_ID uart0
#define SIMU_SBUS2_BAUDRATE 100000


void setupBus2Simulation();
void generateSbus2RcPacket();

//#define SEND_TOTAL_HOLD  // this is made on request in order to sent the total number of hold (in a rpm field) instead of the current % of hold