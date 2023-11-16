#pragma once
#include "stdio.h"
#include "stdint.h"


void setupEsc();
void handleEsc();
void processEscFrame();
void escPioRxHandlerIrq();
void processHW4Frame();
void processHW3Frame();
void processKontronikFrame();
void processZTW1Frame();
int32_t calcTemp(float tempRaw);

/*  Hobbywing V4
1.Frame with 19bytes
2.Frame with 13bytes
Before throttle is raised from 0, signature packets are sent between telemetry packets. This is used to identify the hardware and firmware of the ESC.
Telemetry packets:
Byte | 1 | 2 | 3 | 4 | 5 | 6 |
Value | Package Head (0x9B) | Package Number 1 | Package Number 2 | Package Number 3 | Rx Throttle 1 | Rx Throttle 2 |
7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
Output PWM 1 | Output PWM 2 | RPM 1 | RPM 2 | RPM 3 | Voltage 1 | Voltage 2 | Current 1 | Current 2 | TempFET 1 |
17 | 18 | 19
TempFET 2 | Temp 1 | Temp 2
Signature packets:
Byte | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13
V4LV25/60/80A | 0x9B | 0x9B | 0x03 | 0xE8 | 0x01 | 0x08 | 0x5B | 0x00 | 0x01 | 0x00 | 0x21 | 0x21 | 0xB9
*/
