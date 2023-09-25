#pragma once
#include "stdio.h"
#include "stdint.h"


void setupEscHobbyV4();
void handleEscHobbyV4();
void processNextEscInputByte( uint8_t data);
void escPioRxHandlerIrq();