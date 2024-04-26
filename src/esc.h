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
void processBlhFrame();
int32_t calcTemp(float tempRaw);


uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);
uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen);




