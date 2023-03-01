#pragma once

void setupFportList();

void setupFport();

void fportPioRxHandlerIrq();
void handleFportRxTx(void);
            
bool processNextInputByte( uint8_t c);

bool check_checksum(void);
uint8_t crc_sum8(const uint8_t *p, uint8_t len);

void fportDecodeRcChannels();

void sendNextFportFrame();
void sendOneFport(uint8_t idx);
