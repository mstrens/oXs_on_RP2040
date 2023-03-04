#pragma once

void setupFbusList();

void setupFbus();

void fbusPioRxHandlerIrq();
void handleFbusRxTx(void);
            
bool processNextInputByte( uint8_t c);

bool check_checksum(void);
uint8_t crc_sum8(const uint8_t *p, uint8_t len);

void fbusDecodeRcChannels();

void sendNextFbusFrame();
void sendOneFbus(uint8_t idx);
