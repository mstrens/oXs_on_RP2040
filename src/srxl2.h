#pragma once

#include "srxl_sensors.h"

//void setupFbusList();

void setupSrxl2();

void srxl2PioRxHandlerIrq();
void handleSrxl2RxTx(void);
            
bool srxl2FrameIsvalid();
void srxl2ProcessIncomingFrame();
bool srxl2FillTelemetryFrame();
bool srxl2IsFrameDataAvailable(uint8_t frameIdx);
void srxl2SendHandshake();

void srxl2SendFrame(uint8_t length);  // srxl2TxBuffer is already filled (including CRC)
void srxl2FillTXBuffer(uint8_t frameIdx);

uint16_t srxl2CalculateCrc( uint8_t * buffer , uint8_t length);

void srxl2DecodeRcChannels();

//void sendNextFbusFrame();
//void sendOneFbus(uint8_t idx);

typedef struct {
    STRU_TELE_VARIO_S vario;
    STRU_TELE_GPS_BINARY gps;
    STRU_TELE_RX_MAH voltCurrentCap;

} srxl2Frames_t ;
