#pragma once

#include "srxl_sensors.h"

//void setupFbusList();

typedef struct gpsCoordinateDDDMMmmmm_s {
    int16_t dddmm;
    int16_t mmmm;
} gpsCoordinateDDDMMmmmm_t;

void setupSrxl2();

void srxl2PioRxHandlerIrq();
void handleSrxl2RxTx(void);
            
bool srxl2FrameIsvalid();
void srxl2ProcessIncomingFrame();
bool srxl2FillTelemetryFrame();
bool srxl2IsFrameDataAvailable(uint8_t frameIdx);
void srxl2AlarmUart_callback(uint alarmNum);
void srxl2SendHandshake();
void replyToHandshake();

void srxl2SendFrame(uint8_t length);  // srxl2TxBuffer is already filled (including CRC)
void srxl2FillTXBuffer(uint8_t frameIdx);

uint16_t srxl2CalculateCrc( uint8_t * buffer , uint8_t length);

void srxl2DecodeRcChannels(uint8_t channelOrFailsafe);

static uint32_t dec2bcd(uint16_t dec) ;// convert an u16 to an U32 bcd
static void GPStoDDDMM_MMMM(int32_t gpsDeg7Dec, gpsCoordinateDDDMMmmmm_t *result) ;

//void sendNextFbusFrame();
//void sendOneFbus(uint8_t idx);

typedef struct {
    STRU_TELE_VARIO_S vario;
    STRU_TELE_GPS_BINARY gps;
    STRU_TELE_SPEED airspeed;
    STRU_TELE_ESC esc;
    STRU_TELE_GPS_LOC gpsLoc;
    STRU_TELE_GPS_STAT gpsStats;
    STRU_TELE_RX_MAH voltCurrentCap;
    STRU_TELE_LIPOMON lipoMon;
} __attribute__((packed)) srxl2Frames_t ;
