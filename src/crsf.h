#pragma once

#include "vario.h"
#include "voltage.h"
#include "gps.h"

#define CRSF_ADDRESS_CRSF_RECEIVER 0xEC // address of the receiver (used for telemetry)

typedef enum
{
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_VARIO = 0x07,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_OPENTX_SYNC = 0x10,
    CRSF_FRAMETYPE_RADIO_ID = 0x3A,
    CRSF_FRAMETYPE_RC_CHANNELS_ = 0x16,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,

    //CRSF_FRAMETYPE_ELRS_STATUS = 0x2E, ELRS good/bad packet count and status flags

    CRSF_FRAMETYPE_COMMAND = 0x32,
    // KISS frames
    CRSF_FRAMETYPE_KISS_REQ  = 0x78,
    CRSF_FRAMETYPE_KISS_RESP = 0x79,
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,   // response request using msp sequence as command
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C, // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
    // Ardupilot frames
    CRSF_FRAMETYPE_ARDUPILOT_RESP = 0x80,
} crsf_frame_type_e;



#define CRSF_FRAME_GPS_PAYLOAD_SIZE 8  // to change
#define CRSF_FRAME_VARIO_PAYLOAD_SIZE 2 
#define CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE 8
#define CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE 8  // to change
#define CRSF_FRAME_FLIGHT_MODE_PAYLOAD_SIZE 8  // to change

typedef enum
{
    CRSF_FRAMEIDX_GPS = 0x00,
    CRSF_FRAMEIDX_VARIO = 0x01,
    CRSF_FRAMEIDX_BATTERY_SENSOR = 0x02,
    CRSF_FRAMEIDX_ATTITUDE = 0x03,
    CRSF_FRAMEIDX_FLIGHT_MODE = 0x04,
} crsf_frame_idx_e;

typedef enum
{
    CRSF_FRAME_NOT_READY = 0x00,
    CRSF_FRAME_READY = 0x01,
    CRSF_FRAME_LOCKED = 0x02,
} crsf_frame_status_e;


struct voltageFrameStruct
{
    uint8_t device_addr; // should be 0xEC (=receiver)
    uint8_t frame_size;  // counts size after this byte, so it must be the payload size + 2 (type and crc)
    uint8_t type;        // from crsf_frame_type_e
    uint16_t mVolt ;
    uint16_t current ;
    uint32_t capacity : 24; // there is only one uint32 (splitted in 24 and 8)
    uint32_t remain : 8 ;
    uint8_t crc;
} __attribute__((packed)) ;

struct varioFrameStruct
{
    uint8_t device_addr; // should be 0xEC (=receiver)
    uint8_t frame_size;  // counts size after this byte, so it must be the payload size + 2 (type and crc)
    uint8_t type;        // from crsf_frame_type_e
    int16_t vSpeed ;     // in 0.1 m/sec
    uint8_t crc;
} __attribute__((packed)) ;

struct gpsFrameStruct
{
    uint8_t  device_addr; // should be 0xEC (=receiver)
    uint8_t  frame_size;  // counts size after this byte, so it must be the payload size + 2 (type and crc)
    uint8_t  type;        // from crsf_frame_type_e
    int32_t  latitude;     //( degree / 10`000`000 )
    int32_t  longitude;    // (degree / 10`000`000 )
    uint16_t groundspeed;  // ( km/h / 10 )
    uint16_t heading;      //( degree / 100 )
    uint16_t altitude;     //( meter ­1000m offset )
    uint8_t  numSat;       //( counter )
    uint8_t crc;
} __attribute__((packed)) ;

/*
0x1E Attitude
Payload:
int16_t     Pitch angle ( rad / 10000 )
int16_t     Roll angle ( rad / 10000 )
int16_t     Yaw angle ( rad / 10000 )
*/
struct attitudeFrameStruct
{
    uint8_t  device_addr; // should be 0xEC (=receiver)
    uint8_t  frame_size;  // counts size after this byte, so it must be the payload size + 2 (type and crc)
    uint8_t  type;        // from crsf_frame_type_e
    int16_t  pitch;     //( rad/1000 )
    int16_t  roll;    // (rad / 1000 )
    int16_t  yaw;     // ( rad / 1000 )
    uint8_t crc;
} __attribute__((packed)) ;


void setup_DMA_PIO(); 
void fillCRSFFrame();
void setupCRSF();
bool dataAvailable(uint8_t idx);
void fillFrameBattery(uint8_t idx);
void fillFrameVario(uint8_t idx);
void fillFrameGps(uint8_t idx);
void fillFrameAttitude(uint8_t idx);
void fillOneFrame(uint8_t idx);


