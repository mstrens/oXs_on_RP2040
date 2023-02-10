#pragma once

#include "pico/stdlib.h"
//#include "Button2.h" moved to Button2.cpp
//#include "stdio.h"

#define SPORT_TYPES_MAX 24 // = NUMBER_MAX_IDX

struct field {
    int32_t value;
    bool available;
    uint32_t nextMillis;
    uint16_t interval; // msec
    uint8_t sportDeviceId;
    uint16_t sportFieldId;
} ;


// default SPORT_SENSOR_ID
#define DATA_ID_VARIO  0x00  // = sensor 0 used for Alt and Vspeed
#define DATA_ID_FLVSS  0xA1  //          1
#define DATA_ID_FAS    0x22  //          2
#define DATA_ID_GPS    0x83  //          3 used for all GPS data
#define DATA_ID_RPM    0xE4  //          4
#define DATA_ID_ACC    0x67  //          7
//list of 28 device ID codes is (in sequence)
// 0x00,0xA1,0x22,0x83,0xE4,0x45,0xC6,0x67,0x48,0xE9,0x6A,0xCB,0xAC,0x0D,0x8E,0x2F,0xD0,0x71,0xF2,0x53,0x34,0x95,0x16,0xB7,0x98,0x39,0xBA,0x1B


// FrSky new DATA IDs (2 bytes) (copied from openTX telemetry/frsky_sport.cpp on jan 2023) // those values are not used directly but bits 4 up to 11 are stored in an array in oXs_out_frsky.cpp 
// FrSky new DATA IDs (2 bytes)
#define ALT_FIRST_ID              0x0100
#define ALT_LAST_ID               0x010F
#define VARIO_FIRST_ID            0x0110
#define VARIO_LAST_ID             0x011F
#define CURR_FIRST_ID             0x0200
#define CURR_LAST_ID              0x020F
#define VFAS_FIRST_ID             0x0210
#define VFAS_LAST_ID              0x021F
#define CELLS_FIRST_ID            0x0300
#define CELLS_LAST_ID             0x030F
#define T1_FIRST_ID               0x0400
#define T1_LAST_ID                0x040F
#define T2_FIRST_ID               0x0410
#define T2_LAST_ID                0x041F
#define RPM_FIRST_ID              0x0500
#define RPM_LAST_ID               0x050F
#define FUEL_FIRST_ID             0x0600
#define FUEL_LAST_ID              0x060F
#define ACCX_FIRST_ID             0x0700
#define ACCX_LAST_ID              0x070F
#define ACCY_FIRST_ID             0x0710
#define ACCY_LAST_ID              0x071F
#define ACCZ_FIRST_ID             0x0720
#define ACCZ_LAST_ID              0x072F
#define GPS_LONG_LATI_FIRST_ID    0x0800
#define GPS_LONG_LATI_LAST_ID     0x080F
#define GPS_ALT_FIRST_ID          0x0820
#define GPS_ALT_LAST_ID           0x082F
#define GPS_SPEED_FIRST_ID        0x0830
#define GPS_SPEED_LAST_ID         0x083F
#define GPS_COURS_FIRST_ID        0x0840
#define GPS_COURS_LAST_ID         0x084F
#define GPS_TIME_DATE_FIRST_ID    0x0850
#define GPS_TIME_DATE_LAST_ID     0x085F
#define A3_FIRST_ID               0x0900
#define A3_LAST_ID                0x090F
#define A4_FIRST_ID               0x0910
#define A4_LAST_ID                0x091F
#define AIR_SPEED_FIRST_ID        0x0A00
#define AIR_SPEED_LAST_ID         0x0A0F
#define FUEL_QTY_FIRST_ID         0x0A10
#define FUEL_QTY_LAST_ID          0x0A1F
#define RBOX_BATT1_FIRST_ID       0x0B00
#define RBOX_BATT1_LAST_ID        0x0B0F
#define RBOX_BATT2_FIRST_ID       0x0B10
#define RBOX_BATT2_LAST_ID        0x0B1F
#define RBOX_STATE_FIRST_ID       0x0B20
#define RBOX_STATE_LAST_ID        0x0B2F
#define RBOX_CNSP_FIRST_ID        0x0B30
#define RBOX_CNSP_LAST_ID         0x0B3F
#define SD1_FIRST_ID              0x0B40
#define SD1_LAST_ID               0x0B4F
#define ESC_POWER_FIRST_ID        0x0B50
#define ESC_POWER_LAST_ID         0x0B5F
#define ESC_RPM_CONS_FIRST_ID     0x0B60
#define ESC_RPM_CONS_LAST_ID      0x0B6F
#define ESC_TEMPERATURE_FIRST_ID  0x0B70
#define ESC_TEMPERATURE_LAST_ID   0x0B7F
#define RB3040_OUTPUT_FIRST_ID    0x0B80
#define RB3040_OUTPUT_LAST_ID     0x0B8F
#define RB3040_CH1_2_FIRST_ID     0x0B90
#define RB3040_CH1_2_LAST_ID      0x0B9F
#define RB3040_CH3_4_FIRST_ID     0x0BA0
#define RB3040_CH3_4_LAST_ID      0x0BAF
#define RB3040_CH5_6_FIRST_ID     0x0BB0
#define RB3040_CH5_6_LAST_ID      0x0BBF
#define RB3040_CH7_8_FIRST_ID     0x0BC0
#define RB3040_CH7_8_LAST_ID      0x0BCF
#define X8R_FIRST_ID              0x0C20
#define X8R_LAST_ID               0x0C2F
#define S6R_FIRST_ID              0x0C30
#define S6R_LAST_ID               0x0C3F
#define GASSUIT_TEMP1_FIRST_ID    0x0D00
#define GASSUIT_TEMP1_LAST_ID     0x0D0F
#define GASSUIT_TEMP2_FIRST_ID    0x0D10
#define GASSUIT_TEMP2_LAST_ID     0x0D1F
#define GASSUIT_SPEED_FIRST_ID    0x0D20
#define GASSUIT_SPEED_LAST_ID     0x0D2F
#define GASSUIT_RES_VOL_FIRST_ID  0x0D30
#define GASSUIT_RES_VOL_LAST_ID   0x0D3F
#define GASSUIT_RES_PERC_FIRST_ID 0x0D40
#define GASSUIT_RES_PERC_LAST_ID  0x0D4F
#define GASSUIT_FLOW_FIRST_ID     0x0D50
#define GASSUIT_FLOW_LAST_ID      0x0D5F
#define GASSUIT_MAX_FLOW_FIRST_ID 0x0D60
#define GASSUIT_MAX_FLOW_LAST_ID  0x0D6F
#define GASSUIT_AVG_FLOW_FIRST_ID 0x0D70
#define GASSUIT_AVG_FLOW_LAST_ID  0x0D7F
#define SBEC_POWER_FIRST_ID       0x0E50
#define SBEC_POWER_LAST_ID        0x0E5F
#define DIY_FIRST_ID              0x5100
#define DIY_LAST_ID               0x52FF
#define DIY_STREAM_FIRST_ID       0x5000
#define DIY_STREAM_LAST_ID        0x50FF
#define SERVO_FIRST_ID            0x6800
#define SERVO_LAST_ID             0x680F
#define FACT_TEST_ID              0xF000
#define VALID_FRAME_RATE_ID       0xF010
#define RSSI_ID                   0xF101
#define ADC1_ID                   0xF102
#define ADC2_ID                   0xF103
#define BATT_ID                   0xF104
#define RAS_ID                    0xF105
#define XJT_VERSION_ID            0xF106
#define R9_PWR_ID                 0xF107
#define SP2UART_A_ID              0xFD00
#define SP2UART_B_ID              0xFD01

// added by ms to support ELRS data
#define UPLINK_RSSI_1_ID        0x5200  // to check if this range is valid
#define UPLINK_RSSI_2_ID        0x5201
#define UPLINK_LINK_QUALITY_ID  0x5202
#define UPLINK_SNR_ID           0x5203
#define ACTIVE_ANTENNA_ID       0x5204
#define RF_MODE_ID              0x5205
#define UPLINK_TX_POWER_ID      0x5206
#define DOWNLINK_RSSI_ID        0x5207
#define DOWNLINK_LINK_QUALITY_ID  0x5208
#define DOWNLINK_SNR_ID           0x5209

#define DIY_GPS_NUM_SAT        0X5100
#define DIY_GPS_PDOP           0X5101
#define DIY_GPS_HOME_BEARING   0X5102
#define DIY_GPS_HOME_DISTANCE  0X5103

#define DIY_VOLT3              0X5113
#define DIY_VOLT4              0X5114
#define DIY_PITCH              0X5120
#define DIY_ROLL               0X5121
#define DIY_YAW                0X5122

enum fieldIdx {     // Internal Id for the measurements stored in oXs and that can be sent (some in a different format/field)
      LATITUDE =0,  //  GPS
      LONGITUDE,    //  GPS
      GROUNDSPEED , //  GPS
      HEADING,      //  GPS
      ALTITUDE ,    //  GPS

      NUMSAT ,      //  GPS   5
      GPS_DATE ,    // GPS
      GPS_TIME ,    // GPS
      GPS_PDOP ,    // GPS
      GPS_HOME_BEARING, // GPS

      GPS_HOME_DISTANCE, // GPS  10
      MVOLT,        // volt1  
      CURRENT,  // volt2 must be in seq for voltage.cpp
      RESERVE1, // volt3 must be in seq for voltage.cpp
      RESERVE2, // volt4 must be in seq for voltage.cpp
      
      CAPACITY,    // based on current (volt2)
      TEMP1,       // = Volt3 but saved as temp
      TEMP2,       // = Volt4 but saved as temp
      VSPEED,      // baro       
      RELATIVEALT , // baro      
      
      PITCH,       // imu        20 
      ROLL,        // imu        
      YAW ,        // not used to save data
      RPM ,        // RPM sensor  
      NUMBER_MAX_IDX, // used to count the number of entries  24
};


uint32_t millis() ;

uint32_t micros();

void waitUs(uint32_t delayUs);

bool __no_inline_not_in_flash_func(get_bootsel_button)();

typedef struct
{
    uint8_t type;
    int32_t data;
} queue_entry_t;

void sent2Core0( uint8_t fieldType, int32_t value);

void setupListOfFields();


void enlapsedTime(uint8_t idx);


void startTimerUs(uint8_t idx); 

void getTimerUs(uint8_t idx);