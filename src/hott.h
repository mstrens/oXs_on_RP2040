#pragma once

//#include "Arduino.h"

// Hott protocol v4 delay
#define HOTTV4_TX_DELAY 1000 // Delai entre octets (usec)
#define HOTTV4_REPLY_DELAY 5000 // delay before starting sending a reply to a request (usec)
#define HOTTV4_END_SENDING_DELAY 25000 // delay to wait when dma is empty to let the last byte being sent (e.g. to send 10 bytes) 

// first byte sent by Rx for polling can have 2 values
#define HOTT_BINARY_MODE_REQUEST_ID      0x80
#define HOTT_TEXT_MODE_REQUEST_ID       0x7f

// in binary mode, second byte identifies one of 4 sensor types
#define HOTT_TELEMETRY_VARIO_SENSOR_ID  0x89 //Graupner #33601 Vario Module
#define HOTT_TELEMETRY_GPS_SENSOR_ID    0x8a //Graupner #33600 Gps module
#define HOTTV4_ESC_SENSOR_ID            0x8C
#define HOTT_TELEMETRY_GAM_SENSOR_ID    0x8d // General Air Module ID 
#define HOTT_TELEMETRY_GEA_SENSOR_ID    0x8E // Electric Air Module ID
// note : when sensor relies, it sent first 2 bytes with the value = ??????????????? and then it starts a set of bytes depending on the type of sensor.


// in text mode, second byte identifies one of 4 sensor types
#define HOTT_GPS_SENSOR_TEXT_ID 0xA0 // GPS Module ID          

#define HOTTV4_VARIO_SENSOR_TEXT_ID          0x90   // Vario Module Text ID
#define HOTTV4_GPS_SENSOR_TEXT_ID            0xA0   // GPS Module Text ID
#define HOTTV4_ESC_SENSOR_TEXT_ID            0xC0
#define HOTTV4_GENERAL_AIR_SENSOR_TEXT_ID    0xD0
#define HOTTV4_ELECTRICAL_AIR_SENSOR_TEXT_ID 0xE0   // Electric Air Module Text ID


#define HOTTV4_BUTTON_PREV                   0x07
#define HOTTV4_BUTTON_SET                    0x09
#define HOTTV4_BUTTON_DEC                    0x0B
#define HOTTV4_BUTTON_INC                    0x0D
#define HOTTV4_BUTTON_NEXT                   0x0E
#define HOTTV4_BUTTON_NIL                    0x0F


#define TXHOTTDATA_BUFFERSIZE 45

// structure of GENERAL AIR MODULE 
typedef struct {
  uint8_t start_byte;          //#01 start byte constant value 0x7c
  uint8_t gam_sensor_id;       //#02 EAM sensort id. constat value 0x8d=GENRAL AIR MODULE
  uint8_t warning_beeps;       //#03 1=A 2=B ... 0x1a=Z  0 = no alarm
                  /* VOICE OR BIP WARNINGS
                    Alarme sonore A.. Z, octet correspondant 1 à 26
                    0x00  00  0  No alarm
                    0x01  01  A  
                    0x02  02  B  Negative Difference 2 B
                    0x03  03  C  Negative Difference 1 C
                    0x04  04  D  
                    0x05  05  E  
                    0x06  06  F  Min. Sensor 1 temp. F
                    0x07  07  G  Min. Sensor 2 temp. G
                    0x08  08  H  Max. Sensor 1 temp. H
                    0x09  09  I  Max. Sensor 2 temp. I
                    0xA   10  J  Max. Sens. 1 voltage J
                  0xB   11  K  Max. Sens. 2 voltage K
                  0xC   12  L  
                  0xD   13  M  Positive Difference 2 M
                  0xE   14  N  Positive Difference 1 N
                  0xF   15  O  Min. Altitude O
                  0x10  16  P  Min. Power Voltage P    // We use this one for Battery Warning
                  0x11  17  Q  Min. Cell voltage Q
                  0x12  18  R  Min. Sens. 1 voltage R
                  0x13  19  S  Min. Sens. 2 voltage S
                  0x14  20  T  Minimum RPM T
                  0x15  21  U  
                  0x16  22  V  Max. used capacity V
                  0x17  23  W  Max. Current W
                  0x18  24  X  Max. Power Voltage X
                  0x19  25  Y  Maximum RPM Y
                  0x1A  26  Z  Max. Altitude Z
                        */
  uint8_t sensor_id;                       //#04 constant value 0xd0 for GAM, other values for other modules
  uint8_t alarm_invers1;                   //#05 alarm bitmask. Value is displayed inverted
              //Bit#  Alarm field
            // 0    all cell voltage
            // 1    Battery 1
            // 2    Battery 2
            // 3    Temperature 1
            // 4    Temperature 2
            // 5    Fuel
            // 6    mAh
            // 7    Altitude
  uint8_t alarm_invers2;                    //#06 alarm bitmask. Value is displayed inverted
                //Bit#  Alarm Field
                // 0    main power current
                // 1    main power voltage
                // 2    Altitude
                // 3    m/s                            
                  // 4    m/3s
                  // 5    unknown
                // 6    unknown
                // 7    "ON" sign/text msg active
  uint8_t cell[6];                         //#7 Volt Cell 1 (in 2 mV increments, 210 == 4.20 V)
                                        //#8 Volt Cell 2 (in 2 mV increments, 210 == 4.20 V)
                                        //#9 Volt Cell 3 (in 2 mV increments, 210 == 4.20 V)
                                        //#10 Volt Cell 4 (in 2 mV increments, 210 == 4.20 V)
                                        //#11 Volt Cell 5 (in 2 mV increments, 210 == 4.20 V)
                                        //#12 Volt Cell 6 (in 2 mV increments, 210 == 4.20 V)
  uint16_t  Battery1;                   //#13 LSB battery 1 voltage LSB value. 0.1V steps. 50 = 5.5V only pos. voltages
                                        //#14 MSB 
  uint16_t  Battery2;                   //#15 LSB battery 2 voltage LSB value. 0.1V steps. 50 = 5.5V only pos. voltages
                                        //#16 MSB
  uint8_t temperature1;                    //#17 Temperature 1. Offset of 20. a value of 20 = 0°C
  uint8_t temperature2;                    //#18 Temperature 2. Offset of 20. a value of 20 = 0°C
  uint8_t fuel_procent;                    //#19 Fuel capacity in %. Values 0--100
                                        //graphical display ranges: 0-100% with new firmwares of the radios MX12/MX20/...
  uint16_t fuel_ml;                     //#20 LSB Fuel in ml scale. Full = 65535!
                                        //#21 MSB
  uint16_t rpm;                         //#22 RPM in 10 RPM steps. 300 = 3000rpm
                                        //#23 MSB
  uint16_t altitude;                    //#24 altitude in meters. offset of 500, 500 = 0m
                                        //#25 MSB
  uint16_t climbrate_L;                 //#26 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
                                        //#27 MSB
  uint8_t climbrate3s;                     //#28 climb rate in m/3sec. Value of 120 = 0m/3sec
  uint16_t current;                     //#29 current in 0.1A steps 100 == 10,0A
                                        //#30 MSB current display only goes up to 99.9 A (continuous)
  uint16_t main_voltage;                //#31 LSB Main power voltage using 0.1V steps 100 == 10,0V
                                        //#32 MSB (Appears in GAM display right as alternate display.)
  uint16_t batt_cap;                    //#33 LSB used battery capacity in 10mAh steps
                                        //#34 MSB 
  uint16_t speed;                       //#35 LSB (air?) speed in km/h(?) we are using ground speed here per default
                                        //#36 MSB speed
  uint8_t min_cell_volt;                   //#37 minimum cell voltage in 2mV steps. 124 = 2,48V
  uint8_t min_cell_volt_num;               //#38 number of the cell with the lowest voltage
  uint16_t rpm2;                        //#39 LSB 2nd RPM in 10 RPM steps. 100 == 1000rpm
                                        //#40 MSB
  uint8_t general_error_number;            //#41 General Error Number (Voice Error == 12) TODO: more documentation
  uint8_t pressure;                        //#42 High pressure up to 16bar. 0,1bar scale. 20 == 2.0bar
  uint8_t version;                         //#43 version number (Bytes 35 .43 new but not yet in the record in the display!)
  uint8_t stop_byte;                       //#44 stop byte 0x7D
  uint8_t parity;                          //#45 CHECKSUM CRC/Parity (calculated dynamicaly)
} __attribute__((__packed__)) HOTT_GAM_MSG ;

//GPS
typedef struct {
  uint8_t startByte;               /* Byte 1: 0x7C = Start byte data */
  uint8_t sensorID;                /* Byte 2: 0x8A = GPS Sensor */
  uint8_t alarmTone;               /* Byte 3: 0…= warning beeps */
  uint8_t sensorTextID;            /* Byte 4: 160 0xA0 Sensor ID Neu! */
  uint8_t alarmInverse1;           /* Byte 5: 01 inverse status */
  uint8_t alarmInverse2;           /* Byte 6: 00 inverse status status 1 = kein GPS Signal */
  uint8_t flightDirection;         /* Byte 7: 119 = Flightdir./dir. 1 = 2°; 0° (North), 90° (East), 180° (South), 270° (West) */
  uint8_t GPSSpeedLow;             /* Byte 8: 8 = /GPS speed low byte 8km/h */
  uint8_t GPSSpeedHigh;            /* Byte 9: 0 = /GPS speed high byte */
  
  uint8_t LatitudeNS;              /* Byte 10: 000 = N = 48°39’988 */
  uint8_t LatitudeMinLow;          /* Byte 11: 231 0xE7 = 0x12E7 = 4839 */
  uint8_t LatitudeMinHigh;         /* Byte 12: 018 18 = 0x12 */
  uint8_t LatitudeSecLow;          /* Byte 13: 171 220 = 0xDC = 0x03DC =0988 */
  uint8_t LatitudeSecHigh;         /* Byte 14: 016 3 = 0x03 */
 
  uint8_t longitudeEW;            /* Byte 15: 000  = E= 9° 25’9360 */
  uint8_t longitudeMinLow;         /* Byte 16: 150 157 = 0x9D = 0x039D = 0925 */
  uint8_t longitudeMinHigh;        /* Byte 17: 003 3 = 0x03 */
  uint8_t longitudeSecLow;         /* Byte 18: 056 144 = 0x90 0x2490 = 9360*/
  uint8_t longitudeSecHigh;        /* Byte 19: 004 36 = 0x24 */
  
  uint8_t distanceLow;             /* Byte 20: 027 123 = /distance low byte 6 = 6 m */
  uint8_t distanceHigh;            /* Byte 21: 036 35 = /distance high byte */
  uint8_t altitudeLow;             /* Byte 22: 243 244 = /Altitude low byte 500 = 0m */
  uint8_t altitudeHigh;            /* Byte 23: 001 1 = /Altitude high byte */
  uint8_t resolutionLow;           /* Byte 24: 48 = Low Byte m/s resolution 0.01m 48 = 30000 = 0.00m/s (1=0.01m/s) */
  uint8_t resolutionHigh;          /* Byte 25: 117 = High Byte m/s resolution 0.01m */
  uint8_t unknow1;                 /* Byte 26: 120 = 0m/3s */
  uint8_t GPSNumSat;               /* Byte 27: GPS.Satelites (number of satelites) (1 byte) */
  uint8_t GPSFixChar;              /* Byte 28: GPS.FixChar. (GPS fix character. display, if DGPS, 2D oder 3D) (1 byte) */
  uint8_t HomeDirection;           /* Byte 29: HomeDirection (direction from starting point to Model position) (1 byte) */
  uint8_t angleXdirection;         /* Byte 30: angle x-direction (1 byte) */
  uint8_t angleYdirection;         /* Byte 31: angle y-direction (1 byte) */
  uint8_t angleZdirection;         /* Byte 32: angle z-direction (1 byte) */
  uint8_t gyroXLow;                /* Byte 33: gyro x low byte (2 bytes) */
  uint8_t gyroXHigh;               /* Byte 34: gyro x high byte */
  uint8_t gyroYLow;                /* Byte 35: gyro y low byte (2 bytes) */
  uint8_t gyroYHigh;               /* Byte 36: gyro y high byte */
  uint8_t gyroZLow;                /* Byte 37: gyro z low byte (2 bytes) */
  uint8_t gyroZHigh;               /* Byte 38: gyro z high byte */
  uint8_t vibration;               /* Byte 39: vibration (1 bytes) */
  uint8_t Ascii4;                  /* Byte 40: 00 ASCII Free Character [4] */
  uint8_t Ascii5;                  /* Byte 41: 00 ASCII Free Character [5] */
  uint8_t GPS_fix;                 /* Byte 42: 00 ASCII Free Character [6], we use it for GPS FIX */
  uint8_t version;                 /* Byte 43: 00 version number */
  uint8_t endByte;                 /* Byte 44: 0x7D Ende byte */
  uint8_t chksum;                  /* Byte 45: Parity Byte */
} __attribute__((__packed__)) HOTT_GPS_MSG ;

uint8_t warning_beeps_Hott(void);


void setupHott(void);

void hottPioRxHandlerIrq() ;   // when a byte is received on the tlm bus, read the pio hott fifo and push the data to a queue (to be processed in the main loop)

void handleHottRxTx(void);    // manage TX and RX with a state machine

bool sendHottFrame(uint8_t data);  // generate the frame GAM or GPS according to data; return true if a frame is being sent

bool fillHottGamFrame(); // fill a GAM frame ; return true if a frame is filled

bool fillHottGpsFrame() ; // fill a GAM frame ; return true if a frame is filled

void convertLonLat_Hott( int32_t GPS_LatLon, uint16_t  * degMin , uint16_t * decimalMin ) ;




