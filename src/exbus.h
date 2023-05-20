#pragma once

typedef struct /*_JetiSensorConst*/
{
  uint8_t id;            // sequence nr to be used when frame is send to identify the field (00= oXs, 1 = first field, ...;)
  char    name[20];
  char    unit[7];
  uint8_t dataType;
  uint8_t precision;
  uint8_t length;
}
JetiSensorConst;
typedef const JetiSensorConst JETISENSOR_CONST; 


// list of Jeti type being used
#define EXBUS_TYPE_14 0b00000001  // jeti TYpe = 1
#define EXBUS_TYPE_22 0b00000100  // jeti TYpe = 4
#define EXBUS_TYPE_30 0b00001000  // jeti TYpe = 8
#define EXBUS_TYPE_DT 5         // for date & time
#define EXBUS_TYPE_GPS 9        // for long /lat
#define EXBUS_TYPE_NONE 0XFF      // field not used currently
#define EXBUS_TYPE_DEVICE 0       // identify the name of the device (= "oXs")

void setupExbusList(bool activateAllFields);

void setupExbus();

void exbusPioRxHandlerIrq();
void handleExbusRxTx(void);
            
bool exbusProcessNextInputByte(uint8_t c);

bool check_checksum(void);
uint8_t crc_sum8(const uint8_t *p, uint8_t len);
uint16_t exbusCrc16Update(uint16_t crc, uint8_t data);

void exbusDecodeRcChannels();

void exbusCreateSendTelemetry();
void exbusCreateSendJetiBox();

void exbusCreateTelemetry();
void exbusCreateJetibox();

uint32_t exbusFormatGpsLongLat (int32_t longLat, bool isLong ) ; // return the long or latitude in Jeti format

uint8_t exbusUpdate8Crc (uint8_t crc, uint8_t crc_seed); 
