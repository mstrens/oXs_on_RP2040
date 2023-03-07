#pragma once


#include "stdint.h"

//  BREAKOUT  MS5611  aka  GY63 - see datasheet
//
//  SPI    I2C
//              +--------+
//  VCC    VCC  | o      |
//  GND    GND  | o      |
//         SCL  | o      |
//  SDI    SDA  | o      |
//  CSO         | o      |
//  SDO         | o L    |   L = led
//          PS  | o    O |   O = opening  PS = protocol select
//              +--------+
//
//  PS to VCC  ==>  I2C  (GY-63 board has internal pull up, so not needed)
//  PS to GND  ==>  SPI
//  CS to VCC  ==>  0x76
//  CS to GND  ==>  0x77


#define MS5611_LIB_VERSION                    (F("0.3.8"))

#ifndef MS5611_DEFAULT_ADDRESS
#define MS5611_DEFAULT_ADDRESS                0x77
#endif

#define MS5611_READ_OK                        0
#define MS5611_ERROR_2                        2         // low level I2C error
#define MS5611_NOT_READ                       -999

enum state_t
{  
  UNDEFINED = 0,
  WAIT_FOR_PRESSURE = 1,
  WAIT_FOR_TEMPERATURE = 2 
};

class MS5611
{
public:
  bool    baroInstalled = false; 
  float altitudeCm  ; // in cm 
  int32_t temperature;     // in 1/100 Celsius
  int64_t rawPressure ;  // in 1/10000 mBar so = Pa * 10000
  
  int32_t altIntervalMicros; // enlapstime between 2 calculations of altitude
  explicit MS5611(uint8_t deviceAddress);

  void     begin();
  int      getAltitude(); // return 0 if a new value is calculated; -1 if no calculation was performed; other in case of I2C error

private:
  uint32_t readADC();
  void     command(const uint8_t command);
  void     calculateAltitude();
  int8_t   ms56xx_crc(uint16_t *prom);
  uint8_t  _address;
  //uint8_t  _samplingRate;
  
  //int32_t  _temperature;
  //float    _pressure;
  //float    _pressureOffset;
  //float    _temperatureOffset;
  int      _result;
  uint16_t _calibrationData[8];
  uint32_t _lastRead = 0;
  uint32_t _prevAltMicros = 0;
  state_t  _state ;
  uint32_t _D1;
  uint32_t _D2;
  uint32_t _lastConversionRequest = 0;
  uint32_t _lastTempRequest = 0;
  uint32_t _D2Prev;
};


// -- END OF FILE --

