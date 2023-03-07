#pragma once


#include "stdint.h"


#ifndef SPL06_DEFAULT_ADDRESS
#define SPL06_DEFAULT_ADDRESS                0x77 // or 0X76
#endif

#define SPL06_READ_OK                        0
#define SPL06_ERROR_2                        2         // low level I2C error
#define SPL06_NOT_READ                       -999

// following defines are copied from https://github.com/iNavFlight/inav/blob/master/src/main/drivers/barometer/barometer_spl06.h
#define SPL06_I2C_ADDR                         0x76
#define SPL06_DEFAULT_CHIP_ID                  0x10

#define SPL06_PRESSURE_START_REG               0x00
#define SPL06_PRESSURE_LEN                     3       // 24 bits, 3 bytes
#define SPL06_PRESSURE_B2_REG                  0x00    // Pressure MSB Register
#define SPL06_PRESSURE_B1_REG                  0x01    // Pressure middle byte Register
#define SPL06_PRESSURE_B0_REG                  0x02    // Pressure LSB Register
#define SPL06_TEMPERATURE_START_REG            0x03
#define SPL06_TEMPERATURE_LEN                  3       // 24 bits, 3 bytes
#define SPL06_TEMPERATURE_B2_REG               0x03    // Temperature MSB Register
#define SPL06_TEMPERATURE_B1_REG               0x04    // Temperature middle byte Register
#define SPL06_TEMPERATURE_B0_REG               0x05    // Temperature LSB Register
#define SPL06_PRESSURE_CFG_REG                 0x06    // Pressure config
#define SPL06_TEMPERATURE_CFG_REG              0x07    // Temperature config
#define SPL06_MODE_AND_STATUS_REG              0x08    // Mode and status
#define SPL06_INT_AND_FIFO_CFG_REG             0x09    // Interrupt and FIFO config
#define SPL06_INT_STATUS_REG                   0x0A    // Interrupt and FIFO config
#define SPL06_FIFO_STATUS_REG                  0x0B    // Interrupt and FIFO config
#define SPL06_RST_REG                          0x0C    // Softreset Register
#define SPL06_CHIP_ID_REG                      0x0D    // Chip ID Register
#define SPL06_CALIB_COEFFS_START               0x10
#define SPL06_CALIB_COEFFS_END                 0x21

#define SPL06_CALIB_COEFFS_LEN                 (SPL06_CALIB_COEFFS_END - SPL06_CALIB_COEFFS_START + 1)

// TEMPERATURE_CFG_REG
#define SPL06_TEMP_USE_EXT_SENSOR              (1<<7)

// MODE_AND_STATUS_REG
#define SPL06_MEAS_PRESSURE                    (1<<0)  // measure pressure
#define SPL06_MEAS_TEMPERATURE                 (1<<1)  // measure temperature

#define SPL06_MEAS_CFG_CONTINUOUS              (1<<2)
#define SPL06_MEAS_CFG_PRESSURE_RDY            (1<<4)
#define SPL06_MEAS_CFG_TEMPERATURE_RDY         (1<<5)
#define SPL06_MEAS_CFG_SENSOR_RDY              (1<<6)
#define SPL06_MEAS_CFG_COEFFS_RDY              (1<<7)

// INT_AND_FIFO_CFG_REG
#define SPL06_PRESSURE_RESULT_BIT_SHIFT        (1<<2)  // necessary for pressure oversampling > 8
#define SPL06_TEMPERATURE_RESULT_BIT_SHIFT     (1<<3)  // necessary for temperature oversampling > 8
#define SPL06_FIFO_ENABLE                      (1<<1)  // enable FIFO

#define SPL06_PRESSURE_OVERSAMPLING           0b0010 // 0b0010 =  oversampling 4 => conversion time = 8.4 msec
#define SPL06_TEMPERATURE_OVERSAMPLING        0b0010 // 0b0010 =  oversampling 4 => conversion time = 8.4 msec
// rate can be 64/sec for pressure and 32/sec for temp so a total time of about 806.4
#define SPL06_RATE_32 5 //0b101   rate = 32 * per sec
#define SPL06_RATE_64 6 //0b110   rate = 64 * per sec

#define SPL06_RAW_VALUE_SCALE_FACTOR        3670016.0d   // depends on oversampling : value for oversampling 4 is 3670016

#define SPL06_MEASUREMENT_TIME(oversampling)   ((2 + lrintf(oversampling * 1.6)) + 1) // ms

typedef struct {
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} spl06_coeffs_t;
// end of copy

enum SPL06_state_t
{  
  SPL06_UNDEFINED = 0,
  SPL06_WAIT_FOR_PRESSURE = 1,
  SPL06_WAIT_FOR_TEMPERATURE = 2 
};


class SPL06
{
public:
  bool    baroInstalled = false; 
  float altitudeCm  ; // in cm 
  int32_t temperature;     // in 1/100 Celsius
  int64_t rawPressure ;  // in 1/10000 mBar so = Pa * 10000
  int32_t altIntervalMicros; // enlapstime between 2 calculations of altitude
  
  explicit SPL06(uint8_t deviceAddress);
  void     begin();
  int      getAltitude(); // return 0 if a new value is calculated; -1 if no calculation was performed; other in case of I2C error

private:
    void     calculateAltitude();
    void     requestPressure();
    void     getPressure();
    void     requestTemperature();
    void     getTemperature();
    void     printCalibration();
    uint8_t  _address;
    int      _result;
    SPL06_state_t  _state;
    //uint32_t _lastRead;
    uint32_t _prevAltMicros;
    uint32_t _lastConversionRequest;
    uint32_t _lastTempRequest;
    //uint32_t _D2Prev;
    spl06_coeffs_t spl06_cal;
    // uncompensated pressure and temperature
    int32_t spl06_pressure_raw = 0;
    int32_t spl06_temperature_raw = 0;
};

void setupI2c() ;
// -- END OF FILE --

