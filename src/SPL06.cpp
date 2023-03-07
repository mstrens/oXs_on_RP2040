

#include "SPL06.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "stdio.h"
#include <inttypes.h>
#include "tools.h"
#include "pico/double.h"
#include "param.h"

extern CONFIG config;
extern float actualPressurePa; // used to calculate airspeed

/////////////////////////////////////////////////////
//
// PUBLIC
//
SPL06::SPL06(uint8_t deviceAddress)
{
  _address           = deviceAddress;
}

void SPL06::begin()  // baroInstalled = true when baro exist
{
  baroInstalled = false;
  uint8_t writeCmd[2];
  uint8_t readValue;
  uint8_t regToRead; 
  if ( config.pinScl == 255 or config.pinSda == 255) return; // skip if pins are not defined
  writeCmd[0] = SPL06_RST_REG ;  // reset the device
  writeCmd[1] = 0X89 ; // OX89 means clear fifo and soft reset
  sleep_ms(40);
  //printf("before baro reset\n");
  if (i2c_write_timeout_us (i2c1 , _address, &writeCmd[0] , 2 , false, 1000) <0 ) {
    printf("write error spl06\n");
    return ;  
  }sleep_ms(40) ; // wait that data are loaded from eprom to memory (40 msec in data sheet)
  //printf("before reading baro config\n");
  
  regToRead = SPL06_CHIP_ID_REG ;  // chipid address
  if ( i2c_write_timeout_us (i2c1 , _address, &regToRead , 1 , false,1000) <0) {
    printf("write error for spl06\n");
    return ; // command to get access to one register '0xA0 + 2* offset
  }
  if ( i2c_read_timeout_us (i2c1 , _address , &readValue , 1 , false, 1500) <0)
  {
        printf("Read error for SPL06\n");
        return ;
    }      
  if ( readValue != SPL06_DEFAULT_CHIP_ID) {
      printf("SPL06 has wrong device id\n");
      return ;
  }

  // read factory calibrations from EEPROM.
  uint8_t caldata[SPL06_CALIB_COEFFS_LEN];    
  regToRead = SPL06_CALIB_COEFFS_START ; // this is the address to be read
  if ( i2c_write_timeout_us (i2c1 , _address, &regToRead , 1 , false,1000) <0){
        printf("Write error for calibration SPL06\n");
        return ; // command to get access to one register '0xA0 + 2* offset
  }
  if ( i2c_read_timeout_us (i2c1 , _address , &caldata[0] , SPL06_CALIB_COEFFS_LEN , false, 1500) == PICO_ERROR_TIMEOUT){
        printf("Read error for calibration SPL06\n");
        return ;
    }
    spl06_cal.c0 = (caldata[0] & 0x80 ? 0xF000 : 0) | ((uint16_t)caldata[0] << 4) | (((uint16_t)caldata[1] & 0xF0) >> 4);
    spl06_cal.c1 = ((caldata[1] & 0x8 ? 0xF000 : 0) | ((uint16_t)caldata[1] & 0x0F) << 8) | (uint16_t)caldata[2];
    spl06_cal.c00 = (caldata[3] & 0x80 ? 0xFFF00000 : 0) | ((uint32_t)caldata[3] << 12) | ((uint32_t)caldata[4] << 4) | (((uint32_t)caldata[5] & 0xF0) >> 4);
    spl06_cal.c10 = (caldata[5] & 0x8 ? 0xFFF00000 : 0) | (((uint32_t)caldata[5] & 0x0F) << 16) | ((uint32_t)caldata[6] << 8) | (uint32_t)caldata[7];
    spl06_cal.c01 = ((uint16_t)caldata[8] << 8) | ((uint16_t)caldata[9]);
    spl06_cal.c11 = ((uint16_t)caldata[10] << 8) | (uint16_t)caldata[11];
    spl06_cal.c20 = ((uint16_t)caldata[12] << 8) | (uint16_t)caldata[13];
    spl06_cal.c21 = ((uint16_t)caldata[14] << 8) | (uint16_t)caldata[15];
    spl06_cal.c30 = ((uint16_t)caldata[16] << 8) | (uint16_t)caldata[17];
  
    //printCalibration();

    // SPL06 allows to read continuously pressure and temperature at a specific rate and oversampling
    // several values (up to 32) are stored in a fifo; thery are read form pressure register and the value is 0 when fifo is empty
    // for pressure, we use a rate of 64 and oversampling of 4
    // for temperature, we use a rate of 32 and oversampling of 4
  writeCmd[0] = SPL06_PRESSURE_CFG_REG ;  
  writeCmd[1] = SPL06_PRESSURE_OVERSAMPLING ;
  if (i2c_write_timeout_us (i2c1 , _address, &writeCmd[0] , 2 , false, 1000) <0 ){
   printf("Write error for SPL06\n");
   return ;  
  }
  writeCmd[0] = SPL06_TEMPERATURE_CFG_REG ;  
  writeCmd[1] = SPL06_TEMP_USE_EXT_SENSOR | SPL06_TEMPERATURE_OVERSAMPLING ;
  if (i2c_write_timeout_us (i2c1 , _address, &writeCmd[0] , 2 , false, 1000) <0 ) {
    printf("Write error for SPL06\n"); 
    return ;
  }  
// do not yet activate pressure and temperature ( do not run continously)
//  writeCmd[0] = SPL06_MODE_AND_STATUS_REG ;  
//  writeCmd[1] = 0SPL06_MEAS_PRESSURE | SPL06_MEAS_TEMPERATURE ;
//  if (i2c_write_blocking (i2c1 , _address, &writeCmd , 2 , false) == PICO_ERROR_GENERIC ) return baroInstalled;  
// do not activate fifo
  //writeCmd[0] = SPL06_INT_AND_FIFO_CFG_REG ;  
  //writeCmd[1] = SPL06_FIFO_ENABLE ;
  //if (i2c_write_blocking (i2c1 , _address, &writeCmd , 2 , false) == PICO_ERROR_GENERIC ) return baroInstalled;  

  //printf("end of baro init\n");
  baroInstalled = true; // at this point all is OK.
}

void SPL06::printCalibration(){
    printf("spl06_cal.c0= %f\n",(double) spl06_cal.c0 );
    printf("spl06_cal.c1= %f\n",(double) spl06_cal.c1 );
    printf("spl06_cal.c00= %f\n",(double) spl06_cal.c00 );
    printf("spl06_cal.c10= %f\n",(double) spl06_cal.c10 );
    printf("spl06_cal.c01= %f\n",(double) spl06_cal.c01 );
    printf("spl06_cal.c11= %f\n",(double) spl06_cal.c11 );
    printf("spl06_cal.c20= %f\n",(double) spl06_cal.c20 );
    printf("spl06_cal.c21= %f\n",(double) spl06_cal.c21 );
    printf("spl06_cal.c30= %f\n",(double) spl06_cal.c30 );
}  

void SPL06::requestPressure(){ // return true when cmd is successful
  _result = 0;
  uint8_t writeCmd[2];
  writeCmd[0] = SPL06_MODE_AND_STATUS_REG ;  
  writeCmd[1] = SPL06_MEAS_PRESSURE ;
  if (i2c_write_timeout_us (i2c1 , _address, &writeCmd[0] , 2 , false,1000) <0 ){
    printf("Write error for SPL06\n");
    _result = -1; // _1 shows an error
  }  
}

void SPL06::getPressure(){
    _result = 0;
    uint8_t data[SPL06_PRESSURE_LEN];
    int32_t spl06_pressure;
    uint8_t regToRead = SPL06_PRESSURE_START_REG ;  
    if (i2c_write_timeout_us (i2c1 , _address, &regToRead , 1 , false, 1000) <0 ) {
        printf("Write error for SPL06\n");
        _result = -1; // _1 shows an error
    }
    if (i2c_read_timeout_us (i2c1 , _address , &data[0] , SPL06_PRESSURE_LEN , false , 500) == PICO_ERROR_TIMEOUT){
        printf("Read error for pressure on SPL06\n");
        _result = -1 ;
    }      
    if (_result == 0){
        spl06_pressure = (int32_t)((data[0] & 0x80 ? 0xFF000000 : 0) | (((uint32_t)(data[0])) << 16) | (((uint32_t)(data[1])) << 8) | ((uint32_t)data[2]));
        spl06_pressure_raw = spl06_pressure;
    }
}

void SPL06::requestTemperature(){
  _result = 0;
  uint8_t writeCmd[2];
  writeCmd[0] = SPL06_MODE_AND_STATUS_REG ;  
  writeCmd[1] = SPL06_MEAS_TEMPERATURE ;
  if (i2c_write_timeout_us (i2c1 , _address, &writeCmd[0] , 2 , false, 1000) <0) {
    printf("Write error for SPL06\n");
    _result = -1; // _1 shows an error
  }  
}

void SPL06::getTemperature(){
    _result = 0;
    uint8_t data[SPL06_TEMPERATURE_LEN];
    int32_t spl06_temperature;
    uint8_t regToRead = SPL06_TEMPERATURE_START_REG ;  
    if (i2c_write_timeout_us (i2c1 , _address, &regToRead , 1 , false, 1000) < 0) {
        printf("Write error for SPL06\n");
        _result = -1; // _1 shows an error
    }
    if (i2c_read_timeout_us (i2c1 , _address , &data[0] , SPL06_TEMPERATURE_LEN , false, 1500) <0){
        printf("Read error for temperature on SPL06\n");
        _result = -1 ;
    }      
 
    if (_result == 0){
        spl06_temperature = (int32_t)((data[0] & 0x80 ? 0xFF000000 : 0) | (((uint32_t)(data[0])) << 16) | (((uint32_t)(data[1])) << 8) | ((uint32_t)data[2]));
        spl06_temperature_raw = spl06_temperature;
    }

    
}


// -- END OF FILE --
// Try to get a new pressure and to convert it 
// return 0 if a new value is calculated; -1 if no calculation was performed; other in case of I2C error
// when a value is calculated, then altitude, rawPressure and temperature are calculated.
int SPL06::getAltitude() // Try to get a new pressure ; 
{
  if ( ! baroInstalled) return -1;     // do not process if there is no baro; -1 = no new data
  if ( (microsRp() - _lastConversionRequest) < 9000 ) // at oversampling 4, it takes 8.4 msec to get a conversion
    return -1;
  switch (_state) 
  {
  case SPL06_UNDEFINED :   
    requestPressure(); // ask for pressure conversion in high resolution
    if (_result) return _result;
    _state = SPL06_WAIT_FOR_PRESSURE ;
    _lastConversionRequest = microsRp() ;      
    break;
  case SPL06_WAIT_FOR_PRESSURE :   
    getPressure(); // read pressure, return 0 in case of error; _result =0 if OK.
    if (_result) return _result;
    requestTemperature(); // ask immediately for temperature conversion in high resolution
    if (_result) return _result;
    _state = SPL06_WAIT_FOR_TEMPERATURE ;
    _lastConversionRequest = microsRp() ;
    _lastTempRequest = _lastConversionRequest; 
    break ;  
  case SPL06_WAIT_FOR_TEMPERATURE : 
    getTemperature(); // read temperature, return 0 in case of error; _result =0 if OK.
    if (_result) return _result;
    requestPressure(); // ask for pressure conversion in high resolution
    if (_result) return _result;
    _state = SPL06_WAIT_FOR_PRESSURE ;
    _lastConversionRequest = microsRp() ;      
    calculateAltitude();
    return 0 ; // new data available
    break;
  }
  return -1 ; // no new data 
}

void SPL06::calculateAltitude(){
    // calculate temperature
    const double t_raw_sc = (double)spl06_temperature_raw / SPL06_RAW_VALUE_SCALE_FACTOR;
    const double temp_comp = (double)spl06_cal.c0 * 0.5 + t_raw_sc * (double) spl06_cal.c1;
    temperature = temp_comp * 100 ; // temp in 1/100 of degree
    //printf("temp_raw %f  ", (float) spl06_temperature_raw);
    //printf("temp_raw_sc %f  ", (float) t_raw_sc);
    //printf("temp %f\n", (float) temperature);
    
    // calculate pressure in pascal
    const double p_raw_sc = (double)spl06_pressure_raw / SPL06_RAW_VALUE_SCALE_FACTOR;
    
    const double pressure_cal = (double)spl06_cal.c00 + p_raw_sc * ((double)spl06_cal.c10 + p_raw_sc * ((double)spl06_cal.c20 + p_raw_sc * (double) spl06_cal.c30));
    const double p_temp_comp = t_raw_sc * ((double)spl06_cal.c01 + p_raw_sc * ((double)spl06_cal.c11 + p_raw_sc * (double) spl06_cal.c21));

    const double pressure_comp =  pressure_cal + p_temp_comp; // pressure in Pa
    actualPressurePa = pressure_comp;  // in Pa
    //printf("pres_raw %f  ",(float) spl06_pressure_raw);
    //printf("pres_raw_sc %f  ",(float) p_raw_sc);
    //printf("press_cal %f  ", (float) pressure_cal);
    //printf("p_temp_cmp %f ", (float) p_temp_comp);
    //printf("pressure_comp %f  ", (float) pressure_comp) ;

    altitudeCm = 4433000.0 * (1.0 - pow( actualPressurePa / 101325, 0.1903)); // 101325 is pressure at see level in Pa; altitude is in cm
    altIntervalMicros = _lastTempRequest - _prevAltMicros;
    _prevAltMicros = _lastTempRequest ;
    //printf("Alt %f\n", (float) altitude); 
}

