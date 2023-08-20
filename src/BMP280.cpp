#include "BMP280.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "stdio.h"
#include <inttypes.h>
#include "tools.h"
#include "pico/double.h"
#include "param.h"

extern CONFIG config;
extern float actualPressurePa; // used to calculate airspeed
extern int32_t i2cError;

#ifdef DEBUG
//#define DEBUGI2CBMP280
//#define DEBUGDATA
//#define DEBUGVARIOI2C
#endif
//#define DEBUG_BMP280

extern unsigned long microsRp( void ) ;
extern unsigned long millisRp( void ) ;
extern void delay(unsigned long ms) ;

static BMP280_CALIB_DATA _bmp280_coeffs;   // Last read calibration data will be available here
//static uint8_t           _bmp085Mode;


BMP280::BMP280( uint8_t deviceAddress)
{
  // constructor
  _address           = deviceAddress;
  //varioData.SensorState = 0 ;
}

#define BMP280_CHIP_ID_REG 0xD0 
#define BMP280_CHIP_ID_VALUE 0x58 
    
// **************** Setup the BMP280 sensor *********************
void BMP280::begin() {
  
  uint8_t writeCmd[2];
  uint8_t readValue;
  uint8_t regToRead; 
  uint8_t rxdata;
  uint16_t _calibrationData[13]; // The factory calibration data of the BMP280
  baroInstalled = false;
  if ( config.pinScl == 255 or config.pinSda == 255) return; // skip if pins are not defined
    #ifdef DEBUG  
    printf("Trying to detect BMP280 sensor at I2C Addr=%X\n", _address);
    #endif

    writeCmd[0] = 0xF4 ;  // Register ctrl meas
    writeCmd[1] = 0X33 ; // it means oversampling temperature 1 X , oversampling pressure 8 X and normal mode = continue 
    i2cError = i2c_write_timeout_us (i2c1 , _address, &writeCmd[0] , 2 , false, 1000);
    if (i2cError  <0 ){
      printf("Write error for BMP280: %i\n",i2cError);
      return ;  
    }

// write in register 0xF5 value 0x00 (it means 0.5msec between sampling, no filter, I2C protocol )
    //errorI2C = I2c.write( _address , (uint8_t) 0xF5 , (uint8_t) 0x00 ) ;
    writeCmd[0] = 0xF5 ;  // Register config
    writeCmd[1] = 0X00 ; // OX00 means 0.5msec between sampling, no filter, I2C protocol
    i2cError = i2c_write_timeout_us (i2c1 , _address, &writeCmd[0] , 2 , false,1000);
    if (i2cError  <0) {
        printf("Write error for BMP280: %i\n",i2cError);
        return ;  
    }
 // read and check the device ID (in principe = 0X58 for a bmp280) 
    regToRead = BMP280_CHIP_ID_REG ;  // chipid address
    i2cError = i2c_write_timeout_us(i2c1 , _address, &regToRead , 1 , false,1000);
    if ( i2cError  <0) {
        printf("Write error for BMP280: %i\n",i2cError );
        return ; // command to get access to one register '0xA0 + 2* offset
    }
    i2cError = i2c_read_timeout_us(i2c1 , _address , &readValue , 1 , false, 1500);
    if ( i2cError  <0) {
        printf("Read error for BMP280: %i\n",i2cError);
        return ;
    }     
    if ( readValue != BMP280_CHIP_ID_VALUE) {
        printf("BMP280 has wrong device id\n");
        return ;
    }

// read calibration parameters

    //errorCalibration = false ;
    for (uint8_t i = 1; i <=12; i++) {
    //   errorI2C =  I2c.read( _addr , 0x86 + i*2, 2 ) ; //read 2 bytes from the device after sending the register to be read (first register = 0x86 (=register AC1)
        uint8_t readBuffer[2];
        rxdata = 0x86 + i * 2 ; // this is the address to be read
        i2cError = i2c_write_timeout_us (i2c1 , _address, &rxdata , 1 , false,1000);
        if ( i2cError <0) {
            printf("Write error for BMP280 during calibration: %i\n",i2cError);
            return ; // command to get access to one register '0xA0 + 2* offset
        }
        sleep_ms(1);
        i2cError = i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 2 , false, 1500);
        if ( i2cError  <0) {
            printf("Read error for BMP280 during calibration: %i\n",i2cError);
            return ;
        }
        _calibrationData[i] = (readBuffer[1]<<8 ) | (readBuffer[0] );     
#ifdef DEBUG
        //printf("calibration data #%d = %u \n", i , _calibrationData[i]);
#endif
    } // End for 

    _bmp280_coeffs.dig_T1 = _calibrationData[1];
    _bmp280_coeffs.dig_T2 = _calibrationData[2];
    _bmp280_coeffs.dig_T3 = _calibrationData[3];
    _bmp280_coeffs.dig_P1 = _calibrationData[4];
    _bmp280_coeffs.dig_P2 = _calibrationData[5];
    _bmp280_coeffs.dig_P3 = _calibrationData[6];
    _bmp280_coeffs.dig_P4  = _calibrationData[7];
    _bmp280_coeffs.dig_P5  = _calibrationData[8];
    _bmp280_coeffs.dig_P6  = _calibrationData[9];
    _bmp280_coeffs.dig_P7  = _calibrationData[10];
    _bmp280_coeffs.dig_P8  = _calibrationData[11];
    _bmp280_coeffs.dig_P9  = _calibrationData[12];

#ifdef DEBUG  
    printf("BMP280  sensor is successfully detected\n");  
#endif
    baroInstalled = true; // at this point all is OK.
}  //end of begin


// Try to get a new pressure and to convert it 
// return 0 if a new value is calculated; -1 if no calculation was performed; other in case of I2C error
// when a value is calculated, then altitude, rawPressure and temperature are calculated.
int BMP280::getAltitude() {
    //static uint32_t lastBMP280ReadMicro ;
    //bool newVSpeedCalculated  = false ; 
    if ( ( microsRp() - _lastConversionRequest ) < 20000) return -1;
    int32_t adc_T = 0 ;
    int32_t adc_P = 0 ;
    int32_t t_fine; // t_fine carries fine temperature as global value
    int32_t var1 ; 
    int32_t var2 ;
    //int32_t T; // Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
    uint32_t p; // pressure in pascal
    uint8_t buffer[6];
    uint8_t regToRead = 0xF7 ;  // address reg of the first byte of conversion
    i2cError = i2c_write_timeout_us (i2c1 , _address, &regToRead , 1 , false,1000);
    if ( i2cError <0){ 
        printf("Write error for BMP280: %i\n",i2cError);
        return -1; // command to get access to one register '0xA0 + 2* offset
    }
    i2cError = i2c_read_timeout_us (i2c1 , _address , &buffer[0] , 6 , false, 1500);
    if ( i2cError  <0){
        printf("Read error on BMP280: %i\n",i2cError);
         return -1; 
    }
    adc_P = (buffer[0]<<16) | (buffer[1]<<8) | (buffer[2]) ;
    adc_P = adc_P >> 4 ;
    adc_T = (buffer[3]<<16) | (buffer[4]<<8) | (buffer[5]) ;
    adc_T = adc_T >> 4 ;
    _lastConversionRequest = microsRp() ;
        //errorI2C = I2c.read( _address , (uint8_t) 0xF7 , (uint8_t) 6) ; // read 6 bytes starting from register F7
        //  adc_P = I2c.receive() ;
        //  adc_P <<= 8 ;
        //  adc_P |= I2c.receive() ;
        //  adc_P <<= 8 ;
        //  adc_P |= I2c.receive() ;
        //  adc_P = adc_P >> 4 ;  
        //  adc_T = I2c.receive() ;
        //  adc_T <<= 8 ;
        //  adc_T |= I2c.receive() ;
        //  adc_T <<= 8 ;
        //  adc_T |= I2c.receive()  ;
        //  adc_T = adc_T >> 4 ;
        //  varioData.lastCommandMicros = microsRp() ;

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value

 
          var1 = ((((adc_T>>3) - ((int32_t)_bmp280_coeffs.dig_T1<<1))) * ((int32_t)_bmp280_coeffs.dig_T2)) >> 11;
          var2 = (((((adc_T>>4) - ((int32_t)_bmp280_coeffs.dig_T1)) * ((adc_T>>4) - ((int32_t)_bmp280_coeffs.dig_T1))) >> 12) * ((int32_t)_bmp280_coeffs.dig_T3)) >> 14;
          t_fine = var1 + var2;
          temperature = (t_fine * 5 + 128) >> 8; 
        

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
// Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa

          var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
          var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)_bmp280_coeffs.dig_P6);
          var2 = var2 + ((var1*((int32_t)_bmp280_coeffs.dig_P5))<<1);
          var2 = (var2>>2)+(((int32_t)_bmp280_coeffs.dig_P4)<<16);
          var1 = (((_bmp280_coeffs.dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)_bmp280_coeffs.dig_P2) * var1)>>1))>>18;
          var1 =((((32768+var1))*((int32_t)_bmp280_coeffs.dig_P1))>>15);
          if (var1 == 0) { 
            rawPressure = 0 ; 
          } else {
            p = (((uint32_t)(((int32_t)1048576) - adc_P)-(var2>>12)))*3125;
            if (p < 0x80000000)   {
              p = (p << 1) / ((uint32_t)var1);
            }   else   {
              p = (p / (uint32_t)var1) * 2;
            }
            var1 = (((int32_t)_bmp280_coeffs.dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
            var2 = (((int32_t)(p>>2)) * ((int32_t)_bmp280_coeffs.dig_P8))>>13;
            rawPressure = (uint32_t)((int32_t)p + ((var1 + var2 + _bmp280_coeffs.dig_P7) >> 4)) ;  // in pascal
          }
          actualPressurePa = (float) rawPressure ;    
          altitudeCm = 4433000.0 * (1.0 - pow( ( (double) actualPressurePa)  / 101325.0, 0.1903)); // 101325 is pressure at see level in Pa; altitude is in cm *100
        altIntervalMicros = _lastConversionRequest - _prevAltMicros;
        _prevAltMicros = _lastConversionRequest ;
        #ifdef DEBUG_BMP280  
            printf("temp=%d   Press=%d   Alt=%d\n",temperature,rawPressure,altitude);  
        #endif


        return 0 ; // new data available
        
}  // end getAltitude     









