#include "xgzp6897D.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "stdio.h"
#include <inttypes.h>
#include "tools.h"
#include "param.h"
#include "hardware/watchdog.h"
#include <math.h>
#include "config.h"

extern CONFIG config;
extern float actualPressurePa ; // this value is updated when baro1,2 or 3 is installed
extern float difPressureAirspeedSumPa; // calculate a moving average on x values
extern uint32_t difPressureAirspeedCount;
extern float difPressureCompVspeedSumPa; // calculate a moving average on x values
extern uint32_t difPressureCompVspeedCount;
extern float temperatureKelvin;     // in Kelvin , used when airspeed is calculated
extern int32_t i2cError;

#define XGZP_CONFIG_REGISTER 0X30 ;  // register to control the sensor (sleep time, start, continuous mode)
#define XGZP_CONFIG_VALUE 0X0B; // = 00001011 : 0000 = no sleep time = 0 ; 1 = request a conversion ; 011 = continuous conversion (after 0 sleeptime) 
#define XGZP_PRESSURE_REGISTER 0X06  ;  // register with the pressure ( 3 bytes for pressures and 2 for temp )

XGZP::XGZP(uint8_t deviceAddress)  // class to handle the sensor
{
  _address = deviceAddress;
}

// **************** Setup the XGZP sensor *********************
void XGZP::begin() {
    airspeedInstalled = false;
    uint8_t writeCmd[2];
    //uint8_t readValue;
    //uint8_t regToRead; 
    if ( config.pinScl == 255 or config.pinSda == 255) return; // skip if pins are not defined
    writeCmd[0] = XGZP_CONFIG_REGISTER ;  
    writeCmd[1] = XGZP_CONFIG_VALUE ; 
    // set the config/control register
    if (i2c_write_timeout_us (i2c1 , _address, &writeCmd[0] , 2 , false, 1000) <0 ) {
        printf("write error XGZP\n");
        return ;  
    }
    prevReadUs = microsRp();
    airspeedInstalled = true; // at this point all is OK.
}    
    

/****************************************************************************/
/* readSensor - Read differential pressure                                  */
/****************************************************************************/
void XGZP::getDifPressure() {
    if ( ! airspeedInstalled) return ;     // do not process if there is no sensor
    uint32_t now = microsRp(); 
    if ( (now - prevReadUs) < 20000 ) return ;// it take about 20000 usec for a conversion
    // set on pressure register
    prevReadUs = now;
    uint8_t writeCmd[1];
    writeCmd[0] = XGZP_PRESSURE_REGISTER ;  
    if (i2c_write_timeout_us (i2c1 , _address, &writeCmd[0] , 2 , false, 1000) <0 ) {
        printf("error writing a cmd to XGZP (airspeed sensor)\n");
        return; // no action when i2c error
    }
    uint8_t readBuffer[3];
    i2cError = i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 3 , false, 1500);
    if ( i2cError  < 0)  {
        printf("error reading XGZP (airspeed sensor): %i\n",i2cError);
        return; 
    }  
    // no I2C error in reading the pressure
    int32_t difPressureAdc; 
    difPressureAdc =  (readBuffer[0] << 16) + (readBuffer[1] << 8 ) + (readBuffer[2])  ;  
    if ( calibrated == false) {
        calibrateCount++ ;
        if (calibrateCount == 64 ) { // after 256 reading , we can calculate the offset 
            offset =  (  ((float) difPressureCalSum) / 32.0 ) ; //there has been 32 reading (64-32)                     
            calibrated = true ;
        } else if  (calibrateCount >= 32  ){ // after 32 reading, we can start cummulate the ADC values in order to calculate the offset 
            difPressureCalSum += difPressureAdc ;
        } // end calibration
    }  else { // sensor is calibrated
        // XGZP sensor use a K factor that differs based on the sensor sensitivity
        difPressurePa = (((float) difPressureAdc) - offset) / XGZP_K_FACTOR ;
        difPressureAirspeedSumPa += difPressurePa; // calculate a moving average on x values
        difPressureAirspeedCount++;                // count the number of conversion
        difPressureCompVspeedSumPa += difPressurePa; // calculate a moving average on x values
        difPressureCompVspeedCount++;                // count the number of conversion
                        
    }    

}            
