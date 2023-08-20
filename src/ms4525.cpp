#include "ms4525.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "stdio.h"
#include <inttypes.h>
#include "tools.h"
#include "param.h"
#include "hardware/watchdog.h"
#include <math.h>

extern CONFIG config;
float actualPressurePa = 101325.0; // this value is updated when baro1,2 or 3 is installed
extern float difPressureAirspeedSumPa; // calculate a moving average on x values
extern uint32_t difPressureAirspeedCount;
extern float difPressureCompVspeedSumPa; // calculate a moving average on x values
extern uint32_t difPressureCompVspeedCount;
extern float temperatureKelvin;     // in Kelvin , used when airspeed is calculated
extern int32_t i2cError;





MS4525::MS4525(uint8_t deviceAddress)
{
  _address = deviceAddress;
}

// **************** Setup the 4525DO sensor *********************
void MS4525::begin() {
    airspeedInstalled = false;
    if ( config.pinScl == 255 or config.pinSda == 255) return; // skip if pins are not defined
    // read the sensor to get the initial temperature
    i2cError = i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 4 , false, 1500);
    if ( i2cError  < 0)  {
        printf("error read MS4525 (airspeed sensor) at startup: %i \n",i2cError);
    return ;
    }
    int32_t temperatureAdc ;   // in steps ADC  
    if ( ( readBuffer[0] & 0xC0 ) == 0) { // msb bit must be 1 when a conversion is available  
        temperatureAdc =    (readBuffer[2] << 8) + readBuffer[3] ;
        temperatureAdc = (0xFFE0 & temperatureAdc) >> 5;
        temperatureKelvin = (0.097703957f * (float) temperatureAdc)  + 223.0 ; // in kelvin 
    } else { 
        printf("error reading temperature : %x\n", readBuffer[0]);
        temperatureKelvin = 300 ;
    }
    airspeedInstalled = true;
    prevReadUs = microsRp(); 
}  //end of begin


/****************************************************************************/
/* readSensor - Read differential pressure + calculate airspeed             */
/****************************************************************************/
void MS4525::getDifPressure() {
    if ( ! airspeedInstalled) return ;     // do not process if there is no sensor
    uint32_t now = microsRp(); 
    if ( (now - prevReadUs) < 5000 ) return ;// it take about 500 usec for a conversion
    // read a new pressure
    prevReadUs = now;
    i2cError = i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 2 , false, 1500);
    if ( i2cError  < 0)  {
        printf("error read MS4525 (airspeed sensor): %i\n",i2cError);
        return; 
    }  
    // no I2C error in reading the pressure
    int32_t difPressureAdc; 
    if ( ( readBuffer[0] & 0xC0 ) == 0) {  
        difPressureAdc =  ( ( (readBuffer[0] << 8) + readBuffer[1] ) & 0x3FFF) - 0x2000  ; // substract in order to have a zero value 
        // difPressureAdc = 14745 - 8192 ; // test should give 1 psi = 6894 pa = 105 m/sec = 370 km/h
        // difPressureAdc = 1638 - 8192 ; // test should give -1 psi = 6894 pa
        if ( calibrated4525 == false) {
            calibrateCount++ ;
            if (calibrateCount == 256 ) { // after 256 reading , we can calculate the offset 
                offset =  (  ((float) difPressureCalSum) / 128.0 ) ; //there has been 128 reading (256-128)                     
                calibrated4525 = true ;
            } else if  (calibrateCount >= 128  ){ // after 128 reading, we can start cummulate the ADC values in order to calculate the offset 
                difPressureCalSum += difPressureAdc ;
            } // end calibration
        }  else { // sensor is calibrated
            // with MS4525DO_001 a range of 2 PSI gives 80% of 16383 (= max of 14bits);
            // 1 PSI = 6894,76 Pascal ; so 1 unit of ADC = 2/ (80% * 16383) * 6894,76) 
            // differantial_pressure_Pa =  ((DifPressureAdc  ) * 1.052) ; 
            #define MS4525_ADC_TO_PA 1.052
            difPressurePa = (((float) difPressureAdc) - offset) * MS4525_ADC_TO_PA ;
            difPressureAirspeedSumPa += difPressurePa; // calculate a moving average on x values
            difPressureAirspeedCount++;                // count the number of conversion
            difPressureCompVspeedSumPa += difPressurePa; // calculate a moving average on x values
            difPressureCompVspeedCount++;                // count the number of conversion
                            
        }    
    } else {
        printf("error : first byte of read MS4525 = %x  at %d\n", readBuffer[0] , (int) now );  
    }

}            
