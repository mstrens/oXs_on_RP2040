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






MS4525::MS4525(uint8_t deviceAddress)
{
  _address = deviceAddress;
}

// **************** Setup the 4525DO sensor *********************
void MS4525::begin() {
    airspeedInstalled = false;
    if ( config.pinScl == 255 or config.pinSda == 255) return; // skip if pins are not defined
    // read the sensor to get the initial temperature
    if ( i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 4 , false, 1500) < 0)  {
        printf("error read MS4525 (airspeed sensor) at startup \n");
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
    if ( i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 2 , false, 1500) < 0)  {
        printf("error read MS4525 (airspeed sensor)\n");
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
        printf("first byte of read MS4525 = %x  at %d\n", readBuffer[0] , (int) now );  
    }

}            

/*
                difPressureAdc_0SumValue += difPressureAdc_0;
                difPressureAdc_0SumCount++ ; 
                #define FILTERING4525_ADC_MIN        0.001   // 
                #define FILTERING4525_ADC_MAX        0.01 // 
                #define FILTERING4525_ADC_MIN_AT       10 // when abs(delta between ADC and current value) is less than MIN_AT , apply MIN  
                #define FILTERING4525_ADC_MAX_AT       100 // when abs(delta between ADC and current value) is more than MAX_AT , apply MAX (interpolation in between)
                abs_deltaDifPressureAdc =  difPressureAdc_0 - smoothDifPressureAdc ;
                if (abs_deltaDifPressureAdc < 0) abs_deltaDifPressureAdc = - abs_deltaDifPressureAdc;
                if (abs_deltaDifPressureAdc <= FILTERING4525_ADC_MIN_AT) {
                    expoSmoothFactor = FILTERING4525_ADC_MIN ;  
                } else if (abs_deltaDifPressureAdc >= FILTERING4525_ADC_MAX_AT)  {
                    expoSmoothFactor = FILTERING4525_ADC_MAX ; 
                } else {
                    expoSmoothFactor = FILTERING4525_ADC_MIN + ( FILTERING4525_ADC_MAX - FILTERING4525_ADC_MIN) * (abs_deltaDifPressureAdc - FILTERING4525_ADC_MIN_AT) / (FILTERING4525_ADC_MAX_AT - FILTERING4525_ADC_MIN_AT) ;
                }
                smoothDifPressureAdc += expoSmoothFactor * ( difPressureAdc_0 - smoothDifPressureAdc ) ; 
                float abs_smoothDifPressureAdc;
                if ( smoothDifPressureAdc >= 0) {abs_smoothDifPressureAdc = smoothDifPressureAdc;
                } else { abs_smoothDifPressureAdc = -smoothDifPressureAdc;
                }
               // calculate airspeed based on pressure, altitude and temperature
               // airspeed (m/sec) = sqr(2 * differential_pressure_in_Pa / air_mass_kg_per_m3) 
               // air_mass_kg_per_m3 = pressure_in_pa / (287.05 * (Temp celcius + 273.15))
               // and differantial_pressure_Pa =  ((smoothDifPressureAdc  ) * 1.052) ;  // with MS4525DO_001 a range of 2 PSI gives 80% of 16383 (= max of 14bits); 1 PSI = 6894,76 Pascal ; so 1 unit of ADC = 2/ (80% * 16383) * 6894,76) 
               // so airspeed m/sec =sqr( 2 * 287.05 * 1.052 * differential_pressure_pa * (temperature Celsius + 273.15) / pressure_in_pa )
               // rawAirSpeed cm/sec =  24,58 * 100 * sqrt( (float) abs(smoothDifPressureAdc) * temperature4525  /  actualPressurePa) ); // in cm/sec ; actual pressure must be in pa (so 101325 about at sea level)
#ifdef AIRSPEED_AT_SEA_LEVEL_AND_15C
               smoothAirSpeed =  131.06 * sqrt( (float) ( abs_smoothDifPressureAdc ) ); // indicated airspeed is calculated at 15 Celsius and 101325 pascal
#else               
               smoothAirSpeedCmS =  2458 * sqrt( (float) ( abs_smoothDifPressureAdc * temperatureKelvin / actualPressurePa) ); // in cm/sec ; actual pressure must be in pa (so 101325 about at sea level)
#endif              
                if ( smoothDifPressureAdc < 0 ) smoothAirSpeedCmS = - smoothAirSpeedCmS ; // apply the sign
                // publish the new value every 200 ms
                if ( (millisRp() - prevAirspeedMsAvailable) > 200) { // make the new value available once per 200 msec
                    prevAirspeedMsAvailable = millisRp();
                    //if ( smoothAirSpeedCmS >  0) {  // normally send only if positive and greater than 300 cm/sec , otherwise send 0 but for test we keep all values to check for drift  
                    sent2Core0(RELATIVEALT, (int32_t) smoothAirSpeedCmS); 
//#ifdef AIRSPEED_IN_KMH  // uncomment this line if AIR speed has to be in knot instead of km/h
//                  sent2Core0(AIRSPEED, (int32_t) (smoothAirSpeedCmS * 0.36)) ; // from cm/sec to 1/10 km/h
//#else
//                  sent2Core0(RELATIVEALT, (int32_t) (smoothAirSpeed * 0.1943844492)) ; // from cm/sec to 1/10 knot/h
//#endif
//              } else {
//                  airSpeedData.airSpeed.value = 0 ;
//              }    

                }
            
            } // end processing a value from sensor (when calibrated)
        } // end when data is valid
    } // end no error on I2C    

// check if offset must be reset
//              if (airSpeedData.airspeedReset) { // adjust the offset if a reset command is received from Tx
//                    offset4525 =  offset4525  + smoothDifPressureAdc ;
//                    airSpeedData.airspeedReset = false ; // avoid that offset is changed again and again if PPM do not send a command
//              }
 

} // End of readSensor
*/