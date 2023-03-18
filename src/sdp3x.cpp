#include "sdp3x.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "stdio.h"
#include <inttypes.h>
#include "tools.h"
#include "param.h"
#include "hardware/watchdog.h"
#include <math.h>

extern CONFIG config;
extern float actualPressurePa; // this value is updated when baro1,2 or 3 is installed
extern float difPressureAirspeedSumPa; // calculate a moving average on x values
extern uint32_t difPressureAirspeedCount;
extern float difPressureCompVspeedSumPa; // calculate a moving average on x values
extern uint32_t difPressureCompVspeedCount;
extern float temperatureKelvin;     // in Kelvin , used when airspeed is calculated


SDP3X::SDP3X(uint8_t deviceAddress)
{
  _address = deviceAddress;
}


// **************** Setup the SDP3X sensor *********************
void SDP3X::begin() {
    airspeedInstalled = false;
    if ( config.pinScl == 255 or config.pinSda == 255) return; // skip if pins are not defined
#ifdef DEBUG_SDP3X_SIMULATION // use dummy setup in case of simulation
  nextPressureReadMillis = millis() + 2;    //
  nextAirSpeedMillis  = nextPressureReadMillis + 200 ; 
  temperatureCelsius = 23.4  ; 
  temperatureKelvin = 273.15 + temperatureCelsius ; // in Kelvin
  dpScaleSdp3x =  966.0 / 1013.0 / 60 ; //60=SDP31 , 240=SDP32
#else // not a simulation
    
    //uint8_t readBuffer[9];
    int32_t i2cError;
    
    // set the sensor in continous mode with averaging (send a command 0X3615)
    uint8_t cmdData[2] = { 0X36 , 0X15} ; 
    if ( i2c_write_timeout_us (i2c1 , _address, &cmdData[0] , 2 , true , 1000) <0 ) {
        printf("error write command to sdp3x\n");
        return ; 
    }
    sleep_ms(100); // wait 20msec in order to get the first data (datasheet says 8 msec) 
    //if ( i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 9 , false, 5500) < 0)  {
    /*
    i2cError = i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 1 , true, 5500);   
    printf("read 1 byte from sdp3x = %d \n", (int) i2cError);
    i2cError = i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 2 , true, 5500) ;    
    printf("read 2 bytes from sdp3x = %d \n", (int) i2cError);
    i2cError =  i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 3 , true, 5500) ;    
    printf("read 3 bytes from sdp3x = %d \n", (int) i2cError);
    i2cError =  i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 4 , true, 5500) ;    
    printf("read 4 bytes from sdp3x = %d \n", (int) i2cError);
    i2cError =  i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 5 , true, 5500) ;    
    printf("read 5 bytes from sdp3x = %d \n", (int) i2cError);
    i2cError =  i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 6 , true, 5500);    
    printf("read 6 bytes from sdp3x = %d \n", (int) i2cError);
    i2cError =  i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 7 , true, 5500) ;    
    printf("read 7 bytes from sdp3x = %d \n", (int) i2cError);
    i2cError =  i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 8 , true, 5500) ;    
    printf("read 8 bytes from sdp3x = %d \n", (int) i2cError);
    */
    i2cError =  i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 9 , true, 5500) ;    
    if (i2cError < 0) {
        printf("read 9 bytes from sdp3x = %d \n", (int) i2cError);    
        return ; // skip if there is an error
    }  
    // first 2 bytes = pressure, byte 3 = CRC, byte 4 & 5 = temp,  bytes 6, 7, 8 = scale factor
    //nextPressureReadMillis = millisRp() + 2;    //
    //nextAirSpeedMillis  = nextPressureReadMillis + 200 ; 
    temperatureCelsius =   ((float)(( ((int16_t)readBuffer[3]) << 8)  | readBuffer[4])) / 200.0  ;
    temperatureKelvin = 273.15 + temperatureCelsius ; // in Kelvin
    dpScaleSdp3x =  966.0 / 1013.25 / ((float)((int16_t)readBuffer[6] << 8 | readBuffer[7]));
    // datasheet says that we have to apply a correction of 966/actual pressure in mbar; it is estimated with 1013
#endif // endif for simulation
    printf("sdp3x has been succesfuly detected\n");
    airspeedInstalled = true;
    prevReadUs = microsRp(); 
}  //end of setup

/****************************************************************************/
/* readSensor - Read differential pressure                                  */
/****************************************************************************/
void SDP3X::getDifPressure() {
    if ( ! airspeedInstalled) return ;     // do not process if there is no sensor
    uint32_t now = microsRp();
    if ( (now - prevReadUs) < 2000 ) return ;// it takes about 500 usec for a conversion
    // read a new pressure
    prevReadUs = now;
    if ( i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 2 , true, 1500) < 0)  {
        printf("error read sdp3x (airspeed sensor)\n");
        return;
    }
    //printf("%x %x\n", readBuffer[0], readBuffer[1]);
    difPressurePa =  (float) ((int16_t) (readBuffer[0] << 8 | readBuffer[1] & 0X00FF)) * dpScaleSdp3x     ; // diffPressure in pa
    //printf("%x %x %f\n", readBuffer[1], readBuffer[0] , difPressurePa);
    difPressureAirspeedSumPa += difPressurePa; // calculate a moving average on x values
    difPressureAirspeedCount++;                // count the number of conversion
    difPressureCompVspeedSumPa += difPressurePa; // calculate a moving average on x values
    difPressureCompVspeedCount++;                // count the number of conversion
}

/*
    unsigned long airSpeedMillis = millis() ;
    
    
#ifdef DEBUG_SDP3X_SIMULATION // use dummy read in case of simulation
        if (true){ // added to keep the same if level as when no simulation
            airSpeedData.difPressureAdc_zero =  0x7FFF * dpScaleSdp3x     ;  // diffPressure in pa
#else // not in simulation
        I2CErrorCodeSdp3x = I2c.read( _address,  2 ) ; //read 2 bytes from the device;
        if( I2CErrorCodeSdp3x == 0) { // when no read error, we calculate
      	    data[0] = I2c.receive() ;
       	    data[1] = I2c.receive() ;
            airSpeedData.difPressureAdc_zero =  ((int16_t) (data[0] << 8) + data[1] ) * dpScaleSdp3x     ;  // diffPressure in pa
#endif // end of simulation               
#define FILTERING_SDP3X_MIN        0.01   // 
#define FILTERING_SDP3X_MAX        0.1 // 
#define FILTERING_SDP3X_MIN_AT       10 // when abs is less than MIN_AT , apply MIN  
#define FILTERING_SDP3X_MAX_AT       100 // when abs is more than MAX_AT , apply MAX (interpolation in between)
            float abs_deltaDifPressure =  abs(airSpeedData.difPressureAdc_zero - smoothDifPressure) ;
            if (abs_deltaDifPressure <= FILTERING_SDP3X_MIN_AT) {
                expoSmooth_auto = FILTERING_SDP3X_MIN ;  
            } else if (abs_deltaDifPressure >= FILTERING_SDP3X_MAX_AT)  {
                expoSmooth_auto = FILTERING_SDP3X_MAX ; 
            } else {
                expoSmooth_auto = FILTERING_SDP3X_MIN + ( FILTERING_SDP3X_MAX - FILTERING_SDP3X_MIN) * (abs_deltaDifPressure - FILTERING_SDP3X_MIN_AT) / (FILTERING_SDP3X_MAX_AT - FILTERING_SDP3X_MIN_AT) ;
            }
            smoothDifPressure += expoSmooth_auto * ( airSpeedData.difPressureAdc_zero - smoothDifPressure ) ; // 
          
               // calculate airspeed based on pressure, altitude and temperature
               // airspeed (m/sec) = sqr(2 * differential_pressure_in_Pa / air_mass_kg_per_m3) 
               // air_mass_kg_per_m3 = pressure_in_pa / (287.05 * (Temp celcius + 273.15))
               // so airspeed m/sec =sqr( 2 * 287.05 * differential_pressure_pa * (temperature Celsius + 273.15) / pressure_in_pa )
               // so at 15° and 1013hpa in cm/sec = 127.79 (=sqr(2*287.05*288.15/101300))
               // or rawAirSpeed cm/sec =  2396 * sqrt( (float) abs(smoothDifPressureAdc) * temperatureKelvin  /  actualPressure) ); // in cm/sec ; actual pressure must be in pa (so 101325 about at sea level)
#ifdef AIRSPEED_AT_SEA_LEVEL_AND_15C
            airSpeedData.smoothAirSpeed =  127.79 * sqrt( (float) ( abs(smoothDifPressure) ) ); // indicated airspeed is calculated at 15 Celsius and 101325 pascal
#else               
            airSpeedData.smoothAirSpeed =  2396.0 * sqrt( (float) ( abs(smoothDifPressure) * temperatureKelvinSdp3x  /  actualPressure) ); // in cm/sec ; actual pressure must be in pa (so 101325 about at sea level)
#endif              
            if ( smoothDifPressure < 0 ) airSpeedData.smoothAirSpeed = - airSpeedData.smoothAirSpeed ; // apply the sign
              
#ifdef DEBUG_SDP3X_RAWDATA  
            static bool firstRawData = true ;
            if ( firstRawData ) {
                printer->println(F("at,  difPressure , expoSmooth_auto ,  smoothDifPressure, smoothAirSpeed, ")) ;
                firstRawData = false ;
            } else {
                printer->print( airSpeedMillis ); printer->print(F(" , "));
                printer->print(  airSpeedData.difPressureAdc_zero); printer->print(F(" , "));
                printer->print( expoSmooth_auto ); printer->print(F(" , "));
                printer->print( smoothDifPressure ); printer->print(F(" , ")); 

                printer->print( airSpeedData.smoothAirSpeed * 3.6 / 100); printer->print(F(" , ")); 
                printer->println(" ") ; 
            }       
#endif
        } // end no error on I2C    
        
        if (airSpeedMillis > nextAirSpeedMillis) { // publish airspeed only once every xx ms
              nextAirSpeedMillis = airSpeedMillis + 200 ;
              if ( airSpeedData.smoothAirSpeed >  0) {  // normally send only if positive and greater than 300 cm/sec , otherwise send 0 but for test we keep all values to check for drift  
#ifdef AIRSPEED_IN_KMH  // uncomment this line if AIR speed has to be in knot instead of km/h
                  airSpeedData.airSpeed.value = airSpeedData.smoothAirSpeed * 0.36 ; // from cm/sec to 1/10 km/h
#else
                  airSpeedData.airSpeed.value = airSpeedData.smoothAirSpeed * 0.1943844492 ; // from cm/sec to 1/10 knot/h
#endif
              } else {
                  airSpeedData.airSpeed.value = 0 ;
              }    
              airSpeedData.airSpeed.available = true ;  
        }  // end of process every xx millis
    } // end of process every 2 millis
} // End of readSensor


*/