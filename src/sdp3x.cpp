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
extern int32_t i2cError;

SDP3X::SDP3X(uint8_t deviceAddress)
{
  _address = deviceAddress;
}


// **************** Setup the SDP3X sensor *********************
void SDP3X::begin() {
    airspeedInstalled = false;
    if ( config.pinScl == 255 or config.pinSda == 255) return; // skip if pins are not defined
    // fill buffer with dummy values (just to check if they are different after a read)
    for (uint8_t i=0; i<9; i++){ //
        readBuffer[i] = i;
    }    
    
#ifdef DEBUG_SDP3X_SIMULATION // use dummy setup in case of simulation
    nextPressureReadMillis = millis() + 2;    //
    nextAirSpeedMillis  = nextPressureReadMillis + 200 ; 
    temperatureCelsius = 23.4  ; 
    temperatureKelvin = 273.15 + temperatureCelsius ; // in Kelvin
    dpScaleSdp3x =  966.0 / 1013.0 / 60 ; //60=SDP31 , 240=SDP32
#else // not a simulation
    

    #ifdef USE_ADP810_INSTEAD_OF_SDPxx  // ADP has no continuous conversion mode and request a conversion after each read
    uint8_t cmdData[2] = { 0X37 , 0X2D} ; // request a conversion 
    i2cError = i2c_write_timeout_us (i2c1 , _address, &cmdData[0] , 2 , false , 1000) ;
    if ( i2cError <0 ) {
        printf("error write command to sdp3x : %i\n",i2cError);
        return ; 
    }
    sleep_ms(100); // wait for end of conversion (doc says 10 ms)
    #else
   // set the sensor in continous mode with averaging (send a command 0X3615)
    uint8_t cmdData[2] = { 0X36 , 0X15} ; 
    i2cError = i2c_write_timeout_us (i2c1 , _address, &cmdData[0] , 2 , false , 1000);
    if (  i2cError < 0 ) {
        printf("error write command to sdp3x: %i\n",i2cError );
        return ; 
    }
    sleep_ms(100); // wait 100msec in order to get the first data (datasheet says 8 msec) 
    //if ( i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 9 , false, 5500) < 0)  {
    #endif
    i2cError =  i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 9 , false, 5500) ;    
    if (i2cError < 0) {
        printf("error when reading 9 bytes from sdp3x = %d \n", (int) i2cError);    
        return ; // skip if there is an error
    }  
    // just usefull for debugging
    #ifdef DEBUG
    printf("SDP first read: ");
    for (uint8_t i = 0; i<9 ; i++){
        printf(" %X ",readBuffer[i]);
    }
    printf("\n");
    #endif
    // first 2 bytes = pressure, byte 3 = CRC, byte 4 & 5 = temp,  bytes 6, 7, 8 = scale factor
    //nextPressureReadMillis = millisRp() + 2;    //
    //nextAirSpeedMillis  = nextPressureReadMillis + 200 ; 
    temperatureCelsius =   ((float)(( ((int16_t)readBuffer[3]) << 8)  | readBuffer[4])) / 200.0  ;
    temperatureKelvin = 273.15 + temperatureCelsius ; // in Kelvin
    dpScaleSdp3x =  966.0 / 1013.25 / ((float)((int16_t)readBuffer[6] << 8 | readBuffer[7]));
    // datasheet says that we have to apply a correction of 966/actual pressure in mbar; it is estimated with 1013
#endif // endif for simulation
    #ifdef DEBUG
    printf("sdp3x has been succesfuly detected\n");
    #endif
    airspeedInstalled = true;
    prevReadUs = microsRp(); 
}  //end of setup

/****************************************************************************/
/* readSensor - Read differential pressure                                  */
/****************************************************************************/
void SDP3X::getDifPressure() {
    if ( ! airspeedInstalled) return ;     // do not process if there is no sensor
    uint32_t now = microsRp();
    #ifdef USE_ADP810_INSTEAD_OF_SDPxx
    if ( (now - prevReadUs) < 15000 ) return ;// it takes about 500 usec for a conversion with a SDP (it is more than 10ms for a ADP810)
    #else
    if ( (now - prevReadUs) < 2000 ) return ;// it takes about 500 usec for a conversion with a SDP (it is more than 10ms for a ADP810)
    #endif
    // read a new pressure
    prevReadUs = now;
    i2cError = i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 3 , false, 1500) ;
    if ( i2cError  < 0)  {
        printf("error read sdp3x (airspeed sensor): %i\n",i2cError );
        return;
    }
    //printf("%x %x\n", readBuffer[0], readBuffer[1]);
    difPressurePa =  (float) ((int16_t) ( readBuffer[0] << 8 | (readBuffer[1] & 0X00FF))) * dpScaleSdp3x     ; // diffPressure in pa
    //printf("%x %x %f\n", readBuffer[0], readBuffer[1] , difPressurePa);
    difPressureAirspeedSumPa += difPressurePa; // calculate a moving average on x values
    difPressureAirspeedCount++;                // count the number of conversion
    difPressureCompVspeedSumPa += difPressurePa; // calculate a moving average on x values
    difPressureCompVspeedCount++;                // count the number of conversion
    #ifdef USE_ADP810_INSTEAD_OF_SDPxx
    uint8_t cmdData[2] = { 0X37 , 0X2D} ; // ask a new conversion (that will be read next time)
    i2cError = i2c_write_timeout_us (i2c1 , _address, &cmdData[0] , 2 , false , 1000);
    if ( i2cError  <0 ) {
        printf("error write command to adp810: %i\n",i2cError);
        return ; 
    }
    #endif
}

