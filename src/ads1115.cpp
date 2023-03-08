#include "ads1115.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "stdio.h"
#include <inttypes.h>
#include "tools.h"
#include "pico/double.h"
#include "param.h"


extern CONFIG config;




ADS1115::ADS1115(uint8_t addr, uint8_t idx)
{  // constructor
  ads_Addr=addr;
  ads_idx = idx;
}


// **************** Setup the ADS1115 sensor *********************
void ADS1115::begin() {
#ifdef DEBUG  
  printf("ADS1115 sensor I2C Addr=%X\n", ads_Addr);
#endif
    ads_Counter[0] = ads_MaxCount[ads_idx][0] ;
    ads_Counter[1] = ads_MaxCount[ads_idx][1] ;
    ads_Counter[2] = ads_MaxCount[ads_idx][2] ;
    ads_Counter[3] = ads_MaxCount[ads_idx][3] ;
/*  
#if defined(AN_ADS1115_IS_CONNECTED) && (AN_ADS1115_IS_CONNECTED == YES ) && defined(ADS_MEASURE) && defined(ADS_CURRENT_BASED_ON)
  adsCurrentData.milliAmps.available = false ;
  adsCurrentData.consumedMilliAmps.available = false ;
#endif            
*/  
  adsInstalled = false;
  if ( config.pinScl == 255 or config.pinSda == 255) return; // skip if pins are not defined
  
  
  ads_requestNextConv() ; // Write next config and ask for conversion
  if (I2CErrorCodeAds1115 == 0) adsInstalled = true;  
#ifdef DEBUG  
  if (adsInstalled ) {
    printf("Set up Ads1115 done and OK.\n");  
  } else {
    printf("Error in Set up of ads1115.\n");  
  }
#endif
}  //end of setup


/****************************************************************************/
/* readSensor - Read ADS115                                                 */
/*********************    *******************************************************/
bool ADS1115::readSensor() {  // return true when there is a new average data to calculate
    if ( ! adsInstalled) return false;  
    if ( ( millisRp() - ads_MilliAskConv ) <= (uint32_t) ( (  0x88 >> ads_Rate[ads_idx][ads_CurrentIdx]) + 2) ) return false;
    // when delay of conversion expires (NB delay is 137 msec when ads_rate = 0, and goes down up to 3ms then is divided by 2 for each increase ) 
    if( I2CErrorCodeAds1115 != 0 ) { // if there is an error on previous I2C request for a conversion
        printf("Write error ads1115 in previous request conversion\n");
        ads_requestNextConv() ;
        return false ;
    }
    // read the config status to see if conversion is really done
    uint8_t adsConfigReg  = 0x01;
    uint8_t adsConfigMsb;
    if (i2c_write_timeout_us (i2c1 , ads_Addr, &adsConfigReg , 1 , false, 1000) <0) { 
        printf("Write error ads1115"); //if there is no error on previous I2C request
        return false;
    }  
    if (i2c_read_timeout_us (i2c1 , ads_Addr , &adsConfigMsb , 1 , false , 1500) <0) {
        printf("read error ads1115"); //if there is no error on previous I2C request
        return false;
    }
    if ( ( adsConfigMsb & 0X80) == 0) return false; // conversion is not yet done
    //printf("conversion ads115 in %d ms\n", (millisRp() - ads_MilliAskConv)); 
    
    // at this stage conversion is done
    uint8_t adsReg = 0X0; // 0X0 = adress of conversion register
    uint8_t data[2]; // buffer to read adc
    // send the Address, 0 = conversion register (in order to be able to read the conversion register)
    if (i2c_write_timeout_us (i2c1 , ads_Addr, &adsReg , 1 , false, 1000) <0 ) { 
        printf("Write error ads1115"); //if there is no error on previous I2C request
        return false;
    }  
    if (i2c_read_timeout_us (i2c1 , ads_Addr , &data[0] , 2 , false , 1500) <0) {
        printf("read error ads1115"); //if there is no error on previous I2C request
        return false;
    }
    uint16_t valueAdc ;
    valueAdc = data[0] << 8 | data[1] ;
    //printf("idx=%d : %d adc at%d\n", ads_CurrentIdx , (int) valueAdc , (int) millisRp());
    ads_SumOfConv[ads_CurrentIdx] +=   (int16_t) valueAdc ;
    ads_Counter[ads_CurrentIdx]-- ;
    if ( ads_Counter[ads_CurrentIdx] == 0 ) {
        float adcToMvoltScaling ;
        if (ads_Gain[ads_idx][ads_CurrentIdx]) {
            adcToMvoltScaling = (0X2000 >>  ads_Gain[ads_idx][ads_CurrentIdx] ) / 32768.0 ;  // When ads_Gain[] = 1, it means that 32768 = 4096 mvolt; 4096 = 0x1000) , when 2, it is 2048 mvolt, ...
        } else {
            adcToMvoltScaling =  6144  / 32768.0 ;  // When ads_Gain[] = 0, it means that 32768 = 6144 mvolt
        }
        ads_Value[ads_CurrentIdx] = round( ((float) ads_SumOfConv[ads_CurrentIdx] / (float) ads_MaxCount[ads_idx][ads_CurrentIdx] * adcToMvoltScaling ) * ads_Scale[ads_idx][ads_CurrentIdx] ) + ads_Offset[ads_idx][ads_CurrentIdx];
        //printf("Adc=%d pin=%d : %d mVolt at%d\n", ads_idx , ads_CurrentIdx , ads_Value[ads_CurrentIdx], millisRp());
        ads_Available[ads_CurrentIdx] = true ;
        ads_SumOfConv[ads_CurrentIdx] = 0 ;            // reset the sum 
        ads_Counter[ads_CurrentIdx] = ads_MaxCount[ads_idx][ads_CurrentIdx] ;   // reset the counter to the number of count before averaging
        sent2Core0( ADS_1_1 + ( ads_idx * 4) + ads_CurrentIdx , ads_Value[ads_CurrentIdx]); // save the value
        //#define DEBUG_ADC
        #ifdef DEBUG_ADC
            printf("Adc %i : %d mVolt\n", ads_idx , (int) ads_Value[ads_CurrentIdx]);
        #endif
    }
    ads_requestNextConv() ;
    return true ;
} // end of readSensor


void ADS1115::ads_requestNextConv(void) {
    I2CErrorCodeAds1115 = 0 ; // 0 means that there is no I2C error
    do {
      ads_CurrentIdx++ ;
      if( ads_CurrentIdx > 3 ) ads_CurrentIdx = 0 ;
    } while (  ads_Measure[ads_idx][ads_CurrentIdx] == ADS_OFF ) ;
// perhaps this line has to be splitted in 2 in order to let multiplexer, gain and rate to set up before asking for a conversion.

    uint8_t dataToWrite[3] ;
    dataToWrite[0] = ADS1115_POINTER_CONFIGURATION; // this means that we are adressing the configuration register
    // bit 7 = 1 when write means start a single conversion
    // bits 6,5,4= which pin to measure (e.g. A0 to ground)
    // bits 3,2,1= gain = max voltage to measure
    // bit 0 = 1 means single conversion
    dataToWrite[1] = (( 1 << 7 | ads_Measure[ads_idx][ads_CurrentIdx] << 4 | ads_Gain[ads_idx][ads_CurrentIdx] << 1 | 1  ) ) ;
    // bits 7,6,5 = conversion rate
    // bit 4 = comparator mode; 0= traditional / 1= windows
    // bit 3 = comparator polarity ; not used
    // bit 2 = latching comparator ; not used
    // bits 1,0 = 11 means comparator disabled
    dataToWrite[2] = ( ads_Rate[ads_idx][ads_CurrentIdx] << 5 | 0B11 );
        // bit 15 says that a conversion is requested, bit 8 says on shot mode, bits 0 and 1 = 11 says comparator is disabled.
    
    if (i2c_write_timeout_us (i2c1 , ads_Addr , &dataToWrite[0] , 3 , false,1000) < 0 ) {
        printf("Write error for ads1115\n");
        I2CErrorCodeAds1115 = -1; // -1 shows an error
    }    
    //I2CErrorCodeAds1115 = I2c.write((uint8_t) ads_Addr , (uint8_t) 0X01 , (uint8_t) 2 , &dataToWrite[0] ) ; // send the Address, 1 = config register , 2 bytes , pointer to the data to write
//    if ( I2CErrorCodeAds1115 ) I2CErrorCodeAds1115 = I2c.write( (uint8_t) ads_Addr , (uint8_t) 0X01 , (uint8_t) 2 , &dataToWrite[0] ) ; // retry once if there is an error (probably we should add a clear of I2C bus in between)
#ifdef DEBUGADS1115REQUESTCONV  
      printer->print(F("At ")); printer->print(millisRp()) ;
      printer->print(F(" cmd=")); printer->print(dataToWrite[0], HEX) ;
      printer->print(F(" ")); printer->print( dataToWrite[1] , HEX ) ;
      printer->println(" ");
#endif
    ads_MilliAskConv = millisRp() ;
    //printf("request %x %x %x idx=%d at%d  ",ads_CurrentIdx , dataToWrite[1] , dataToWrite[2], ads_CurrentIdx , ads_MilliAskConv );
    
} // end of Ads_requestNextConv


#if defined(AN_ADS1115_IS_CONNECTED) && (AN_ADS1115_IS_CONNECTED == YES ) && defined(ADS_MEASURE) && defined(ADS_CURRENT_BASED_ON) // this part is compiled only when the config ask for current
void OXS_ADS1115::ads_calculateCurrent(void) {
    
    static int32_t sumCurrent = 0 ;
    static uint16_t cnt ;
    static uint32_t milliTmp ;
    static uint32_t lastCurrentMillis ;
    sumCurrent +=  ads_Conv[ads_CurrentIdx].value ;
    cnt++ ;
    milliTmp = millisRp() ;
  if  (lastCurrentMillis == 0)  {
    lastCurrentMillis = milliTmp ;
  }
  else if (  (milliTmp - lastCurrentMillis ) > 200 )  {   // calculate average only once per 200 millisec
      adsCurrentData.milliAmps.value = ((sumCurrent / cnt) - MVOLT_AT_ZERO_AMP ) * 1000 / MVOLT_PER_AMP ;
//      if (currentData.milliAmps.value < 0) currentData.milliAmps.value = 0 ;
      adsCurrentData.milliAmps.available = true ;
      floatConsumedMilliAmps += ((float) adsCurrentData.milliAmps.value) * (milliTmp - lastCurrentMillis ) / 3600.0 /1000.0 ;   
      adsCurrentData.consumedMilliAmps.value = (int32_t) floatConsumedMilliAmps ;
      adsCurrentData.consumedMilliAmps.available = true ;
      lastCurrentMillis =  milliTmp ;
#ifdef DEBUGCURRENT
      printer->print("At time  = ");
      printer->print(milliTmp);
      printer->print(" Cnt = ");
      printer->print(cnt);
      printer->print(" average current =  ");
      printer->print(adsCurrentData.milliAmps.value);
      printer->print(" consumed milliAmph =  ");
      printer->println(adsCurrentData.consumedMilliAmps.value);
#endif
      sumCurrent = 0;
      cnt = 0;
  } 
}
#endif


#if defined(ADS_AIRSPEED_BASED_ON) and (ADS_AIRSPEED_BASED_ON >= ADS_VOLT1) and (ADS_AIRSPEED_BASED_ON <= ADS_VOLT_4) // this part is compiled only when required
float ads_sumDifPressureAdc_0 ;
uint8_t ads_cntDifPressureAdc_0 ;

void OXS_ADS1115::ads_calculate_airspeed( int16_t ads_difPressureAdc ) {
  // convert ads_volt to pressure.
  static int32_t ads_pressure;
  static int32_t difPressureSum ;
  static float offset7002 ;
  static int16_t calibrateCount7002 ;
  static boolean calibrated7002 = false ;
  static float ads_difPressureAdc_0 ;
  static float ads_abs_deltaDifPressureAdc ;
  static float ads_smoothDifPressureAdc ;
  static float expoSmooth7002_adc_auto ;
  static int32_t ads_smoothAirSpeed ; 
  uint32_t ads_airSpeedMillis ;
  static uint32_t ads_nextAirSpeedMillis ;
//#define DEBUG_AIRSPEED_WITH_DUMMY_ADS_DATA
#ifdef DEBUG_AIRSPEED_WITH_DUMMY_ADS_DATA
  static int16_t dummy_ads_value ; 
  ads_difPressureAdc = ((millisRp() / 1000 ) % (100) ) * 300 ; 
#endif 
  if ( calibrated7002 == false) {
       calibrateCount7002++ ;
       if (calibrateCount7002 == 256 ) { // after 256 reading , we can calculate the offset 
         offset7002 =  (  (float) difPressureSum / 128.0 ) ; //there has been 128 reading (256-128)                     
         calibrated7002 = true ;
       } else if  (calibrateCount7002 >= 128  ){ // after 128 reading, we can start cummulate the ADC values in order to calculate the offset 
          difPressureSum += ads_difPressureAdc ;
       } // end calibration
  }  else { // sensor is calibrated
                    ads_difPressureAdc_0 = ( ads_difPressureAdc - offset7002 )  ;
                    ads_sumDifPressureAdc_0 += ads_difPressureAdc_0 ;
                    ads_cntDifPressureAdc_0++ ;
#define FILTERING7002_ADC_MIN        0.001 // 
#define FILTERING7002_ADC_MAX        0.01  // 
#define FILTERING7002_ADC_MIN_AT       10  // when abs(delta between ADC and current value) is less than MIN_AT , apply MIN  
#define FILTERING7002_ADC_MAX_AT       100 // when abs(delta between ADC and current value) is more than MAX_AT , apply MAX (interpolation in between)
                    ads_abs_deltaDifPressureAdc =  abs(ads_difPressureAdc_0 - ads_smoothDifPressureAdc) ;
                    if (ads_abs_deltaDifPressureAdc <= FILTERING7002_ADC_MIN_AT) {
                       expoSmooth7002_adc_auto = FILTERING7002_ADC_MIN ;  
                    } else if (ads_abs_deltaDifPressureAdc >= FILTERING7002_ADC_MAX_AT)  {
                       expoSmooth7002_adc_auto = FILTERING7002_ADC_MAX ; 
                    } else {
                       expoSmooth7002_adc_auto = FILTERING7002_ADC_MIN + ( FILTERING7002_ADC_MAX - FILTERING7002_ADC_MIN) * (ads_abs_deltaDifPressureAdc - FILTERING7002_ADC_MIN_AT) / (FILTERING7002_ADC_MAX_AT - FILTERING7002_ADC_MIN_AT) ;
                    }
                    if ( ( ads_smoothDifPressureAdc <=2 ) && ( ads_smoothDifPressureAdc >= -2 ) ) expoSmooth7002_adc_auto *= 0.2 ; 
                    ads_smoothDifPressureAdc += expoSmooth7002_adc_auto * ( ads_difPressureAdc_0 - ads_smoothDifPressureAdc ) ; // 

               // calculate airspeed based on pressure, altitude and temperature
               // airspeed (m/sec) = sqr(2 * differential_pressure_in_Pa / air_mass_kg_per_m3) 
               // air_mass_kg_per_m3 = pressure_in_pa / (287.05 * (Temp celcius + 273.15))
               // and differantial_pressure_Pa =  ((smoothDifPressureAdc  ) * 2048 / 32768) ;  // with 7002, 1 mvolt = 1 pa and ads1115 ADC gives 32768 when volt = 2048 mvolt; so 1 step ADC = 2048 / 32768
                //2048 is used because we supposed that ads gain is set on 2048 ; MPXV7002 provides 1 mvolt per pa wit 5 volt Vcc; it is ratiometric so there should be some Vcc correction
               // so airspeed m/sec =sqr( 2 * 287.05 *  2048 / 32768 * smoothDifPressureAdc * (temperature Celsius + 273.15) / pressure_in_pa )
               // rawAirSpeed cm/sec =  5,99 * 100 * sqrt( (float) abs(smoothDifPressureAdc) * temperature4525  /  actualPressure) ); // in cm/sec ; actual pressure must be in Pa (so 101325 about at sea level
               //                    =  32.32 * sqrt( (float) abs(smoothDifPressureAdc) ); // in cm/sec ; if pressure is standard = 101325 and temp = 15 C°)
               //                    =  10256 * sqrt( (float) abs(smoothDifPressureAdc) /  actualPressure) ); // in cm/sec ; temp is supposed to be 20 C°, pressure is in Pa
#ifdef AIRSPEED_AT_SEA_LEVEL_AND_15C
               ads_smoothAirSpeed =  32.32 * sqrt( (float) ( abs(ads_smoothDifPressureAdc) ) ); // indicated airspeed is calculated at 15 Celsius and 101325 pascal
#else               
               ads_smoothAirSpeed =  10256.0 * sqrt( (float) ( abs(ads_smoothDifPressureAdc)  /  (float) actualPressure) ); // in cm/sec ; actual pressure must be in pa (so 101325 about at sea level)
#endif              
             if ( ads_smoothDifPressureAdc < 0 ) ads_smoothAirSpeed = - ads_smoothAirSpeed ; // apply the sign
      
  }  // end of test on calibration
  ads_airSpeedMillis = millisRp() ;
  if ( ads_airSpeedMillis  > ads_nextAirSpeedMillis){ // publish airspeed only once every xx ms
              ads_nextAirSpeedMillis = ads_airSpeedMillis + 200 ;
//              if ( ads_smoothAirSpeed >  0) {  // normally send only if positive and greater than 300 cm/sec , otherwise send 0 but for test we keep all values to check for drift  
#ifdef AIRSPEED_IN_KMH  // uncomment this line if AIR speed has to be in knot instead of km/h
                  adsAirSpeedData.airSpeed.value = ads_smoothAirSpeed * 0.36 ; // from cm/sec to 1/10 km/h
#else
                  adsAirSpeedData.airSpeed.value = ads_smoothAirSpeed * 0.1943844492 ; // from cm/sec to 1/10 knot/h
#endif
//              } else {
//                  adsAirSpeedData.airSpeed.value = 0 ;
//              }    
              adsAirSpeedData.airSpeed.available = true ; 

// check if offset must be reset
              if (adsAirSpeedData.airspeedReset) { // adjust the offset if a reset command is received from Tx
                    offset7002 =  offset7002  + ads_smoothDifPressureAdc ;
                    ads_smoothDifPressureAdc = 0 ;
                    adsAirSpeedData.airspeedReset = false ; // avoid that offset is changed again and again if PPM do not send a command
              }
#ifdef DEBUGADSAIRSPEEDDATA
                  static bool firstRawData = true ;
                  if ( firstRawData ) {
                          printer->println(F("at,  difPressureAdc ,difPressADC_0 , ads_smoothDifPressureAdc , actualPressure , adsAirSpeedData.airSpeed, offset, reset  ")) ;
                        firstRawData = false ;
                  } else {
                        printer->print( ads_airSpeedMillis ); printer->print(F(" , "));
                        printer->print( ads_difPressureAdc ); printer->print(F(" , "));
                        printer->print( ads_difPressureAdc_0); printer->print(F(" , "));
                        printer->print( ads_smoothDifPressureAdc * 1000); printer->print(F(" , ")); 
                        printer->print( actualPressure ); printer->print(F(" , ")); 
                        printer->print( ads_smoothAirSpeed * 3.6 / 100 ); printer->print(F(" , "));
                        printer->print( offset7002 * 1000); printer->print(F(" , ")); 
                        if ( ads_smoothDifPressureAdc == 0) printer->print(F(" reset")) ;
                        
                        printer->println(" ") ; 
                  }       
#endif


 
  } // end test on millisRp()
  
}
#endif // end of conditional compiling for calculate airspeed.

