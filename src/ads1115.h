#pragma once

#include "stdint.h"


//#include <Arduino.h>
#include "config.h"

// values to say how to use a ads measurement
#define ADS_VOLT_1 0
#define ADS_VOLT_2 1
#define ADS_VOLT_3 2
#define ADS_VOLT_4 3
#define ADS_VOLT_5 4
#define ADS_VOLT_6 5
#define ADS_AIRSPEED 6
#define ADS_CURRENT 7 
#define ADS_NOT_USED 8

// values to configure the ads multiplexer
#define A0_TO_A1 0
#define A0_TO_A3 1
#define A1_TO_A3 2
#define A2_TO_A3 3
#define A0_TO_GND 4
#define A1_TO_GND 5
#define A2_TO_GND 6
#define A3_TO_GND 7
#define ADS_OFF 8

// values to configure the programable ads gain amplifier
#define MV6144 0 
#define MV4096 1
#define MV2048 2
#define MV1024 3
#define MV512 4
#define MV256 5

// value to configure the rate of ads conversion (value is the delay in msec that must be wait before getting the conversion ; this take care of the tolerance of 10% foreseen in the datasheet 
#define MS137 0
#define MS69 1
#define MS35 2
#define MS18 3 
#define MS9 4 
#define MS5 5 
#define MS3 6
#define MS2 7

/* i2c_write does not work if I pass the register address as a #define. It only works if the address is a const uint8_t and it is passed as a pointer (&)*/
static const uint8_t ADS1115_POINTER_CONVERSION = 0x00;
static const uint8_t ADS1115_POINTER_CONFIGURATION = 0x01;
static const uint8_t ADS1115_POINTER_LO_THRESH = 0x02;
static const uint8_t ADS1115_POINTER_HI_THRESH = 0x03;

class ADS1115 {
public:
  ADS1115(uint8_t addr, uint8_t idx) ;
  
  void begin();
  bool  readSensor( void ); //return true if an averaged has just been calculated  
  void ads_requestNextConv(void) ;
  bool    adsInstalled = false; 
//  void ads_calculateCurrent(void) ;
//  void ads_calculate_airspeed( int16_t ads_difPressureADC ) ;
//#if defined(AN_ADS1115_IS_CONNECTED) && (AN_ADS1115_IS_CONNECTED == YES ) && defined(ADS_MEASURE) && defined(ADS_CURRENT_BASED_ON)
//  struct CURRENTDATA adsCurrentData ;
//  float floatConsumedMilliAmps ; // in mA
//#endif

//#if defined(AN_ADS1115_IS_CONNECTED) && (AN_ADS1115_IS_CONNECTED == YES ) && defined(ADS_MEASURE) && defined(ADS_AIRSPEED_BASED_ON)
//  struct AIRSPEEDDATA adsAirSpeedData ;
//#endif
  
private: 
uint32_t ads_MilliAskConv  ; 
uint8_t ads_Addr;
uint8_t ads_idx ;         // make the difference between ads 1 and 2 
uint8_t ads_CurrentIdx = 0;
uint8_t I2CErrorCodeAds1115 ; 
int32_t ads_SumOfConv[4] ; // summarise all conversion in order to calculate average 
uint8_t ads_Counter[4]  ;
int32_t ads_Value[4] ; //averaged conversion including offset and scale
int32_t ads_Available[4] ; //averaged conversion including offset and scale

uint8_t ads_Last_Conv_Idx ;

}; // end class OXS_ADS1115

const uint8_t ads_Measure[2][4] = { { ADS1_MEASURE} , {ADS2_MEASURE } }; //  how to configure the multiplexer
const uint8_t ads_Gain[2][4] = { { ADS1_FULL_SCALE_VOLT } , { ADS2_FULL_SCALE_VOLT } }; //  how to configure the programmable gain amplifier
const uint8_t ads_Rate[2][4] = { { ADS1_RATE } , { ADS2_RATE } }; // how to configure the time of conversion
const float ads_Offset[2][4] =  { { ADS1_OFFSET } , { ADS2_OFFSET } };
const float ads_Scale[2][4] =  { { ADS1_SCALE } , { ADS2_SCALE } };
const uint8_t ads_MaxCount[2][4] = { { ADS1_AVERAGING_ON } , { ADS2_AVERAGING_ON } }; //number of conversion before averaging
  

