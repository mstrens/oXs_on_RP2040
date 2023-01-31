#pragma once

#include "hardware/adc.h"

#include "config.h"
//#include "config_advanced.h"
//#include "config_macros.h"

#define MAX_NBR_VOLTAGES 4 // number of pins available for ADC
#define VOLTAGEINTERVAL 10 // minimum interval between 2 reads (msec)
#define SUM_COUNT_MAX_VOLTAGE 10 // number of ADC conversions to calculate averages

class VOLTAGE
{
public:
    oneMeasurement_t mVolt[MAX_NBR_VOLTAGES];
    explicit VOLTAGE(void);
    void begin();
    void getVoltages(void) ;
private:
    uint8_t pin[MAX_NBR_VOLTAGES]  =  { 26, 27, 28 ,29};            // pin number to use to read each voltage (See hardware setting in oXs_config.h)  
    float offset[MAX_NBR_VOLTAGES] = { 0.0 } ;               // offset to apply while converting ADC to millivolt (See setting in oXs_config.h)  
    float mVoltPerStep[MAX_NBR_VOLTAGES] ;       // rate to apply while converting ADC to millivolt (See setting in oXs_config.h)  
    int32_t sumVoltage[MAX_NBR_VOLTAGES] = { 0,0,0};       // used to calculate average voltage
    float consumedMah = 0 ;

};
/*
struct VOLTAGEDATA {
//  bool available;    // to remove afterward
  uint16_t vrefMilliVolts;          // in mV the internal measured voltage Reference ; to remove afterward

  struct ONE_MEASUREMENT mVolt[6] ;  // in mV 
//  int32_t mVolt[6] ;             // in mV 
//  bool mVoltAvailable[6] ;
  
  byte mVoltPin[6] ;            // pin number to use to read each voltage (See hardware setting in oXs_config.h)  
  int offset[6] ;               // offset to apply while converting ADC to millivolt (See setting in oXs_config.h)  
  float mVoltPerStep[6] ;       // rate to apply while converting ADC to millivolt (See setting in oXs_config.h)  

  bool atLeastOneVolt ;         // true if there is at least one voltage to measure (added because otherwise a while in cpp never end)
  
  int32_t sumVoltage[6] ;       // used to calculate average voltage     

  uint8_t maxNumberOfCells ;    // used to fill in the max number of cells
  uint32_t mVoltCell[6] ;
  bool mVoltCell_Available [6];
  uint32_t mVoltCellMin ;
  bool mVoltCellMin_Available ;
   uint32_t mVoltCellTot ;
  bool mVoltCellTot_Available ;
 
#if defined(PROTOCOL) && ( (PROTOCOL == FRSKY_SPORT) || ( PROTOCOL == FRSKY_HUB ) || (PROTOCOL == FRSKY_SPORT_HUB ) ) //if Frsky protocol is used  
struct ONE_MEASUREMENT mVoltCell_1_2 ; 
struct ONE_MEASUREMENT mVoltCell_3_4 ;  
struct ONE_MEASUREMENT mVoltCell_5_6 ;  
#endif
};


class OXS_VOLTAGE {
  public:
#ifdef DEBUG  
    OXS_VOLTAGE(HardwareSerial &print);
#else
    OXS_VOLTAGE( uint8_t x );
#endif
    VOLTAGEDATA voltageData ;
	void setupVoltage( void );
	void readSensor();
	void resetValues();
    void convertNtcVoltToTemp (int32_t &voltage ) ;
    
  private:
#ifdef DEBUG  
     HardwareSerial* printer;
#endif
     int readVoltage( int value) ;  // read the voltage from the sensor specify by value
     void voltageNrIncrease() ; 
     uint32_t calculateCell(int32_t V0 , int32_t V1 , int32_t V2 , uint8_t cellId , uint8_t  maxNumberOfCells) ;  
};

extern bool lowVoltage ;
*/
//#endif


