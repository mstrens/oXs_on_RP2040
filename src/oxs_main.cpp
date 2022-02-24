/*

// to do
// check the conversion of pressure and altitude because it is a mix of float and int from 2 sources
// create a class to  manage the communication with the receiver


#include <Arduino.h>
#include "Wire.h"

#include "pico/stdlib.h"
#include "hardware/pio.h"
//#include "uart_tx.pio.h"
#include "pico/util/queue.h"

#include <config_basic.h>
#include <MS5611.h>
#include <vario.h>
#include <voltage.h>
#include <crsf.h>

//  BREAKOUT  MS5611  aka  GY63 - see datasheet
//
//  SPI    I2C
//              +--------+
//  VCC    VCC  | o      |
//  GND    GND  | o      |
//         SCL  | o      |
//  SDI    SDA  | o      |
//  CSO         | o      |
//  SDO         | o L    |   L = led
//          PS  | o    O |   O = opening  PS = protocol select
//              +--------+
//
//  PS to VCC  ==>  I2C  (GY-63 board has internal pull up, so not needed)
//  PS to GND  ==>  SPI
//  CS to VCC  ==>  0x76
//  CS to GND  ==>  0x77
#define YES 1
#define NO  0
#define FIRST_BARO_SENSOR_USE   MS5611
#define USE_VARIO1

#define DEBUG

#if defined(FIRST_BARO_SENSOR_USE) && (FIRST_BARO_SENSOR_USE == MS5611)
MS5611 baro1(0x77);
queue_t altitudeQ;
  typedef struct altitudeItem {
  uint32_t altitude;
  uint32_t interval;
} altitudeItem_t;

const int ALTITUDE_QUEUE_LENGTH = 2;
#endif

#if defined(USE_VARIO1)
VARIO vario1 ;
//struct repeating_timer timerVario1;
#endif
#if defined(ARDUINO_MEASURES_VOLTAGES) && (ARDUINO_MEASURES_VOLTAGES == YES)
VOLTAGE voltage ;    
#endif
  



uint32_t count , start, stop = 0; // just to test
//prototypes

//bool vario1_timer_callback(struct repeating_timer *t)
//{
//  return true; // tue says to repeat the timer 
//}


void setup() {
  
  #if defined( DEBUG)
  Serial.begin(115200);
  while(!Serial);
  #endif


  // pehaps a while here
  // When a pressure sensor is used
  //    presure1.begin() 
  #if defined(FIRST_BARO_SENSOR_USE) && (FIRST_BARO_SENSOR_USE == MS5611)
  if (baro1.begin() == true)
  {
    Serial.println("MS5611 found.");
  }
  else
  {
    Serial.println("MS5611 not found. halt.");
    while (1);
  }
  //queue_init(&altitudeQ, sizeof(altitudeItem_t), ALTITUDE_QUEUE_LENGTH); //initialize the queue to get altitude from the preesure sensor
  #endif
  #if defined(ARDUINO_MEASURES_VOLTAGES)
  
  #endif
  
  //add_repeating_timer_us(-9200, vario1_timer_callback, NULL, &timerVario1);
  


  //    presure2.begin() // if there is second barosensor
  // when there is a GPS
  //    gps.begin()
}

void loop() {
  //getSensors() ;
  //calculateFields()
  //generateFrame();
  //sendTelemetry()

  start = micros();
  #if defined (USE_VARIO1)
  int result = baro1.getAltitude();   // uses default OSR_ULTRA_LOW  (fastest)
  if (result == 0) vario1.calculateAltVspeed(&baro1) ; // if no error, calculate smooth alt, Vspeed and set some flags
  #endif
  #if defined(ARDUINO_MEASURES_VOLTAGES)
  voltage.getVoltages();
  #endif  
  fillCRSFFrame();
  
  stop = micros();

  if (count % 20 == 0)
  {
    Serial.println();
    Serial.println("CNT\tDUR\tRES\tTEMP\tPRES");
  }

  Serial.print(count);
  count++;
  Serial.print("\t");
  Serial.print(stop - start);
  Serial.print("\t");
  Serial.print(result);
  Serial.print("\t");
  Serial.print(baro1.getTemperature(), 2);
  Serial.print("\t");
  Serial.print(baro1.getPressure(), 2);
  Serial.println();
}

*/