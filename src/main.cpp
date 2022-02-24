#include <Arduino.h>
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include <config_basic.h>
#include <MS5611.h>
#include <vario.h>
#include <voltage.h>
#include <gps.h>
#include <crsf.h>

#define PICO_I2C1_SDA_PIN 2   // pin 2 = SDA
#define PICO_I2C1_SCL_PIN 3   // pin 3 = SCL
//     GPS uses pin 4 UART1 TX (in gps.cpp)
//     GPS uses pin 5 UART1 RX
//     CRSF TX use PIO and pin 8 (in crsf.cpp)

#if defined(ARDUINO_MEASURES_VOLTAGES) && (ARDUINO_MEASURES_VOLTAGES == YES)
VOLTAGE voltage ;    // class to handle voltages
#endif
#if defined( VARIO1) && (VARIO1 == MS5611)
MS5611 baro1( (uint8_t) 0x77  );    // class to handle MS5611; adress = 0x77 or 0x76
VARIO vario1;
#endif
#if defined(A_GPS_IS_CONNECTED) && (A_GPS_IS_CONNECTED == YES)
queue_t uart1Queue ;
GPS gps;
#endif

void setupI2c(){
//#define IC2USED i2c1
    i2c_init( i2c1, 400 * 1000);
    gpio_set_function(PICO_I2C1_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_I2C1_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_I2C1_SDA_PIN);
    gpio_pull_up(PICO_I2C1_SCL_PIN); 
}


void getSensors(void){
#if defined(ARDUINO_MEASURES_VOLTAGES) && (ARDUINO_MEASURES_VOLTAGES == YES)
  voltage.getVoltages();
#endif
#if defined( VARIO1) && (VARIO1 == MS5611)
  baro1.getAltitude();
  vario1.calculateAltVspeed(&baro1);    
#endif
#if defined(A_GPS_IS_CONNECTED) && (A_GPS_IS_CONNECTED == YES)
  gps.readGps();
#endif
  
}
void mergeSeveralSensors(void){
}

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  //Serial.begin(115200);
  setup_DMA_PIO(); // set up the DMA but do not yet start it
  setupI2c();
  #if defined(ARDUINO_MEASURES_VOLTAGES) && (ARDUINO_MEASURES_VOLTAGES == YES)
  voltage.begin();
  #endif
  #if defined( VARIO1) && (VARIO1 == MS5611)
    baro1.begin();    
  #endif
  #if defined(A_GPS_IS_CONNECTED) && (A_GPS_IS_CONNECTED == YES)
  gps.setupGps();
  #endif
  setupCRSF();    
}

void loop() {
  getSensors();
  mergeSeveralSensors();
  fillCRSFFrame();
    

    //put_on_DMA_PIO("Hello, world! (from PIO!)\n" );
    //printf("hello\n");
    //sleep_ms(100);
  //printf("hello");
  //delay(1000); 
}

