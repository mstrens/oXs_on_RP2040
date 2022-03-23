#include "pico/stdlib.h"
#include "stdio.h"
#include "pico/util/queue.h"
#include <config.h>
#include "MS5611.h"
#include <vario.h>
#include <voltage.h>
#include <gps.h>
#include <tusb.h>
#include <crsf.h>
#include <param.h>
#include "sbus_pwm.h"
#include "hardware/watchdog.h"

// to do 
// PWM : support 8 channels when 1 has been tested
// PWM and Sbus : support failsafe

#define DEBUG

VOLTAGE voltage ;    // class to handle voltages

MS5611 baro1( (uint8_t) 0x77  );    // class to handle MS5611; adress = 0x77 or 0x76
VARIO vario1;

queue_t uart0Queue ; // queue is used to transfer the data from the uart0 used by GPS
GPS gps;

// CRSF is managed with 2 pio and not with the internal uart1 in order to freely select the pins

void getSensors(void){
  voltage.getVoltages();
  baro1.getAltitude();
  vario1.calculateAltVspeed(&baro1);    
  //gps.readGps();
}

void mergeSeveralSensors(void){
}


void setup() {
  stdio_init_all();
  #ifdef DEBUG
  uint16_t counter = 50;   
  while ( (!tud_cdc_connected()) && (counter--)) { sleep_ms(100);  }
  //while ( (!tud_cdc_connected()) ) { sleep_ms(100);  }
  #endif
  //printf("before setup config\n");
  
  setupConfig(); // retrieve the config parameters (crsf baudrate, voltage scale & offset, type of gps, failsafe settings)
  //printf("after setup config\n");
  
  setupI2c();      // setup I2C
  //printf("after setup i2c\n");
  voltage.begin();
  //printf("after setup voltage\n");
  baro1.begin();    
  //printf("after setup baro\n");sleep_ms(500);
  gps.setupGps();  //use UART 0 on pins 12 13
  //printf("after setup gps\n");sleep_ms(500);
  setupCRSF();  // setup the 2 pio (for TX and RX) and the DMA (for TX) and the irq handler (for Rx); use pin 9 (Rx) and 10(TX)
  //printf("after setup crsf\n");sleep_ms(500);
  setupSbusPio(); 
  setupPwm(); 
  watchdog_enable(500, 1); // require an update once every 500 msec
}

void loop() {
  
  watchdog_update();
  getSensors();
  mergeSeveralSensors();
  fillCRSFFrame();
  handleCrsfRx();
  fillSbusFrame();
  updatePWM();
  handleUSBCmd();
  tud_task();
}

int main(){
  setup();
  while(1) loop();
}