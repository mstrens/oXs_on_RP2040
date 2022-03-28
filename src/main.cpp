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
  gps.readGps();
}

void mergeSeveralSensors(void){
}


void setup() {
  stdio_init_all();
  uint16_t counter = 50; 
  if ( watchdog_caused_reboot() ) counter = 0; // avoid the UDC wait time when reboot is caused by the watchdog   
  while ( (!tud_cdc_connected()) && (counter--)) { sleep_ms(100);  }
  
  setupConfig(); // retrieve the config parameters (crsf baudrate, voltage scale & offset, type of gps, failsafe settings)
  
  setupI2c();      // setup I2C
  voltage.begin();
  baro1.begin();    
  gps.setupGps();  //use UART 0 on pins 12 13
  setupCRSF();  // setup the 2 pio (for TX and RX) and the DMA (for TX) and the irq handler (for Rx); use pin 9 (Rx) and 10(TX)
  setupSbusPio(); 
  setupPwm(); 
  watchdog_enable(500, 1); // require an update once every 500 msec
}

void loop() {
  watchdog_update();
  getSensors();
  mergeSeveralSensors();
  fillCRSFFrame();
  watchdog_update();
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