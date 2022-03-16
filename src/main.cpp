#include "pico/stdlib.h"
#include "stdio.h"
#include "pico/util/queue.h"
#include <config.h>
#include "MS5611.h"
#include <vario.h>
#include <voltage.h>
#include <gps.h>
#include <crsf.h>
#include <param.h>
#include "sbus_pwm.h"
#include "hardware/watchdog.h"

// to do 
// PWM : support 8 channels when 1 has been tested
// PWM and Sbus : support failsafe


VOLTAGE voltage ;    // class to handle voltages

MS5611 baro1( (uint8_t) 0x77  );    // class to handle MS5611; adress = 0x77 or 0x76
VARIO vario1;

queue_t uart0Queue ; // queue is used to transfer the data from the pio uart GPS
GPS gps;


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
  setupConfig(); // retrieve the config parameters (crsf baudrate, voltage scale & offset, type of gps, failsafe settings)
  setup_DMA_PIO(); // set up the DMA used to manage Sport Uart but do not yet start it
  setupI2c();      // setup I2C
  voltage.begin();
  baro1.begin();    
  gps.setupGps();
  setupCRSF();
  setupSbusPio(); 
  setupPwm(); 
  watchdog_enable(500, 1); // require an update once every 100 msec

  //sleep_ms(1000)  ;
}

void loop() {
  watchdog_update();
  getSensors();
  mergeSeveralSensors();
  fillCRSFFrame();
  fillSbusFrame();
  updatePWM();
  handleUSBCmd();
}

int main(){
  setup();
  while(1) loop();
}