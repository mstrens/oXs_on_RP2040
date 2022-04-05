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
#include "sbus_out_pwm.h"
#include "sbus_in.h"
#include "hardware/watchdog.h"
#include "sport.h"
#include "param.h"
#include "tools.h"

// Look at file config.h for more details
//
// This project can be interfaced with a ELRS (it uses CRSF protocol) or a FRSKY receiver (it uses Sbus for Rc channels and Sport for telemetry )
// Sbus as output is based only on CRSF and uses pin GPIO 0 (PIO TX), pio0, sm 2 and one dma 
// PWM uses pins 1 up to 8
// CRSF uses GPIO 9 (pio RX on pio0 sm 1 + one IRQ PIO0_IRQ_0 + one queue)  and GPIO 10 (pio TX on pio0 sm 0 + one dma)
// Sbus as input (for Frsky) uses pin GPIO 9.
// Sport (for Frsky) uses pin GPIO 10 
//    For Sbus in, we use UART1 RX (easier for 8N2) + one IRQ UART1_IRQ + one queue
//    For Sport we use pio0 and 
//          for Rx sm1 + one IRQ PIO0_IRQ_0 + one queue (sportRxQueue)
//          for Tx sm0 (Tx) + one dma
// GPS uses gpio 12 (UART0-TX) and gpio 13 (UART0-RX) + one IRQ UART0_IRQ + one queue (gpsQueue)
// I2C uses pins : GPIO 14 = PICO_I2C1_SDA_PIN  and  GPIO 15 = PICO_I2C1_SCL_PIN =  
// Analog read uses GPIO pins 26 up to 29

//#define DEBUG

VOLTAGE voltage ;    // class to handle voltages

MS5611 baro1( (uint8_t) 0x77  );    // class to handle MS5611; adress = 0x77 or 0x76
VARIO vario1;

queue_t gpsQueue ; // queue is used to transfer the data from the uart0 used by GPS
GPS gps;

// CRSF is managed with 2 pio and not with the internal uart1 in order to freely select the pins

extern CONFIG config;

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
  //if ( watchdog_caused_reboot() ) counter = 0; // avoid the UDC wait time when reboot is caused by the watchdog   
  while ( (!tud_cdc_connected()) && (counter--)) { sleep_ms(100);  }
    
  setupConfig(); // retrieve the config parameters (crsf baudrate, voltage scale & offset, type of gps, failsafe settings)
  setupListOfFields(); // initialise the list of fields being used
  voltage.begin();
  setupI2c();      // setup I2C
  baro1.begin();   
  gps.setupGps();  //use UART 0 on pins 12 13
  if ( config.protocol == 'C'){
    setupCRSF();  // setup the 2 pio (for TX and RX) and the DMA (for TX) and the irq handler (for Rx); use pin 9 (Rx) and 10(TX)
    setupSbusOutPio();
  } else if (config.protocol == 'S') {
    setupSport();
    setupSbusIn();
  } 
  setupPwm(); 
  printConfig();
  watchdog_enable(500, 1); // require an update once every 500 msec
}

void loop() {
  //printf(".\n");
  watchdog_update();
  getSensors();
  mergeSeveralSensors();
  watchdog_update();
  if ( config.protocol == 'C'){
    fillCRSFFrame();
    handleCrsfRx();
  } else if (config.protocol == 'S') {
    handleSportRxTx();
    handleSbusIn();
  } 
  watchdog_update();
  fillSbusFrame();
  updatePWM();
  handleUSBCmd();
  tud_task();
}

int main(){
  setup();
  while(1) loop();
}