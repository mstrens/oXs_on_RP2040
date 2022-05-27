#include "pico/stdlib.h"
#include "stdio.h"
#include "pico/util/queue.h"
#include <config.h>
#include "hardware/i2c.h"
#include "MS5611.h"
#include "SPL06.h"
#include <vario.h>
#include <voltage.h>
#include <gps.h>
#include <tusb.h>
#include <crsf.h>
#include <param.h>
#include "hardware/pio.h"
#include "sbus_out_pwm.h"
#include "sbus_in.h"
#include "hardware/watchdog.h"
#include "sport.h"
#include "jeti.h"
#include "param.h"
#include "tools.h"
#include "ws2812.h"
#include "rpm.h"

// to do : add current and rpm telemetry fields to jeti protocol
//         support ex bus jeti protocol on top of ex jeti protocol
//         support Frsky Fport on top of sbus+sport protocol  

// Look at file config.h for more details
//
// This project can be interfaced with 
//    - a ELRS (it uses CRSF protocol) receiver for telemetry and to generate SBus and PWM signals
//    - or a FRSKY receiver for telemetry (Sport) and to generate PWM signals based on the Frsky Sbus
//    - or a Jeti receiver for telemetry only (connected to Ex pin of receiver)
// A Sbus signal can be generated (only for CRSF) and uses pin GPIO 0 (PIO TX), pio0, sm 2 and one dma
// PWM signals are generated on:
//     - pins GPIO1 up to GPIO8 (without use of a pio)
//     - pin GPIO11 (with pio1 sm0)
//     - pin GPIO0 (with  pio1 sm1) when Sbus out is not activated (see config below)
//
// CRSF uses GPIO 9 (pio RX on pio0 sm 1 + one IRQ PIO0_IRQ_0 + one queue)  and GPIO 10 (pio TX on pio0 sm 0 + one dma)
// Sbus as input (for Frsky) uses pin GPIO 9 and UART1 RX (easier for 8N2) + one IRQ UART1_IRQ + one queue.
// Sport (for Frsky) uses pin GPIO 10 and : 
//      - for Rx pio0 + sm1 + one IRQ PIO0_IRQ_0 + one queue (sportRxQueue)
//      - for Tx pio0 + sm0 (Tx) + one dma
// GPS uses gpio 12 (UART0-TX) and gpio 13 (UART0-RX) + one IRQ UART0_IRQ + one queue (gpsQueue)
// I2C uses pins:
//      - GPIO 14 = PICO_I2C1_SDA_PIN
//      - GPIO 15 = PICO_I2C1_SCL_PIN  
// Analog read uses GPIO pins 26 up to 28 (so 3 max)
// RPM uses GPIO pin 29

// So pio 0 sm0 is used for CRSF Tx  or for Sport TX
//        0   1             CRSF Rx  or for Sport Rx
//        0   2            sbus out             
//        1   0 is used for one PWM          
//        1   1 is used for one PWM
//        1   2 is used for RPM
//        1   3 is used for RGB led

 

//#define DEBUG

VOLTAGE voltage ;    // class to handle voltages

MS5611 baro1( (uint8_t) 0x77  );    // class to handle MS5611; adress = 0x77 or 0x76
SPL06 baro2( (uint8_t) 0x76  );    // class to handle MS5611; adress = 0x77 or 0x76

VARIO vario1;

queue_t gpsQueue ; // queue is used to transfer the data from the uart0 used by GPS
GPS gps;

// CRSF is managed with 2 pio and not with the internal uart1 in order to freely select the pins

extern CONFIG config;

#define PICO_I2C1_SDA_PIN 14  
#define PICO_I2C1_SCL_PIN 15  

uint32_t lastBlinkMillis;

void setupI2c(){
    i2c_init( i2c1, 400 * 1000);
    gpio_set_function(PICO_I2C1_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_I2C1_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_I2C1_SDA_PIN);
    gpio_pull_up(PICO_I2C1_SCL_PIN); 
}

void getSensors(void){
  voltage.getVoltages();
  if ( baro1.baroInstalled){
    if ( baro1.getAltitude() == 0) { // if an altitude is calculated
      vario1.calculateAltVspeed(baro1.altitude , baro1.altIntervalMicros ); // Then calculate Vspeed ... 
    }
  } else if ( baro2.baroInstalled){
    if ( baro2.getAltitude() == 0) { // if an altitude is calculated
      vario1.calculateAltVspeed(baro2.altitude , baro2.altIntervalMicros); // Then calculate Vspeed ... 
    }
  }  
  gps.readGps();
  readRpm();
}

void mergeSeveralSensors(void){
}



void setup() {
  stdio_init_all();
  watchdog_enable(1500, 0); // require an update once every 500 msec
  setupLed();
  setRgbColor(10,0,0);
  #ifdef DEBUG
  uint16_t counter = 15; 
  //if ( watchdog_caused_reboot() ) counter = 0; // avoid the UDC wait time when reboot is caused by the watchdog   
  while ( (!tud_cdc_connected()) && (counter--)) { 
    sleep_ms(100); 
    watchdog_update();
    }
  #endif
  setRgbColor(0,0,10);  
  watchdog_update();
  setupConfig(); // retrieve the config parameters (crsf baudrate, voltage scale & offset, type of gps, failsafe settings)
  watchdog_update();
  setupListOfFields(); // initialise the list of fields being used
  watchdog_update();
  voltage.begin();
  setupI2c();      // setup I2C
  baro1.begin();  // check MS5611; when ok, baro1.baroInstalled  = true
  watchdog_update();
  baro2.begin();  // check SPL06;  when ok, baro2.baroInstalled  = true
  watchdog_update();
  /*
  if ( baro1.baroInstalled){
    printf("MS5611 detected\n");
  } else {
    printf("MS5611 not detected\n");
  }
  if ( baro2.baroInstalled){
    printf("SPL06 detected\n");
  } else {
    printf("SPL06 detected\n");
  }
  */
  gps.setupGps();  //use UART 0 on pins 12 13
  watchdog_update();
  if ( config.protocol == 'C'){
    setupCRSF();  // setup the 2 pio (for TX and RX) and the DMA (for TX) and the irq handler (for Rx); use pin 9 (Rx) and 10(TX)
    if (config.gpio0 == 0) { // configure the pio for SBUS only if Sbus is activated in config.
      setupSbusOutPio();
    }  
  } else if (config.protocol == 'S') {
    setupSport();
    setupSbusIn();
  } else if (config.protocol == 'J') {
    setupJeti();
  }
  watchdog_update();
  setupPwm();
  setupPioPwm();
  watchdog_update();
  setupRpm(); // this function perform the setup of pio Rpm
  printConfig();
  watchdog_update();
  setRgbColor(0,10,0);
  watchdog_enable(500, 0); // require an update once every 500 msec
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
  } else if (config.protocol == 'J') {
    handleJetiTx();
  } 
  watchdog_update();
  if ((config.gpio0 == 0) && ( config.protocol == 'C') ){ // generate SBUS only if Sbus is activated in config and protocol = CRSF.
    fillSbusFrame();
  }
  updatePWM();
  updatePioPwm();
  handleUSBCmd();
  tud_task();
  if (( millis() - lastBlinkMillis) > 300 ){
    toggleRgb();
    lastBlinkMillis = millis();
  }
}

int main(){
  setup();
  while(1) loop();
}