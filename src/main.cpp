#include "pico/stdlib.h"
#include "stdio.h"
#include "config.h"
#include "hardware/i2c.h"
#include "MS5611.h"
#include "SPL06.h"
#include "BMP280.h"
#include <vario.h>
#include <voltage.h>
#include <gps.h>
#include <tusb.h>
#include <crsf_in.h>
#include <crsf_out.h>
#include <param.h>
#include "hardware/pio.h"
#include "sbus_out_pwm.h"
#include "sbus_in.h"
#include "hardware/watchdog.h"
#include "sport.h"
#include "jeti.h"
#include "hott.h"
#include "mpx.h"
#include "param.h"
#include "tools.h"
#include "ws2812.h"
#include "rpm.h"
#include "EMFButton.h"
#include "ads1115.h"
#include "mpu.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "ds18b20.h"


// to do : add current and rpm telemetry fields to jeti protocol
//         support ex bus jeti protocol on top of ex jeti protocol
//         support Frsky Fport on top of sbus+sport protocol
//         add switching 8 gpio from one channel
//         cleanup code for MP6050 (select one algo from the 3, keeping averaging of accZ, avoid movind data from var to var)
//         try to detect MS5611 and other I2C testing the different I2C addresses
//         in readme add the wiring of GPS and of other I2C devices+ comments about MP6050
//         if ds18b20 would be supported, then change the code in order to avoid long waiting time that should block other tasks.
//         reactivate boot button and test if it works for failsafe setting (it blocks core1 and so it is perhaps an issue)
//         stop core1 when there is no I2C activity while saving the config (to avoid I2C conflict)


// Look at file in folder "doc" for more details
//
// So pio 0 sm0 is used for CRSF Tx  or for Sport TX or JETI TX or HOTT TX or MPX TX
//        0   1                         for Sport Rx            or HOTT RX or MPX RX
//        0   2            sbus out             

//        1   0 is used for gps Tx  (was used for one pwm)        
//        1   1 is used for gps Rx  (was used for one pwm)
//        1   3 is used for RGB led
// So UART0 is used for Secondary crsf of Sbus in ( was GPS before)
//    UART1 is used for primary crsf in of SBUS IN (was only Sbus in before)
 
// Pin that can be used are:
// C1 = 0/15  ... C16 = 0/15
// GPS_TX = 0/29
// GPS_RX = 0/29
// PRI = 5 ,9, 21 ,25  (UART1)
// SEC = 1, 13 , 17 ,29 (UART0) 
// SBUS_OUT = 0/29
// TLM = 0/29
// VOLT1= 26/29 ... VOLT4 = 26/29
// SDA = 2, 6, 10, 14, 18, 22, 26  (I2C1)
// SCL = 3, 7, 11, 15, 19, 23, 27  (I2C1)
// RPM = 0/29 
// LED = 16


VOLTAGE voltage ;    // class to handle voltages

MS5611 baro1( (uint8_t) 0x77  );    // class to handle MS5611; adress = 0x77 or 0x76
SPL06 baro2( (uint8_t) 0x76  );    // class to handle SPL06; adress = 0x77 or 0x76
BMP280 baro3( (uint8_t) 0x76) ;    // class to handle BMP280; adress = 0x77 or 0x76

ADS1115 adc1( I2C_ADS_Add1 , 0) ;     // class to handle first ads1115 (adr pin connected to grnd)
ADS1115 adc2( I2C_ADS_Add2 , 1) ;     // class to handle second ads1115 (adr pin connected to vdd)

VARIO vario1;

//queue_t gpsQueue ; // queue is used to transfer the data from the uart0 used by GPS
GPS gps;

// objet to manage the mpu6050
MPU mpu(1);

// CRSF is managed with 2 pio and not with the internal uart1 in order to freely select the pins

EMFButton btn (3, 0); // button object will be associated to the boot button of rp2040; requires a special function to get the state (see tool.cpp)
                       // parameters are not used with RP2040 boot button 
extern uint32_t lastRcChannels;
extern CONFIG config;
bool configIsValid = true;
bool configIsValidPrev = true;
bool blinking = true ;
uint8_t ledState = STATE_NO_SIGNAL;
uint8_t prevLedState = STATE_NO_SIGNAL;

uint32_t lastBlinkMillis;

queue_t qSensorData;       // send one sensor data to core0; when type=0XFF, it means a command then data= the command (e.g.0XFFFFFFFF = save config)
queue_t qSendCmdToCore1;
bool core1SetupDone = false;
void core1_main(); // prototype of core 1 main function

extern field fields[];  // list of all telemetry fields and parameters used by Sport


void setupI2c(){
    if ( config.pinScl == 255 || config.pinSda == 255) return; // skip if pins are not defined
    // send 10 SCL clock to force sensor to release sda
    /*
    gpio_init(config.pinSda);
    gpio_init(config.pinScl);
    gpio_set_dir(config.pinSda, GPIO_IN);
    gpio_set_dir(config.pinScl, GPIO_OUT);
    gpio_pull_up(config.pinSda);
    gpio_pull_up(config.pinScl);
    sleep_us(10);
    while ( gpio_get(config.pinSda) == 0) {;
        
        for (uint8_t i=0; i<9; i++){
            gpio_put(config.pinScl, 1);
            sleep_us(10);
            gpio_put(config.pinScl, 0);
            sleep_us(10);
            printf("trying to unlock I2C\n");
        }
    }    
    gpio_put(config.pinScl, 1);
    gpio_set_dir(config.pinSda, GPIO_OUT);
    gpio_put(config.pinScl, 0);
    sleep_us(10);
    gpio_put(config.pinSda, 0);
    sleep_us(10);
    gpio_put(config.pinScl, 1);
    sleep_us(10);
    gpio_put(config.pinSda, 1);
    sleep_us(10);
    gpio_set_dir(config.pinSda, GPIO_IN);
    if ( gpio_get(config.pinSda) == 0) printf("I2C still locked\n");    
    */
    // initialize I2C     
    i2c_init( i2c1, 400 * 1000);
    gpio_set_function(config.pinSda, GPIO_FUNC_I2C);
    gpio_set_function(config.pinScl, GPIO_FUNC_I2C);
    gpio_pull_up(config.pinSda);
    gpio_pull_up(config.pinScl); 
}

void setupSensors(){     // this runs on core1!!!!!!!!
      voltage.begin();      
      setupI2c();      // setup I2C
      baro1.begin();  // check MS5611; when ok, baro1.baroInstalled  = true
      baro2.begin();  // check SPL06;  when ok, baro2.baroInstalled  = true
      baro3.begin(); // check BMP280;  when ok, baro3.baroInstalled  = true
      #ifdef USE_ADS1115
      adc1.begin() ; 
      adc2.begin() ;
      #endif 
      mpu.begin(); 
    //blinkRgb(0,10,0);
      gps.setupGps();  //use a Pio
      #ifdef USEDS18B20
      ds18b20Setup(); 
      #endif
      core1SetupDone = true;
}

void getSensors(void){      // this runs on core1 !!!!!!!!!!!!
  voltage.getVoltages();
  if ( baro1.baroInstalled){
    if ( baro1.getAltitude() == 0) { // if an altitude is calculated
      vario1.calculateAltVspeed(baro1.altitude , baro1.altIntervalMicros ); // Then calculate Vspeed ... 
    }
  } else if ( baro2.baroInstalled){
    if ( baro2.getAltitude() == 0) { // if an altitude is calculated
      vario1.calculateAltVspeed(baro2.altitude , baro2.altIntervalMicros); // Then calculate Vspeed ... 
    }
  } else if (baro3.baroInstalled) {
    if ( baro3.getAltitude() == 0) { // if an altitude is calculated
      vario1.calculateAltVspeed(baro3.altitude , baro3.altIntervalMicros); // Then calculate Vspeed ... 
    }
  }
  #ifdef USE_ADS1115
  adc1.readSensor(); 
  adc2.readSensor();
  #endif 
  mpu.getAccZWorld();  
  gps.readGps();
  readRpm();
  #ifdef USE_DS18B20
  ds18b20Read(); 
  #endif
}

void mergeSeveralSensors(void){
}

void setColorState(){    // set the colors based on the RF link
    lastBlinkMillis = millis(); // reset the timestamp for blinking
    switch (ledState) {
        case STATE_OK:
            setRgbColorOn(0, 10, 0); //green
            break;
        case STATE_PARTLY_OK:
            setRgbColorOn(10, 5, 0); //yellow
            break;
        case STATE_FAILSAFE:
            setRgbColorOn(0, 0, 10); //blue
            break;
        default:
            setRgbColorOn(10, 0, 0); //red
            break;     
    }
}

enum bootButtonStates_t {NOT_ARMED, ARMED, SAVED};
bootButtonStates_t bootButtonState =  NOT_ARMED;

void handleBootButton(){ 
    // check boot button; after double click, change LED to fix blue and next HOLD within 5 sec save the current channnels as Failsafe values
    static uint32_t bootArmedMillis = 0;
    btn.tick();
    if ( ( btn.hasClicks() == 2) && ( (millis() - lastRcChannels ) < 100 ) ) { // double click + a recent frame exist
        bootArmedMillis = millis();
        bootButtonState =  ARMED;
        setRgbColorOn(0, 0, 10); //blue
        //printf("armed\n");
    } else if (bootButtonState == ARMED) {
        if (btn.hasOnlyHeld() && ( (millis() - lastRcChannels ) < 100 )) {     // saved when long hold + recent frame exist
            bootButtonState =  SAVED;
            setRgbColorOn(5, 5, 5);
            bootArmedMillis = millis();
            cpyChannelsAndSaveConfig();   // copy the channels values and save them into the config.
            //printf("saving failsafe\n");
        } else if ( (millis() - bootArmedMillis) > 5000) {
            bootButtonState =  NOT_ARMED;  // reset if no hold withing the 5 sec
            setColorState();               // restore colors based on RF link
            //printf("loosing armed\n");
        }
    } else if ((bootButtonState == SAVED) && ((millis() - bootArmedMillis) > 2000) ){
        bootButtonState =  NOT_ARMED;  // reset after 2 sec
        setColorState();               // restore colors based on RF link 
        //printf("done\n");
    }
}

void setup() {
  stdio_init_all();
  setupLed();
  setRgbColorOn(10,0,10); // start with 2 color
  #ifdef DEBUG
  uint16_t counter = 10;                      // after an upload, watchdog_cause_reboot is true.
  //if ( watchdog_caused_reboot() ) counter = 0; // avoid the UDC wait time when reboot is caused by the watchdog   
  while ( (!tud_cdc_connected()) && (counter--)) { 
  //while ( (!tud_cdc_connected()) ) { 
    sleep_ms(200);
    toggleRgb();
    }
    sleep_ms(2000);
  #endif
  
  if (watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
    } else {
        printf("Clean boot\n");
        sleep_ms(1000); // wait that GPS is initialized
    }
  setRgbColorOn(0,0,10);  // switch to blue during the setup of different sensors/pio/uart
  
  setupConfig(); // retrieve the config parameters (crsf baudrate, voltage scale & offset, type of gps, failsafe settings)  
  if (configIsValid){ // continue with setup only if config is valid
      setupListOfFields(); // initialise the list of fields being used
      queue_init(&qSensorData, sizeof(queue_entry_t) , 50) ; // max 50 groups of 5 bytes.  create queue to get data from core1
      queue_init(&qSendCmdToCore1, 1, 10); 
      multicore_launch_core1(core1_main);// start core1 and so start I2C sensor discovery
      uint32_t setup1StartUs = micros();  
      while (! core1SetupDone){
        sleep_us(100);
        if ((micros() - setup1StartUs) > 2000) {
            printf("Attention: setup on core 1 did not ended within timeout\n");
            continue;
        }
      }
      //rintf("Setup1 takes %d usec\n",micros() - setup1StartUs);
      if ( config.protocol == 'C'){
        setupCrsfIn();  // setup one/two uart and the irq handler (for primary Rx) 
        setupCrsf2In();  // setup one/two uart and the irq handler (for secondary Rx) 
        setupCrsfOut(); //  setup 1 pio/sm (for TX ) and the DMA (for TX)   
      } else if (config.protocol == 'S') {
        setupSbusIn();
        setupSbus2In();
        setupSport();
      } else if (config.protocol == 'J') {
        setupSbusIn();
        setupSbus2In();
        setupJeti();
      } else if (config.protocol == 'H') {
        setupSbusIn();
        setupSbus2In();
        setupHott();
      } else if (config.protocol == 'M') {
        setupSbusIn();
        setupSbus2In();
        setupMpx();
      }
      if (config.pinSbusOut != 255) { // configure 1 pio/sm for SBUS out (only if Sbus out is activated in config).
          setupSbusOutPio();
        }
      setupPwm();
      setupRpm(); // this function perform the setup of pio Rpm
      watchdog_enable(3500, 0); // require an update once every 500 msec
  } 
  
  printConfig(); // config is not valid
  setRgbColorOn(10,0,0); // set color on red (= no signal)
  
}

void getSensorsFromCore1(){
    queue_entry_t entry;
    //printf("qlevel= %d\n",queue_get_level(&qSensorData));
    
    while( !queue_is_empty(&qSensorData)){
        if ( queue_try_remove(&qSensorData,&entry)){
            if (entry.type >= NUMBER_MAX_IDX) {
                if (entry.type == 0XFF && entry.data == 0XFFFFFFFF) {  // this is a command to save the config.
                    watchdog_enable(15000,false);
                    sleep_ms(1000);
                    printf("Calibration has been done\n");
                    printConfigOffsets();
                    printf("\nConfig will be saved\n\n");
                    sleep_ms(1000);
                    saveConfig();
                    printf("config has been saved\n");  
                    printf("Device will reboot\n\n");
                    watchdog_enable(1500,false);
                    sleep_ms(1000);
                    watchdog_reboot(0, 0, 100); // this force a reboot!!!!!!!!!!
                } else {
                    printf("error : invalid type of sensor = %d\n", entry.type);
                }    
            } else {
                fields[entry.type].value = entry.data;
                fields[entry.type].available = true ;
                //printf("t=%d  %10.0f\n",entry.type ,  (float)entry.data);
            }    
        }
    }
}

void loop() {
  //debugBootButton();
  if (configIsValid){
      getSensorsFromCore1();
      mergeSeveralSensors();
      watchdog_update();
      if ( config.protocol == 'C'){
        fillCRSFFrame();
        handleCrsfIn();
        handleCrsf2In();
        fillSbusFrame();
      } else if (config.protocol == 'S') {
        handleSportRxTx();
        handleSbusIn();
        handleSbus2In();
        fillSbusFrame();
      } else if (config.protocol == 'J') {
        handleJetiTx();
        handleSbusIn();
        handleSbus2In();
        fillSbusFrame();
      } else if (config.protocol == 'H') {
        handleHottRxTx();
        handleSbusIn();
        handleSbus2In();
        fillSbusFrame();
      } else if (config.protocol == 'M') {
        handleMpxRxTx();
        handleSbusIn();
        handleSbus2In();
        fillSbusFrame();
      } 
      watchdog_update();
      updatePWM();
            //updatePioPwm();
  }
  watchdog_update();
  //if (tud_cdc_connected()) {
  handleUSBCmd();  // process the commands received from the USB
  tud_task();      // I suppose that this function has to be called periodicaly
  //}  
  if ( configIsValidPrev != configIsValid) {
    configIsValidPrev = configIsValid;
    if (configIsValid) {
        blinking = true; // setRgbColorOn(0,10,0); // red , green , blue
    } else {
        blinking = false; // setRgbColorOn(10,0,0);
        setRgbOn();  
    }
  }

//  handleBootButton(); // check boot button; after double click, change LED to fix blue and next HOLD within 5 sec save the current channnels as Failsafe values
  if (( bootButtonState == ARMED) || ( bootButtonState == SAVED)){
    //setRgbColorOn(0, 0, 10); //blue
  } else if ( ledState != prevLedState){
    //printf(" %d\n ",ledState);
    prevLedState = ledState;
    setColorState();     
  } else if ( blinking && (( millis() - lastBlinkMillis) > 300 ) ){
    toggleRgb();
    lastBlinkMillis = millis();
  }
  //if (get_bootsel_button()) {
  //  printf("p\n");
  //} 
  //enlapsedTime(0);
}

// initialisation of core 1 that capture the sensor data
void setup1(){
    multicore_lockout_victim_init();
    setupSensors();    
}
// main loop on core 1 in order to read the sensors and send the data to core0
void loop1(){
    uint8_t qCmd;
    getSensors(); // get sensor
    if ( ! queue_is_empty(&qSendCmdToCore1)){
        queue_try_remove(&qSendCmdToCore1, &qCmd);
        if ( qCmd == 0X01) { // 0X01 is the code to request a calibration
            mpu.calibrationExecute();
        }
    }
}

void core1_main(){
    setup1();
    while(1) loop1();
}

int main(){
  setup();
  while(1) loop();
}