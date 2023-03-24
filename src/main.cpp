#include "pico/stdlib.h"
#include "stdio.h"
#include "config.h"
#include "hardware/i2c.h"
#include "MS5611.h"
#include "SPL06.h"
#include "BMP280.h"
#include "ms4525.h"
#include "sdp3x.h"
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
#include "tools.h"
#include "sport.h"
#include "jeti.h"
#include "hott.h"
#include "mpx.h"
#include "ibus.h"
#include "sbus2_tlm.h"
#include "fbus.h"
#include "srxl2.h"
//#include "param.h"

#include "ws2812.h"
#include "rpm.h"
#include "EMFButton.h"
#include "ads1115.h"
#include "mpu.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "ds18b20.h"
#include "hardware/timer.h"
// to do : add current and rpm telemetry fields to jeti protocol
//         support ex bus jeti protocol on top of ex jeti protocol
//         support Frsky Fbus on top of sbus+sport protocol
//         add switching 8 gpio from one channel
//         cleanup code for MP6050 (select one algo from the 3, keeping averaging of accZ, avoid movind data from var to var)
//         try to detect MS5611 and other I2C testing the different I2C addresses
//         if ds18b20 would be supported, then change the code in order to avoid long waiting time that should block other tasks.
//         reactivate boot button and test if it works for failsafe setting (it blocks core1 and so it is perhaps an issue)
//         stop core1 when there is no I2C activity while saving the config (to avoid I2C conflict)
//         add airspeed field and compensated Vspeed to all protocols (currently it is only in sport)
//         add spektrum protocol (read the bus already in set up, change baudrate, fill all fields in different frames)
//         add command to force to prefill fields[] with some values

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

MS4525 ms4525 ( (uint8_t) MS4525_ADDRESS ) ; // 0x28 is the default I2C adress of a 4525DO sensor)
SDP3X sdp3x( (uint8_t) SDPXX_ADDRESS) ;      // 0X21 is the default I2C address of asdp31,... sensor (diffrent for sdp8xx)

VARIO vario1;

//queue_t gpsQueue ; // queue is used to transfer the data from the uart0 used by GPS
GPS gps;

// objet to manage the mpu6050
MPU mpu(1);

field fields[NUMBER_MAX_IDX];  // list of all telemetry fields and parameters that can be measured (not only used by Sport)


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
volatile bool core1SetupDone = false;
void core1_main(); // prototype of core 1 main function

uint8_t forcedFields = 0; // use to debug a protocol; force the values when = 'P' (positive) or 'N' (negative)

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
      
      //sleep_ms(3000);
      //printf("start core1 setup\n");
      //startTimerUs(0); //xxxxx to debug only 
      voltage.begin();      
      //printf("voltage done\n");
      setupI2c();      // setup I2C
      //printf("i2C done\n");
      baro1.begin();  // check MS5611; when ok, baro1.baroInstalled  = true
      //printf("baro1 done\n");
      if (! baro1.baroInstalled) {
        baro2.begin();  // check SPL06;  when ok, baro2.baroInstalled  = true
        //printf("baro2 done\n");
      }
      if (! ( baro1.baroInstalled or baro2.baroInstalled)) {
        baro3.begin(); // check BMP280;  when ok, baro3.baroInstalled  = true
        //printf("baro3 done\n");
      }
      adc1.begin() ; 
      //printf("adc1 done\n");
      adc2.begin() ;
      //printf("adc2 done\n");
      mpu.begin(); 
      //printf("mpu done\n");
    //blinkRgb(0,10,0);
      gps.setupGps();  //use a Pio
      //printf("gps done\n");
      ms4525.begin();
      if (! ms4525.airspeedInstalled) {
        sdp3x.begin();
      }
      #ifdef USEDS18B20
      ds18b20Setup(); 
      #endif
      setupRpm(); // this function perform the setup of pio Rpm
      //printf("rpm done\n");
      
      core1SetupDone = true;
      //printf("end core1 setup\n") ;    
      //getTimerUs(0);   // xxxxx
}

void getSensors(void){      // this runs on core1 !!!!!!!!!!!!
  voltage.getVoltages();
  if ( baro1.baroInstalled){
    if ( baro1.getAltitude() == 0) { // if an altitude is calculated
      vario1.calculateAltVspeed(baro1.altitudeCm , baro1.altIntervalMicros ); // Then calculate Vspeed ...   
    }
  } else if ( baro2.baroInstalled){
    if ( baro2.getAltitude() == 0) { // if an altitude is calculated
      vario1.calculateAltVspeed(baro2.altitudeCm , baro2.altIntervalMicros); // Then calculate Vspeed ... 
    }
  } else if (baro3.baroInstalled) {
    if ( baro3.getAltitude() == 0) { // if an altitude is calculated
      vario1.calculateAltVspeed(baro3.altitudeCm , baro3.altIntervalMicros); // Then calculate Vspeed ... 
    }
  }
  adc1.readSensor(); 
  adc2.readSensor();
  mpu.getAccZWorld();  
  gps.readGps();
  if (ms4525.airspeedInstalled){
    ms4525.getDifPressure();
    calculateAirspeed( );
    vario1.calculateVspeedDte();
  }
  if (sdp3x.airspeedInstalled){
    sdp3x.getDifPressure();
    calculateAirspeed( );
    vario1.calculateVspeedDte();
  } 
  readRpm();
  #ifdef USE_DS18B20
  ds18b20Read(); 
  #endif
}

void mergeSeveralSensors(void){
}

void setColorState(){    // set the colors based on the RF link
    lastBlinkMillis = millisRp(); // reset the timestamp for blinking
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
    if ( ( btn.hasClicks() == 2) && ( (millisRp() - lastRcChannels ) < 100 ) ) { // double click + a recent frame exist
        bootArmedMillis = millisRp();
        bootButtonState =  ARMED;
        setRgbColorOn(0, 0, 10); //blue
        //printf("armed\n");
    } else if (bootButtonState == ARMED) {
        if (btn.hasOnlyHeld() && ( (millisRp() - lastRcChannels ) < 100 )) {     // saved when long hold + recent frame exist
            bootButtonState =  SAVED;
            setRgbColorOn(5, 5, 5);
            bootArmedMillis = millisRp();
            cpyChannelsAndSaveConfig();   // copy the channels values and save them into the config.
            //printf("saving failsafe\n");
        } else if ( (millisRp() - bootArmedMillis) > 5000) {
            bootButtonState =  NOT_ARMED;  // reset if no hold withing the 5 sec
            setColorState();               // restore colors based on RF link
            //printf("loosing armed\n");
        }
    } else if ((bootButtonState == SAVED) && ((millisRp() - bootArmedMillis) > 2000) ){
        bootButtonState =  NOT_ARMED;  // reset after 2 sec
        setColorState();               // restore colors based on RF link 
        //printf("done\n");
    }
}

volatile uint8_t testU8 = 0;
void test_callback(uint alarmNum){
    testU8++;
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
  // test
  //int32_t testValue = -10;
  //printf("rounding -10 = %d\n" , ( int_round(testValue , 100) ) +500);
  //testValue = -60;
  //printf("rounding -60 = %d\n" , ( int_round(testValue , 100) ) +500);
  //testValue = -200;
  //printf("rounding -200 = %d\n" , ( int_round(testValue , 100) ) +500);
  //testValue = +10;
  //printf("rounding +10 = %d\n" , ( int_round(testValue , 100) ) +500);
  //testValue = +60;
  //printf("rounding +60 = %d\n" , ( int_round(testValue , 100) ) +500);
  //uint8_t readBuffer[2] = {0XFF, 0XFE}; 
  //printf("test %f\n",(float) ((int16_t) (readBuffer[0] << 8 | readBuffer[1] & 0X00FF)));
  //int16_t test = 0X7B03;
  //int32_t testi32 = (int32_t) test;
  //uint8_t testFirst = * (&test);
  //printf("test %X %X %X\n",  test , testi32 , testFirst) ; 
  //if (hardware_alarm_is_claimed(0)) printf("alarm 0 is used\n");
  //if (hardware_alarm_is_claimed(1)) printf("alarm 1 is used\n");
  //if (hardware_alarm_is_claimed(2)) printf("alarm 2 is used\n");
  //if (hardware_alarm_is_claimed(3)) printf("alarm 3 is used\n");
  //hardware_alarm_set_callback(2 , test_callback);
  //hardware_alarm_set_target(2 , time_us_64()+200);
  //hardware_alarm_set_target(2 , time_us_64()+700);
  //sleep_us(100);
  //printf("testU8= %d\n", (int) testU8);
  //sleep_us(200);
  //printf("testU8= %d\n", (int) testU8);
  //sleep_us(700);
  //printf("testU8= %d\n", (int) testU8);
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
      for (uint8_t i = 0 ;  i< NUMBER_MAX_IDX ; i++){ // initialise the list of fields being used 
        fields[i].value= 0;
        fields[i].available= false;
      }  
      queue_init(&qSensorData, sizeof(queue_entry_t) , 50) ; // max 50 groups of 5 bytes.  create queue to get data from core1
      queue_init(&qSendCmdToCore1, 1, 10); // queue to send a cmd to core 1 (e.g. to perform a calibration of mp6050)
      #ifdef DEBUG
      sleep_ms(2000); // xxxxxxxxxxx to remove after debug
      #endif
      multicore_launch_core1(core1_main);// start core1 and so start I2C sensor discovery
      uint32_t setup1StartUs = microsRp();  
      while ( core1SetupDone == false) {
            if ((microsRp() - setup1StartUs) > (2 * 1000000)) {
                sleep_ms(3000);
                printf("Attention: setup on core 1 did not ended within timeout\n");
                break   ;
            }
      }
      uint32_t core1SetupUs = microsRp() - setup1StartUs ; 
      printf("Setup1 takes %d usec\n",(int) core1SetupUs) ;
      if ( config.protocol == 'C'){   //crsf
        setupCrsfIn();  // setup one/two uart and the irq handler (for primary Rx) 
        setupCrsf2In();  // setup one/two uart and the irq handler (for secondary Rx) 
        setupCrsfOut(); //  setup 1 pio/sm (for TX ) and the DMA (for TX)   
      } else if (config.protocol == 'S') { // sport
        setupSbusIn();
        setupSbus2In();
        setupSport();
      } else if (config.protocol == 'J') {   //jeti
        setupSbusIn();
        setupSbus2In();
        setupJeti();
      } else if (config.protocol == 'H') {   //hott
        setupSbusIn();
        setupSbus2In();
        setupHott();
      } else if (config.protocol == 'M') {   //Mpx
        setupSbusIn();
        setupSbus2In();
        setupMpx();
      } else if (config.protocol == 'I') {  // ibus
        setupSbusIn();
        setupSbus2In();
        setupIbus();
      } else if (config.protocol == '2')  {  // Sbus2 futaba
        setupSbusIn();
        setupSbus2In();
        setupSbus2Tlm();
      } else if (config.protocol == 'F') {   // fBus frsky
        setupFbus();
        setupSbus2In();
        //setupSbus2Tlm();
      } else if (config.protocol == 'L') {   // srxl2 Spektrum
        setupSrxl2();
        //setupSbus2In(); // to do add a second input
        //setupSbus2Tlm();
      }
      if (config.pinSbusOut != 255) { // configure 1 pio/sm for SBUS out (only if Sbus out is activated in config).
          setupSbusOutPio();
        }
      setupPwm();
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
                    printf("\nCalibration has been done");
                    printConfigOffsets();
                    printf("\nConfig will be saved\n\n");
                    sleep_ms(1000);
                    saveConfig();
                    printf("Config has been saved\n");  
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
                fields[entry.type].onceAvailable = true ;
                //printf("t=%d  %10.0f\n",entry.type ,  (float)entry.data);
            }    
        }
    }
    if ((forcedFields == 1) || (forcedFields == 2)) fillFields(forcedFields); // force dummy vallues for debuging a protocol
}

void loop() {
  //debugBootButton();
  if (configIsValid){
      getSensorsFromCore1();
      mergeSeveralSensors();
      watchdog_update();
      if ( config.protocol == 'C'){   //elrs/crsf
        fillCRSFFrame();
        handleCrsfIn();
        handleCrsf2In();
        fillSbusFrame();
      } else if (config.protocol == 'S') {  // sport
        handleSportRxTx();
        handleSbusIn();
        handleSbus2In();
        fillSbusFrame();
      } else if (config.protocol == 'J') {  //jeti
        handleJetiTx();
        handleSbusIn();
        handleSbus2In();
        fillSbusFrame();
      } else if (config.protocol == 'H') {  //Hott
        handleHottRxTx();
        handleSbusIn();
        handleSbus2In();
        fillSbusFrame();
      } else if (config.protocol == 'M') {  // multiplex
        handleMpxRxTx();
        handleSbusIn();
        handleSbus2In();
        fillSbusFrame();
      } else if (config.protocol == 'I') {  // Ibus flysky
        handleIbusRxTx();
        handleSbusIn();           //???????????is this OK
        handleSbus2In();
        fillSbusFrame();
      } else if (config.protocol == '2') { // Sbus2 Futaba
        handleSbusIn();
        handleSbus2In();
        fillSbusFrame();
      } else if (config.protocol == 'F') {  // Fbus frsky
        handleFbusRxTx();
        handleSbus2In();
        fillSbusFrame();
      } else if (config.protocol == 'L') {  // SRXL2 Spektrum
        handleSrxl2RxTx();
        //handleSbus2In();  // to do processa second inpunt
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
  } else if ( blinking && (( millisRp() - lastBlinkMillis) > 300 ) ){
    toggleRgb();
    lastBlinkMillis = millisRp();
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