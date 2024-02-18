#include "pico/stdlib.h"
#include "stdio.h"
#include "config.h"
#include "hardware/i2c.h"
#include "MS5611.h"
#include "SPL06.h"
#include "BMP280.h"
#include "ms4525.h"
#include "sdp3x.h"
#include "XGZP6897D.h"
#include "vario.h"
#include "voltage.h"
#include "gps.h"
#include "tusb.h"
#include "crsf_in.h"
#include "crsf_out.h"
#include "param.h"
#include "hardware/pio.h"
#include "sbus_out_pwm.h"
#include "sbus_in.h"
#include "hardware/watchdog.h"
#include "tools.h"
#include "sport.h"
#include "jeti.h"
#include "exbus.h"
#include "hott.h"
#include "mpx.h"
#include "ibus.h"
#include "ibus_in.h"
#include "sbus2_tlm.h"
#include "fbus.h"
#include "srxl2.h"
#include "sequencer.h"
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
#include "logger.h"
#include "esc.h"
#include "gyro.h"
#include "lora.h"

// to do : add rpm, temp telemetry fields to jeti protocol
//         try to detect MS5611 and other I2C testing the different I2C addresses
//         if ds18b20 would be supported, then change the code in order to avoid long waiting time that should block other tasks.
//         stop core1 when there is no I2C activity while saving the config (to avoid I2C conflict)
//         add airspeed field and compensated Vspeed to all protocols (currently it is only in sport and some other)
//         add spektrum protocol (read the bus already in set up, change baudrate, fill all fields in different frames)

//         test 16 rc channel values in log interface 
//         test logger param in config parameters
//         test tlm data in log interface
//         it seems that in ELRS protocol, PWM are not generated since some version.
//         use Rc channels with gyro correction to the signal Sbus out. 
//         in mpu, when we apply offsets for acc and gyro, we should check that we do not exceed the 16 bits (or put the values in 32 bits)

//         Test Lora locator functionality that has been added.

// Look at file in folder "doc" for more details
//
// So pio 0 sm0 is used for CRSF Tx  or for Sport TX or JETI TX or HOTT TX or MPX TX or SRXL2 or Ibus (it uses max 6 bytes for Hott and a dma)
//        0   1                         for Sport Rx            or HOTT RX or MPX RX or SRXL2 or Ibus (it uses 9 bytes, 1 irq0 and no dma)
//        0   2            sbus out                                                            (it uses 4 bytes and one dma)       
//        0   3            esc  Rx                                                             (it uses 9 bytes, 1 irq and no dma)

//        1   0 is used for gps Tx  ; is unclaim when GPS config is done and reused             (it uses 4 bytes no dma)      
//        1   0 is also used for gps Rx                                                         (it uses 9 bytes, 1 irq1 and no dma)

//        1   1  is used for rpm                                                                       (it uses 3 bytes , no Irq, no dma)
//        1   2 is used for logger uart Tx                                                             (it uses 4 bytes, no irq and one dma) 
//        1   3 is used for RGB led                                                                    (it uses 4 bytes, no irq and no dma)     
// So UART0 is used for Secondary crsf in or Sbus in ( was GPS before)
//    UART1 is used for primary crsf in or SBUS IN (was only Sbus in before)
 
// note : GPS setup (on core 1) can end when core 1 is already in main loop; setup of logger (pio/dma) has to wait that gps set up is done 

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

int32_t i2cError = 0;

MS5611 baro1( (uint8_t) 0x77  );    // class to handle MS5611; adress = 0x77 or 0x76
SPL06 baro2( (uint8_t) 0x76  );    // class to handle SPL06; adress = 0x77 or 0x76
BMP280 baro3( (uint8_t) 0x76) ;    // class to handle BMP280; adress = 0x77 or 0x76

ADS1115 adc1( I2C_ADS_Add1 , 0) ;     // class to handle first ads1115 (adr pin connected to grnd)
ADS1115 adc2( I2C_ADS_Add2 , 1) ;     // class to handle second ads1115 (adr pin connected to vdd)

MS4525 ms4525 ( (uint8_t) MS4525_ADDRESS ) ; // 0x28 is the default I2C adress of a 4525DO sensor)
SDP3X sdp3x( (uint8_t) SDPXX_ADDRESS) ;      // 0X21 is the default I2C address of asdp31,... sensor (diffrent for sdp8xx)
XGZP  xgzp( (uint8_t) XGZP_ADDRESS );        // 0x6D is the only one I2C adress

VARIO vario1;

//queue_t gpsQueue ; // queue is used to transfer the data from the uart0 used by GPS
GPS gps;

// objet to manage the mpu6050
MPU mpu(1);

LOGGER logger;

field fields[NUMBER_MAX_IDX];  // list of all telemetry fields and parameters that can be measured (not only used by Sport)

// remapping from sbus value to pwm value
uint16_t fromSbusMin = FROM_SBUS_MIN;
uint16_t toPwmMin = TO_PWM_MIN; 
uint16_t fromSbusMax = FROM_SBUS_MAX;
uint16_t toPwmMax = TO_PWM_MAX; 

// CRSF is managed with 2 pio and not with the internal uart1 in order to freely select the pins

EMFButton btn (3, 0); // button object will be associated to the boot button of rp2040; requires a special function to get the state (see tool.cpp)
                       // parameters are not used with RP2040 boot button 
extern uint32_t lastRcChannels;
extern bool newRcChannelsReceivedForLogger;  // used to know when we have to update the logger data


extern CONFIG config;
bool configIsValid = true;
bool configIsSaved = true ;
bool configIsValidPrev = true;
bool multicoreIsRunning = false;
bool blinking = true ;
uint8_t ledState = STATE_NO_SIGNAL;
uint8_t prevLedState = STATE_NO_SIGNAL;

uint32_t lastBlinkMillis;

extern SEQUENCER seq;
extern struct gyroMixer_t gyroMixer ; // contains the parameters provided by the learning process for each of the 16 Rc channel
extern bool gyroIsInstalled ;
queue_t qSensorData;       // send one sensor data to core0; when type=0XFF, it means a command then data= the command (e.g.0XFFFFFFFF = save config)
queue_t qSendCmdToCore1;
volatile bool core1SetupDone = false;
void core1_main(); // prototype of core 1 main function

uint8_t forcedFields = 0; // use to debug a protocol; force the values when = 'P' (positive) or 'N' (negative)

int32_t cameraPitch;
int32_t cameraRoll;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;

extern bool calibrateImuGyro ; // recalibrate the gyro or not at reset (avoid it after a watchdog reset)

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
    //blinkRgb(0,10,0,500,1000000); blink red, green, blue at 500 msec for 1000 0000 X
      gps.setupGps();  //use a Pio and 1 sm (in fact reuse the same sm for RX after TX)
      //printf("gps done\n");
      ms4525.begin();
      if (! ms4525.airspeedInstalled) {
        sdp3x.begin();
        if (! sdp3x.airspeedInstalled) {
            xgzp.begin();
        }
      }
      #ifdef USEDS18B20
      ds18b20Setup(); 
      #endif
      setupRpm(); // this function perform the setup of pio Rpm
      //printf("rpm done\n");
      
    setupEsc() ; 

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
  if (xgzp.airspeedInstalled){
    xgzp.getDifPressure();
    calculateAirspeed( );
    vario1.calculateVspeedDte();
  }
  

  readRpm();
  handleEsc();
  #ifdef USE_DS18B20
  ds18b20Read(); 
  #endif
}

//void mergeSeveralSensors(void){
//}

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
        case STATE_GYRO_CAL_MIXER_NOT_DONE:
            setRgbColorOn(10, 0, 0); //red
            break;
        case STATE_GYRO_CAL_MIXER_DONE:
            setRgbColorOn(10, 5, 0); //yellow
            break;
        case STATE_GYRO_CAL_LIMIT:
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
  //bool clockChanged; 
  //clockChanged = set_sys_clock_khz(133000, false);
  set_sys_clock_khz(133000, false);
  #ifdef DEBUG
  uint16_t counter = 10;                      // after an upload, watchdog_cause_reboot is true.
  //if ( watchdog_caused_reboot() ) counter = 0; // avoid the UDC wait time when reboot is caused by the watchdog   
  while ( (!tud_cdc_connected()) && (counter--)) { 
  //while ( (!tud_cdc_connected()) ) { 
    sleep_ms(100);
    //toggleRgb();
    }
  sleep_ms(2000);  // in debug mode, wait a little to let USB on PC be able to display all messages
  uint8_t a1[2] = {1, 2};
  int debug = 2;
  debugAX("aa", a1 , 2);
  #endif
  
  if (watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
        calibrateImuGyro = false;
    } else {
        printf("Clean boot\n");
        calibrateImuGyro = true;
        //sleep_ms(1000); // wait that GPS is initialized
    }
  setupConfig(); // retrieve the config parameters (crsf baudrate, voltage scale & offset, type of gps, failsafe settings)  
  setupSequencers(); // retrieve the sequencer parameters (defsMax, stepsMax, defs and steps)
  setupGyroMixer(); 
  checkConfigAndSequencers();     // check if config and sequencers are valid (print error message; configIsValid is set on true or false)
  setupLed();
  setRgbColorOn(0,0,10);  // switch to blue during the setup of different sensors/pio/uart
  if (configIsValid) { // continue with setup only if config is valid 
      for (uint8_t i = 0 ;  i< NUMBER_MAX_IDX ; i++){ // initialise the list of fields being used 
        fields[i].value= 0;
        fields[i].available= false;
        fields[i].onceAvailable = false;
      }  
      queue_init(&qSensorData, sizeof(queue_entry_t) , 50) ; // max 50 groups of 5 bytes.  create queue to get data from core1
      queue_init(&qSendCmdToCore1, 1, 10); // queue to send a cmd to core 1 (e.g. to perform a calibration of mp6050)
      multicore_launch_core1(core1_main);// start core1 and so start I2C sensor discovery
      multicoreIsRunning = true; // to know if multicore is running or not
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
      } else if (config.protocol == 'J') {   //jeti non exbus
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
        setupIbusIn();
        //setupSbus2In();
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
      } else if (config.protocol == 'E') {   // jeti Exbus
        setupExbus();
        //setupSbus2In(); // to do add a second input
        //setupSbus2Tlm();
      }
      if (config.pinSbusOut != 255) { // configure 1 pio/sm for SBUS out (only if Sbus out is activated in config).
          setupSbusOutPio();
        }
      setupPwm();
      if ( config.pinLogger != 255) {
        logger.begin();           // set up the logger
      }
      if ((config.gyroChanControl <= 16) and (mpu.mpuInstalled)) { // when the channel to control the gyro is defined and a mpu.installed
        gyroIsInstalled=true;
      }
      
      watchdog_enable(3500, 0); // require an update once every 500 msec
  } 
  printConfigAndSequencers(); 
  setRgbColorOn(10,0,0); // set color on red (= no signal)
  // to detect end of setup
  //printf("end of set up\n");
  //while (1) {watchdog_update();};

  /*
  if (clockChanged){
    printf("clock is changed to 133mHz\n");
  } else {
    printf("clock is not changed\n");
  }
   
    //checkLedColors(); // this program does not end and so main loop is not called.
  if(gyroIsInstalled){
    printf("gyro is installed\n");
  }  else{ 
    printf("gyro is not installed\n");
  }
  */
}

void getSensorsFromCore1(){
    queue_entry_t entry;
    //printf("qlevel= %d\n",queue_get_level(&qSensorData));

    bool loggerHeaderSent = false;  // used to sent the synchro byte 0X7E only once per function call.
    
    while( !queue_is_empty(&qSensorData)){
        if ( queue_try_remove(&qSensorData,&entry)){
            if (entry.type >= NUMBER_MAX_IDX) {
                if (entry.type == SAVE_CONFIG_ID && entry.data == 0XFFFFFFFF) {  // this is a command to save the config.
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
                } else if (entry.type == CAMERA_PITCH_ID) {
                    cameraPitch = entry.data;
                } else if (entry.type == CAMERA_ROLL_ID) {
                    cameraRoll = entry.data;
                } else if (entry.type == GYRO_X_ID) {  
                    gyroX = (15*gyroX + entry.data) >> 4;  // here some filtering because oXs use a BW=188 for mpu instead of BW=5 for flightstab in gyro setting.
                    //gyroX = entry.data;
                } else if (entry.type == GYRO_Y_ID) { 
                    gyroY = (15*gyroY + entry.data) >>4 ;
                    //gyroY = entry.data; 
                } else if (entry.type == GYRO_Z_ID) { 
                    gyroZ = (15*gyroZ + entry.data) >> 4;
                    //gyroZ = entry.data;
                } else {
                    printf("error : invalid type of sensor = %d\n", entry.type);
                }    
            } else {
                fields[entry.type].value = entry.data;
                fields[entry.type].available = true ;
                if (fields[entry.type].onceAvailable == false){
                    fields[entry.type].onceAvailable = true ;
                    // update sportMaxBandwidth for some protocols
                    if ( (config.protocol == 'S') || (config.protocol == 'F') )  calculateSportMaxBandwidth(); 
                }    
                if (config.pinLogger != 255) { // when logger is on
                    if ( loggerHeaderSent == false) {  // log once per function call the synchro byte 
                        logger.logByteNoStuff(0X7E);
                        logger.logTimestampMs(millisRp());
                        loggerHeaderSent = true; 
                    }
                    logger.logint32withStuff(entry.type ,entry.data);    // log the type (index) and value of the tlm field
                }                                                     // !!! can wait that previous buffer is totally sent by dma
                //printf("t=%d  %10.0f\n",entry.type ,  (float)entry.data);
            }    
        }
    }
    if ((forcedFields == 1) || (forcedFields == 2)) fillFields(forcedFields); // force dummy values for debuging a protocol
}
/*
void printTest(int testVal){
    static uint32_t prevTime = 0;
    if ((millisRp() - prevTime) > 10) { // once per 5 sec
        prevTime = millisRp();
        printf("test value=%i\n",testVal);
        if (configIsValid) {printf("config is valid\n");} else {printf("config is not valid\n");} 
    }
}
*/

#define MAIN_LOOP 0
void loop() {
  //debugBootButton();
    //startTimerUs(MAIN_LOOP);                            // start a timer to measure enlapsed time

  if ((configIsValid) and (configIsSaved)) {
      getSensorsFromCore1(); // this also generate the LOG signal on pio UART
      //mergeSeveralSensors();
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
        handleIbusIn();           
        //handleSbus2In();
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
      } else if (config.protocol == 'E') {  // Jeti Exbus
        handleExbusRxTx();
        //handleSbus2In();  // to do processa second inpunt
        fillSbusFrame();
      }
      watchdog_update();
      if (gyroIsInstalled) {
        calibrateGyroMixers();   // check if user ask for gyro calibration 
      }
      updatePWM(); // update PWM pins only based on channel value (not sequencer); this will call applyGyroCorrections if gyro is used
      
      sequencerLoop();  // update PWM pins based on sequencer
      if ((config.pinLogger != 255) && (newRcChannelsReceivedForLogger)) { // when logger is on and new RC data have been converted in uint16
        newRcChannelsReceivedForLogger = false; // reset the flag allowing a log of RC channels
        logger.logAllRcChannels();  // log all rc channels
      }       
  }
  //alarmTimerUs(MAIN_LOOP, 1000);    //  print a warning if enlapsed time exceed xx usec

  watchdog_update();
  //if (tud_cdc_connected()) {
  //printf("before handleUSBCmd\n");sleep_ms(100); 
  
  handleUSBCmd();  // process the commands received from the USB
  //printTest((int) seq.defsMax);
  tud_task();      // I suppose that this function has to be called periodicaly
  //}  
  //printf("after tud-task\n");sleep_ms(100); 
  if ( configIsValidPrev != configIsValid) {
    configIsValidPrev = configIsValid;
    if ((configIsValid) and (configIsSaved)){
        blinking = true; // setRgbColorOn(0,10,0); // red , green , blue
    } else {
        blinking = false; // setRgbColorOn(10,0,0);
        setRgbOn();  
    }
  }
  //printf("before handleBootButton\n");sleep_ms(100); 
  handleBootButton(); // check boot button; after double click, change LED to fix blue and next HOLD within 5 sec save the current channnels as Failsafe values
  //printf("after handleBootButton\n");sleep_ms(100); 
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
  //printf("end of loop\n");sleep_ms(100); 
  if (config.pinSpiCs != 255) {
    loraHandle() ;
  }  
}

// initialisation of core 1 that capture the sensor data
void setup1(){
    multicore_lockout_victim_init();
    setupSensors();    
}
// main loop on core 1 in order to read the sensors and send the data to core0
#define MAIN_LOOP1 1
void loop1(){
    //startTimerUs(MAIN_LOOP1);
    uint8_t qCmd;
    getSensors(); // get sensor
    // get some request from core0
    if ( ! queue_is_empty(&qSendCmdToCore1)){
        queue_try_remove(&qSendCmdToCore1, &qCmd);
        if ( qCmd == REQUEST_HORIZONTAL_MPU_CALIB) { // 0X01 is the code to request an horizontal calibration
            mpu.calibrationHorizontalExecute();
        } else if ( qCmd == REQUEST_VERTICAL_MPU_CALIB) { // 0X02 is the code to request a vertical calibration
            mpu.calibrationVerticalExecute();
        }
    }
    //alarmTimerUs(MAIN_LOOP1, 500);
}

void core1_main(){
    setup1();
    while(1) loop1();
}

int main(){
  setup();
  while(1) loop();
}