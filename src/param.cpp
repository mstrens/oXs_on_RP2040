
#include "param.h"
#include "pico/stdlib.h"
#include "config.h"
#include "stdio.h"
#include "MS5611.h"
#include "SPL06.h"
#include "BMP280.h"
#include "ads1115.h"
#include "ms4525.h"
#include "xgzp6897D.h"
#include "sdp3x.h"
#include <string.h>
#include <ctype.h>
#include "gps.h"
#include "hardware/flash.h"
#include <inttypes.h>
#include "stdlib.h"
#include  "hardware/sync.h"
#include "hardware/watchdog.h"
#include "crsf_out.h"
#include "pico/multicore.h"
#include "mpu.h"
#include "pico/util/queue.h"
#include "tools.h"
#include "jeti.h"
#include "exbus.h"
#include "hardware/pio.h"  // needed for sbus_out_pwm.h
#include "sbus_out_pwm.h"  // needed to print the PWM values
#include "sequencer.h"
#include <errno.h>   // used by strtol() to check for errors 
#include "gyro.h"
#include "crsf_in.h"
#include "sbus_in.h"

// commands could be in following form:
// C1 = 0/15  ... C16 = 0/15
// GPS_TX = 0/29
// GPS_RX = 0/29
// PRI = 5, 9, 21, 25  (UART1)
// SEC = 1, 13, 17,29 (UART0) 
// SBUS_OUT = 0/29
// TLM = 0/29
// VOLT1= 26/29 ... VOLT4 = 26/29
// SDA = 2, 6, 10, 14, 18, 22, 26  (I2C1)
// SCL = 3, 7, 11, 15, 19, 23, 27  (I2C1)
// RPM = 0/29 
// LED = 16
// PROTOCOL = C, S, J , M , I , F, 2, L, E
// for RP2040_zero, pin 16 = LED
// When no config is found in memory, a default config is loaded (defined in config.h)
// When a pin is not used, value = 0xFF
// When a pin is used twice, the config is not valid and a LED will blink Red 
// to store the pins, variable will be pinChannels[16], pinGpsTx, pinGpsRx, pinPrimIn, pinSecIn, 
//                                     pinSbusOut,pinTlm, pinVolt[4]  pinSda, pinScl,pinRpm, pinLed
// Spi SCK = 10, 14, 26 (for spi1)
// Spi Mosi= 11, 15, 27 (for spi1)
// spi Miso = 8, 12, 24, 28 (for spi1)

#define CMD_BUFFER_LENTGH 3000
uint8_t cmdBuffer[CMD_BUFFER_LENTGH];
uint16_t cmdBufferPos = 0;

extern GPS gps;
extern sbusFrame_s sbusFrame;
extern uint32_t lastRcChannels;


CONFIG config;
uint8_t debugTlm = 'N';
uint8_t debugSbusOut = 'N';

uint8_t pinCount[30] = {0};

// for sequencer
int tempIntTable[10]; // temporary table to store n integers converted from the serial buffer (starting from pvalue)
uint8_t nextSequencerBegin;    // true when a step is the first of the next sequencer
uint8_t nextSequenceBegin;    // true when a step is the first of the next sequence

SEQUENCER seq;
extern  SEQ_DATA seqDatas[16];   // internal table to remember the state of each sequencer
extern bool seqDatasToUpload;    // flag to say if seqDatas[] must be updated or not

bool pinIsduplicated ;
extern bool configIsValid; 
extern bool configIsSaved; 

extern bool multicoreIsRunning; 
volatile bool isPrinting = false;
extern field fields[];  // list of all telemetry fields and parameters used by Sport

extern MS5611 baro1 ;
extern SPL06  baro2 ;
extern BMP280 baro3 ; 

extern MS4525 ms4525;
extern SDP3X  sdp3x;
extern XGZP   xgzp;

extern ADS1115 adc1 ;
extern ADS1115 adc2 ;    

extern MPU mpu;
extern queue_t qSendCmdToCore1;

extern uint8_t forcedFields;

extern float dteCompensationFactor;

extern sbusFrame_s sbusFrame;

extern uint16_t pwmTop; // just used for debugging

extern gyroMixer_t gyroMixer ; // contains the parameters provided by the learning process for each of the 16 Rc channel

extern int8_t orientationList[36][6];
extern const char* mpuOrientationNames[8];
extern bool orientationIsWrong; 

extern bool locatorInstalled;


void handleUSBCmd(void){
    int c;
    while (1) {
        c = getchar_timeout_us(5);
        //printf("%X\n", (uint8_t) c);
        
        if ( c== PICO_ERROR_TIMEOUT) return;
        //printf("%X\n", (uint8_t) c);
        
        if (cmdBufferPos >= (CMD_BUFFER_LENTGH -1) ){  // discard previous char when buffer is full
            cmdBufferPos = 0;
        }
        if ( c != '\n') {
            cmdBuffer[cmdBufferPos++] = c & 0xFF; // save the char
           // printf("%c\n", (uint8_t) c);
        } else {
            cmdBuffer[cmdBufferPos] = 0x00 ; // put the char end of string
            cmdBufferPos = 0;                // reset the position
            processCmd();                    // process the cmd line
        }
    }
}

char * pkey = NULL;
char * pvalue = NULL;
    

void processCmd(){
    printf("processing cmd\n");
    bool updateConfig = false;      // after some cheks, says if we can save the config
    bool updateSequencers = false;    // after some checks, says if we ca save the sequencers   
    char *ptr;
    uint32_t ui;
    uint32_t ui2;
    int32_t integerValue;
    double db;    
    pkey = NULL;
    pvalue = NULL;
    //printf("buffer0= %X\n", cmdBuffer[0]);
    if (cmdBuffer[0] == 0x0D){ // when no cmd is entered we print the current config
        printConfigAndSequencers();
        return; 
    }
    if (cmdBuffer[0] == '?'){ // when help is requested we print the instruction
        isPrinting = true; //use to discard incoming data from Sbus... while printing a long text (to avoid having the queue saturated)
        printf("\nCommands can be entered to change the config parameters (format is XXXX  or  XXXX=YYYY e.g. PRI=5)\n");
        
        printf("GPIO's being used (to disable, set the value to 255)\n");
        printf("  Function                  Command   Valid GPIO's\n");   
        printf("  Primary channels input    PRI     = 5, 9, 21, 25\n");
        printf("  Secondary channels input  SEC     = 1, 13, 17, 29\n");
        printf("  Telemetry                 TLM     = 0, 1, 2, ..., 29\n");
        printf("  GPS Rx                    GPS_RX  = 0, 1, 2, ..., 29\n");
        printf("  GPS Tx                    GPS_TX  = 0, 1, 2, ..., 29\n");
        printf("  Sbus OUT                  SBUS_OUT= 0, 1, 2, ..., 29\n");
        printf("  RPM (only for Sport)      RPM     = 0, 1, 2, ..., 29\n");
        printf("  SDA (baro sensor)         SDA     = 2, 6, 10, 14, 18, 22, 26\n");
        printf("  SCL (baro sensor)         SCL     = 3, 7, 11, 15, 19, 23, 27\n");
        printf("  PWM Channels 1, ..., 16   C1 / C16= 0, 1, 2, ..., 15\n");
        printf("  Voltage 1, ..., 4         V1 / V4 = 26, 27, 28, 29\n");
        printf("  RGB led                   RGB     = 0, 1, 2, ..., 29\n");
        printf("  Logger                    LOG     = 0, 1, 2, ..., 29\n");
        printf("  ESC                       ESC_PIN = 0, 1, 2, ..., 29\n");
        printf("  Lora  CS                  SPI_CS  = 0, 1, 2, ..., 29\n");
        printf("        SCK                 SPI_SCK = 10, 14, 26\n");
        printf("        MOSI                SPI_MOSI = 11, 15, 27\n");
        printf("        MISO                SPI_MISO = 8, 12, 24, 28\n");

        printf("Rf protocol                 PROTOCOL= Y        Y is S(Sport Frsky), F(Fbus Frsky), C(CRSF/ELRS), H(Hott), M(Mpx)\n");
        printf("                                               2(Sbus2 Futaba), J(Jeti), E(jeti Exbus), L (spektrum SRXL2) ,or I(IBus/Flysky)\n");
        printf("    CRSF baudrate:          CRSFBAUD = 420000\n");
                
        printf("Type of ESC :               ESC_TYPE = YYY      YYY is HW4(Hobbywing V4), ZTW1(ZTW mantis),KON (Kontronik) or BLH(BlHeli)\n");
        printf("Logger baudrate :           LOGBAUD = 115200\n");
        printf("Refresh rate of servos      PWMHZ = 50          Value in range 50...333 (apply for PWM and sequencer)\n");
        printf("Voltage scale x(1,2,3,4)    SCALEx = nnn.ddd    e.g. SCALE1=2.3 or SCALE3=0.123\n")  ;
        printf("                            SCALEx = 0          Avoid sending voltage x to the Transmitter (for Frsky or Jeti)\n")  ;
        printf("Voltage offset x(1,2,3,4)   OFFSETx = nnn.ddd   e.g. OFFSET1=0.6789\n")  ;
        printf("Temperature (from V3, V4)   TEMP = Y            Y=1 if one TMP36 on V3, Y=2 if a second one is on V4\n");
        printf("GPS type                    GPS = Y             U=Ublox configured by oXs, E=Ublox configured Externally, C = CADIS\n");
        printf("RPM multiplicator           RPM_MULT = Y.Y      e.g. 0.5 to divide RPM by 2\n");
        printf("Led inversion               LED = N             N=normal , I=inverted\n");
        printf("Failsafe mode               FAILSAFE = H        Set failsafe to Hold mode\n")  ;
        printf("         values             SETFAILSAFE         Values are set on the current positions\n")  ;
        
        printf("Rc channels  (1...16 or 255 for not in use)    can be used to manage airspeed/gyro/sequencers\n");
        printf("    Airspeed                ACC = YY            To select Vspeed and compensation ratio\n");   
        printf("    Gyro Mode/Gain          GMG = YY            To select mode/gain\n");
        printf("    Gyro Stick Aileron      GSA = YY            Gyro only : Original aileron stick (without mix/limits/trim)\n");
        printf("    Gyro Stick Elevator     GSE = YY               ""                 elevator \n");
        printf("    Gyro Stick Rudder       GSR = YY               ""                 rudder   \n");
        
        printf("Gyro Gain on Roll           GGR = YYYY          Gain on roll axis (-127/127)\n");
        printf("             Pitch          GGP = YYYY             ""    pitch\n");
        printf("             Yaw            GGY = YYYY             ""    Yaw\n");
        printf("     Gain on stick Throw    GGT = Y             1 (corr. on full throw) , 2 (on half) , 3 (on quater)\n");
        printf("     Max Rotate             GMR = Y             1 (Very low) , 2 (low) , 3 (medium) , 4 (high)\n");
        printf("     stick Rotate Enable    GRE =Y              1 (disabled) , 2 (enabled)\n");
        printf("     Stabilize mode         GST = YYY           YY = ON or OFF(=hold mode replace stabilize mode)\n");
        printf("     Orientation(Hoz./Vert.)GOH = X or GOV = X  0(front X+), 1(back X-), 2(left Y+), 3(right Y-), 4(up Z+), 5(down Z-)\n");
        printf("     PID parameters         PIDx = kpA kiA kdA kpE kiE kdE kpR kiR kdR       x=N(normal), H(hold), S(stab)\n");
        printf("                                                kp, ki, kd are the values of one PID; A,E,R means for Aileron, Elevator, Rudder\n");

        printf("Sequencers                  SEQ = YYYY          See Readme section to see how to fill YYYY\n");
        printf("                            SEQ = DEL           Erase all sequencer\n");
        printf("Force MPU6050 calibration   MPUCAL=Y            Y = H (Horizontal and still) or V (Vertical=nose up)\n");
        printf("Testing                     FV                  Field Values (display all telemetry internal values)\n")  ;
        printf("                            FVP                 Field Values Positieve (force the tlm values to positieve dummy values\n")  ;
        printf("                            FVN                      idem with negatieve values\n")  ;
        printf("                            PWM                 Display the current PWM values (in micro sec)\n");
        printf("\n");
        printf("To get the current config, just press Enter; to save it in flash, send SAVE; to get list of commands send ""?""\n");
        printf("   Note: some changes require a reset to be applied (e.g. to unlock I2C bus)\n");
        isPrinting = false;
        return;  
    }
    if (cmdBuffer[0] != 0x0){
        char * equalPos = strchr( (char*)cmdBuffer, '=');  // search position of '='
        
        if (equalPos != NULL){ // there is a '=' so search for value
            *equalPos = 0x0;      // replace '=' with End of string    
            equalPos++;           // point to next position  
            pvalue = skipWhiteSpace(equalPos);
            removeTrailingWhiteSpace(pvalue);
        }    
        pkey =  skipWhiteSpace((char*)cmdBuffer);  
        removeTrailingWhiteSpace(pkey);
    }
    upperStr(pkey);
    upperStr(pvalue);
    // pkey point to the key (before '=')
    // pvalue point to the value
    printf("\nCmd to execute: ");   
    if (pkey) printf("  %s", pkey);
    if (pvalue) printf("=%s", pvalue);
    printf("\n");
    
    // change PRI pin
    if ( strcmp("PRI", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ((config.protocol != 'E' && config.protocol != 'F' && config.protocol != 'L') &&
                   ( !(ui == 5 or ui == 21 or ui == 9 or ui ==25 or ui ==255))) {
            printf("Error : PRI pin must be 5 ,21 , 9 , 25 or 255 for most protocols (except Exbus, Fbus and SRXL2)\n");
        } else if ( (config.protocol == 'E' || config.protocol == 'F' || config.protocol == 'L') &&
                   ( !(ui <=29 or ui ==255))) {
            printf("Error : PRI pin must be in range 0/29 or 255 for Exbus, Fbus and SRXL2 protocol\n");
        } else {    
            config.pinPrimIn = ui;
            printf("Pin for primary channels input = %u\n" , config.pinPrimIn);
            updateConfig = true;
        }
    }
    // change SEC pin
    if ( strcmp("SEC", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui == 1 or ui == 17 or ui == 13 or ui ==29 or ui ==255)) {
            printf("Error : pin must 1, 13, 17, 29 or 255");
        } else {    
            config.pinSecIn = ui;
            printf("Pin for secondary channels input = %u\n" , config.pinSecIn);
            updateConfig = true;
        }
    }
    // change TLM pin
    if ( strcmp("TLM", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui <=29 or ui ==255)) {
            printf("Error : pin must be in range 0/29 or 255\n");
        } else {    
            config.pinTlm = ui;
            printf("Pin for telemetry = %u\n" , config.pinTlm );
            updateConfig = true;
        }
    }
    // change GPS Rx pin
    if ( strcmp("GPS_RX", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui <=29 or ui ==255)) {
            printf("Error : pin must be in range 0/29 or 255\n");
        } else {    
            config.pinGpsRx = ui;
            printf("Pin for GPS Rx = %u\n" , config.pinGpsRx );
            updateConfig = true;
        }
    }
    // change GPS Tx pin
    if ( strcmp("GPS_TX", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui <=29 or ui ==255)) {
            printf("Error : pin must be in range 0/29 or 255\n");
        } else {    
            config.pinGpsTx = ui;
            printf("Pin for GPS Tx = %u\n" , config.pinGpsTx );
            updateConfig = true;
        }
    }
    // change Sbus out pin
    if ( strcmp("SBUS_OUT", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui <=29 or ui ==255)) {
            printf("Error : pin must be in range 0/29 or 255\n");
        } else {    
            config.pinSbusOut = ui;
            printf("Pin for Sbus output = %u\n" , config.pinSbusOut );
            updateConfig = true;
        }
    }
    // change for RPM pin
    if ( strcmp("RPM", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui <=29 or ui ==255)) {
            printf("Error : pin must be in range 0/29 or 255\n");
        } else {    
            config.pinRpm = ui;
            printf("Pin for RPM = %u\n" , config.pinRpm );
            updateConfig = true;
        }
    }
    // change for SDA pin
    if ( strcmp("SDA", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui==2 or ui==6 or ui==10 or ui==14 or ui==18 or ui==22 or ui==26 or ui ==255)) {
            printf("Error : pin must be 2, 6, 10, 14, 18, 22, 26 or 255\n");
        } else {    
            config.pinSda = ui;
            printf("Pin for SDA (baro) = %u\n" , config.pinSda );
            updateConfig = true;
        }
    }
    // change for SCL pin
    if ( strcmp("SCL", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui==3 or ui==7 or ui==11 or ui==15 or ui==19 or ui==23 or ui==27 or ui ==255)) {
            printf("Error : pin must be 3, 7, 11, 15, 19, 23, 27 or 255\n");
        } else {    
            config.pinScl = ui;
            printf("Pin for Scl (baro) = %u\n" , config.pinScl );
            updateConfig = true;
        }
    }
    if ( strcmp("MPUCAL", pkey) == 0 ) {  
        if (!mpu.mpuInstalled) {
        printf("Calibration not done: no MP6050 installed\n");
        } else if (!((strcmp("H", pvalue) == 0) || (strcmp("V", pvalue) == 0) )){  // only possible to request an horizontal or a vertical calibration
            printf("Calibration not done: type must be H (Horizontal and still) or V (Vertical = nose up)\n"); 
        } else {    
            uint8_t data = REQUEST_VERTICAL_MPU_CALIB ;
            if (strcmp("H", pvalue) == 0){
              data = REQUEST_HORIZONTAL_MPU_CALIB; //  = execute calibration
            } 
            queue_try_add(&qSendCmdToCore1 , &data);
            return; // retun here to avoid other messages
        }
    }
    
    // change for channels
    if ( *pkey == 'C' && * (pkey+1) >= '0' && * (pkey+1) <= '9') {
        pkey++; // skip 'C' char to extract the digits
        ui2 = strtoul(pkey, NULL, 10);
        //printf("channel is = %u\n", (int) ui2);
        if ( (ui2==0 or ui2>16 )){
            printf("Error : Channel number must be in range 1 / 16\n");
        } else {
            ui = strtoul(pvalue, &ptr, 10);
            if ( *ptr != 0x0){
                printf("Error : pin must be an unsigned integer\n");
            } else if ( !(ui<16 or ui ==255)) {
                printf("Error : pin must be in range 0 / 15 or 255\n");
            } else {    
                config.pinChannels[ui2-1] = ui;
                printf("Pin for channel %" PRIu32 " = %u\n" , ui2 , config.pinChannels[ui2-1] );
                updateConfig = true;
            }    
        }
    }
    // change for voltages
    if ( *pkey == 'V' && * (pkey+1) >= '0' && * (pkey+1) <= '9') {
        pkey++; // skip 'V' char to extract the digits
        ui2 = strtoul(pkey, NULL, 10);
        if ( (ui2==0 or ui2>4 )){
            printf("Error : Voltage number must be in range 1 / 4\n");
        } else {
            ui = strtoul(pvalue, &ptr, 10);
            if ( *ptr != 0x0){
                printf("Error : pin must be an unsigned integer\n");
            } else if ( ! ( ((ui>=26 and ui<=29)) or ui ==255) ) {
                printf("Error : pin must be in range 26 / 29 or 255\n");
            } else {    
                config.pinVolt[ui2-1] = ui;
                printf("Pin for voltage %" PRIu32 " = %u\n" , ui2, config.pinVolt[ui2-1] );
                updateConfig = true;
            }    
        }
    }
        
    // change crsf baudrate
    if ( strcmp("CRSFBAUD", pkey) == 0 ) { // if the key is CRSFBAUD
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : CRSF baudrate must be an unsigned integer\n");
        } else {
            config.crsfBaudrate = ui;
            printf("CRSF baudrate = %" PRIu32 "\n" , config.crsfBaudrate);
            updateConfig = true;
        }
    }
    
    // change debugTlm
    if ( strcmp("DEBUGTLM", pkey) == 0 ) { // if the key is DEBUGTLM
        if (strcmp("Y", pvalue) == 0) {
            debugTlm = 'Y';
        } else if (strcmp("N", pvalue) == 0) {
            debugTlm = 'N';
            //updateConfig = true; // this is not saved
        } else  {
            printf("Error : DEBUGTLM must be Y or N\n");
        }
    }
    
    // change debugSbusOut
    if ( strcmp("DEBUGSBUSOUT", pkey) == 0 ) { // if the key is DEBUGSBUSOUT
        if (strcmp("Y", pvalue) == 0) {
            debugSbusOut = 'Y';
        } else if (strcmp("N", pvalue) == 0) {
            debugSbusOut = 'N';
            //updateConfig = true; // this is not saved
        } else  {
            printf("Error : DEBUGSBUSOUT must be Y or N\n");
        }
    }
    
    // print current values of all telemetry fields
    if ( strcmp("FV", pkey) == 0 ) { 
            printFieldValues();
            return;
    }
    // force dummy positive value
    if ( strcmp("FVP", pkey) == 0 ) { 
            forcedFields = 1;
            fillFields(forcedFields);
            if (config.protocol == 'E') setupExbusList(true); // rebuild the list of fields being sent in jeti exbus
            if (config.protocol == 'J') initListOfJetiFields(true); // rebuild the list of fields being sent in jeti ex
            printFieldValues();
            printf("Internal telemetry fields are now filled with POSITIVE dummy values\n");
            printf("To get real values again, you have to power down\n");
            
            return;
    }
// force dummy negative value
    if ( strcmp("FVN", pkey) == 0 ) { 
            forcedFields = 2;
            fillFields(forcedFields);
            if (config.protocol == 'E') setupExbusList(true); // rebuild the list of fields being sent in jeti exbus
            if (config.protocol == 'J') initListOfJetiFields(true); // rebuild the list of fields being sent in jeti ex
            
            printFieldValues();
            printf("Internal telemetry fields are now filled with NEGATIVE dummy values\n");
            printf("To get real values again, you have to power down\n");
            return;
    }
    // print current values of all PWM fields
    if ( strcmp("PWM", pkey) == 0 ) { 
            printPwmValues();
            return;
    }
    // change protocol
    if ( strcmp("PROTOCOL", pkey) == 0 ) { // 
        if (strcmp("S", pvalue) == 0) {
            config.protocol = 'S';
            updateConfig = true;
        } else if (strcmp("C", pvalue) == 0) {
            config.protocol = 'C';
            updateConfig = true;
        } else if (strcmp("J", pvalue) == 0) {
            config.protocol = 'J';
            updateConfig = true;
        } else if (strcmp("E", pvalue) == 0) {
            config.protocol = 'E';
            updateConfig = true;
        } else if (strcmp("H", pvalue) == 0) {
            config.protocol = 'H';
            updateConfig = true;
        } else if (strcmp("M", pvalue) == 0) {
            config.protocol = 'M';
            updateConfig = true;
        } else if (strcmp("F", pvalue) == 0) {
            config.protocol = 'F';
            updateConfig = true;
        } else if (strcmp("I", pvalue) == 0) {
            config.protocol = 'I';
            updateConfig = true;
        } else if (strcmp("2", pvalue) == 0) {
            config.protocol = '2';
            updateConfig = true;
        } else if (strcmp("L", pvalue) == 0) {
            config.protocol = 'L';
            updateConfig = true;
        } else  {
            printf("Error : protocol must be S(Sport Frsky), F(Fbus Frsky), C(CRSF=ELRS), J(Jeti), E(jeti Exbus), H(Hott), M(Mpx), 2(Sbus2 Futaba), L(SRXL2 Spektrum) or I(Ibus/Flysky)\n");
        }
    }
    
    
    // change scale
    if (( strcmp("SCALE1", pkey) == 0 ) || ( strcmp("SCALE2", pkey) == 0 )\
         || ( strcmp("SCALE3", pkey) == 0 )  || ( strcmp("SCALE4", pkey) == 0 ) ){ 
        db = strtod(pvalue,&ptr);
        if (*ptr != 0x0) {
            printf("Error : value is not a valid float\n");
        } else {
            updateConfig = true;
            if (*(pkey+5) == '1' ) {config.scaleVolt1 = db;}
            else if (*(pkey+5) == '2' ) {config.scaleVolt2 = db;}
            else if (*(pkey+5) == '3' ) {config.scaleVolt3 = db;}
            else if (*(pkey+5) == '4' ) {config.scaleVolt4 = db;}
            else {
                printf("Error : x must be 1...4 in SCALEx\n");
                updateConfig = false;
            }
        }
    }
    // change offset
    if (( strcmp("OFFSET1", pkey) == 0 ) || ( strcmp("OFFSET2", pkey) == 0 )\
         || ( strcmp("OFFSET3", pkey) == 0 )  || ( strcmp("OFFSET4", pkey) == 0 ) ){ 
        db = strtod(pvalue,&ptr);
        if (*ptr != 0x0) {
            printf("Error : value is not a valid float\n");
        } else {
            updateConfig = true;
            if (*(pkey+6) == '1' ) {config.offset1 = db;}
            else if (*(pkey+6) == '2' ) {config.offset2 = db;}
            else if (*(pkey+6) == '3' ) {config.offset3 = db;}
            else if (*(pkey+6) == '4' ) {config.offset4 = db;}
            else {
                printf("Error : x must be 1...4 in OFFSETx\n");
                updateConfig = false;
            }
        }
    }
    // change GPS
    if ( strcmp("GPS", pkey) == 0 ) {
        if (strcmp("U", pvalue) == 0) {
            config.gpsType = 'U';
            updateConfig = true;
        } else if (strcmp("E", pvalue) == 0) {
            config.gpsType = 'E';
            updateConfig = true;
        } else if (strcmp("C", pvalue) == 0) {
            config.gpsType = 'C';
            updateConfig = true;
        } else  {
            printf("Error : GPS type must be U, E or C\n");
        }
    }
    // change RPM multipicator
    if ( strcmp("RPM_MULT", pkey) == 0 ) {
        db = strtod(pvalue,&ptr);
        if (*ptr != 0x0) {
            printf("Error : value is not a valid float\n");
        } else {
            config.rpmMultiplicator = db;
            updateConfig = true;
        }
    }
    

    
    // change gpio0
    /*
    if ( strcmp("GPIO0", pkey) == 0 ) {
        if (strcmp("SBUS", pvalue) == 0) {
            config.gpio0 = 0 ;
            printf("gpio0 = Sbus\n" );
            updateConfig = true;
        } else {
            ui = strtoul(pvalue, &ptr, 10);
            if ( *ptr != 0x0 || (ui == 0) || ui > 16){
                printf("Error : GPIO0 must be SBUS or an integer between 1 and 16");
            } else {
                config.gpio0 = ui;
                printf("GPIO0 = %u\n" , (unsigned int) config.gpio0);
                updateConfig = true;
            }
        }    
    }
    
    // change gpio1
    if ( strcmp("GPIO1", pkey) == 0 ) {
            ui = strtoul(pvalue, &ptr, 10);
            if ( *ptr != 0x0 || (ui == 0) || ui > 13){
                printf("Error : GPIO1 must be an integer between 1 and 13");
            } else {
                config.gpio1 = ui;
                printf("GPIO1 = %u\n" , (unsigned int) config.gpio1);
                updateConfig = true;
            }
    }
    
    
    // change gpio5
    if ( strcmp("GPIO5", pkey) == 0 ) {
            ui = strtoul(pvalue, &ptr, 10);
            if ( *ptr != 0x0 || (ui == 0) || ui > 13){
                printf("Error : GPIO5 must be an integer between 1 and 13");
            } else {
                config.gpio5 = ui;
                printf("GPIO5 = %u\n" , (unsigned int) config.gpio5);
                updateConfig = true;
            }
    }

    
    // change gpio11
    if ( strcmp("GPIO11", pkey) == 0 ) {
            ui = strtoul(pvalue, &ptr, 10);
            if ( *ptr != 0x0 || (ui == 0) || ui > 16){
                printf("Error : GPIO11 must be an integer between 1 and 16");
            } else {
                config.gpio11 = ui;
                printf("GPIO11 = %u\n" , (unsigned int) config.gpio11);
                updateConfig = true;
            }
    }
    */
    
    // change failsafe mode
    if ( strcmp("FAILSAFE", pkey) == 0 ) {
        if (strcmp("H", pvalue) == 0) {
            config.failsafeType = 'H';
            updateConfig = true;
        } else  {
            printf("Error : FAILSAFE mode must be H\n");
        }
    }
    // set failsafe to the current values
    if ( strcmp("SETFAILSAFE", pkey) == 0 ) { // if the key is Failsafe
        if ( lastRcChannels ) {
            config.failsafeType = 'C'; // remove 'H' for HOLD
            memcpy( &config.failsafeChannels , &sbusFrame.rcChannelsData, sizeof(config.failsafeChannels));
            updateConfig = true;
        } else {
            printf("Error : No RC channels have been received yet. FAILSAFE values are unknown\n");
        }    
    }
    // change number of temperature sensors
    if ( strcmp("TEMP", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : value must be an unsigned integer\n");
        } else if ( !(ui==0 or ui==1 or ui==2 or ui ==255)) {
            printf("Error : value must be 0, 1, 2 or 255\n");
        } else {    
            config.temperature = ui;
            printf("Number of temperature sensors = %u\n" , config.temperature );
            updateConfig = true;
        }
    }
    // change Vspeed compensation channel 
    if ( strcmp("ACC", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : channel must be an unsigned integer\n");
        } else if ( !(ui >= 1 or ui <= 16 or ui ==255)) {
            printf("Error : channel must be 1...16 or 255");
        } else {    
            config.VspeedCompChannel = ui;
            printf("Vspeed compensation channel = %u\n" , config.VspeedCompChannel);
            updateConfig = true;
        }
    }

    // change for RGB led gpio
    if ( strcmp("RGB", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : gpio must be an unsigned integer\n");
        } else if ( !(ui <=29 or ui==255)) {
            printf("Error : gpio must be in range 0/29 or 255\n");
        } else {    
            config.pinLed = ui;
            printf("gpio for RGB led = %u\n" , config.pinLed );
            updateConfig = true;
        }
    }
    
    // change led color
    if ( strcmp("LED", pkey) == 0 ) {
        if (strcmp("N", pvalue) == 0) {
            config.ledInverted = 'N';
            updateConfig = true;
        } else if (strcmp("I", pvalue) == 0) {
            config.ledInverted = 'I';
            updateConfig = true;
        } else  {
            printf("Error : LED color must be N (normal) or I(inverted)\n");
        }
    }

    // change for Log pin
    if ( strcmp("LOG", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : gpio must be an unsigned integer\n");
        } else if ( !(ui <=29 or ui==255)) {
            printf("Error : gpio must be in range 0/29 or 255\n");
        } else {    
            config.pinLogger = ui;
            printf("Gpio for Logger = %u\n" , config.pinLogger );
            updateConfig = true;
        }
    }
    
    // change logger baudrate
    if ( strcmp("LOGBAUD", pkey) == 0 ) { // if the key is LOGBAUD
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : logger baudrate must be an unsigned integer\n");
        } else {
            config.loggerBaudrate = ui;
            printf("Logger baudrate = %" PRIu32 "\n" , config.loggerBaudrate);
            updateConfig = true;
        }
    }

    // change for Esc pin
    if ( strcmp("ESC_PIN", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui <=29 or ui ==255)) {
            printf("Error : pin must be in range 0/29 or 255\n");
        } else {    
            config.pinEsc = ui;
            printf("Pin for ESC = %u\n" , config.pinEsc );
            updateConfig = true;
        }
    }
    
    // change for type of esc
    if ( strcmp("ESC_TYPE", pkey) == 0 ) { 
        if (strcmp("HW4", pvalue) == 0) {
            config.escType = HW4 ;
            printf("escType is now HW4 (Hobbywing)\n");
            updateConfig = true; 
        //} else if  (strcmp("HW3", pvalue) == 0) {
        //    config.escType = HW3 ;
        //    updateConfig = true;
        } else if (strcmp("KON", pvalue) == 0) {
            config.escType = KONTRONIK ;
            printf("escType is now KON (Kontronik)\n");
            updateConfig = true;
        } else if (strcmp("ZTW1", pvalue) == 0) {
            config.escType = ZTW1 ;
            printf("escType is now ZTW1\n");
            updateConfig = true;
        } else if (strcmp("BLH", pvalue) == 0) {
            config.escType = BLH ;
            printf("escType is now BLH (BlHeli)\n");
            updateConfig = true;
        } else {    
            printf("Error : ESC_TYPE must be HW4, ZTW1, KON or BLH\n");
        }
    }

    // change PWM HZ
    if ( strcmp("PWMHZ", pkey) == 0 ) { // if the key is PWMHZ
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : PWMHZ must be an unsigned integer\n");
        } else if ((ui < 50 ) || (ui > 333)){
            printf("Error : PWMHZ must be in range 50...333 (included)\n");
        } else {
            config.pwmHz = ui;
            printf("PwmHz = %" PRIu32 "\n" , config.pwmHz);
            updateConfig = true;
        }
    }

    // change gyro mode/gain channel 
    if ( strcmp("GMG", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : channel must be an unsigned integer\n");
        } else if ( !(ui >= 1 or ui <= 16 or ui ==255)) {
            printf("Error : channel must be 1...16 or 255");
        } else {    
            config.gyroChanControl = ui;
            printf("Gyro mode/gain channel = %u\n" , config.gyroChanControl);
            updateConfig = true;
        }
    }

    // change gyro stick aileron channel 
    if ( strcmp("GSA", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : channel must be an unsigned integer\n");
        } else if ( !(ui >= 1 or ui <= 16 or ui ==255)) {
            printf("Error : channel must be 1...16 or 255");
        } else {    
            config.gyroChan[0] = ui;
            printf("Gyro stick aileron = %u\n" , config.gyroChan[0]);
            updateConfig = true;
        }
    }

    // change gyro stick elevator channel 
    if ( strcmp("GSE", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : channel must be an unsigned integer\n");
        } else if ( !(ui >= 1 or ui <= 16 or ui ==255)) {
            printf("Error : channel must be 1...16 or 255");
        } else {    
            config.gyroChan[1] = ui;
            printf("Gyro stick elevator channel = %u\n" , config.gyroChan[1]);
            updateConfig = true;
        }
    }

    // change gyro stick rudder channel 
    if ( strcmp("GSR", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : channel must be an unsigned integer\n");
        } else if ( !(ui >= 1 or ui <= 16 or ui ==255)) {
            printf("Error : channel must be 1...16 or 255");
        } else {    
            config.gyroChan[2] = ui;
            printf("Gyro stick rudder channel = %u\n" , config.gyroChan[2]);
            updateConfig = true;
        }
    }

    // change gyro gain Roll 
    if ( strcmp("GGR", pkey) == 0 ) { 
        integerValue = strtol(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : gain must be an integer\n");
        } else if ( !(integerValue >= -127 or integerValue < 127)) {
            printf("Error : gain must be in range -127/+127");
        } else {    
            config.vr_gain[0] = (int8_t) integerValue;
            printf("Gain on roll axis= %i\n" , config.vr_gain[0]);
            updateConfig = true;
        }
    }

    // change gyro gain Pitch 
    if ( strcmp("GGP", pkey) == 0 ) { 
        integerValue = strtol(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : gain must be an integer\n");
        } else if ( !(integerValue >= -127 or integerValue < 127)) {
            printf("Error : gain must be in range -127/+127");
        } else {    
            config.vr_gain[1] = (int8_t) integerValue;
            printf("Gain on pitch axis= %i\n" , config.vr_gain[1]);
            updateConfig = true;
        }
    }
    
    // change gyro gain Yaw 
    if ( strcmp("GGY", pkey) == 0 ) { 
        integerValue = strtol(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : gain must be an integer\n");
        } else if ( !(integerValue >= -127 or integerValue < 127)) {
            printf("Error : gain must be in range -127/+127");
        } else {    
            config.vr_gain[2] = (int8_t) integerValue;
            printf("Gain on yaw axis= %i\n" , config.vr_gain[2]);
            updateConfig = true;
        }
    }

    // change gyro stick gain throw 
    if ( strcmp("GGT", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : gain stick throw must be an unsigned integer\n");
        } else if ( !(ui >= 1 or ui <= 3)) {
            printf("Error : gain stick throw must be 1,2 or 3\n");
        } else {    
            config.stick_gain_throw = (enum STICK_GAIN_THROW) ui;
            printf("Gyro gain stick throw = %u\n" , (uint32_t) config.stick_gain_throw);
            updateConfig = true;
        }
    }

    // change gyro max rotate 
    if ( strcmp("GMR", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : max rotate must be an unsigned integer\n");
        } else if ( !(ui >= 1 or ui <= 3)) {
            printf("Error : max rotate must be 1,2,3 or 4\n");
        } else {    
            config.max_rotate = (enum MAX_ROTATE) ui;
            printf("Gyro max rotate = %u\n" , (uint32_t) config.max_rotate);
            updateConfig = true;
        }
    }

    // change gyro rotate enable in mode rate 
    if ( strcmp("GRE", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : max rotate enable must be an unsigned integer\n");
        } else if ( !(ui >= 1 or ui <= 2)) {
            printf("Error : max rotate enable must be 1 or 2\n");
        } else {    
            config.rate_mode_stick_rotate = (enum RATE_MODE_STICK_ROTATE) ui;
            printf("Max rotate enable in rate mode = %u\n" , (uint32_t) config.stick_gain_throw);
            updateConfig = true;
        }
    }

    // change gyro autolevel
    if ( strcmp("GST", pkey) == 0 ) { 
        if (strcmp("ON", pvalue) == 0) {
            config.gyroAutolevel=true;
            updateConfig = true;
        } else if (strcmp("OFF", pvalue) == 0) {
            config.gyroAutolevel=false;
            updateConfig = true;
        } else {
            printf("For GST command the value must be ON (Stabilize) or OFF (Hold)\n");
        }
    }

    // Change gyro horizontal orientation
    if ( strcmp("GOH", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : gyro orientation must be an unsigned integer\n");
        } else if ( !(ui <= 5)) {
            printf("Error : gyro orientation must be <= 5\n");
        } else {    
            config.mpuOrientationH = ui;
            printf("Gyro horizontal orientation is  ");
            printf(mpuOrientationNames[config.mpuOrientationH]);
            printf("\n");
            setupOrientation(); // based on config.mpuOrientation fill orientationX,Y,Z and signX, Y, Z and orientationIsWrong
            updateConfig = true;
        }
    }

    // Change gyro vertical orientation
    if ( strcmp("GOV", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : gyro orientation must be an unsigned integer\n");
        } else if ( !(ui <= 5)) {
            printf("Error : gyro orientation must be <= 5\n");
        } else {    
            config.mpuOrientationV = ui;
            printf("Gyro vertical orientation is ");
            printf(mpuOrientationNames[config.mpuOrientationV]);
            printf("\n");
            setupOrientation(); // based on config.mpuOrientation fill orientationX,Y,Z and signX, Y, Z and orientationIsWrong
            updateConfig = true;
        }
    }


    // get PID for rate mode
    if ( strcmp("PIDN", pkey) == 0 ) { 
        if (getPid(0)){ // true when valid syntax is decoded and pid structure has been updated ;
                                  // we will save the structure and reboot; during reboot we will check if config is valid
            updateConfig = true;
        } else {
            printf("\nError in syntax or in a parameter: command PIDN= is discarded\n");
        }  
    }

    // get PID for rate mode
    if ( strcmp("PIDH", pkey) == 0 ) { 
        if (getPid(1)){ // true when valid syntax is decoded and pid structure has been updated ;
                                  // we will save the structure and reboot; during reboot we will check if config is valid
            updateConfig = true;
        } else {
            printf("\nError in syntax or in a parameter: command PIDH= is discarded\n");
        }  
    }

    // get PID for rate mode
    if ( strcmp("PIDS", pkey) == 0 ) { 
        if (getPid(2)){ // true when valid syntax is decoded and pid structure has been updated ;
                                  // we will save the structure and reboot; during reboot we will check if config is valid
            updateConfig = true;
        } else {
            printf("\nError in syntax or in a parameter: command PIDS= is discarded\n");
        }  
    }

    // change for LORA SPI CS pin
    if ( strcmp("SPI_CS", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui <=29 or ui ==255)) {
            printf("Error : pin must be in range 0/29 or 255\n");
        } else {    
            config.pinSpiCs = ui;
            printf("Pin for SPI CS = %u\n" , config.pinSpiCs );
            updateConfig = true;
        }
    }

    // change for LORA SPI SCK pin (10, 14, 26)
    if ( strcmp("SPI_SCK", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui ==10 or ui ==14 or ui ==26 or ui ==255)) {
            printf("Error : pin SPI_SCK must be 10,14,26 or 255\n");
        } else {    
            config.pinSpiSck = ui;
            printf("Pin for SPI SCK = %u\n" , config.pinSpiSck );
            updateConfig = true;
        }
    }

    // change for LORA SPI MOSI pin (11, 15, 27)
    if ( strcmp("SPI_MOSI", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui ==11 or ui ==15 or ui ==27 or ui ==255)) {
            printf("Error : pin SPI_MOSI must be 11, 15, 27 or 255\n");
        } else {    
            config.pinSpiMosi = ui;
            printf("Pin for SPI MOSI = %u\n" , config.pinSpiMosi );
            updateConfig = true;
        }
    }


    // change for LORA SPI MISO pin (8, 12, 24, 28)
    if ( strcmp("SPI_MISO", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui ==8 or ui ==12 or ui ==24 or ui ==28 or ui ==255)) {
            printf("Error : pin SPI_MISO must be 8, 12, 24, 28 or 255\n");
        } else {    
            config.pinSpiMiso = ui;
            printf("Pin for SPI MISO = %u\n" , config.pinSpiMiso );
            updateConfig = true;
        }
    }



    // get Sequencer definition
    if ( strcmp("SEQ", pkey) == 0 ) { 
        if (strcmp("DEL", pvalue) == 0) {
            seq.defsMax=0;
            seq.stepsMax=0;
            //sequencerIsValid=false;
            updateConfig = true;
            printf("All definitions for sequencer are deleted\n");
        } else {    
            if (getAllSequencers()){ // true when valid syntax is decoded and seq structure has been updated ;
                                  // we will save the structure and reboot; during reboot we will check if config is valid
                updateConfig = true;
            } else {
                printf("\nError in syntax or in a parameter: command SEQ= is discarded\n");
            }
        }  
    }

    /*
    // get steps for Sequencer
    if ( strcmp("STEP", pkey) == 0 ) { 
        if (seq.defsMax == 0){
            printf("\nError in command STEP=: number of sequencers = 0; fill SEQ= command before entering STEP=\n");
            return;
        }
        if (getStepsSequencers()){ // true when valid syntax is decoded and step structure has been updated (not yet the seqDatas[] table);
                                  // we will save the structure and reboot; during reboot we will check if config is valid
            updateConfig = true;
        } else { 
            // in case of error, we just discard the command
            printf("\nError in syntax or in a parameter: command STEP= is discarded\n");
            return;
        }
    }
    */
    
    // save the config
    if ( strcmp("SAVE", pkey) == 0 ) { 
        saveConfig();
        saveSequencers();
        printf("config has been saved\n");  
        printf("Device will reboot but it could be that a reset or a (power down + power on) is required\n\n");
        watchdog_enable(1500,false);
        sleep_ms(1000);
        watchdog_reboot(0, 0, 100); // this force a reboot!!!!!!!!!!
        sleep_ms(5000);
        printf("OXS did not rebooted after 5000 ms\n");
    }   

    if (updateConfig) {
        printf("config has not yet been saved; use SAVE command to save it!!\n");  
        
        configIsSaved = false;    //set a flag to say that config must still be saved
        // disable interrupt to avoid having error msg about queue being full because part of main loop is not executed
        uart_set_irq_enables(CRSF_UART_ID, false, false);
        uart_set_irq_enables(CRSF2_UART_ID, false, false);
        uart_set_irq_enables(SBUS_UART_ID, false, false);
        uart_set_irq_enables(SBUS2_UART_ID, false, false);
        // there are 3 irq used from pio/sm: pio0+sm1, pio0+sm3 ,  pio1+sm0
        pio_set_irq0_source_enabled(pio0 ,  pis_sm1_rx_fifo_not_empty , false ); // pio/sm for Exbus, Hott, ibus,Mpx,sport, srxl2
        pio_set_irq1_source_enabled(pio0 ,  pis_sm3_rx_fifo_not_empty , false ); // pio/sm for ESC
        pio_set_irq0_source_enabled(pio1 ,  pis_sm0_rx_fifo_not_empty , false ); // pio/sm for GPS
    }
        
    if ( strcmp("N", pkey) == 0 ) {
        nextSimuSeqChVal();
        return;
    }    
    if ( strcmp("A", pkey) == 0 ) printAttitudeFrame(); // print Attitude frame with vario data
    if ( strcmp("G", pkey) == 0 ) printGpsFrame();      // print GPS frame
    if ( strcmp("B", pkey) == 0 ) printBatteryFrame();   // print battery frame 
    //printConfigAndSequencers();                                       // print the current config
    printf("\n >> \n");
}


void addPinToCount(uint8_t pinId){
    if ( pinId != 255) {
        if (pinId > 29 ) {
            printf("Error in parameters: one pin number is %u : it must be <30", pinId);
            configIsValid = false;
        } else {
            pinCount[pinId]++;
        }
    }
}

void checkConfigAndSequencers(){     // set configIsValid 
    // each pin can be used only once (also in sequencer)
    // if SDA is defined SCL must be defined too, and the opposite
    // if GPS_TX is defined GPS_RX must be defined too and the opposite
    watchdog_update(); //sleep_ms(500);
    bool atLeastOnePwmPin = false;
    //pinIsduplicated = false; 
    for (uint8_t i = 0 ; i<30; i++) pinCount[i] = 0; // reset the counter
    configIsValid = true;
    addPinToCount(config.pinGpsTx); 
    addPinToCount(config.pinGpsRx);
    addPinToCount(config.pinPrimIn);
    addPinToCount(config.pinSecIn);
    addPinToCount(config.pinSbusOut);
    addPinToCount(config.pinTlm);
    addPinToCount(config.pinSda);
    addPinToCount(config.pinScl);
    addPinToCount(config.pinRpm);
    addPinToCount(config.pinLed);
    for (uint8_t i = 0 ; i<16 ; i++) {addPinToCount(config.pinChannels[i]);}
    for (uint8_t i = 0 ; i<16 ; i++) {
        if (config.pinChannels[i] != 255) atLeastOnePwmPin = true ;}
    for (uint8_t i = 0 ; i<4 ; i++) {addPinToCount(config.pinVolt[i]);}
    addPinToCount(config.pinLogger);
    addPinToCount(config.pinEsc);
    addPinToCount(config.pinSpiCs);
    addPinToCount(config.pinSpiSck);
    addPinToCount(config.pinSpiMosi);
    addPinToCount(config.pinSpiMiso);
    for (uint8_t i = 0 ; i<seq.defsMax ; i++) {
        if (seq.defs[i].pin > 29 ) {
            printf("Error in sequencer: one pin number is %u : it must be <30", seq.defs[i].pin);
            configIsValid = false;
        } else {
            pinCount[seq.defs[i].pin]++;   
        }
    }
    for (uint8_t i = 0 ; i<30; i++) {
        if (pinCount[i] > 1) {
            printf("Error in parameters: pin %u is used %u times\n", i , pinCount[i]);
            configIsValid=false;
            pinIsduplicated= true;
        }          
    }
    if ( (config.pinSda != 255 and config.pinScl==255) or
         (config.pinScl != 255 and config.pinSda==255) ) {
        printf("Error in parameters: SDA and SCL must both be defined or unused\n");
        configIsValid=false;
    }
    if ( (config.pinGpsTx != 255 and config.pinGpsRx==255) or 
         (config.pinGpsRx != 255 and config.pinGpsTx==255) ) {
        printf("Error in parameters: GPS_TX and GPS_RX must both be defined or unused\n");
        configIsValid=false;
    }
    if ( (config.pinSbusOut != 255 and config.pinPrimIn==255 and config.pinSecIn==255) ) { //check that when Sbus out is defined, PrimIn is defined too
        printf("Error in parameters: a pin is defined for Sbus Out but not for Primary nor Secondary channels input (PRI or SEC)\n");
        configIsValid=false;
    }
    if ( (atLeastOnePwmPin and config.pinPrimIn==255 and config.pinSecIn==255) ) { //check that when pwm is defined, PrimIn is defined too
        printf("Error in parameters: at least one PWM pin is defined but no pin for Primary nor Secondary channels input (PRI or SEC)\n");
        configIsValid=false;
    }
    
    //if ( (config.pinSecIn != 255 and config.pinPrimIn ==255) ) { //check that Prim is defined ff pinSec is defined
    //    printf("Error in parameters: pin is defined for secondary channels input but not for Primary channels input (PRI)\n");
    //    configIsValid=false;
    //}
    //if ( (config.pinSbusOut != 255 and ( config.protocol == 'S' or config.protocol == 'J') ) ) { //check that Prim is defined ff pinSec is defined
    //    printf("Warning: Sbus signal will not be generated for Sport or Jeti protocol\n");
    //}
    if (config.temperature == 1 and config.pinVolt[2] == 255){
        printf("Error in parameters: when 1 temperature sensor is used (TEMP = 1), a pin for V3 must be defined too)\n");
        configIsValid=false;
    }
    if (config.temperature == 2 && (config.pinVolt[2] == 255 || config.pinVolt[3] == 255)){
        printf("Error in parameters: when 2 temperature sensors are used (TEMP = 2), a pin for V3 and for V4 must be defined too)\n");
        configIsValid=false;
    }
    if ( config.protocol != 'E' && config.protocol != 'F' && config.protocol != 'L' &&
        ( !(config.pinPrimIn == 5 or config.pinPrimIn == 21 or config.pinPrimIn == 9 or config.pinPrimIn ==25 or config.pinPrimIn ==255))) {
            printf("Error : PRI pin must be 5 ,21 , 9 , 25 or 255 for most protocols (except Exbus, Fbus and SRXL2)\n");
            configIsValid=false;
    }
    if (config.protocol == '2' && config.pinPrimIn == 255){
        printf("Error in parameters: For Futaba Sbus2 protocol, a pin must be defined for Primary channels input (PRI)\n");
        configIsValid=false;
    }
    if (config.protocol == '2' && config.pinTlm != 255 && ( config.pinTlm != (config.pinPrimIn - 1)) ){
        printf("Error in parameters: For Futaba SBUS2 protocol, TLM pin (when defined) must be equal to (PRI pin -1) \n");
        configIsValid=false;
    }
    if (config.protocol == 'F' && config.pinPrimIn == 255  ){
        printf("Error in parameters: For Frsky Fbus, a pin must be defined for Primary channels input (PRI)\n");
        configIsValid=false;
    }
    if (config.protocol == 'E' && config.pinPrimIn == 255  ){
        printf("Error in parameters: For JEti Exbus, a pin must be defined for Primary channels input (PRI)\n");
        configIsValid=false;
    }
    if (config.protocol == 'F' && config.pinTlm != 255  ){
        printf("Error in parameters: For Frsky Fbus, TLM pin may not be defined (but PRI must be defined)\n");
        configIsValid=false;
    }
    if (config.protocol == 'E' && config.pinTlm != 255  ){
        printf("Error in parameters: For jeti Exbus, TLM pin may not be defined (but PRI must be defined)\n");
        configIsValid=false;
    }
    if (config.protocol == 'L' && config.pinPrimIn == 255  ){
        printf("Error in parameters: For Spektrum SRXL2, a pin must be defined for Primary channels input (PRI)\n");
        configIsValid=false;
    }
    if (config.protocol == 'L' && config.pinTlm != 255  ){
        printf("Error in parameters: For Spektrum SRXL2, TLM pin may not be defined (but PRI must be defined)\n");
        configIsValid=false;
    }
    if (!( config.VspeedCompChannel >= 1 or config.VspeedCompChannel <= 16 or config.VspeedCompChannel ==255)){
        printf("Error in parameters: Vspeed compensation channel must be in range 1...16 or 255\n");
        configIsValid=false;
    }
    if ( (config.pinLogger != 255) && ( config.loggerBaudrate < 9600) || (config.loggerBaudrate > 1000000 )){
        printf("Error in parameters: Logger baudrate must be in range 9600...1000000\n");
        configIsValid=false;
    }
    //if ( (config.pinEsc != 255) && (config.pinVolt[0]!=255) ) {
    //    printf("Error in parameters: When gpio is defined for ESC, gpio for Volt1 (V1) must be undefined (=255)\n");
    //    configIsValid=false;
    //}    
    
    //if ( (config.pinEsc != 255) && (config.pinVolt[1]!=255)) {
    //    printf("Error in parameters: When gpio is defined for ESC, gpio for current = Volt2 (V2) must be undefined (=255)\n");
    //    configIsValid=false;
    //}
        
    //if ( (config.pinEsc != 255) && (config.pinRpm!=255)) {
    //    printf("Error in parameters: When gpio is defined for ESC, gpio for RPM must be undefined (=255)\n");
    //    configIsValid=false;
    //}
    //if ( (config.pinEsc != 255) && (config.temperature!=255) && (config.temperature!=0)) {
    //    printf("Error in parameters: When gpio is defined for ESC, parameter about number of temperature (TEMP) must be 0 or 255\n");
    //    configIsValid=false;
    //}    
    if ( (config.pinEsc != 255) && (config.escType!=HW3) && (config.escType!=HW4) && \
            (config.escType!=KONTRONIK) && (config.escType!=ZTW1) && (config.escType!=BLH)) {
        printf("Error in parameters: When gpio is defined for ESC, esc type must be HW4, ZTW1, KON or BLH\n");
        configIsValid=false;
    }    
    if ( (config.pwmHz < 50) || (config.pwmHz > 333)){
        printf("Error in parameters: pwmHz must be in range 50...333 (included)\n");
        configIsValid=false;
    }    
    if ((config.gyroChanControl != 255) and ( config.gyroChan[0]==255 or config.gyroChan[1]==255 or config.gyroChan[2]==255)){
        printf("Error in parameters: when gyro mode/gain Rc channel is defined (not 255), Rc channels must also be defined for Roll, Pitch and Yaw).\n");
        configIsValid=false;
    }
    if ((config.gyroChanControl != 255) and ( config.pinScl==255 or config.pinSda==255)){
        printf("Error in parameters: when gyro mode/gain Rc channel is defined (not 255), SCL and SDA must be defined (to allow I2C for mpu6050)\n");
        configIsValid=false;
    }
    if  ((config.gyroChanControl != 255) and ( config.gyroChan[0]==config.gyroChanControl or config.gyroChan[1]==config.gyroChanControl\
             or config.gyroChan[2]==config.gyroChanControl or config.gyroChan[0]==config.gyroChan[1] or config.gyroChan[0]==config.gyroChan[2] \
             or config.gyroChan[1]==config.gyroChan[2] ) ) {
        printf("Error in parameters: when gyro mode/gain Rc channel is defined (not 255), all 4 channels must be different from each other   %i\n",config.gyroChanControl);
        configIsValid=false;
    }    

    if (config.mpuOrientationH>5){
        printf("Error in parameters: gyro horizontal orientation (%i) is not valid\n",config.mpuOrientationH);
        orientationIsWrong= true;
        // do not set configIsValid on false because we have to be able to process the gyro calibration
    }
    if (config.mpuOrientationV>5){
        printf("Error in parameters: gyro vertical orientation (%i) is not valid\n",config.mpuOrientationH);
        orientationIsWrong= true;
        // do not set configIsValid on false because we have to be able to process the gyro calibration
    }
    if ((config.mpuOrientationH<=5) && (config.mpuOrientationV<=5)){
        if (orientationList[config.mpuOrientationH *6 + config.mpuOrientationV][3]==0) {
            // when the combination H and V is not valid, sign is 0 instead of 1 or -1
            printf("Error in parameters: gyro horizontal (%i) and vertical (%i) orientations are not compatible\n",config.mpuOrientationH,config.mpuOrientationV);
            orientationIsWrong= true;
        }
    }
    if ((config.pinSpiCs != 255) && (config.pinSpiSck==255 or config.pinSpiMosi==255 or config.pinSpiMiso==255)){
        printf("Error in parameters: when SPI_CS is not 255, then SPI_CS, SPI_MOSI and SPI_MISO must all be defined (different from 255)\n");
        configIsValid=false;    
    }

    checkSequencers();
    if ( configIsValid == false) {
        printf("\nAttention: error in config parameters\n");
    } else {
        printf("\nConfig parameters are OK\n");
    }
    if ( configIsSaved == false) {
        printf("\nAttention: some config parameters are not saved; use SAVE command\n");
    }
    
//    if ( sequencerIsValid == false) {
//        printf("\nAttention: error in sequencer parameters\n");
//    } else {
//        printf("\nSequencer parameters are OK\n");
//    }
    
    printf("Press ? + Enter to get help about the commands\n");
}




void printConfigAndSequencers(){   // print all and perform checks
    //startTimerUs(0) ;  // to debug only - to know how long it takes to print the config
    isPrinting = true;
    uint8_t version[] =   VERSION ;
    printf("\nVersion = %s \n", version)  ;
    printf("    Function                GPIO  Change entering XXX=yyy (yyy=255 to disable)\n");   
    printf("Primary channels input    = %4u  (PRI     = 5, 9, 21, 25)\n", config.pinPrimIn);
    printf("Secondary channels input  = %4u  (SEC     = 1, 13, 17, 29)\n", config.pinSecIn);
    printf("Telemetry . . . . . . . . = %4u  (TLM     = 0, 1, 2, ..., 29)\n", config.pinTlm );
    printf("GPS Rx  . . . . . . . . . = %4u  (GPS_RX  = 0, 1, 2, ..., 29)\n", config.pinGpsRx );
    printf("GPS Tx  . . . . . . . . . = %4u  (GPS_TX  = 0, 1, 2, ..., 29)\n", config.pinGpsTx );
    printf("Sbus OUT  . . . . . . . . = %4u  (SBUS_OUT= 0, 1, 2, ..., 29)\n", config.pinSbusOut );
    printf("RPM   . . . . . . . . . . = %4u  (RPM     = 0, 1, 2, ..., 29)\n", config.pinRpm );
    printf("SDA (I2C sensors) . . . . = %4u  (SDA     = 2, 6, 10, 14, 18, 22, 26)\n", config.pinSda );
    printf("SCL (I2C sensors) . . . . = %4u  (SCL     = 3, 7, 11, 15, 19, 23, 27)\n", config.pinScl );
    printf("PWM Channels 1, 2, 3 ,4   = %4u %4u %4u %4u (C1 / C16= 0, 1, 2, ..., 15)\n", config.pinChannels[0] , config.pinChannels[1] , config.pinChannels[2] , config.pinChannels[3]);
    printf("PWM Channels 5, 6, 7 ,8   = %4u %4u %4u %4u\n", config.pinChannels[4] , config.pinChannels[5] , config.pinChannels[6] , config.pinChannels[7]);
    printf("PWM Channels 9,10,11,12   = %4u %4u %4u %4u\n", config.pinChannels[8] , config.pinChannels[9] , config.pinChannels[10] , config.pinChannels[11]);
    printf("PWM Channels 13,14,15,16  = %4u %4u %4u %4u\n", config.pinChannels[12] , config.pinChannels[13] , config.pinChannels[14] , config.pinChannels[15]);
    printf("Voltage 1, 2, 3, 4        = %4u %4u %4u %4u (V1 / V4 = 26, 27, 28, 29)\n", config.pinVolt[0] , config.pinVolt[1], config.pinVolt[2] , config.pinVolt[3]);
    printf("RGB led . . . . . . . . . = %4u  (RGB    = 0, 1, 2, ..., 29)\n", config.pinLed);
    printf("Logger  . . . . . . . . . = %4u  (LOG    = 0, 1, 2, ..., 29)\n", config.pinLogger );
    printf("ESC . . . . . . . . . . . = %4u  (ESC_PIN= 0, 1, 2, ..., 29)\n", config.pinEsc );
    printf("Locator CS  . . . . . . . = %4u  (SPI_CS = 0, 1, 2, ..., 29)\n", config.pinSpiCs );
    printf("        SCK . . . . . . . = %4u  (SPI_SCK= 10, 14, 26)\n", config.pinSpiSck );
    printf("        MOSI  . . . . . . = %4u  (SPI_MOSI=11, 15, 27)\n", config.pinSpiMosi );
    printf("        MISO  . . . . . . = %4u  (SPI_MISO=8, 12, 24, 28)\n", config.pinSpiMiso );
    
    if (config.escType == HW4) {
        printf("    Esc type is HW4 (Hobbywing V4)\n")  ;
    } else if (config.escType == HW3) {
        printf("Esc type is HW3 (Hobbywing V3)\n")  ;
    } else if (config.escType == KONTRONIK) {
        printf("Esc type is KON (Kontronik)\n")  ;
    } else if (config.escType == ZTW1) {
        printf("Esc type is ZTW1 (ZTW mantis)\n")  ;
    } else if (config.escType == BLH) {
        printf("Esc type is BLH (BlHeli)\n")  ;
    } else {
        printf("Esc type is not defined\n")  ;
    }    

    watchdog_update(); //sleep_ms(500);
    if (config.protocol == 'S'){
            printf("\nProtocol is Sport (Frsky)\n")  ;
        } else if (config.protocol == 'C'){
            printf("\nProtocol is CRSF (=ELRS)\n")  ;
        } else if (config.protocol == 'J'){
            printf("\nProtocol is Jeti (non Exbus)\n")  ;    
        } else if (config.protocol == 'E'){
            printf("\nProtocol is Jeti (Exbus)\n")  ;    
        } else if (config.protocol == 'H'){
            printf("\nProtocol is Hott\n")  ;    
        } else if (config.protocol == 'M'){
            printf("\nProtocol is Mpx\n")  ;    
        } else if (config.protocol == 'I'){
            printf("\nProtocol is ibus(Flysky)\n")  ;    
        } else if (config.protocol == '2'){
            printf("\nProtocol is Sbus2(Futaba)\n")  ;    
        } else if (config.protocol == 'F'){
            printf("\nProtocol is Fbus(Frsky)\n")  ;    
        } else if (config.protocol == 'L'){
            printf("\nProtocol is SRXL2 (Spektrum)\n")  ;    
        } else {
            printf("\nProtocol is unknow\n")  ;
        }
    printf("CRSF baudrate   = %" PRIu32 "\n", config.crsfBaudrate)  ;
    printf("Logger baudrate = %" PRIu32 "\n", config.loggerBaudrate)  ;
    printf("PWM is generated at = %i Hz\n", (int) config.pwmHz)  ;
    
    printf("Voltage parameters:\n")  ;
    printf("    Scales : %f , %f , %f , %f \n", config.scaleVolt1 , config.scaleVolt2 ,config.scaleVolt3 ,config.scaleVolt4 )  ;
    printf("    Offsets: %f , %f , %f , %f \n", config.offset1 , config.offset2 ,config.offset3 ,config.offset4 )  ;
    if ( config.pinVolt[2] !=255 && config.temperature == 1) {
        printf("    One temperature sensor is connected on V3\n");
    } else if (config.pinVolt[2] !=255 && config.pinVolt[3] !=255 && config.temperature == 2){
         printf("    Temperature sensors are connected on V3 and V4\n");
    } else {
        printf("    No temperature sensors are connected on V3 and V4\n");
    }
    printf("RPM multiplier = %f\n", config.rpmMultiplicator);
    if (baro1.baroInstalled) {
        printf("Baro sensor is detected using MS5611\n")  ;
        printf("    Sensitivity min = %i (at %i)   , max = %i (at %i)\n", SENSITIVITY_MIN, SENSITIVITY_MIN_AT, SENSITIVITY_MAX, SENSITIVITY_MAX_AT);
        printf("    Hysteresis = %i \n", VARIOHYSTERESIS);        
    } else if (baro2.baroInstalled) {
        printf("Baro sensor is detected using SPL06\n")  ;
        printf("    Sensitivity min = %i (at %i)   , max = %i (at %i)\n", SENSITIVITY_MIN, SENSITIVITY_MIN_AT, SENSITIVITY_MAX, SENSITIVITY_MAX_AT);
        printf("    Hysteresis = %i \n", VARIOHYSTERESIS);        
    } else if (baro3.baroInstalled) {
        printf("Baro sensor is detected using BMP280\n")  ;
        printf("    Sensitivity min = %i (at %i)   , max = %i (at %i)\n", SENSITIVITY_MIN, SENSITIVITY_MIN_AT, SENSITIVITY_MAX, SENSITIVITY_MAX_AT);
        printf("    Hysteresis = %i \n", VARIOHYSTERESIS);        
    } else {
        printf("Baro sensor is not detected\n")  ;
    }
    if (ms4525.airspeedInstalled) {
        printf("Aispeed sensor is detected using MS4525\n")  ;        
    } else if (sdp3x.airspeedInstalled) {
        printf("Airspeed sensor is detected using SDP3X\n")  ;
    } else if (xgzp.airspeedInstalled) {
        printf("Airspeed sensor is detected using XGZP....\n")  ;
    } else {
        printf("Airspeed sensor is not detected\n")  ;
    } 
    if (config.VspeedCompChannel != 255){
        printf("    Vspeed compensation channel = %i\n", config.VspeedCompChannel);
    } else {
        printf("    No Vspeed compensation channel defined; oXs uses default settings\n");
    }
    if (adc1.adsInstalled) {
        printf("First analog to digital sensor is detected using ads1115\n")  ;
        printf("    Measurement setup: %i , %i , %i ,%i\n", ads_Measure[0][0], ads_Measure[0][1], ads_Measure[0][2], ads_Measure[0][3]) ;
        printf("    Gains: %i , %i , %i ,%i\n", ads_Gain[0][0], ads_Gain[0][1], ads_Gain[0][2], ads_Gain[0][3]) ;
        printf("    Rates: %i , %i , %i ,%i\n", ads_Rate[0][0], ads_Rate[0][1], ads_Rate[0][2], ads_Rate[0][3]) ;
        printf("    Offsets: %f , %f , %f ,%f\n", ads_Offset[0][0], ads_Offset[0][1], ads_Offset[0][2], ads_Offset[0][3]) ;
        printf("    Scales: %f , %f , %f ,%f\n", ads_Scale[0][0], ads_Scale[0][1], ads_Scale[0][2], ads_Scale[0][3]) ;
        printf("    Averaged on: %i , %i , %i ,%i\n", ads_MaxCount[0][0], ads_MaxCount[0][1], ads_MaxCount[0][2], ads_MaxCount[0][3]) ;
    } else {
        printf("First analog to digital sensor is not detected\n")  ;
    }
    if (adc2.adsInstalled) {
        printf("Second analog to digital sensor is detected using ads1115\n")  ;
        printf("    Measurement setup: %i , %i , %i ,%i\n", ads_Measure[1][0], ads_Measure[1][1], ads_Measure[1][2], ads_Measure[1][3]) ;
        printf("    Gains: %i , %i , %i ,%i\n", ads_Gain[1][0], ads_Gain[1][1], ads_Gain[1][2], ads_Gain[1][3]) ;
        printf("    Rates: %i , %i , %i ,%i\n", ads_Rate[1][0], ads_Rate[1][1], ads_Rate[1][2], ads_Rate[1][3]) ;
        printf("    Offsets: %f , %f , %f ,%f\n", ads_Offset[1][0], ads_Offset[1][1], ads_Offset[1][2], ads_Offset[1][3]) ;
        printf("    Scales: %f , %f , %f ,%f\n", ads_Scale[1][0], ads_Scale[1][1], ads_Scale[1][2], ads_Scale[1][3]) ;
        printf("    Averaged on: %i , %i , %i ,%i\n", ads_MaxCount[1][0], ads_MaxCount[1][1], ads_MaxCount[1][2], ads_MaxCount[1][3]) ;
    }  else {
        printf("Second analog to digital sensor is not detected\n")  ;
    } 

    if (config.gpsType == 'U'){
            printf("Foreseen GPS type is Ublox (configured by oXs) :")  ;
        } else if (config.gpsType == 'C'){
            printf("Foreseen GPS type is Ublox (configured externally) :")  ;
        } else if (config.gpsType == 'C'){
            printf("Foreseen GPS type is CADIS  :")  ;
        } else {
            printf("Foreseen GPS type is unknown  :")  ;
        }
    if (gps.gpsInstalled && gps.GPS_fix) {
        printf("GPS is detected and has a fix\n")  ;
    } else if (gps.gpsInstalled ) {
        printf("GPS is detected but has not (yet) a fix\n")  ;
    } else {
        printf("GPS is not (yet) detected\n")  ;
    }
    if (config.ledInverted == 'I'){
        printf("Led color is inverted\n")  ;
    } else {
        printf("Led color is normal (not inverted)\n")  ;
    }
    //if (config.gpio0 == 0){
    //    printf("GPIO0 is used to output a Sbus signal\n");
    //} else if ( config.gpio0 < 17 ){
    //    printf("GPIO0 generates channel %u\n", (unsigned int) config.gpio0);
    //} else {
    //    printf("GPIO0 : Error in configuration\n");
    //}
    //if (config.gpio1 > 0 && config.gpio1 < 17){
    //    printf("GPIO1 (and GPIO2, 3, 4) generates channel %u (and next)\n", (unsigned int) config.gpio1);
    //} else {
    //    printf("GPIO1 : Error in configuration\n");
    //}
    //if (config.gpio5 > 0 && config.gpio5 < 17){
    //    printf("GPIO5 (and GPIO6, 7, 8) generates channel %u (and next)\n", (unsigned int) config.gpio5);
    //} else {
    //    printf("GPIO5 : Error in configuration\n");
    //}
    //if (config.gpio11 > 0 && config.gpio11 < 17){
    //    printf("GPIO11 generates channel %u \n", (unsigned int) config.gpio11);
    //} else {
    //    printf("GPIO11 : Error in configuration\n");
    //}
    if ( config.failsafeType == 'H'){
        printf("Failsafe type is HOLD\n")  ;
    } else {
        printf("Failsafe uses predefined values\n")  ;
        printf("     Chan 1...4  = %5d %5d %5d %5d\n", (int) fmap( config.failsafeChannels.ch0  )\
                                                        , (int) fmap( config.failsafeChannels.ch1 )\
                                                        , (int) fmap( config.failsafeChannels.ch2 )\
                                                        , (int) fmap( config.failsafeChannels.ch3 ) );
        printf("     Chan 5...8  = %5d %5d %5d %5d\n", (int) fmap( config.failsafeChannels.ch4 )\
                                                        , (int) fmap( config.failsafeChannels.ch5 )\
                                                        , (int) fmap( config.failsafeChannels.ch6 )\
                                                        , (int) fmap( config.failsafeChannels.ch7 ) );
        printf("     Chan 9...12 = %5d %5d %5d %5d\n", (int) fmap( config.failsafeChannels.ch8 )\
                                                        , (int) fmap( config.failsafeChannels.ch9 )\
                                                        , (int) fmap( config.failsafeChannels.ch10 )\
                                                        , (int) fmap( config.failsafeChannels.ch11 ) );
        printf("     Chan 13...16= %5d %5d %5d %5d\n", (int) fmap( config.failsafeChannels.ch12 )\
                                                        , (int) fmap( config.failsafeChannels.ch13 )\
                                                        , (int) fmap( config.failsafeChannels.ch14 )\
                                                        , (int) fmap( config.failsafeChannels.ch15 ) );
    }    
    if (config.pinSpiCs != 255){
        if (locatorInstalled ){
            printf("Lora module for locator is detected\n")  ;   
        } else {
            printf("Lora module for locator is not detected\n")  ;   
        }     
    }
    if(mpu.mpuInstalled){
        printf("Acc/Gyro is detected using MP6050\n")  ;
        printf("     Acceleration offsets X, Y, Z = %i , %i , %i\n", config.accOffsetX , config.accOffsetY , config.accOffsetZ);
        printf("     Gyro offsets         X, Y, Z = %i , %i , %i\n", config.gyroOffsetX , config.gyroOffsetY , config.gyroOffsetZ); 
        printf("     Orientation          Horizontal is ");
        uint8_t nameIdx;
        nameIdx = config.mpuOrientationH;
        if (nameIdx > 6) nameIdx=6; 
        printf(mpuOrientationNames[nameIdx]) ;
        printf("     Vertical is ");
        nameIdx = config.mpuOrientationV;
        if (nameIdx > 6) nameIdx=6; 
        printf(mpuOrientationNames[nameIdx] );
        printf("\n"); 
    } else {
       printf("Acc/Gyro is not detected\n")  ;     
    }
    printGyro();
    watchdog_update(); //sleep_ms(500);
    printSequencers(); 
    checkConfigAndSequencers();
    //getTimerUs(0);        // print the time enlapsed is the function print.

    isPrinting = false;
} // end printConfigAndSequencers()


#define FLASH_CONFIG_OFFSET (256 * 1024)
const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_CONFIG_OFFSET);

void saveConfig() {
    //sleep_ms(1000); // let some printf to finish
    uint8_t buffer[FLASH_PAGE_SIZE] ;
    memset(buffer, 0xff, FLASH_PAGE_SIZE);
    memcpy(&buffer[0], &config, sizeof(config));
    printf("size of config is %i\n", sizeof(config));
    // Note that a whole number of sectors must be erased at a time.
    // irq must be disable during flashing
    watchdog_enable(3000 , true);
    if (multicoreIsRunning) multicore_lockout_start_blocking();
    uint32_t irqStatus = save_and_disable_interrupts();
    flash_range_erase(FLASH_CONFIG_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_CONFIG_OFFSET, buffer, FLASH_PAGE_SIZE);
    restore_interrupts(irqStatus);
    if (multicoreIsRunning) multicore_lockout_end_blocking();
    //sleep_ms(1000);
    //printf("New config has been saved\n");
    //printConfig(); 
}

void cpyChannelsAndSaveConfig() {    // used when pressing boot button to save failsafe value
    config.failsafeType = 'C'; // remove 'H' for HOLD
    memcpy( &config.failsafeChannels , &sbusFrame.rcChannelsData, sizeof(config.failsafeChannels));
    saveConfig();        
}

void upperStr( char *p){
    if (p == NULL ) return;
    while ( *p != 0){
        *p = toupper(*p);
        p++;
    }    
}

char * skipWhiteSpace(char * str)
{
	char *cp = str;
	if (cp)
		while (isspace(*cp))
			++cp;
	return cp;
}

void removeTrailingWhiteSpace( char * str)
{
	if (str == nullptr)
		return;
	char *cp = str + strlen(str) - 1;
	while (cp >= str && isspace(*cp))
		*cp-- = '\0';
}

void setupConfig(){   // The config is uploaded at power on
    if (*flash_target_contents == CONFIG_VERSION ) {
        memcpy( &config , flash_target_contents, sizeof(config));
        if (config.pwmHz == 0XFFFF) config.pwmHz = _pwmHz; // set default value when it has not been defined manually

    } else {
        config.version = CONFIG_VERSION;
        config.pinChannels[0] = _pinChannels_1;
        config.pinChannels[1] = _pinChannels_2;
        config.pinChannels[2] = _pinChannels_3;
        config.pinChannels[3] = _pinChannels_4;
        config.pinChannels[4] = _pinChannels_5;
        config.pinChannels[5] = _pinChannels_6;
        config.pinChannels[6] = _pinChannels_7;
        config.pinChannels[7] = _pinChannels_8;
        config.pinChannels[8] = _pinChannels_9;
        config.pinChannels[9] = _pinChannels_10;
        config.pinChannels[10] = _pinChannels_11;
        config.pinChannels[11] = _pinChannels_12;
        config.pinChannels[12] = _pinChannels_13;
        config.pinChannels[13] = _pinChannels_14;
        config.pinChannels[14] = _pinChannels_15;
        config.pinChannels[15] = _pinChannels_16;
        config.pinGpsTx = _pinGpsTx;
        config.pinGpsRx = _pinGpsRx;
        config.pinPrimIn = _pinPrimIn;
        config.pinSecIn = _pinSecIn; 
        config.pinSbusOut = _pinSbusOut;
        config.pinTlm = _pinTlm;
        config.pinVolt[0] = _pinVolt_1;
        config.pinVolt[1] = _pinVolt_2;
        config.pinVolt[2] = _pinVolt_3;
        config.pinVolt[3] = _pinVolt_4;
        config.pinSda = _pinSda;
        config.pinScl = _pinScl;
        config.pinRpm = _pinRpm;
        config.pinLed = _pinLed;
        config.protocol = _protocol; // default = sport
        config.crsfBaudrate = _crsfBaudrate;
        config.scaleVolt1 = _scaleVolt1;
        config.scaleVolt2 = _scaleVolt2;
        config.scaleVolt3 = _scaleVolt3;
        config.scaleVolt4 = _scaleVolt4;
        config.offset1 = _offset1;
        config.offset2 = _offset2;
        config.offset3 = _offset3;
        config.offset4 = _offset4;
        config.gpsType = _gpsType ;
        config.rpmMultiplicator = _rpmMultiplicator;
        //config.gpio0 = 0;
        //config.gpio1 = 1;
        //config.gpio5 = 6;
        //config.gpio11 = 11;
        config.failsafeType = _failsafeType;
        config.failsafeChannels.ch0 = 1<<10 ; // set default failsafe value to 1/2 of 11 bits
        config.failsafeChannels.ch1 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch2 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch3 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch4 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch6 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch6 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch7 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch8 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch9 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch10 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch11 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch12 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch13 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch14 = config.failsafeChannels.ch0 ;
        config.failsafeChannels.ch15 = config.failsafeChannels.ch0 ;
        config.accOffsetX = 0;
        config.accOffsetY = 0;
        config.accOffsetZ = 0;
        config.gyroOffsetX = 0;
        config.gyroOffsetY = 0;
        config.gyroOffsetZ= 0;
        config.temperature = _temperature;
        config.VspeedCompChannel = _VspeedCompChannel;
        config.ledInverted = _ledInverted; 
        config.pinLogger = _pinLogger;
        config.loggerBaudrate =_loggerBaudrate;
        config.pinEsc = _pinEsc ;
        config.escType = _escType; 
        config.pwmHz = _pwmHz;
        config.gyroChanControl = _gyroChanControl ; // Rc channel used to say if gyro is implemented or not and to select the mode and the general gain. Value must be in range 1/16 or 255 (no gyro)
        config.gyroChan[0] = _gyroChan_AIL;
        config.gyroChan[1] = _gyroChan_ELV;
        config.gyroChan[2] = _gyroChan_RUD;
        config.vr_gain[0]  = _vr_gain_AIL;
        config.vr_gain[1]  = _vr_gain_ELV;
        config.vr_gain[2]  = _vr_gain_RUD;
        config.stick_gain_throw = (enum STICK_GAIN_THROW)_stick_gain_throw;
        config.max_rotate = (MAX_ROTATE)_max_rotate;
        config.rate_mode_stick_rotate = (enum RATE_MODE_STICK_ROTATE)_rate_mode_stick_rotate;
        config.gyroAutolevel = _gyroAutolevel;
        config.mpuOrientationH = _mpuOrientationH;
        config.mpuOrientationV = _mpuOrientationV;
        config.pid_param_rate.output_shift = _pid_param_rate_output_shift;
        config.pid_param_hold.output_shift = _pid_param_hold_output_shift;
        config.pid_param_stab.output_shift = _pid_param_stab_output_shift;
        config.pid_param_rate.kp[0] =  _pid_param_rate_KP_AIL;
        config.pid_param_rate.kp[1] =  _pid_param_rate_KP_ELV;
        config.pid_param_rate.kp[2] =  _pid_param_rate_KP_RUD;
        config.pid_param_hold.kp[0] =  _pid_param_hold_KP_AIL;
        config.pid_param_hold.kp[1] =  _pid_param_hold_KP_ELV;
        config.pid_param_hold.kp[2] =  _pid_param_hold_KP_RUD;
        config.pid_param_stab.kp[0] =  _pid_param_stab_KP_AIL;
        config.pid_param_stab.kp[1] =  _pid_param_stab_KP_ELV;
        config.pid_param_stab.kp[2] =  _pid_param_stab_KP_RUD;
            
        config.pid_param_rate.ki[0] =  _pid_param_rate_KI_AIL;
        config.pid_param_rate.ki[1] =  _pid_param_rate_KI_ELV;
        config.pid_param_rate.ki[2] =  _pid_param_rate_KI_RUD;
        config.pid_param_hold.ki[0] =  _pid_param_hold_KI_AIL;
        config.pid_param_hold.ki[1] =  _pid_param_hold_KI_ELV;
        config.pid_param_hold.ki[2] =  _pid_param_hold_KI_RUD;
        config.pid_param_stab.ki[0] =  _pid_param_stab_KI_AIL;
        config.pid_param_stab.ki[1] =  _pid_param_stab_KI_ELV;
        config.pid_param_stab.ki[2] =  _pid_param_stab_KI_RUD;
        
        config.pid_param_rate.kd[0] =  _pid_param_rate_KD_AIL;
        config.pid_param_rate.kd[1] =  _pid_param_rate_KD_ELV;
        config.pid_param_rate.kd[2] =  _pid_param_rate_KD_RUD;
        config.pid_param_hold.kd[0] =  _pid_param_hold_KD_AIL;
        config.pid_param_hold.kd[1] =  _pid_param_hold_KD_ELV;
        config.pid_param_hold.kd[2] =  _pid_param_hold_KD_RUD;
        config.pid_param_stab.kd[0] =  _pid_param_stab_KD_AIL;
        config.pid_param_stab.kd[1] =  _pid_param_stab_KD_ELV;
        config.pid_param_stab.kd[2] =  _pid_param_stab_KD_RUD;

        config.pinSpiCs = _pinSpiCs;
        config.pinSpiSck = _pinSpiSck;
        config.pinSpiMosi = _pinSpiMosi;
        config.pinSpiMiso = _pinSpiMiso;

    }
            
} 

void printConfigOffsets(){
    printf("\nOffset Values in config:\n");
	printf("Acc. X = %d, Y = %d, Z = %d\n", (int) config.accOffsetX , (int) config.accOffsetY, (int) config.accOffsetZ);    
    printf("Gyro. X = %d, Y = %d, Z = %d\n", (int) config.gyroOffsetX , (int) config.gyroOffsetY, (int) config.gyroOffsetZ);
}

void printFieldValues(){
    printf("\n");
    for (uint8_t i=0; i< NUMBER_MAX_IDX ;i++){
        if (fields[i].onceAvailable ){
            switch (i) {
                case LATITUDE:
                    printf("GPS Latitude = %.7f degree\n", ((float) fields[i].value) / 10000000.0);
                    break;
                case LONGITUDE:
                    printf("GPS Longitude = %.7f degree\n", ((float) fields[i].value) / 10000000.0);
                    break;
                case GROUNDSPEED:
                    printf("GPS Groundspeed = %d cm/s\n", (int) fields[i].value) ;
                    break;
                case HEADING:
                    printf("GPS Heading = %f degree\n", ((float) fields[i].value) / 100.0) ;
                    break;
                case ALTITUDE:
                    printf("GPS Altitude = %d cm\n", (int) fields[i].value) ;
                    break;
                case NUMSAT:
                    printf("GPS Num sat. = %d\n", (int) fields[i].value) ;
                    break;
                case GPS_DATE:
                    printf("GPS Date J M A = %d %d %d \n", (uint8_t) (fields[i].value >>8) , (uint8_t) (fields[i].value >> 16) ,
                        (uint8_t) (fields[i].value >> 24) ) ;
                    break;
                case GPS_TIME:
                    printf("GPS Time H M S = %d %d %d \n", (uint8_t) (fields[i].value >>24) , (uint8_t) (fields[i].value >> 16) ,
                        (uint8_t) (fields[i].value >> 8) ) ;
                    break;
                case GPS_PDOP:
                    printf("GPS Pdop = %d \n", (int) fields[i].value) ;
                    break;
                case GPS_HOME_BEARING:
                    printf("GPS Home bearing = %d degree\n", (int) fields[i].value) ;
                    break;
                case GPS_HOME_DISTANCE:
                    printf("GPS Home distance = %d m\n", (int) fields[i].value) ;
                    break;
                case MVOLT:
                    printf("Volt 1 = %d mVolt\n", (int) fields[i].value) ;
                    break;
                case CURRENT:
                    printf("Current (Volt 2) = %d mA\n", (int) fields[i].value) ;
                    break;
                case RESERVE1:
                    printf("Volt 3 = %d mVolt\n", (int) fields[i].value) ;
                    break;
                case RESERVE2:
                    printf("Volt 4 = %d mVolt\n", (int) fields[i].value) ;
                    break;        
                case CAPACITY:
                    printf("Capacity (using current) = %d mAh\n", (int) fields[i].value) ;
                    break;        
                case TEMP1:
                    printf("Temp 1 (Volt 3) = %d degree\n", (int) fields[i].value) ;
                    break;        
                case TEMP2:
                    printf("Temp 2 (Volt 4) = %d degree\n", (int) fields[i].value) ;
                    break;        
                case VSPEED:
                    printf("Vspeed = %d cm/s\n", (int) fields[i].value) ;
                    break;        
                case RELATIVEALT:
                    printf("Baro Rel altitude = %d cm\n", (int) fields[i].value) ;
                    break;        
                case PITCH:
                    printf("Pitch = %f degree\n", (float) fields[i].value * 0.01) ;
                    break;        
                case ROLL:
                    printf("Roll = %f degree\n", (float) fields[i].value * 0.01) ;
                    break;        
                case YAW:
                    printf("Yaw = %f degree\n", (float) fields[i].value * 0.01);
                    break;        
                case RPM:
                    printf("RPM = %d Hertz\n", (int) fields[i].value) ;
                    break;
                case ADS_1_1:
                    printf("Ads 1 1 = %d mVolt\n", (int) fields[i].value) ;
                    break;        
                case ADS_1_2:
                    printf("Ads 1 2 = %d mVolt\n", (int) fields[i].value) ;
                    break;        
                case ADS_1_3:
                    printf("Ads 1 3 = %d mVolt\n", (int) fields[i].value) ;
                    break;        
                case ADS_1_4:
                    printf("Ads 1 4 = %d mVolt\n", (int) fields[i].value) ;
                    break;        
                case ADS_2_1:
                    printf("Ads 2 1 = %d mVolt\n", (int) fields[i].value) ;
                    break;        
                case ADS_2_2:
                    printf("Ads 2 2 = %d mVolt\n", (int) fields[i].value) ;
                    break;        
                case ADS_2_3:
                    printf("Ads 2 3 = %d mVolt\n", (int) fields[i].value) ;
                    break;        
                case ADS_2_4:
                    printf("Ads 2 4 = %d mVolt\n", (int) fields[i].value) ;
                    break;
                case AIRSPEED:
                    printf("Airspeed = %d cm/s\n", (int) fields[i].value) ;
                    break;
                case AIRSPEED_COMPENSATED_VSPEED:
                    printf("Compensated Vspeed = %d cm/s\n", (int) fields[i].value) ;
                    break;
                case SBUS_HOLD_COUNTER:
                    printf("Sbus hold counter = %d\n", (int) fields[i].value) ;
                    break;
                case SBUS_FAILSAFE_COUNTER:
                    printf("Sbus failsafe counter = %d\n", (int) fields[i].value) ;
                    break;
                case GPS_CUMUL_DIST :
                    printf("Gps cumulative distance = %d\n", (int) fields[i].value) ;
                    break;
                case ACC_X :
                    printf("Acc X = %fg\n", (float) fields[i].value * 0.001) ;
                    break;
                case ACC_Y :
                    printf("Acc Y = %fg\n", (float) fields[i].value * 0.001) ;
                    break;
                case ACC_Z :
                    printf("Acc Z = %fg\n", (float) fields[i].value * 0.001) ;
                    break;
                case RESERVE3:
                    printf("Reserve 3 = %d\n", (int) fields[i].value) ;
                    break;
                case RESERVE4:
                    printf("Reserve 4 = %d\n", (int) fields[i].value) ;
                    break;
                case RESERVE5:
                    printf("Reserve 5 = %d\n", (int) fields[i].value) ;
                    break;
                case RESERVE6:
                    printf("Reserve 6 = %d\n", (int) fields[i].value) ;
                    break;
                case RESERVE7:
                    printf("Reserve 7 = %d\n", (int) fields[i].value) ;
                    break;
                            
            } // end switch
        }
    }
    if (config.VspeedCompChannel != 255){
        printf("Vspeed compensation = %.2f\n", dteCompensationFactor);
    }
    printf("pwmTop= %i\n",pwmTop);
}

void printPwmValues(){
    if ( lastRcChannels == 1){
        printf("PWM values are not available - no rc channels data have been received\n");
    } else {
        uint16_t c[16];
        c[0] = (uint16_t) sbusFrame.rcChannelsData.ch0 ;
        c[1] = (uint16_t) sbusFrame.rcChannelsData.ch1 ;
        c[2] = (uint16_t) sbusFrame.rcChannelsData.ch2 ;
        c[3] = (uint16_t) sbusFrame.rcChannelsData.ch3 ;
        c[4] = (uint16_t) sbusFrame.rcChannelsData.ch4 ;
        c[5] = (uint16_t) sbusFrame.rcChannelsData.ch5 ;
        c[6] = (uint16_t) sbusFrame.rcChannelsData.ch6 ;
        c[7] = (uint16_t) sbusFrame.rcChannelsData.ch7 ;
        c[8] = (uint16_t) sbusFrame.rcChannelsData.ch8 ;
        c[9] = (uint16_t) sbusFrame.rcChannelsData.ch9 ;
        c[10] = (uint16_t) sbusFrame.rcChannelsData.ch10 ;
        c[11] = (uint16_t) sbusFrame.rcChannelsData.ch11 ;
        c[12] = (uint16_t) sbusFrame.rcChannelsData.ch12 ;
        c[13] = (uint16_t) sbusFrame.rcChannelsData.ch13 ;
        c[14] = (uint16_t) sbusFrame.rcChannelsData.ch14 ;
        c[15] = (uint16_t) sbusFrame.rcChannelsData.ch15 ;
        printf("PWM values us (sbus) 1... 8 ");
        for (uint8_t i = 0; i<8; i++){
            printf(" %5d(%5d)", (int) fmap( c[i]  ), c[i]);
        }
        printf("\n");
        printf("PWM values us (sbus) 9...16 ");
        for (uint8_t i = 8; i<16; i++){
            printf(" %5d(%5d)", (int) fmap( c[i] ) , c[i]);
        }
        printf("\n");
        
    }
}

//********************************** Sequencer *****************************************
/*
For the sequencer we have to fill 2 tables
- one with 5 items per sequencer (key = "SEQ")
- one with 4 items per step (key = "STEP")
note : the table with steps can contain several sequences; 
we consider that a new sequence begins each time the first item (= channel range) changes
furthermore we consider that when the next first item is lower than the previous first item, then the steps become part of next sequencer
For each table, items are comma separated.
Each set of items starts with { and end with}
whitespaces are skipped
Whe define a function that takes 1 param : number of items to read (so e.g. {1,2,3,4})
the function read from a pointer up to a '0'(or the number of item) and return true if we find the right number of param.


*/

#define FLASH_SEQUENCER_OFFSET FLASH_CONFIG_OFFSET + (4 * 1024) // Sequencer is 4K after config parameters
const uint8_t *flash_sequencer_contents = (const uint8_t *) (XIP_BASE + FLASH_SEQUENCER_OFFSET);

uint8_t seqIdx = 0;        // count the sequencer
uint16_t sequenceIdx = 0;  // count the sequence
uint8_t stepIdx = 0;       // count the steps
SEQ_DEF seqDefsTemp[16];   // temporary structure to avoid any change to seq in case of error detected here
SEQ_STEP stepsTemp[SEQUENCER_MAX_NUMBER_OF_STEPS]; // temporary structure to avoid any change to seq in case of error detected here


void setupSequencers(){   // The config is uploaded at power on
    if (*flash_sequencer_contents == SEQUENCER_VERSION ) {
        memcpy( &seq , flash_sequencer_contents, sizeof(seq));
        seqDatasToUpload = true; // set a flag to update the table seqDatas[] when perfroming a checkSequencer()
        //printf("loaded param defsmax=%i  stepsMax=%i\n", seq.defsMax, seq.stepsMax); 
        //seq.defsMax = 0 ; // for testing only to be modified
        //seq.stepsMax = 0 ; // for testing only to be modified
    } else {
        seq.version = SEQUENCER_VERSION;
        seq.defsMax = 0 ;
        seq.stepsMax = 0 ; 
    }
} 

void checkSequencers(){
    // this function use flag (seqDatasToUpdate) to say if the table seqData has to be updated or not
    // it must be to be updated after a restart, a SEQ or a STEP command, not after a ENTER that only print the current config
    
    // set configIsValid = false when an error is detected
    watchdog_update(); //sleep_ms(500);
    if ( seq.defsMax == 0) {
        //printf("No sequencer defined\n");
        return ; // skip when sequencer are not defined
    }
    if (seq.stepsMax == 0) {
        printf("Error in sequencer steps: no steps defined while %i sequencers are defined\n", seq.defsMax);
        configIsValid = false;
        return;
    }
    uint8_t seqIdx = 0;  // index of current sequencer
    uint16_t sequenceIdx = 0; // index of current sequence
    uint16_t stepIdx = 0;   // index of current step
    uint16_t prevStepIdx = 0;
    uint8_t rangeNumber = 0; // used to check that each sequencer has at least 2 sequences 
    uint32_t currentSeqMillis = millisRp();
    CH_RANGE prevRange = seq.steps[stepIdx].chRange; 
    while (stepIdx < seq.stepsMax) {                     // process all steps
        //printf("Seq=%i   Step=%i  range=%i", (int) seqIdx+1, (int) stepIdx+1 , (int) rangeNumber );
        if (( seq.steps[stepIdx].nextSequencerBegin == 1 ) && (stepIdx > 0)) {   // When the next (not the first one) sequencer begin , close the current
            if ( rangeNumber < 2) {
                printf("Error in sequencer: only one sequence for sequencers %i\n", seqIdx+1);
                configIsValid = false;
                return;
            }
            if (seqDatasToUpload) {
                seqDatas[seqIdx].stepEndAtIdx = prevStepIdx;                 // store end of current sequencer
            }
            seqIdx++;                                                    // handle next sequencer
        }
        if (seq.steps[stepIdx].nextSequencerBegin == 1 ) {    // for each begin of sequencer 
            //if (seqIdx >= seq.defsMax) {
            //    printf("Error in sequencer: number of sequencers found in STEP exceeds number of sequencers defined in sequencer%i)\n",seq.defsMax );
            //    configIsValid = false;
            //    return;
            //}
            if (seqDatasToUpload) {
                // initilize seqDatas for new sequencer
                seqDatas[seqIdx].stepStartAtIdx = stepIdx;
                seqDatas[seqIdx].state = STOPPED;
                seqDatas[seqIdx].currentChValueUs = 0; // use a dummy channel value in order to force a change when a channel value will be received from Rx
                seqDatas[seqIdx].lastreceivedRange = dummy; // use a dummy channel value in order to force a change when a channel value will be received from Rx
                seqDatas[seqIdx].currentStepIdx = 0xFFFF;  // use a dummy value at startup (to detect when range )
                seqDatas[seqIdx].delayedStepIdx = 0xFFFF;  // use a dummy value to say that there is no delayed step.
                //seqDatas[seqIdx].lastActionAtMs = 0;
                seqDatas[seqIdx].lastOutputVal = seq.defs[seqIdx].defValue ; // set default value
                seqDatas[seqIdx].nextActionAtMs = 0; // 0 means that we have still to apply the default value      
            }
            rangeNumber = 1; // restat a new counting of the number of different RC values
            prevRange = seq.steps[stepIdx].chRange;    // set Prev equal to allow counting the number of different RC channel values (ranges) 
        }
        
        
        if ( seq.steps[stepIdx].chRange > prevRange) rangeNumber++; // Count the number of Rc channel values (range) for this sequencer
        
        if ( ( seq.steps[stepIdx].chRange < prevRange )  && (seq.steps[stepIdx].nextSequencerBegin == 0 )){  
            printf("Error in sequencer steps: in the same sequencer, Rc values of step n+1 (%i) must be >= to the value of step n\n", seqIdx+1);
            configIsValid = false;
            return;
        }
        if ( seq.defs[seqIdx].type == 1) { // when seq has type ANALOG, PWM value must be between 0/100  
            if (seq.steps[stepIdx].value < 0 || ( seq.steps[stepIdx].value > 100 && seq.steps[stepIdx].value != 127))  {
                printf("Error in sequencer steps: for sequencer %i  step %i, type is ANALOG(=1); PWM output value must then be in range 0/100 or 127(=stop)\n"\
                    ,seqIdx + 1 , stepIdx + 1 );
                configIsValid = false;
                return;
            }
        }
        prevRange = seq.steps[stepIdx].chRange ; 
        prevStepIdx = stepIdx;
        stepIdx++;
    } // end while
    if ( rangeNumber < 2) {
        printf("Error in sequencer steps: only one sequence for step item number %i\n",(int) stepIdx );
        configIsValid = false;
        return;
    }
    if( (seqIdx + 1) != seq.defsMax) {
        printf("Error in sequencer steps: number of sequencers (%i) detected in STEP do not match the number of sequencers in SEQ (%i)\n", seqIdx+1 , seq.defsMax);
        configIsValid = false;
        return;
    }
    if (seqDatasToUpload) {
        seqDatas[seqIdx].stepEndAtIdx = prevStepIdx; // for the last sequencer, register the last valid stepIdx
    }
    if (seq.steps[stepIdx].nextSequenceBegin == 1) sequenceIdx++; // count the number of sequence
    //sequencerIsValid =  true ; // no error in sequencer detected
    //printf("%i pins are controlled by a sequencer; setup is valid\n", seq.defsMax);   
    
    //#define DEBUG_PRINT_SEQDATAS
    #ifdef DEBUG_PRINT_SEQDATAS
        for (uint8_t i = 0; i<seq.defsMax;i++){
            printf("Start=%i End=%i chVal=%i state=%i range=%i first=%i currStep=%i smmoth=%i nextMs=%i outVal=%i\n",\
            seqDatas[i].stepStartAtIdx , seqDatas[i].stepEndAtIdx , (int) seqDatas[i].state , seqDatas[i].currentChValue , (int) seqDatas[i].currentRange, \
            seqDatas[i].firstStepIdx , seqDatas[i].currentStepIdx , seqDatas[i].smoothUpToMs ,\
            seqDatas[i].nextActionAtMs , seqDatas[i].lastOutputVal);
        }
        printf("\n");
    #endif
    seqDatasToUpload = false; // reset the flag asking for an update of table seqDatas[]
    watchdog_update(); //sleep_ms(500);
}

void printSequencers(){
    //printf("\nSequencer struct uses %i bytes\n", sizeof(seq));
    //printf("Sequencer def[] uses %i bytes\n", sizeof(seq.defs));
    //printf("Sequencer steps[] uses %i bytes\n", sizeof(seq.steps));
    watchdog_update(); //sleep_ms(500); // for tesing to be modified
    uint8_t seqIdx = 0; 
    if( seq.defsMax == 0 ){
        printf("\nNo sequencers are defined\n");
        return;
    }
    isPrinting = true;
    printf("\nNumber of: sequencers=%i   sequences=%i   steps= %i\n", seq.defsMax , seq.sequencesMax, seq.stepsMax);
    printf("Sequencer = [  Gpio  Type(0=servo,1=analog) Clock(msec) ChannelNr Default Min Max ]\n");
    printf("Sequence  = (  RC_value(-100...100)  to_Repeat  Uninterrupted   Only_priority_interrupted   is_a_Priority_seq  )\n");
    printf("Step      = {  Smooth(clocks) Pwm%(-100...100) Keep(clocks)  }\n");
    printf("SEQ=");
    for (uint16_t i = 0 ; i < seq.stepsMax; i++){
        // for each step, look if the step is the first of a sequencer
        if ( seq.steps[i].nextSequencerBegin == 1){
            printf("\n[ %i %i %i %i %i %i %i ] ", seq.defs[seqIdx].pin , (int) seq.defs[seqIdx].type , seq.defs[seqIdx].clockMs ,\
             seq.defs[seqIdx].channel , seq.defs[seqIdx].defValue , seq.defs[seqIdx].minValue , seq.defs[seqIdx].maxValue );
            seqIdx++;
        }
        if ( seq.steps[i].nextSequenceBegin == 1){
            printf("\n  ( %i", seq.steps[i].chRange);
            if (seq.steps[i].toRepeat == 1) { printf(" R");}
            if (seq.steps[i].neverInterrupted == 1) {printf(" U");}
            if (seq.steps[i].priorityInterruptOnly == 1) {printf(" O");}
            if (seq.steps[i].isPriority == 1) {printf(" P");}
            printf(" )\n    ");
        }
        printf("{%i %i %i} ", seq.steps[i].smooth , seq.steps[i].value , seq.steps[i].keep);             
    }
    printf("\n");    
    isPrinting = false;
}

void saveSequencers() {
    //sleep_ms(1000); // let some printf to finish
    //uint8_t buffer[FLASH_PAGE_SIZE] = {0xff};
    //memcpy(&buffer[0], &seq, sizeof(seq));
    // Note that a whole number of sectors must be erased at a time.
    // irq must be disable during flashing
    watchdog_enable(5000 , true);
    if ( multicoreIsRunning) multicore_lockout_start_blocking();
    uint32_t irqStatus = save_and_disable_interrupts();
    flash_range_erase(FLASH_SEQUENCER_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_SEQUENCER_OFFSET,  (uint8_t*) &seq, sizeof(seq));
    //flash_range_program(FLASH_SEQUENCER_OFFSET,  buffer, FLASH_PAGE_SIZE);
    
    restore_interrupts(irqStatus);
    if (multicoreIsRunning) multicore_lockout_end_blocking();
    //sleep_ms(1000);
    //printf("New config has been saved\n");
    //printConfig(); 
}

bool getAllSequencers(){  // try to get sequencer definition from a string pointed by pvalue; return true if valid
                          //  (then seq structure is modified)
                       // when true we still have to check if this is valid with config (for use of pins and with steps)
    seqIdx = 0;        // count the sequencer
    sequenceIdx = 0;  // count the sequence
    stepIdx = 0;       // count the steps
    
    while ( (*pvalue) != 0X00 ) { // while not end of value buffer
        if (seqIdx >= 16) {
            printf("Error : to many sequencer definitions; max is 16\n");
            return false; 
        }
        if ( parseOneSequencer() == false){ // this will also parse the sequences and the steps
            printf("Error converting the sequencer definition numner %i\n",seqIdx+1);
            return false; 
        }
        seqIdx++;    // count number of sequencers
    } // end while
    memcpy(&seq.defs , seqDefsTemp , sizeof(seqDefsTemp));  // copy all sequencer definitions
    memcpy(&seq.steps , stepsTemp, sizeof(stepsTemp));      // copy all sequence & steps definitions
    seq.defsMax = seqIdx;        // store the number of sequencers
    seq.sequencesMax = sequenceIdx ; // store the number of sequences
    seq.stepsMax = stepIdx;      // store the number of steps
    seqDatasToUpload = true;    // set a flag to force an upload of seqDatas[] during the check process.
    printf("Number of sequencers= %i   sequences=%i   steps=%i\n",seq.defsMax, seq.sequencesMax , seq.stepsMax);
    return true;
}


bool parseOneSequencer(){  // try to read a string  with n integer space separated set between [ ]
                            // and then try to get all sequences and steps for this sequencer
    char * ptr ;                        // get the pos of first non converted integer 
    pvalue =  skipWhiteSpace(pvalue);   // skip space at the begining
    if (( * pvalue) != '['){            // first char must be {
        printf("Error : sequencer must begin with [ \n");
        return false ;
    }
    nextSequencerBegin = 1;
    pvalue++;
    for (uint8_t i = 0 ; i < 7; i++) {       // try to convert 7 integers
        errno = 0;
        tempIntTable[i] =  strtol(pvalue , &ptr ,10);  // convert to integer starting from pvalue; ptr point to the first non converted char; skip whitespace before
                                                   // *ptr = 0 when no error
        if ( ( ptr == pvalue ) || ( ptr == NULL)) {
            printf("Error : parameter %i of sequencer %i can't be converted to an integers\n",i,seqIdx+1 );
            return false;    
        }
        pvalue = ptr; 
        //printf(" seq %i = %i\n", i , tempIntTable[i]);
    }
    pvalue =  skipWhiteSpace(pvalue);
    if (( * pvalue) != ']') {
        printf("Error : parameters of sequencer %i must end with ] after 7 values\n",seqIdx+1);
        return false ;
    }
    if (tempIntTable[0] < 0 || tempIntTable[0] > 15){
        printf("Error : for sequencer number %i, gpio must be in range 0 / 15\n", seqIdx+1);
        return false;
    }
    if (tempIntTable[1] < 0 || tempIntTable[1] > 1){
        printf("Error : for sequencer number %i, type must be 0 (SERVO) or 1(ANALOG)\n" , seqIdx+1);
        return false;
    }
    if (tempIntTable[2] < 20 || tempIntTable[2] > 100000){
        printf("Error : for sequencer number %i, clock must be in range 20 / 100000 (msec)\n", seqIdx+1);
        return false;
    }
    if (tempIntTable[3] < 1 || tempIntTable[3] > 16){
        printf("Error : for sequencer number %i, channel must be in range 1 / 16\n", seqIdx+1);
        return false;
    }
    if (tempIntTable[1] == 0) {  // for SERVO
        if (tempIntTable[4] < -125 || tempIntTable[4] > 125){
            printf("Error : for sequencer number %i, when type = SERVO, default PWM value must be in range -125 / +125\n", seqIdx+1);
            return false;
        }
    } else {                 // for ANALOG
        if (tempIntTable[4] < 0 || tempIntTable[4] > 100){
            printf("Error : for sequencer number %i, when type = ANALOG, default PWM value must be in range 0 / 100\n", seqIdx+1);
            return false;
        }
    }
    if (tempIntTable[1] == 0) {  // for SERVO
        if (tempIntTable[5] < -125 || tempIntTable[5] > 125){
            printf("Error : for sequencer number %i, when type = SERVO, min PWM value must be in range -125 / +125\n", seqIdx+1);
            return false;
        }
    } else {                 // for ANALOG
        if (tempIntTable[5] < 0 || tempIntTable[5] > 100){
            printf("Error : for sequencer number %i, when type = ANALOG, min PWM value must be in range 0 / 100\n", seqIdx+1);
            return false;
        }
    }if (tempIntTable[1] == 0) {  // for SERVO
        if (tempIntTable[6] < -125 || tempIntTable[6] > 125){
            printf("Error : for sequencer number %i, when type = SERVO, max PWM value must be in range -125 / +125\n", seqIdx+1);
            return false;
        }
    } else {                 // for ANALOG
        if (tempIntTable[6] < 0 || tempIntTable[6] > 100){
            printf("Error : for sequencer number %i, when type = ANALOG, max PWM value must be in range 0 / 100\n", seqIdx+1);
            return false;
        }
    }
    // here we have a valid set of parameter for a sequencer that we can save in a temp structure
    seqDefsTemp[seqIdx].pin = tempIntTable[0];
    seqDefsTemp[seqIdx].type = (SEQ_OUTPUT_TYPE) tempIntTable[1];
    seqDefsTemp[seqIdx].clockMs = tempIntTable[2];
    seqDefsTemp[seqIdx].channel = tempIntTable[3];
    seqDefsTemp[seqIdx].defValue = tempIntTable[4];
    seqDefsTemp[seqIdx].minValue = tempIntTable[5];
    seqDefsTemp[seqIdx].maxValue = tempIntTable[6];    
    pvalue++;
    pvalue =  skipWhiteSpace(pvalue);
    if (( * pvalue) != '(') {
        printf("Error : after [...] for sequencer %i there must a ( to define a sequence\n",seqIdx+1);
        return false ;
    }
    // parse all sequences and steps from one sequencer (up to the end or up to next sequencer)
    while ( (( * pvalue) != 0x00)  && (( * pvalue) != '[')) {    
        // for each sequence
        if ( parseOneSequence() == false ) {  // when true, stepsTemp will be filled for this sequencer
            return false;
        }
        sequenceIdx++;                       // prepare next sequence.
    }        
    return true;
}    

bool parseOneSequence() { // parse one sequence and all steps from this sequence (up to the end or up to next sequence or sequencer)
    // seqIdx, sequenceIdx, stepIdx and stepsTemp[] are used and updated
    char * ptr ;
    // look for one sequence and n steps    
    if(( * pvalue) != '(') {     // "(" is used to mark a new sequence
        printf("Error: expecting a sequence beginning with ( for sequencer %i\n", seqIdx+1);
        return false;
    }    
    nextSequenceBegin = 1;
    pvalue++;                       // skip '(' 
    pvalue =  skipWhiteSpace(pvalue);   // skip next spaces
    errno = 0;
    tempIntTable[0] =  strtol(pvalue , &ptr ,10);  // convert Rc value to integer 
                                                    // *ptr = 0 when no error
    if ((ptr == pvalue) || ( ptr == NULL)) {
        printf("Error: for sequence %i, parameter RC channel value can't be converted to an integer\n",sequenceIdx+1);
        return false;    
    }
    pvalue = ptr;
    if (tempIntTable[0] < -100 || tempIntTable[0] > 100){
        printf("Error: for sequence %i, Rc channel value must in range -100...100 (included)\n" , sequenceIdx+1);
        return false;    
    }
    //int  modulo10 = tempIntTable[0] % 10;
    //if (( modulo10 != 0) && ( modulo10 != 5) && ( modulo10 != -5) ) {
        if ( (tempIntTable[0] % 5) != 0  ) {
        printf("Error: for sequence %i, Rc channel value is % but must be a multiple of 5 (5,10,15,20,...100, -5,-10,-15,...-100)\n"
         , sequenceIdx+1 , tempIntTable[0]);
        //printf("wrong value is %i, modulo is %i\n", tempIntTable[0], tempIntTable[0] % 10);
        return false;    
    }
    tempIntTable[1] = 0; // set 4 optional flags to 0
    tempIntTable[2] = 0;
    tempIntTable[3] = 0;
    tempIntTable[4] = 0;
    for (uint8_t i=0;i<4;i++){ // search for a R, U O or P
        pvalue =  skipWhiteSpace(pvalue);   // skip next spaces
        if (( * pvalue) == 'R' ){
            tempIntTable[1]=1;
        } else if (( * pvalue) == 'U' ){
            tempIntTable[2]=1;
        } else if (( * pvalue) == 'O' ){
            tempIntTable[3]=1;
        } else if (( * pvalue) == 'P' ){
            tempIntTable[4]=1;
        } else if (( * pvalue) == ')' ){
            break;
        } else {
            printf("Error: sequence definition %i contains an invalid optional character (can only be R,U,O and/or P)\n", sequenceIdx+1);
            return false;
        }
        pvalue++;
        pvalue =  skipWhiteSpace(pvalue);   // skip next spaces  
    } // end for (4 optional sequence param)
    if ( (tempIntTable[2]==1) && (tempIntTable[3]==1)){
        printf("Error: in sequence definition %i: options U and O may not be used toegether\n",sequenceIdx+1);
        return false;    
    }
    pvalue =  skipWhiteSpace(pvalue);   // skip next spaces  
    if (( * pvalue) != ')'){            // sequence must end with )
        printf("Error: sequence definition %i must have a ) after max 4 optionnal characters\n",sequenceIdx+1);
        return false ;
    }
    pvalue++; // skip )
    pvalue =  skipWhiteSpace(pvalue);   // skip next spaces  
    if (( * pvalue) != '{'){            // a step befinning with { must exist after a sequence 
        printf("Error: after sequence definition %i we must have a steps beginning with {\n",sequenceIdx+1);
        return false ;
    }
    // parse one or several steps
    while ( (( * pvalue) != 0X00) && (( * pvalue) != '(') && (( * pvalue) != '[')) {     // get one or several steps
        if (parseOneStep() == false) {  // data are stored in stepsTemp[] 
            return false;
        }
        if (stepIdx == SEQUENCER_MAX_NUMBER_OF_STEPS) {
            printf("Error: to many steps; maximum is %i\n",SEQUENCER_MAX_NUMBER_OF_STEPS);
        return false ;
        }
        stepIdx++;   // prepare for next step
    }
    return true;
}

bool parseOneStep(){
    char * ptr;
    pvalue =  skipWhiteSpace(pvalue);   // skip next spaces  
    if (( * pvalue) != '{'){            // first char must be {
        printf("Error: in sequence %i, each step parameters must begin with { ; step=%i\n",sequenceIdx+1, stepIdx+1);
        return false ;
    }
    pvalue++;
    for (uint8_t i = 5 ; i < 8; i++) {
        errno = 0;
        tempIntTable[i] =  strtol(pvalue , &ptr ,10);  // convert to integer starting from pvalue; ptr point to the first non converted char; skip whitespace before
                                                // *ptr = 0 when no error
        if( ( ptr == pvalue ) || (ptr == NULL) ){
            printf("Error : for step %i, parameter %i can't be converted to an integer\n", stepIdx+1, i+1);
            return false;    
        }
        pvalue = ptr; 
        //printf(" seq %i = %i\n", i , tempIntTable[i]);
    }  // end for 
    // Check each of the 3 parameters                
    if (tempIntTable[5] < 0 || tempIntTable[5] > 255){
        printf("Error: for step %i, smooth must be in range 0 / 255 (included)\n" , stepIdx+1);
        return false;
    }
    if (tempIntTable[6] < -125 || tempIntTable[6] > 127 || tempIntTable[6] == 126)  {
        printf("Error: for step number %i, output value must be in range -125 / 125 (included) or 127 (stop at current position)\n", stepIdx+1);
        return false;
    }
    if (tempIntTable[5] >0 && tempIntTable[6] == 127)  {
        printf("Error: for step number %i, smooth must be 0 when output value is 127 (stop at current position)\n", stepIdx+1);
        return false;
    }
    if (tempIntTable[7] < 0 || tempIntTable[7] > 255){
        printf("Error: for step number %i, keep must be in range 0 / 255 (included)\n" , stepIdx+1);
        return false;
    }
    pvalue =  skipWhiteSpace(pvalue);
    if (( * pvalue) != '}') {
        printf("Error : for step %i, group of parameters must end with } after 3 values\n",stepIdx+1);
        return false ;
    }
    pvalue++;
    pvalue =  skipWhiteSpace(pvalue);   // skip next spaces  
    // here we have a valid step (that includes also the sequence parameters)
    // all parameters of one step have been processed and are valid; store them in temp
    stepsTemp[stepIdx].chRange = ( CH_RANGE) tempIntTable[0];
    stepsTemp[stepIdx].toRepeat = tempIntTable[1];
    stepsTemp[stepIdx].neverInterrupted =  tempIntTable[2];
    stepsTemp[stepIdx].priorityInterruptOnly =  tempIntTable[3];
    stepsTemp[stepIdx].isPriority =  tempIntTable[4];
    stepsTemp[stepIdx].smooth = tempIntTable[5];
    stepsTemp[stepIdx].value = tempIntTable[6];
    stepsTemp[stepIdx].keep = tempIntTable[7];
    stepsTemp[stepIdx].nextSequencerBegin = nextSequencerBegin;
    stepsTemp[stepIdx].nextSequenceBegin = nextSequenceBegin; 
    nextSequencerBegin = 0; // reset the flags
    nextSequenceBegin = 0;
    //printf("rc val= %i  R=%i  U=%i  O=%i  P=%i sm=%i pos=%i ke=%i nrb=%i nsb=%i\n", \
    //            stepsTemp[stepIdx].chRange , stepsTemp[stepIdx].toRepeat , stepsTemp[stepIdx].neverInterrupted , \
    //            stepsTemp[stepIdx].priorityInterruptOnly , stepsTemp[stepIdx].isPriority , stepsTemp[stepIdx].smooth,\
    //            stepsTemp[stepIdx].value , stepsTemp[stepIdx].keep ,\
    //            stepsTemp[stepIdx].nextSequencerBegin , stepsTemp[stepIdx].nextSequenceBegin );
    return true; 
}  // end of handling one step; 

void printGyro(){
    if(config.gyroChanControl>16) {
        printf("\nGyro is not configured\n");
        return;
    }
    printf("\nGyro configuration is:\n");
    printf("   Channels for :                     mode/gain=%i  ,  Ail stick=%i  ,  Elv stick=%i  ,  Rud stick=%i\n", \
        config.gyroChanControl , config.gyroChan[0]  , config.gyroChan[1]  , config.gyroChan[2] );
    printf("   Gain per axis (-128/127):                           Roll=%i          Pitch=%i         Yaw=%i\n", config.vr_gain[0] , config.vr_gain[1] , config.vr_gain[2]);
    printf("   Gain on throw :                    %i    (1=on full throw, 2=on half, 3=on quater)\n", (int)config.stick_gain_throw);
    printf("   Max rotate    :                    %i    (1=Very low , 2=low , 3=medium , 4=high)\n", (int) config.max_rotate);
    printf("   Stick rotate enabled in rate mode: %i    (1=disabled , 2=enabled)\n", (int) config.rate_mode_stick_rotate);
    if (config.gyroAutolevel) {
        printf("   Stabilize mode :               ON    (Hold mode is disabled)\n");
    } else {
        printf("   Stabilize mode :               OFF   (Hold mode is enabled)\n");
    }    
    printf("    PID         ---Roll(aileron)---         --Pitch(elevator)--        ----Yaw(rudder)----\n");
    printf("   Mode          Kp      Ki      Kd          Kp      Ki      Kd         Kp      Ki      Kd\n");
    printf("   Normal  PIDN= %-5i   %-5i   %-5i       %-5i   %-5i   %-5i      %-5i   %-5i   %-5i\n",\
            config.pid_param_rate.kp[0], config.pid_param_rate.ki[0],config.pid_param_rate.kd[0],\
            config.pid_param_rate.kp[1], config.pid_param_rate.ki[1],config.pid_param_rate.kd[1],\
            config.pid_param_rate.kp[2], config.pid_param_rate.ki[2],config.pid_param_rate.kd[2] );
    printf("   Hold    PIDH= %-5i   %-5i   %-5i       %-5i   %-5i   %-5i      %-5i   %-5i   %-5i\n",\
            config.pid_param_hold.kp[0], config.pid_param_hold.ki[0],config.pid_param_hold.kd[0],\
            config.pid_param_hold.kp[1], config.pid_param_hold.ki[1],config.pid_param_hold.kd[1],\
            config.pid_param_hold.kp[2], config.pid_param_hold.ki[2],config.pid_param_hold.kd[2] );
    printf("   Stab.   PIDS= %-5i   %-5i   %-5i       %-5i   %-5i   %-5i      %-5i   %-5i   %-5i\n",\
            config.pid_param_stab.kp[0], config.pid_param_stab.ki[0],config.pid_param_stab.kd[0],\
            config.pid_param_stab.kp[1], config.pid_param_stab.ki[1],config.pid_param_stab.kd[1],\
            config.pid_param_stab.kp[2], config.pid_param_stab.ki[2],config.pid_param_stab.kd[2] );    
    printGyroMixer();
}

void printGyroMixer(){     // this function is also called at the end of the gyroMixer calibration process
    if (gyroMixer.isCalibrated == false){
        printf("Gyro mixers must be calibrated\n");
    } else {
        // when calibrated
        printf("\nGyro mixers are calibrated:\n");
        printf("Sticks centered at:    Ail=%i%%   Elv=%i%%   Rud=%i%%\n", pc(gyroMixer.neutralUs[config.gyroChan[0]-1]-1500),\
            pc(gyroMixer.neutralUs[config.gyroChan[1]-1]-1500), pc(gyroMixer.neutralUs[config.gyroChan[2]-1]-1500));
        printf("Gyro corrections (from center pos in %%) on:\n");
        for (uint8_t i = 0; i<16;i++) {
            if ( gyroMixer.used[i]) {
                printf("Channel %-2i center=%-4i rollRight=%-4i rollLeft=%-4i pitchUp%-4i pitchDown=%-4i yawRight=%-4i yawLeft=%-4i min=%-4i max=%-4i\n",\
                i+1 , pc(gyroMixer.neutralUs[i]-1500) , pc(gyroMixer.rateRollRightUs[i]), pc(gyroMixer.rateRollLeftUs[i]), \
                pc(gyroMixer.ratePitchUpUs[i]) , pc(gyroMixer.ratePitchDownUs[i]),\
                pc(gyroMixer.rateYawRightUs[i]), pc(gyroMixer.rateYawLeftUs[i]),\
                pc(gyroMixer.minUs[i]-1500) , pc(gyroMixer.maxUs[i]-1500) ) ;
            }
        }
    }
}

#define FLASH_GYROMIXER_OFFSET FLASH_CONFIG_OFFSET + (8 * 1024) // Sequencer is 4K after config parameters
const uint8_t *flash_gyroMixer_contents = (const uint8_t *) (XIP_BASE + FLASH_GYROMIXER_OFFSET);

void saveGyroMixer() {
    //sleep_ms(1000); // let some printf to finish
    //uint8_t buffer[FLASH_PAGE_SIZE] = {0xff};
    //memcpy(&buffer[0], &seq, sizeof(seq));
    // Note that a whole number of sectors must be erased at a time.
    // irq must be disable during flashing
    watchdog_enable(5000 , true);
    if ( multicoreIsRunning) multicore_lockout_start_blocking();
    uint32_t irqStatus = save_and_disable_interrupts();
    flash_range_erase(FLASH_GYROMIXER_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_GYROMIXER_OFFSET,  (uint8_t*) &gyroMixer, sizeof(gyroMixer));
    //flash_range_program(FLASH_SEQUENCER_OFFSET,  buffer, FLASH_PAGE_SIZE);
    
    restore_interrupts(irqStatus);
    if (multicoreIsRunning) multicore_lockout_end_blocking();
    //sleep_ms(1000);
    //printf("New config has been saved\n");
    //printConfig(); 
}

void setupGyroMixer(){   // The config is uploaded at power on
    if (*flash_gyroMixer_contents == GYROMIXER_VERSION ) {
        memcpy( &gyroMixer , flash_gyroMixer_contents, sizeof(gyroMixer));        
    } else {
        gyroMixer.version = GYROMIXER_VERSION;
        gyroMixer.isCalibrated = false ;    
        for (uint8_t i=0; i<16 ; i++){ // set all mixer as unsued (for gyro) and with default values
            gyroMixer.used[i] = false;
            gyroMixer.neutralUs[i] = 1500 ;
            gyroMixer.minUs[i] = 1000 ;
            gyroMixer.maxUs[i] = 2000 ;
            gyroMixer.rateRollLeftUs[i] = 0 ; // 0 means that when stick is at the end pos, this output RC channel is not impacted 
            gyroMixer.rateRollRightUs[i] = 0 ; 
            gyroMixer.ratePitchUpUs[i] = 0 ;  
            gyroMixer.ratePitchDownUs[i] = 0 ;
            gyroMixer.rateYawLeftUs[i] = 0  ;
            gyroMixer.rateYawRightUs[i] = 0 ;   
        }
        
    }
} 

bool getPid(uint8_t mode){  // get all pid parameters for one mode; return true if valid; config is then updated
    // we expect getting 9 parameters (uint16) kp ki kd for ail, then for elv and for rud; all are space delimited
    char * ptr ;                        // get the pos of first non converted integer 
    pvalue =  skipWhiteSpace(pvalue);   // skip space at the begining
    int32_t tempTable[9];
    for (uint8_t i = 0 ; i < 9; i++) {       // try to convert 9 integers
        errno = 0;
        tempTable[i] =  strtol(pvalue , &ptr ,10);  // convert to integer starting from pvalue; ptr point to the first non converted char; skip whitespace before
                                                   // *ptr = 0 when no error
        if ( ( ptr == pvalue ) || ( ptr == NULL)) {
            printf("Error : PID must have 9 values; parameter %i is missing or can't be converted to an integer\n",i+1 );
            return false;    
        }
        if ((tempTable[i]>32000) or (tempTable[i]<-32000)){
            printf("Error : parameters %i of PID is not in range -32000/32000\n",i+1 );
            return false;        
        }
        pvalue = ptr; 
        //printf(" seq %i = %i\n", i , tempIntTable[i]);
    }
    pvalue =  skipWhiteSpace(pvalue);   // skip space at the begining
    if (( * pvalue) != 0 ){   // last char must be 0 (end of string) 
        printf("Error : more than 9 values detected for PID parameters\n");
        return false;
    }
    struct _pid_param tempPid;
    for (uint8_t i=0; i<3; i++){ //for ail, elv, rud)
        tempPid.kp[i] = tempTable[i*3];
        tempPid.ki[i] = tempTable[i*3+1];
        tempPid.kd[i] = tempTable[i*3+2];
    }
    if (mode == 0) { // normal
        tempPid.output_shift = _pid_param_rate_output_shift;
        memcpy(&config.pid_param_rate, &tempPid , sizeof(tempPid));
    } else if (mode == 1) { // hold
        tempPid.output_shift = _pid_param_hold_output_shift;
        memcpy(&config.pid_param_hold, &tempPid , sizeof(tempPid));
    } else {   // stab mode
        tempPid.output_shift = _pid_param_stab_output_shift;
        memcpy(&config.pid_param_stab, &tempPid , sizeof(tempPid));
    }
    return true;            
} 
