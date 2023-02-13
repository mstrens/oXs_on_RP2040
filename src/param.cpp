
#include "param.h"
#include "pico/stdlib.h"
#include "config.h"
#include "stdio.h"
#include "MS5611.h"
#include "SPL06.h"
#include "BMP280.h"
#include "ads1115.h"
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
// PROTOCOL = C, F, J
// for RP2040_zero, pin 16 = LED
// When no config is found in memory, a default config is loaded (defined in config.h)
// When a pin is not used, value = 0xFF
// When a pin is used twice, the config is not valid and a LED will blink Red 
// to store the pins, variable will be pinChannels[16], pinGpsTx, pinGpsRx, pinPrimIn, pinSecIn, 
//                                     pinSbusOut,pinTlm, pinVolt[4]  pinSda, pinScl,pinRpm, pinLed


#define CMD_BUFFER_LENTGH 80
uint8_t cmdBuffer[CMD_BUFFER_LENTGH];
uint8_t cmdBufferPos = 0;

extern GPS gps;
extern sbusFrame_s sbusFrame;
extern uint32_t lastRcChannels;

CONFIG config;
uint8_t debugTlm = 'N';
uint8_t debugSbusOut = 'N';

uint8_t pinCount[30] = {0};
extern bool configIsValid; 

extern field fields[];  // list of all telemetry fields and parameters used by Sport

extern MS5611 baro1 ;
extern SPL06  baro2 ;
extern BMP280 baro3 ; 

extern ADS1115 adc1 ;
extern ADS1115 adc2 ;    

extern MPU mpu;
extern queue_t qSendCmdToCore1;


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

void processCmd(){
    printf("processing cmd\n");
    bool updateConfig = false;
    char *ptr;
    uint32_t ui;
    uint32_t ui2;
    double db;    
    char * pkey = NULL;
    char * pvalue = NULL;
    //printf("buffer0= %X\n", cmdBuffer[0]);
    if (cmdBuffer[0] == 0x0D){ // when no cmd is entered we print the current config
        
    }
    if (cmdBuffer[0] == '?'){ // when help is requested we print the instruction
        printf("\nCommands can be entered to change the config parameters\n");
        printf("- To activate a function, select the pin and enter function code = pin number (e.g. PRI=1)\n");
        printf("    Function                  Code        Valid pins number\n");   
        printf("    Primary channels input    PRI     = 5, 9, 21, 25\n");
        printf("    Secondary channels input  SEC     = 1, 13, 17, 29\n");
        printf("    Telemetry                 TLM     = 0, 1, 2, ..., 29\n");
        printf("    GPS Rx                    GPS_RX  = 0, 1, 2, ..., 29\n");
        printf("    GPS Tx                    GPS_TX  = 0, 1, 2, ..., 29\n");
        printf("    Sbus OUT                  SBUS_OUT= 0, 1, 2, ..., 29\n");
        printf("    RPM (only for Sport)      RPM     = 0, 1, 2, ..., 29\n");
        printf("    SDA (baro sensor)         SDA     = 2, 6, 10, 14, 18, 22, 26\n");
        printf("    SCL (baro sensor)         SCL     = 3, 7, 11, 15, 19, 23, 27\n");
        printf("    PWM Channels 1, ..., 16   C1 / C16= 0, 1, 2, ..., 15\n");
        printf("    Voltage 1, ..., 4         V1 / V4 = 26, 27, 28, 29\n");
        printf("- To disable a function, set pin number to 255\n\n");

        printf("-To debug on USB/serial the telemetry frames, enter DEBUGTLM=Y or DEBUGTLM=N (default)\n");
        printf("-To change the protocol, enter PROTOCOL=x where x=S for Sport, C for CRSF/ELRS, J for Jeti, H for Hott or M for Mpx\n");
        printf("-To change the CRSF baudrate, enter e.g. BAUD=420000\n");
        printf("-To change voltage scales, enter SCALEx=nnn.ddd e.g. SCALE1=2.3 or SCALE3=0.123\n")  ;
        printf("     Enter SCALEx=0 to avoid sending voltage x to the Transmitter (for Frsky or Jeti)\n")  ;
        printf("-If a TMP36 is used on V3, enter TEMP=1 (if a second one is on V4, enter TEMP=2)");
        printf("-To change voltage offset, enter OFFSETx=nnn.ddd e.g. OFFSET1=0.6789\n")  ;
        printf("-To change GPS type: for an Ublox, enter GPS=U and for a CADIS, enter GPS=C\n");
        printf("-To change RPM multiplicator, enter e.g. RPM_MULT=0.5 to divide RPM by 2\n");
        printf("-To force a calibration of MP6050, enter MPUCAL\n");
    //    printf("-To select the signal generated on:\n");
    //    printf("     GPIO0 : enter GPIO0=SBUS or GPIO0=xx where xx = 01 up to 16\n");
    //    printf("     GPIO1 : enter GPIO1=xx where xx = 01 up to 13 (GPIO2...4 will generate channel xx+1...3)\n");
    //    printf("     GPIO5 : enter GPIO5=xx where xx = 01 up to 13 (GPIO6...8 will generate channel xx+1...3)\n");
    //    printf("     GPIO11: enter GPIO11=xx where xx = 01 up to 16\n");
        printf("-To select the failsafe mode to HOLD, enter FAILSAFE=H\n")  ;
        printf("-To set the failsafe values on the current position, enter SETFAILSAFE\n")  ;
        printf("-To get the current config, just press Enter\n");
        printf("   Note: some changes require a reset to be applied (e.g. to unlock I2C bus)\n");
        return;  
    }
    if (cmdBuffer[0] != 0x0){
        char * equalPos = strchr( (char*)cmdBuffer, '=');
        
        if (equalPos != NULL){ // there is = so search for value
            *equalPos = 0x0;
            equalPos++;
            pvalue = skipWhiteSpace(equalPos);
            removeTrailingWhiteSpace(pvalue);
        }    
        pkey =  skipWhiteSpace((char*)cmdBuffer);
        removeTrailingWhiteSpace(pkey);
    }
    upperStr(pkey);
    upperStr(pvalue);
    printf("\nCmd to execute: ");   
    if (pkey) printf("  %s", pkey);
    if (pvalue) printf("=%s", pvalue);
    printf("\n");
    // change PRI pin
    if ( strcmp("PRI", pkey) == 0 ) { 
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : pin must be an unsigned integer\n");
        } else if ( !(ui == 5 or ui == 21 or ui == 9 or ui ==25 or ui ==255)) {
            printf("Error : pin must be 5 ,21 , 9 , 25 or 255\n");
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
        requestMpuCalibration();
        return; // do not continue in order to avoid printing config while config print some data too.
        /*
        db = strtod(pvalue,&ptr);
        if (*ptr != 0x0) {
            printf("Error : value is not a valid float\n");
        } else {
            config.rpmMultiplicator = db;
            updateConfig = true;
        }
        */
    }
    
    // change for channels
    if ( *pkey == 'C' ) {
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
    if ( *pkey == 'V' ) {
        pkey++; // skip 'C' char to extract the digits
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
        
    // change baudrate
    if ( strcmp("BAUD", pkey) == 0 ) { // if the key is BAUD
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : baudrate must be an unsigned integer\n");
        } else {
            config.crsfBaudrate = ui;
            printf("baud = %" PRIu32 "\n" , config.crsfBaudrate);
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
    

    // change protocol
    if ( strcmp("PROTOCOL", pkey) == 0 ) { // if the key is BAUD
        if (strcmp("S", pvalue) == 0) {
            config.protocol = 'S';
            updateConfig = true;
        } else if (strcmp("C", pvalue) == 0) {
            config.protocol = 'C';
            updateConfig = true;
        } else if (strcmp("J", pvalue) == 0) {
            config.protocol = 'J';
            updateConfig = true;
        } else if (strcmp("H", pvalue) == 0) {
            config.protocol = 'H';
            updateConfig = true;
        } else if (strcmp("M", pvalue) == 0) {
            config.protocol = 'M';
            updateConfig = true;
        } else  {
            printf("Error : protocol must be S (Sport), C (CRSF=ELRS), J (Jeti), H (Hott) or M (Mpx)\n");
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
        } else if (strcmp("C", pvalue) == 0) {
            config.gpsType = 'C';
            updateConfig = true;
        } else  {
            printf("Error : GPS type must be U or C\n");
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
    if (updateConfig) {
        saveConfig();
        printf("config has been saved\n");  
        printf("Device will reboot\n\n");
        watchdog_enable(1500,false);
        sleep_ms(1000);
        watchdog_reboot(0, 0, 100); // this force a reboot!!!!!!!!!!
    }    
    if ( strcmp("A", pkey) == 0 ) printAttitudeFrame(); // print Attitude frame with vario data
    if ( strcmp("G", pkey) == 0 ) printGpsFrame();      // print GPS frame
    if ( strcmp("B", pkey) == 0 ) printBatteryFrame();   // print battery frame 
    printConfig();                                       // print the current config
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

void checkConfig(){
    // each pin can be used only once
    // if SDA is defined SCL must be defined too, and the opposite
    // if GPS_TX is defined GPS_RX must be defined too and the opposite
    bool atLeastOnePwmPin = false;
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
    for (uint8_t i = 0 ; i<16 ; i++) {addPinToCount(config.pinChannels[i]);}
    for (uint8_t i = 0 ; i<16 ; i++) {
        if (config.pinChannels[i] != 255) atLeastOnePwmPin = true ;}
    for (uint8_t i = 0 ; i<4 ; i++) {addPinToCount(config.pinVolt[i]);}
    for (uint8_t i = 0 ; i<30; i++) {
        if (pinCount[i] > 1) {
            printf("Error in parameters: pin %u is used %u times\n", i , pinCount[i]);
            configIsValid=false;
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
    if ( configIsValid == false) {
        printf("\nAttention: error in config parameters\n");
    } else {
        printf("\nConfig parameters are OK\n");
    }
    printf("Press ? + Enter to get help about the commands\n");
}

void printConfig(){
    uint8_t version[] =   VERSION ;
    printf("\nVersion = %s \n", version)  ;
    printf("    Function                Pin   Change entering XXX=yyy (yyy=255 to disable)\n");   
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
    
    if (config.protocol == 'S'){
            printf("\nProtocol is Sport (Frsky)\n")  ;
        } else if (config.protocol == 'C'){
            printf("\nProtocol is CRSF (=ELRS)\n")  ;
        } else if (config.protocol == 'J'){
            printf("\nProtocol is Jeti (Ex)\n")  ;    
        } else if (config.protocol == 'H'){
            printf("\nProtocol is Hott\n")  ;    
        } else if (config.protocol == 'M'){
            printf("\nProtocol is Mpx\n")  ;    
        } else {
            printf("\nProtocol is unknow\n")  ;
        }
    printf("CRSF baudrate = %" PRIu32 "\n", config.crsfBaudrate)  ;
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
    if(mpu.mpuInstalled){
        printf("Acc/Gyro is detected using MP6050\n")  ;
        printf("     Acceleration offsets X, Y, Z = %i , %i , %i\n", config.accOffsetX , config.accOffsetY , config.accOffsetZ);
        printf("     Gyro offsets         X, Y, Z = %i , %i , %i\n", config.gyroOffsetX , config.gyroOffsetY , config.gyroOffsetZ); 
    } else {
       printf("Acc/Gyro is not detected\n")  ;     
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
            printf("Foreseen GPS type is Ublox  :")  ;
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
    printf("     Chan 1...4  = %" PRIu32 " , %" PRIu32 " , %" PRIu32 " , %" PRIu32 "\n", (uint32_t) config.failsafeChannels.ch0\
                                                    , (uint32_t) config.failsafeChannels.ch1\
                                                    , (uint32_t) config.failsafeChannels.ch2\
                                                    , (uint32_t) config.failsafeChannels.ch3);
    printf("     Chan 5...8  = %" PRIu32 " , %" PRIu32 " , %" PRIu32 " , %" PRIu32 "\n", (uint32_t) config.failsafeChannels.ch4\
                                                    , (uint32_t) config.failsafeChannels.ch5\
                                                    , (uint32_t) config.failsafeChannels.ch6\
                                                    , (uint32_t) config.failsafeChannels.ch7);
    printf("     Chan 9...12 = %" PRIu32 " , %" PRIu32 " , %" PRIu32 " , %" PRIu32 "\n", (uint32_t) config.failsafeChannels.ch8\
                                                    , (uint32_t) config.failsafeChannels.ch9\
                                                    , (uint32_t) config.failsafeChannels.ch10\
                                                    , (uint32_t) config.failsafeChannels.ch11);
    printf("     Chan 13...16= %" PRIu32 " , %" PRIu32 " , %" PRIu32 " , %" PRIu32 "\n", (uint32_t) config.failsafeChannels.ch12\
                                                    , (uint32_t) config.failsafeChannels.ch13\
                                                    , (uint32_t) config.failsafeChannels.ch14\
                                                    , (uint32_t) config.failsafeChannels.ch15);

    }
    checkConfig();

}


#define FLASH_TARGET_OFFSET (256 * 1024)
const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

void saveConfig() {
    //sleep_ms(1000); // let some printf to finish
    uint8_t buffer[FLASH_PAGE_SIZE] = {0xff};
    memcpy(&buffer[0], &config, sizeof(config));
    // Note that a whole number of sectors must be erased at a time.
    // irq must be disable during flashing
    watchdog_enable(3000 , true);
    multicore_lockout_start_blocking();
    uint32_t irqStatus = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, buffer, FLASH_PAGE_SIZE);
    restore_interrupts(irqStatus);
    multicore_lockout_end_blocking();
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
    } else {
        config.version = CONFIG_VERSION;
        for (uint8_t i=0 ; i<16 ; i++) { config.pinChannels[i] = 0XFF; }
        config.pinGpsTx = 0xFF;
        config.pinGpsRx = 0xFF;
        config.pinPrimIn = 0xFF;
        config.pinSecIn = 0xFF; 
        config.pinSbusOut = 0xFF;
        config.pinTlm = 0xFF;
        for (uint8_t i=0 ; i<4 ; i++) { config.pinVolt[i] = {0xFF}; }
        config.pinSda = 0xFF;
        config.pinScl = 0xFF;
        config.pinRpm = 0xFF;
        config.pinLed = 16;
        config.protocol = 'S'; // default = sport
        config.crsfBaudrate = 420000;
        config.scaleVolt1 = 1.0;
        config.scaleVolt2 = 1.0;
        config.scaleVolt3 = 1.0;
        config.scaleVolt4 = 1.0;
        config.offset1 = 0.0;
        config.offset2 = 0.0;
        config.offset3 = 0.0;
        config.offset4 = 0.0;
        config.gpsType = 'U' ;
        config.rpmMultiplicator = 1.0;
        //config.gpio0 = 0;
        //config.gpio1 = 1;
        //config.gpio5 = 6;
        //config.gpio11 = 11;
        config.failsafeType = 'H';
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
        config.temperature = 255;
    }
    
} 


void requestMpuCalibration()  // 
{
    if (!mpu.mpuInstalled) {
        printf("Calibration not done: no MP6050 installed\n");
        return ;
    }
    uint8_t data = 0X01; // 0X01 = execute calibration
    printf("Before calibration:\n");
    printConfigOffsets();
    sleep_ms(1000); // wait that message is printed
    queue_try_add(&qSendCmdToCore1 , &data);

}    

void printConfigOffsets(){
    printf("\nOffset Values in config:\n");
	printf("Acc. X = %d, Y = %d, Z = %d\n", config.accOffsetX , config.accOffsetY, config.accOffsetZ);    
    printf("Gyro. X = %d, Y = %d, Z = %d\n", config.gyroOffsetX , config.gyroOffsetY, config.gyroOffsetZ);
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
                    printf("GPS Groundspeed = %d cm/s\n", fields[i].value) ;
                    break;
                case HEADING:
                    printf("GPS Heading = %f degree\n", ((float) fields[i].value) / 100.0) ;
                    break;
                case ALTITUDE:
                    printf("GPS Altitude = %d cm\n", fields[i].value) ;
                    break;
                case NUMSAT:
                    printf("GPS Num sat. = %d cm\n", fields[i].value) ;
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
                    printf("GPS Pdop = %d \n", fields[i].value) ;
                    break;
                case GPS_HOME_BEARING:
                    printf("GPS Home bearing = %d degree\n", fields[i].value) ;
                    break;
                case GPS_HOME_DISTANCE:
                    printf("GPS Home distance = %d m\n", fields[i].value) ;
                    break;
                case MVOLT:
                    printf("Volt 1 = %d mVolt\n", fields[i].value) ;
                    break;
                case CURRENT:
                    printf("Current (Volt 2) = %d mA\n", fields[i].value) ;
                    break;
                case RESERVE1:
                    printf("Volt 3 = %d mVolt\n", fields[i].value) ;
                    break;
                case RESERVE2:
                    printf("Volt 4 = %d mVolt\n", fields[i].value) ;
                    break;        
                case CAPACITY:
                    printf("Capacity (using current) = %d mAh\n", fields[i].value) ;
                    break;        
                case TEMP1:
                    printf("Temp 1 (Volt 3) = %d degree\n", fields[i].value) ;
                    break;        
                case TEMP2:
                    printf("Temp 2 (Volt 4) = %d degree\n", fields[i].value) ;
                    break;        
                case VSPEED:
                    printf("Vspeed = %d cm/s\n", fields[i].value) ;
                    break;        
                case RELATIVEALT:
                    printf("Baro Rel altitude = %d cm\n", fields[i].value) ;
                    break;        
                case PITCH:
                    printf("Pitch = %d degree\n", fields[i].value) ;
                    break;        
                case ROLL:
                    printf("Roll = %d degree\n", fields[i].value) ;
                    break;        
                case YAW:
                    printf("Yaw = %d degree\n", fields[i].value) ;
                    break;        
                case RPM:
                    printf("RPM = %d Hertz\n", fields[i].value) ;
                    break;        
            } // end switch
        }
    }
}
