
#include "param.h"
#include "pico/stdlib.h"
#include "config.h"
#include "stdio.h"
#include "MS5611.h"
#include "SPL06.h"
#include <string.h>
#include <ctype.h>
#include "gps.h"
#include "hardware/flash.h"
#include <inttypes.h>
#include "stdlib.h"
#include  "hardware/sync.h"
#include "hardware/watchdog.h"

#define CMD_BUFFER_LENTGH 80
uint8_t cmdBuffer[CMD_BUFFER_LENTGH];
uint8_t cmdBufferPos = 0;

extern GPS gps;
extern sbusFrame_s sbusFrame;
extern uint32_t lastRcChannels;

CONFIG config;

extern MS5611 baro1 ;
extern SPL06  baro2 ;


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
    double db;    
    char * pkey = NULL;
    char * pvalue = NULL;
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
    // change baudrate
    if ( strcmp("BAUD", pkey) == 0 ) { // if the key is BAUD
        ui = strtoul(pvalue, &ptr, 10);
        if ( *ptr != 0x0){
            printf("Error : baudrate must be a unsigned integer");
        } else {
            config.crsfBaudrate = ui;
            printf("baud = %" PRIu32 "\n" , config.crsfBaudrate);
            updateConfig = true;
        }
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
        } else  {
            printf("Error : protocol must be S (Sport) ,  C (CRSF=ELRS) or J (Jeti)\n");
        }
    }
    
    
    // change scale
    if (( strcmp("SCALE1", pkey) == 0 ) || ( strcmp("SCALE2", pkey) == 0 )\
         || ( strcmp("SCALE3", pkey) == 0 )  || ( strcmp("SCALE4", pkey) == 0 ) ){ 
        db = strtod(pvalue,&ptr);
        if (*ptr != 0x0) {
            printf("Error : value is not a valid float");
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
            printf("Error : value is not a valid float");
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
    
    // change gpio0
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
    if (updateConfig) saveConfig();
    if ( strcmp("A", pkey) == 0 ) printAttitudeFrame(); // print Attitude frame with vario data
    if ( strcmp("G", pkey) == 0 ) printGpsFrame();      // print GPS frame
    if ( strcmp("B", pkey) == 0 ) printBatteryFrame();   // print battery frame 
    printConfig();                                       // print the current config
    printf("\n >> \n ");
}


void printConfig(){
    uint8_t version[] =   VERSION ;
    printf("\nVersion = %s \n", version)  ;
    if (config.protocol == 'S'){
            printf("\nProtocol is Sport (Frsky)\n")  ;
        } else if (config.protocol == 'C'){
            printf("\nProtocol is CRSF (=ELRS)\n")  ;
        } else if (config.protocol == 'J'){
            printf("\nProtocol is Jeti (Ex)\n")  ;    
        } else {
            printf("\nProtocol is unknow\n")  ;
        }
    printf("CRSF baudrate = %" PRIu32 "\n", config.crsfBaudrate)  ;
    printf("Voltage parameters:\n")  ;
    printf("    Scales : %f , %f , %f , %f \n", config.scaleVolt1 , config.scaleVolt2 ,config.scaleVolt3 ,config.scaleVolt4 )  ;
    printf("    Offsets: %f , %f , %f , %f \n", config.offset1 , config.offset2 ,config.offset3 ,config.offset4 )  ;
    if (baro1.baroInstalled) {
        printf("Baro sensor is detected using MS5611\n")  ;
        printf("    Sensitivity min = %i (at %i)   , max = %i (at %i)\n", SENSITIVITY_MIN, SENSITIVITY_MIN_AT, SENSITIVITY_MAX, SENSITIVITY_MAX_AT);
        printf("    Hysteresis = %i \n", VARIOHYSTERESIS);        
    } else if (baro2.baroInstalled) {
        printf("Baro sensor is detected using SPL06\n")  ;
        printf("    Sensitivity min = %i (at %i)   , max = %i (at %i)\n", SENSITIVITY_MIN, SENSITIVITY_MIN_AT, SENSITIVITY_MAX, SENSITIVITY_MAX_AT);
        printf("    Hysteresis = %i \n", VARIOHYSTERESIS);        
    } else {
        printf("Baro sensor is not detected\n")  ;
    }   
    if (config.gpsType == 'U'){
            printf("Foreseen GPS type is Ublox  :")  ;
        } else if (config.gpsType == 'C'){
            printf("Foreseen GPS type is CADIS  :")  ;
        } else {
            printf("Foreseen GPS type is unknown  :")  ;
        }
    if (gps.gpsInstalled) {
        printf("GPS is detected\n")  ;
    } else {
        printf("GPS is not (yet) detected\n")  ;
    }
    if (config.gpio0 == 0){
        printf("GPIO0 is used to output a Sbus signal\n");
    } else if ( config.gpio0 < 17 ){
        printf("GPIO0 generates channel %u\n", (unsigned int) config.gpio0);
    } else {
        printf("GPIO0 : Error in configuration\n");
    }
    if (config.gpio1 > 0 && config.gpio1 < 17){
        printf("GPIO1 (and GPIO2, 3, 4) generates channel %u (and next)\n", (unsigned int) config.gpio1);
    } else {
        printf("GPIO1 : Error in configuration\n");
    }
    if (config.gpio5 > 0 && config.gpio5 < 17){
        printf("GPIO5 (and GPIO6, 7, 8) generates channel %u (and next)\n", (unsigned int) config.gpio5);
    } else {
        printf("GPIO5 : Error in configuration\n");
    }
    if (config.gpio11 > 0 && config.gpio11 < 17){
        printf("GPIO11 generates channel %u \n", (unsigned int) config.gpio11);
    } else {
        printf("GPIO11 : Error in configuration\n");
    }
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
    printf("\nCommands can be entered to change the config parameters\n");
    printf("-To change the protocol, for Sport enter PROTOCOL=S, for CRSF/ELRS enter PROTOCOL=C, for Jeti enter PROTOCOL=J\n");
    printf("-To change the CRSF baudrate, enter e.g. BAUD=420000\n");
    printf("-To change voltage scales, enter SCALEx=nnn.ddd e.g. SCALE1=2.3 or SCALE3=0.123\n")  ;
    printf("     Enter SCALEx=0 to avoid sending value x to the Transmitter (for Frsky or Jeti)\n")  ;
    printf("-To change voltage offset, enter OFFSETx=nnn.ddd e.g. OFFSET1=0.6789\n")  ;
    printf("-To change GPS type: for an Ublox, enter GPS=U and for a CADIS, enter GPS=C\n");
    printf("-To select the signal generated on:\n");
    printf("     GPIO0 : enter GPIO0=SBUS or GPIO0=xx where xx = 01 up to 16\n");
    printf("     GPIO1 : enter GPIO1=xx where xx = 01 up to 13 (GPIO2...4 will generate channel xx+1...3)\n");
    printf("     GPIO5 : enter GPIO5=xx where xx = 01 up to 13 (GPIO6...8 will generate channel xx+1...3)\n");
    printf("     GPIO11: enter GPIO11=xx where xx = 01 up to 16\n");
    printf("-To select the failsafe mode to HOLD, enter FAILSAFE=H\n")  ;
    printf("-To set the failsafe values on the current position, enter SETFAILSAFE\n")  ;
    printf("   Note: some changes require a reset to be applied\n"); 
}


#define FLASH_TARGET_OFFSET (256 * 1024)
const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

void saveConfig() {
    uint8_t buffer[FLASH_PAGE_SIZE] = {0xff};
    memcpy(&buffer[0], &config, sizeof(config));
    // Note that a whole number of sectors must be erased at a time.
    // irq must be disable during flashing
    uint32_t irqStatus = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, buffer, FLASH_PAGE_SIZE);
    restore_interrupts(irqStatus);
    printf("New config has been saved\n");
    watchdog_reboot(0, 0, 10000);
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
        config.gpio0 = 0;
        config.gpio1 = 1;
        config.gpio5 = 6;
        config.gpio11 = 11;
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
    }
    
} 
