#include "hardware/pio.h"
#include "rpm.pio.h"
#include "tools.h"
#include "param.h"
#include <string.h>
#include <inttypes.h>
#include "stdio.h"
#include "rpm.h"


extern CONFIG config;
extern field fields[];  // list of all telemetry fields and parameters used by Sport


PIO pioRpm = pio1; // we use pio 1; 
uint smRpm = 2;  // we use the state machine 2 for rpm 

//#define PIN_RPM 29
#define RPM_COUNTER_INTERVAL_USEC 100000 // 100 msec  

uint32_t currentRpmUsec;
uint32_t previousRpmUsec;
uint32_t currentRpmCounter;
uint32_t previousRpmCounter;
float rpmScaling;


void setupRpm(){
    if (config.pinRpm == 255) return; // skip when pin is not defined
    setupRpmPio( );
    rpmScaling = 1000000.0 * config.rpmMultiplicator; // 1000000 = nbr of microsec in a sec
    //printf("RPM is set up\n");
}
void setupRpmPio(){
    // setup the PIO for RPM
        uint offsetRpm = pio_add_program(pioRpm, &rpm_program);
        rpm_program_init(pioRpm, smRpm, offsetRpm, config.pinRpm);
        previousRpmUsec = microsRp();
        previousRpmCounter = getRpmCounter();

}
void readRpm(){
    if (config.pinRpm == 255) return; // skip when pin is not defined
    //printf("read rpm\n");
    if (rpmScaling != 0.0){
        currentRpmUsec = microsRp();
        if ( ( currentRpmUsec - previousRpmUsec ) > RPM_COUNTER_INTERVAL_USEC ) {
            currentRpmCounter = getRpmCounter();
            uint32_t rpm = ((previousRpmCounter - currentRpmCounter) * rpmScaling ) / ( ( currentRpmUsec - previousRpmUsec )) ;
            //printf("PrevC= %" PRIu32 "  currentC=%" PRIu32 "  dif=%" PRIu32 "  interv=%" PRIu32 "  rpm=%" PRIu32 "\n", previousRpmCounter , currentRpmCounter ,
            //        (previousRpmCounter - currentRpmCounter) , ( currentRpmUsec - previousRpmUsec ), rpm);
            previousRpmUsec = currentRpmUsec;
            previousRpmCounter = currentRpmCounter;
            sent2Core0( RPM, (int32_t) rpm );       
        }
    }    
}

uint32_t getRpmCounter(){
    uint moveY2Isr = pio_encode_mov( pio_isr , pio_y); //(enum pio_src_dest dest, enum pio_src_dest src)
    pio_sm_exec (pioRpm, smRpm, moveY2Isr);// move Y reg to ISR
    uint pushIsr2Rxfifo = pio_encode_push(false,false); //(bool if_full, bool block)
    pio_sm_exec (pioRpm, smRpm, pushIsr2Rxfifo);// push ISR to RxFifo
    return pio_sm_get (pioRpm, smRpm); // read Rxfifo
}