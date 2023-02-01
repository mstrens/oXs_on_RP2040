#include "tools.h"
#include "pico/stdlib.h"
#include "config.h"
#include "stdio.h"
#include "pico/util/queue.h"
#include "pico/multicore.h"

extern queue_t qSensorData; 



// useful for button but moved to button2.cpp
/////////////////////////////////////////////////////////////////
// Added by Mstrens to be able to get state of boot button on rp2040
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
// Picoboard has a button attached to the flash CS pin, which the bootrom
// checks, and jumps straight to the USB bootcode if the button is pressed
// (pulling flash CS low). We can check this pin in by jumping to some code in
// SRAM (so that the XIP interface is not required), floating the flash CS
// pin, and observing whether it is pulled low.
//
// This doesn't work if others are trying to access flash at the same time,
// e.g. XIP streamer, or the other core.

bool __no_inline_not_in_flash_func(get_bootsel_button)() {
    const uint CS_PIN_INDEX = 1;
    //startTimerUs(0);
    multicore_lockout_start_blocking();
    // Must disable interrupts, as interrupt handlers may be in flash, and we
    // are about to temporarily disable flash access!
    uint32_t flags = save_and_disable_interrupts();

    // Set chip select to Hi-Z
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    // Note we can't call into any sleep functions in flash right now
    //for (volatile int i = 0; i < 1000; ++i);

    // The HI GPIO registers in SIO can observe and control the 6 QSPI pins.
    // Note the button pulls the pin *low* when pressed.
    bool button_state = !(sio_hw->gpio_hi_in & (1u << CS_PIN_INDEX));

    // Need to restore the state of chip select, else we are going to have a
    // bad time when we return to code in flash!
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    restore_interrupts(flags);
    multicore_lockout_end_blocking();
    //getTimerUs(0);
    return button_state;
}

//#include "hardware/sync.h"
//#include "hardware/structs/ioqspi.h"
//#include "hardware/structs/sio.h"

//#include "Button2.h"  // already in tools.h

uint32_t millis(){
    return  to_ms_since_boot( get_absolute_time());
}

uint32_t micros() {
    return  to_us_since_boot(get_absolute_time ());
}

void waitUs(uint32_t delayUs){
    uint32_t nowUs = micros();
    while (( micros() - nowUs) < delayUs) {micros();}
}

void enlapsedTime(uint8_t idx){
    static uint32_t prevTime[10] = {0};
    uint32_t currTime;
    if (idx >= sizeof(prevTime)) return ;
    currTime = micros() ;
    printf("Eus%d=%d\n", idx , currTime-prevTime[idx]);
    prevTime[idx]=currTime;
}

uint32_t startAtUs[10] = {0};
void startTimerUs(uint8_t idx){
    if (idx >= sizeof(startAtUs)) return ;
    startAtUs[idx] = micros() ;
}

void getTimerUs(uint8_t idx){
    if (idx >= sizeof(startAtUs)) return ;
    printf("FSus%d=%d\n", idx , micros()-startAtUs[idx]);
}

void sent2Core0( uint8_t fieldType, int32_t value){
    queue_entry_t entry;
    entry.type = fieldType;
    entry.data = value ;
    queue_try_add(&qSensorData, &entry);
    //printf("sending %d = %10.0f\n", entry.type , (float) entry.data);
}


field fields[SPORT_TYPES_MAX];  // list of all telemetry fields and parameters used by Sport

void setupListOfFields(){
// list of fileds being used
    //latitude , longitude , groundspeed , heading , altitude ,  numSat
    //mVolt , current , temp1 ,  temp2 
    //vSpeed, pitch , roll , yaw
    //uplink_RSSI_1 , uplink_RSSI_2 , uplink_Link_quality , uplink_SNR , active_antenna , rf_Mode ,
    //uplink_TX_Power , downlink_RSSI , downlink_Link_quality , downlink_SNR, 
    
    // relativeAlt , RPM , CAPACITY, REMAIN,
    
    #ifndef SKIP_VOLT1_3_4
    uint16_t listFieldsID[SPORT_TYPES_MAX] = {GPS_LONG_LATI_FIRST_ID , GPS_LONG_LATI_FIRST_ID ,GPS_SPEED_FIRST_ID, GPS_COURS_FIRST_ID , GPS_ALT_FIRST_ID ,DIY_FIRST_ID  ,\
                          VFAS_FIRST_ID , CURR_FIRST_ID , DIY_FIRST_ID+1 , DIY_FIRST_ID+2  ,\
                          VARIO_FIRST_ID, ACCX_FIRST_ID , ACCY_FIRST_ID , ACCZ_FIRST_ID ,\
                          UPLINK_RSSI_1_ID , UPLINK_RSSI_2_ID , UPLINK_LINK_QUALITY_ID , UPLINK_SNR_ID , ACTIVE_ANTENNA_ID, RF_MODE_ID ,\
                          UPLINK_TX_POWER_ID , DOWNLINK_RSSI_ID , DOWNLINK_LINK_QUALITY_ID , DOWNLINK_SNR_ID ,\
                          ALT_FIRST_ID , RPM_FIRST_ID , FUEL_FIRST_ID ,  DIY_FIRST_ID+2 , GPS_TIME_DATE_FIRST_ID , GPS_TIME_DATE_FIRST_ID} ;
    #else
    
    uint16_t listFieldsID[SPORT_TYPES_MAX] = {GPS_LONG_LATI_FIRST_ID , GPS_LONG_LATI_FIRST_ID ,GPS_SPEED_FIRST_ID, GPS_COURS_FIRST_ID , GPS_ALT_FIRST_ID , T1_FIRST_ID  ,\
                          CURR_FIRST_ID , VFAS_FIRST_ID , T2_FIRST_ID , FUEL_FIRST_ID  ,\
                          VARIO_FIRST_ID, ACCX_FIRST_ID , ACCY_FIRST_ID , ACCZ_FIRST_ID ,\
                          UPLINK_RSSI_1_ID , UPLINK_RSSI_2_ID , UPLINK_LINK_QUALITY_ID , UPLINK_SNR_ID , ACTIVE_ANTENNA_ID, RF_MODE_ID ,\
                          UPLINK_TX_POWER_ID , DOWNLINK_RSSI_ID , DOWNLINK_LINK_QUALITY_ID , DOWNLINK_SNR_ID ,\
                          ALT_FIRST_ID , RPM_FIRST_ID } ;
    #endif
    uint8_t listdeviceID[SPORT_TYPES_MAX] = {DATA_ID_GPS, DATA_ID_GPS, DATA_ID_GPS, DATA_ID_GPS, DATA_ID_GPS , DATA_ID_GPS ,\
                            DATA_ID_FAS , DATA_ID_FAS , DATA_ID_FAS , DATA_ID_FAS ,\
                            DATA_ID_VARIO , DATA_ID_ACC , DATA_ID_ACC , DATA_ID_ACC ,\
                            DATA_ID_RPM , DATA_ID_RPM , DATA_ID_RPM , DATA_ID_RPM , DATA_ID_RPM , DATA_ID_RPM,\
                            DATA_ID_RPM , DATA_ID_RPM , DATA_ID_RPM , DATA_ID_RPM ,\
                            DATA_ID_VARIO , DATA_ID_RPM , DATA_ID_RPM , DATA_ID_RPM , DATA_ID_GPS , DATA_ID_GPS };
    uint16_t listInterval[SPORT_TYPES_MAX] = { 500, 500 , 500 , 500 , 500 ,500,\
                            500 , 500 , 500 , 500,\
                            300 , 300 , 300 , 300,\
                            500 , 500 , 500 , 500 , 500, 500,\
                            500 , 500 , 500 , 500 ,\
                            300 , 500 , 500 , 500 , 500 , 500};
    for (uint8_t i = 0 ;  i<sizeof(listdeviceID); i++){
        fields[i].value= 0;
        fields[i].available= false;
        fields[i].nextMillis= 0;
        fields[i].interval= listInterval[i];
        fields[i].deviceId= listdeviceID[i];
        fields[i].fieldId= listFieldsID[i];
    }
}





