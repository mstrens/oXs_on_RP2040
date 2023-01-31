#include "voltage.h"
#include "hardware/adc.h"
#include "tools.h"
#include "stdio.h"
#include <inttypes.h>
#include "param.h"

extern CONFIG config;
extern field fields[SPORT_TYPES_MAX];  // list of all telemetry fields and parameters used by Sport


VOLTAGE::VOLTAGE() {}

void VOLTAGE::begin(void ) {
    if ( config.pinVolt[0] == 255 and config.pinVolt[1] == 255 and config.pinVolt[2] == 255 and config.pinVolt[3] == 255 ) return ;
    adc_init(); // prepare ADC
    for (int cntInit = 0 ; cntInit < 4 ; cntInit++) {
        if ( config.pinVolt[cntInit] != 255) {
            adc_gpio_init(config.pinVolt[cntInit]); // prepare the pin for ADC
            mVolt[cntInit].value = 0;
            mVolt[cntInit].available = false ;
        }    
    } // end for
    // 330 because the max volt is 3.3V and we expect 2 decimals - to check if this is correct
    mVoltPerStep[0] = 0;  // 0 means that the value must not be transmitted (set scale = 0 to avoid sending the data)
    mVoltPerStep[1] = 0;
    mVoltPerStep[2] = 0;
    mVoltPerStep[3] = 0;
    if ( config.scaleVolt1 != 0) mVoltPerStep[0] = 330 / 4095.0  * config.scaleVolt1;
    if ( config.scaleVolt2 != 0) mVoltPerStep[1] = 330 / 4095.0  * config.scaleVolt2;
    if ( config.scaleVolt3 != 0) mVoltPerStep[2] = 330 / 4095.0  * config.scaleVolt3;
    if ( config.scaleVolt4 != 0) mVoltPerStep[3] = 330 / 4095.0  * config.scaleVolt4;  
    offset[0] = config.offset1; 
    offset[1] = config.offset2;
    offset[2] = config.offset3;
    offset[3] = config.offset4;
} // end begin

void VOLTAGE::getVoltages(void){
    static uint8_t sumCount = 0;
    static uint32_t lastVoltagemillis = 0 ;
    static uint32_t enlapsedMillis =0;
    float value;
    if ( config.pinVolt[0] == 255 and config.pinVolt[1] == 255 and config.pinVolt[2] == 255 and config.pinVolt[3] == 255 ) return ;
    enlapsedMillis = millis() - lastVoltagemillis; 
    if ( enlapsedMillis > VOLTAGEINTERVAL ) {
        lastVoltagemillis = millis() ;
        for (int cntInit = 0 ; cntInit < 4 ; cntInit++) {
            if ( config.pinVolt[cntInit] != 255) {
                adc_select_input(cntInit); // select the pin
                sumVoltage[cntInit] += adc_read(); // convert and sum
            }    
        }
        sumCount++;
        if ( sumCount == SUM_COUNT_MAX_VOLTAGE ) {
            sumCount = 0;
            for (int cntInit = 0 ; cntInit < 4 ; cntInit++) {
                if ( config.pinVolt[cntInit] != 255) {  // calculate average only if pin is defined  
                    //fields[cntInit + MVOLT].value = ( ((float) sumVoltage[cntInit]) / (( float) SUM_COUNT_MAX_VOLTAGE) * mVoltPerStep[cntInit]) - offset[cntInit];
                    if (mVoltPerStep[cntInit] !=0) {
                        value =  ( ((float) sumVoltage[cntInit]) / (( float) SUM_COUNT_MAX_VOLTAGE) * mVoltPerStep[cntInit]) - offset[cntInit];
                        sent2Core0( cntInit + MVOLT, (int32_t) value );
                        if (cntInit == 1) { // when we are calculating a current we calculate also the consumption
                            consumedMah += value * enlapsedMillis  / 3600000.0 ;  // in mah.
                            sent2Core0( CAPACITY, (int32_t) value );
                        }
                        //fields[cntInit + MVOLT].available = true ;
                    }
                    sumVoltage[cntInit] = 0 ;
                    //printf("voltage has been measured: %d value= %d \n", cntInit , (int) mVolt[cntInit].value);
                    
                }    
            }    
        }
    }        
}
