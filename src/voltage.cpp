#include "voltage.h"
#include "hardware/adc.h"
#include "tools.h"
#include "stdio.h"
#include <inttypes.h>
#include "param.h"
#include "math.h"

extern CONFIG config;
extern field fields[];  // list of all telemetry fields and parameters used by Sport


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
    // 330 because the max volt is 3.3V and we expect 3 decimals to get it in mvolt
    mVoltPerStep[0] = 0;  // 0 means that the value must not be transmitted (set scale = 0 to avoid sending the data)
    mVoltPerStep[1] = 0;
    mVoltPerStep[2] = 0;
    mVoltPerStep[3] = 0;
    if ( config.scaleVolt1 != 0) mVoltPerStep[0] = 3300 / 4095.0  * config.scaleVolt1;
    if ( config.scaleVolt2 != 0) mVoltPerStep[1] = 3300 / 4095.0  * config.scaleVolt2;
    if ( config.scaleVolt3 != 0) mVoltPerStep[2] = 3300 / 4095.0  * config.scaleVolt3;
    if ( config.scaleVolt4 != 0) mVoltPerStep[3] = 3300 / 4095.0  * config.scaleVolt4;  
    offset[0] = config.offset1; 
    offset[1] = config.offset2;
    offset[2] = config.offset3;
    offset[3] = config.offset4;
} // end begin

void VOLTAGE::getVoltages(void){
    static uint16_t sumCount = 0;
    static uint32_t lastVoltageMicros = 0 ;
    uint32_t enlapsedMicros =0;
    static uint32_t lastConsumedMillis = millisRp();
    float value;
    uint16_t adcValue;
    static uint8_t adcSeq = 0; // sequence 0...3 of pin to be read 
    if ( config.pinVolt[0] == 255 and config.pinVolt[1] == 255 and config.pinVolt[2] == 255 and config.pinVolt[3] == 255 ) return ;
    if ( (microsRp() - lastVoltageMicros) > VOLTAGEINTERVAL ) {  // performs one conversion every X usec
        lastVoltageMicros = microsRp() ;
        if ( config.pinVolt[adcSeq] != 255) {
            adc_read(); // convert and sum
            //printf("V=%i\n",(int) adc_read());
            adcValue = adc_read();
//            if (adcValue < adcMin[adcSeq]) adcMin[adcSeq] = adcValue;
//            if (adcValue > adcMax[adcSeq]) adcMax[adcSeq] = adcValue;
            sumVoltage[adcSeq] += adcValue; // convert and sum
            adcValue = adc_read();
//            if (adcValue < adcMin[adcSeq]) adcMin[adcSeq] = adcValue;
//            if (adcValue > adcMax[adcSeq]) adcMax[adcSeq] = adcValue;
            sumVoltage[adcSeq] += adcValue; // convert and sum
        }     
        adcSeq = (adcSeq + 1) & 0X03 ; // increase seq and keep in range 0..3
        if ( config.pinVolt[adcSeq] != 255) {
            adc_select_input( config.pinVolt[adcSeq] - FIRST_ANALOG_PIN); // select the pin (conversion is done later on when voltage is stabilized)
            //printf("a=%i p=%i V=%i\n", adcSeq, config.pinVolt[adcSeq] , (int) adc_read());
            
        }
    
        sumCount++;
        if ( (sumCount >> 2) == SUM_COUNT_MAX_VOLTAGE ) {   // after XX conversions, calculates averages
            sumCount = 0;
            for (int cntInit = 0 ; cntInit < 4 ; cntInit++) {
                if ( config.pinVolt[cntInit] != 255) {  // calculate average only if pin is defined  
                    //fields[cntInit + MVOLT].value = ( ((float) sumVoltage[cntInit]) / (( float) SUM_COUNT_MAX_VOLTAGE) * mVoltPerStep[cntInit]) - offset[cntInit];
                    if (mVoltPerStep[cntInit] !=0) {
                        value =  ( ((float) sumVoltage[cntInit]) / (( float) SUM_COUNT_MAX_VOLTAGE * 2.0) * mVoltPerStep[cntInit])\
                                 - offset[cntInit];
                        // Volt3 and Volt 4 can be used as temperature or voltage depending on value of config.temperature
                        // volt 2 is used for current and consumed capacity is then calculated too
                        if ( (cntInit == 2) && (config.temperature == 1 || config.temperature == 2) ) {
                            #ifdef RESISTOR_FOR_TEMPERATURE
                            value = ((float) sumVoltage[cntInit]) / (( float) SUM_COUNT_MAX_VOLTAGE * 2.0);
                            convertNtcVoltToTemp(value);
                            #endif
                            sent2Core0( TEMP1 , (int32_t) value );
                        } else if ( (cntInit == 3) && (config.temperature == 2) ) {
                            #ifdef RESISTOR_FOR_TEMPERATURE
                            value = ((float) sumVoltage[cntInit]) / (( float) SUM_COUNT_MAX_VOLTAGE * 2.0);
                            convertNtcVoltToTemp(value);
                            #endif
                            sent2Core0( TEMP2 , (int32_t) value );
                        } else {
                            sent2Core0( cntInit + MVOLT, (int32_t) value ); // save as MVOLT, CURRENT, RESERVE1 or RESERVE2
                        }
                        if (cntInit == 1) { // when we are calculating a current we calculate also the consumption
                            
                            consumedMah += (value * (millisRp() - lastConsumedMillis)) / 3600000.0 ;  // in mah.
                            lastConsumedMillis =  millisRp(); 
                            sent2Core0( CAPACITY, (int32_t) consumedMah );
                        }
                        //adcAvg[cntInit]= sumVoltage[cntInit] / SUM_COUNT_MAX_VOLTAGE ;
                        
                    }
                    sumVoltage[cntInit] = 0 ;
                    //adcMin[cntInit] = 0XFFFF;
                    //adcMax[cntInit] = 0;
                    //printf("voltage has been measured: %d value= %d \n", cntInit , (int) mVolt[cntInit].value);
                    //printf("adc %i  min=%i   max=%i   avg=%i\n", cntInit , adcMin[cntInit] ,adcMax[cntInit] , adcAvg[cntInit] );
                    
                }    
            }    
        }
        
    }        
}

#ifdef RESISTOR_FOR_TEMPERATURE
void  VOLTAGE::convertNtcVoltToTemp (float &adcValue ) {     //Calculate temperature using the Beta Factor equation
        // Convert the thermal stress value to resistance
        // we reuse here the mVolt calculated by oXs. The config must be adapted in a such a way that this mVolt is equal to the raw value returned by the ADC * 1000 (for better accuracy)
        // The calculated temperature is filled back in the voltage field
        float media =  RESISTOR_FOR_TEMPERATURE /  ( (4095.0 / adcValue ) - 1 ) ;// formula is valid when thermistor is connected to ground
        //  float media = RESISTOR_FOR_TEMPERATURE * (1 -  4095.0 / adcValue ); // formula is valid when thermistor is connected to 3.3V instead of to ground
        // T = 1/(A + B* ln(R) + C * ln(R) *ln(R) *ln(R)) 
        float steinhart = log(media) ;
        steinhart = 1 / ( STEINHART_A + STEINHART_B * steinhart + STEINHART_C * steinhart * steinhart * steinhart) ;
        steinhart -= 273.15 ;
        adcValue = steinhart ;    
    }  // end convertNtcVoltToTemp 
#endif
