#include "voltage.h"
 #include "hardware/adc.h"
 


#if defined(ARDUINO_MEASURES_VOLTAGES) && (ARDUINO_MEASURES_VOLTAGES == YES)


VOLTAGE::VOLTAGE() {}

void VOLTAGE::begin(void ) {
    
    #ifdef RESISTOR_TO_GROUND 
  float tempResistorToGround[MAX_NBR_VOLTAGES] = { RESISTOR_TO_GROUND } ;
    #else
  float tempResistorToGround[MAX_NBR_VOLTAGES] = { 0 , 0 , 0 , 0 } ;
    #endif
    #ifdef RESISTOR_TO_VOLTAGE  
  float tempResistorToVoltage[MAX_NBR_VOLTAGES] = { RESISTOR_TO_VOLTAGE } ;
    #else
  float tempResistorToVoltage[MAX_NBR_VOLTAGES] = { 0 , 0 , 0 , 0  } ;
    #endif
    #ifdef SCALE_VOLTAGE   
  float tempScaleVoltage[6] = { SCALE_VOLTAGE }  ;
    #else
  float tempScaleVoltage[6] =  { 1 , 1 , 1 , 1  } ;
    #endif
  for (int cntInit = 0 ; cntInit < MAX_NBR_VOLTAGES ; cntInit++) {
    if ( ( pin[ cntInit ] >= 26  ) && ( pin[ cntInit ] <= 29  ) ) {
      atLeastOneVolt = true ;
    } else {
      pin[cntInit] = 0 ; // discard wrong pins
    }  
    // 330 because the max volt is 3.3V and e expect 2 decimals - to check if this is correct
    if ( tempResistorToGround[cntInit] > 0 && tempResistorToVoltage[cntInit] > 0 && tempScaleVoltage[cntInit] > 0 ) {
      mVoltPerStep[cntInit] = 330 / 4095.0 * ( tempResistorToGround[cntInit] + tempResistorToVoltage[cntInit] ) / tempResistorToGround[cntInit]  * tempScaleVoltage[cntInit];
    } else {
      mVoltPerStep[cntInit] = 330 / 4095.0  * tempScaleVoltage[cntInit];  
    }
    mVolt[cntInit].value = 0;
    mVolt[cntInit].available = false ;
  } // end for
  if(atLeastOneVolt){
        adc_init (); // prepare ADC
        for (int cntInit = 0 ; cntInit < MAX_NBR_VOLTAGES ; cntInit++) {
            if (pin[cntInit]) adc_gpio_init(pin[cntInit]); // prepare the pin for ADC
        }
  }
} // end begin

void VOLTAGE::getVoltages(void){
    static uint8_t sumCount = 0;
    static uint32_t lastVoltagemillis = 0 ;
    if ((atLeastOneVolt) && ((millis() - lastVoltagemillis) > VOLTAGEINTERVAL )) {
        lastVoltagemillis = millis() ;
        for (int cntInit = 0 ; cntInit < MAX_NBR_VOLTAGES ; cntInit++) {
            if (pin[cntInit]) {
                adc_select_input(cntInit); // select the pin
                sumVoltage[cntInit] += adc_read(); // convert and sum
            }
        }
        sumCount++;
        if ( sumCount == SUM_COUNT_MAX_VOLTAGE ) {
            sumCount = 0;
            for (int cntInit = 0 ; cntInit < MAX_NBR_VOLTAGES ; cntInit++) {
                if (pin[cntInit]) {
                    mVolt[cntInit].value = sumVoltage[cntInit] / SUM_COUNT_MAX_VOLTAGE * mVoltPerStep[cntInit] ;
                    mVolt[cntInit].available = true ;
                    sumVoltage[cntInit] = 0 ;
                    //printf("voltage has been measured: %d value= %d \n", cntInit , (int) mVolt[cntInit].value);  
                }
            }    
        }
    }        
}

#endif // end of PIN_VOLTAGE
