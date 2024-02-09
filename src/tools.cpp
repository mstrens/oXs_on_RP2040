#include "tools.h"
#include "pico/stdlib.h"
#include "config.h"
#include "stdio.h"
#include "pico/util/queue.h"
#include "pico/multicore.h"
#include "math.h"
#include "sport.h"
#include "ms4525.h"
#include "sdp3x.h"
#include "XGZP6897D.h"

extern queue_t qSensorData; 
extern field fields[];
extern bool multicoreIsRunning;

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
    
    //return false; // to debug - to be modified
    const uint CS_PIN_INDEX = 1;
    //startTimerUs(0);
    if( multicoreIsRunning) multicore_lockout_start_blocking();
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
    
    if (multicoreIsRunning) multicore_lockout_end_blocking();
    //getTimerUs(0);
    return button_state;
}

int32_t int_round(int32_t n, uint32_t d)
{
    if (d <= 0) return n;
    int32_t offset;
    offset = ((n >= 0) ? d : -(int32_t) d) ;
    offset = offset >>1; 
    //printf("n=%d   d=%d   offset %d  return %d\n", n , d , offset, (n+offset)/ (int32_t)d);
    return (n + offset) / (int32_t) d;
}


uint32_t millisRp(){
    return  to_ms_since_boot( get_absolute_time());
}

uint32_t microsRp() {
    return  to_us_since_boot(get_absolute_time ());
}

void waitUs(uint32_t delayUs){
    uint32_t nowUs = microsRp();
    while (( microsRp() - nowUs) < delayUs) {microsRp();}
}

void enlapsedTime(uint8_t idx){
    static uint32_t prevTime[10] = {0};
    uint32_t currTime;
    if (idx >= sizeof(prevTime)) return ;
    currTime = microsRp() ;
    printf("Eus%d=%d\n", idx , currTime-prevTime[idx]);
    prevTime[idx]=currTime;
}

uint32_t startAtUs[10] = {0};
void startTimerUs(uint8_t idx){
    if (idx >= sizeof(startAtUs)) return ;
    startAtUs[idx] = microsRp() ;
}

void alarmTimerUs(uint8_t idx, uint32_t alarmExceedUs){
    if (idx >= sizeof(startAtUs)) return ;
    if (( microsRp()-startAtUs[idx]) > alarmExceedUs) { 
        printf("FSus %d= %d\n", idx , microsRp()-startAtUs[idx]);
    }    

}

void getTimerUs(uint8_t idx){
    if (idx >= sizeof(startAtUs)) return ;
    printf("FSus %d= %d\n", idx , microsRp()-startAtUs[idx]);
}


bool msgEverySec(uint8_t idx){  // return true when more than 1 sec
    static uint32_t prevMs[5] = {0};
    if (idx >= 5) return false;
    if ((millisRp() - prevMs[idx]) > 1000){
        prevMs[idx]= millisRp();
        return true;
    }
    return false;
}



void sent2Core0( uint8_t fieldType, int32_t value){
    queue_entry_t entry;
    entry.type = fieldType;
    entry.data = value ;
    queue_try_add(&qSensorData, &entry);
    //printf("sending %d = %10.0f\n", entry.type , (float) entry.data);
}


float difPressureAirspeedSumPa = 0 ; // calculate a moving average on x values
uint32_t difPressureAirspeedCount = 0 ;
float difPressureCompVspeedSumPa = 0 ; // calculate a moving average on x values
uint32_t difPressureCompVspeedCount = 0 ;
float temperatureKelvin;
uint32_t prevAirspeedCalculatedUs;
uint32_t prevAirspeedAvailableMs;
//uint32_t prevCompVspeedCalculatedUs;
//uint32_t prevCompVspeedAvailableMs;
float smoothAirspeedCmS = 0;
extern MS4525 ms4525;
extern SDP3X sdp3x;
extern XGZP  xgzp;
extern float actualPressurePa;


void calculateAirspeed(){
    if (ms4525.airspeedInstalled == false && sdp3x.airspeedInstalled == false  && xgzp.airspeedInstalled == false) return; // skip if no sensor installed
    uint32_t nowUs = microsRp(); 
    if ( ( nowUs - prevAirspeedCalculatedUs) < 20000 ) return; // skip if there is less than 20 msec
    prevAirspeedCalculatedUs = nowUs;
    if (difPressureAirspeedCount == 0) return ; // skip if there is no value (normally because it is not yet calibrated)
    float difPressureAvg = difPressureAirspeedSumPa / (float) difPressureAirspeedCount ; // calculate a moving average on x values
    difPressureAirspeedSumPa = 0 ;  // reset 
    difPressureAirspeedCount = 0 ;
    //printf("p=%.2f\n", difPressureAvg);
    float rawAirspeedPa;
    if ( difPressureAvg < 0 ) {
        rawAirspeedPa = -2396 *  sqrt( -difPressureAvg * temperatureKelvin / (float) actualPressurePa );
    } else {   
    // calculate airspeed based on pressure, altitude and temperature
    // airspeed (m/sec) = sqr(2 * differential_pressure_in_Pa / air_mass_kg_per_m3) 
    // air_mass_kg_per_m3 = pressure_in_pa / (287.05 * (Temp celcius + 273.15))
    // so airspeed m/sec =sqr( 2 * 287.05 * differential_pressure_pa * (temperature Celsius + 273.15) / pressure_in_pa )
    // rawAirSpeed cm/sec =  23,96 * 100 * sqrt( (float) abs(smoothDifPressureAdc) * temperature4525  /  actualPressurePa) ); // in cm/sec ;
    // actual pressure must be in pa (so 101325 about at sea level)
    
    //#ifdef AIRSPEED_AT_SEA_LEVEL_AND_15C
    //smoothAirSpeed =  131.06 * sqrt( (float) ( abs_smoothDifPressureAdc ) ); // indicated airspeed is calculated at 15 Celsius and 101325 pascal
        rawAirspeedPa = 2396 *  sqrt( difPressureAvg * temperatureKelvin / (float) actualPressurePa );
    }
    #define EXPOSMOOTH_AIRSPEED_FACTOR 0.1
    smoothAirspeedCmS += ( EXPOSMOOTH_AIRSPEED_FACTOR * ( rawAirspeedPa - smoothAirspeedCmS )) ; 
    // publish the new value every 200 ms
    if ( (millisRp() - prevAirspeedAvailableMs) > 200) { // make the new value available once per 200 msec
        //printf("difP= %f tmp=%f p=%f rs=%f  ss=%f\n" , (float) difPressureAvg , (float)  temperatureKelvin ,
        //     (float) actualPressurePa , (float) rawAirspeedPa , (float) smoothAirspeedCmS *0.036 );
        prevAirspeedAvailableMs = millisRp();
        //if ( smoothAirSpeedCmS >  0) {  // normally send only if positive and greater than 300 cm/sec , otherwise send 0 but for test we keep all values to check for drift  
        sent2Core0(AIRSPEED, (int32_t) smoothAirspeedCmS);     
    }
} 
// check if offset must be reset
//              if (airSpeedData.airspeedReset) { // adjust the offset if a reset command is received from Tx
//                    offset4525 =  offset4525  + smoothDifPressureAdc ;
//                    airSpeedData.airspeedReset = false ; // avoid that offset is changed again and again if PPM do not send a command
//              }

int32_t posFieldValues[] = {    
    891234567L, //  LATITUDE ,  //  GPS special format
    1781234567L, //  LONGITUDE =     //  GPS special format
    2468,        //GROUNDSPEED =  //  GPS cm/s
    17912,      //  HEADING =,      //  GPS 0.01 degree
    135721,     //  ALTITUDE ,    //  GPS cm
    23,         //  NUMSAT ,      //  5 GPS no unit   
    0X170410FF,  //  GPS_DATE ,    // GPS special format AAMMJJFF
    0X22133100,  //  GPS_TIME ,    // GPS special format HHMMSS00
    123,         //  GPS_PDOP ,    // GPS no unit
    179,         //  GPS_HOME_BEARING, // GPS degree

    234,         //  GPS_HOME_DISTANCE, // 10 GPS  in m
    7321,       //  MVOLT,        // volt1   in mVolt
    89321,      //  CURRENT,  // volt2 must be in seq for voltage.cpp in mA (mV)
    15321,      //   RESERVE1, // volt3 must be in seq for voltage.cpp in mV
    14321,      //  RESERVE2, // volt4 must be in seq for voltage.cpp in mV
      
    34321,      //  CAPACITY,    // based on current (volt2) in mAh
    136,        //  TEMP1,       // = Volt3 but saved as temp in degree
    148,        //  TEMP2,       // = Volt4 but saved as temp in degree
    258,        //  VSPEED,      // baro       in cm/s
    246821,     //  RELATIVEALT , // baro      in cm
      
    89,         //  PITCH,       // 20 imu        in degree 
    78,         //  ROLL,        // imu           in degree
    67,         //  YAW ,        // not used to save data  in degree
    369,        //  RPM ,        // RPM sensor    in Herzt
    11111,      //    ADS_1_1,      // Voltage provided by ads1115 nr 1 on pin 1

    12121,      //  ADS_1_2,      // Voltage provided by ads1115 nr 1 on pin 2    25
    13131,      //  ADS_1_3,      // Voltage provided by ads1115 nr 1 on pin 3
    13141,      //  ADS_1_4,      // Voltage provided by ads1115 nr 1 on pin 4
    21212,      //  ADS_2_1,      // Voltage provided by ads1115 nr 2 on pin 1
    22222,      //  ADS_2_2,      // Voltage provided by ads1115 nr 2 on pin 2
      
    23232,      //  ADS_2_3,      // Voltage provided by ads1115 nr 2 on pin 3    30
    24242,      //  ADS_2_4,      // Voltage provided by ads1115 nr 2 on pin 4
    15151,      //  AIRSPEED,    cm/s
    167,         //     AIRSPEED_COMPENSATED_VSPEED,
    98,         //  SBUS_HOLD_COUNTER,
    
    12,          //  SBUS_FAILSAFE_COUNTER,
    135791,       // GPS cumulative dist in m
    2468,         // ACC_X in 0.001G
    3579,         // ACC_Y
    1478,          // Acc_Z

    45678,
    56789,
    67800,
    78901,
    89012

};

int32_t negFieldValues[] = {    
    -891234567L, //  LATITUDE ,  //  GPS special format
    -1781234567L, //  LONGITUDE =     //  GPS special format
    0,        //GROUNDSPEED =  //  GPS cm/s
    -17912,      //  HEADING =,      //  GPS 0.01 degree
    -56721,     //  ALTITUDE ,    //  GPS cm
    0,         //  NUMSAT ,      //  5 GPS no unit   
    0X170410FF,  //  GPS_DATE ,    // GPS special format AAMMJJFF
    0X22133100,  //  GPS_TIME ,    // GPS special format HHMMSS00
    03,         //  GPS_PDOP ,    // GPS no unit
    -179,         //  GPS_HOME_BEARING, // GPS degree

    0,         //  GPS_HOME_DISTANCE, // 10 GPS  in m
    -7321,       //  MVOLT,        // volt1   in mVolt
    -89321,      //  CURRENT,  // volt2 must be in seq for voltage.cpp in mA (mV)
    -15321,      //   RESERVE1, // volt3 must be in seq for voltage.cpp in mV
    -14321,      //  RESERVE2, // volt4 must be in seq for voltage.cpp in mV
      
    0,      //  CAPACITY,    // based on current (volt2) in mAh
    -19,        //  TEMP1,       // = Volt3 but saved as temp in degree
    -21,        //  TEMP2,       // = Volt4 but saved as temp in degree
    -258,        //  VSPEED,      // baro       in cm/s
    -46821,     //  RELATIVEALT , // baro      in cm
      
    -89,         //  PITCH,       // 20 imu        in degree 
    -78,         //  ROLL,        // imu           in degree
    -67,         //  YAW ,        // not used to save data  in degree
    0,        //  RPM ,        // RPM sensor    in Herzt
    -11111,      //    ADS_1_1,      // Voltage provided by ads1115 nr 1 on pin 1

    -12121,      //  ADS_1_2,      // Voltage provided by ads1115 nr 1 on pin 2    25
    -13131,      //  ADS_1_3,      // Voltage provided by ads1115 nr 1 on pin 3
    -13141,      //  ADS_1_4,      // Voltage provided by ads1115 nr 1 on pin 4
    -21212,      //  ADS_2_1,      // Voltage provided by ads1115 nr 2 on pin 1
    -22222,      //  ADS_2_2,      // Voltage provided by ads1115 nr 2 on pin 2
      
    -23232,      //  ADS_2_3,      // Voltage provided by ads1115 nr 2 on pin 3    30
    -24242,      //  ADS_2_4,      // Voltage provided by ads1115 nr 2 on pin 4
    0,      //  AIRSPEED,    cm/s
    -167,         //     AIRSPEED_COMPENSATED_VSPEED,      
    0,         //  SBUS_HOLD_COUNTER,
    
    0,          //  SBUS_FAILSAFE_COUNTER,   
    0,           // cumulative GPS dist m
    -2468,         // ACC_X in 0.001G
    -3579,         // ACC_Y
    -1478,          // Acc_Z

    -45678,
    -56789,
    -67800,
    -78901,
    -89012
        
};
// fill all fields with dummy values (useful to test a protocol)
 void fillFields( uint8_t forcedFields){
    //printf("entering fillFields wi,th %d\n", forcedFields);
    if (forcedFields == 1)  {   // force positive values
        for (uint8_t i = 0; i <  (sizeof(posFieldValues)/sizeof(*posFieldValues)) ; i++){
        //for (uint8_t i = 0; i <  6 ; i++){
        
            fields[i].value = posFieldValues[i];
            fields[i].available = true;
            fields[i].onceAvailable = true;
            //printf("filling for %d\n", i);
        }
    }
    if (forcedFields == 2)  {   // force negative values
        for (uint8_t i = 0; i <  (sizeof(negFieldValues)/sizeof(*negFieldValues)) ; i++){
            fields[i].value = negFieldValues[i];
            fields[i].available = true;
            fields[i].onceAvailable = true; 
        }
    }
 }

uint16_t swapBinary(uint16_t value) {
    return (value >> 8) | (value << 8);
}

int16_t swapBinary(int16_t value) {
    return (value >> 8) | (value << 8);
}

uint32_t swapBinary(uint32_t value) {
    uint32_t tmp = ((value << 8) & 0xFF00FF00) | ((value >> 8) & 0xFF00FF);
    return (tmp << 16) | (tmp >> 16);
}

int32_t swapBinary(int32_t value) {
    int32_t tmp = ((value << 8) & 0xFF00FF00) | ((value >> 8) & 0xFF00FF);
    return (tmp << 16) | (tmp >> 16);
}

