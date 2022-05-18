#include "tools.h"
#include "pico/stdlib.h"
#include "config.h"

uint32_t millis(){
    return  to_ms_since_boot( get_absolute_time());
}

uint32_t micros() {
    return  to_us_since_boot(get_absolute_time ());
}


field fields[SPORT_TYPES_MAX];  // list of all telemetry fields and parameters used by Sport

void setupListOfFields(){
// list of fileds being used
    //latitude , longitude , groundspeed , heading , altitude ,  numSat
    //mVolt , current , capacity ,  remain 
    //vSpeed, pitch , roll , yaw
    //uplink_RSSI_1 , uplink_RSSI_2 , uplink_Link_quality , uplink_SNR , active_antenna , rf_Mode ,
    //uplink_TX_Power , downlink_RSSI , downlink_Link_quality , downlink_SNR, 
    
    // relativeAlt
    
    #ifndef SKIP_VOLT1_3_4
    uint16_t listFieldsID[SPORT_TYPES_MAX] = {GPS_LONG_LATI_FIRST_ID , GPS_LONG_LATI_FIRST_ID ,GPS_SPEED_FIRST_ID, GPS_COURS_FIRST_ID , GPS_ALT_FIRST_ID ,T1_FIRST_ID  ,\
                          VFAS_FIRST_ID , CURR_FIRST_ID , T2_FIRST_ID , FUEL_FIRST_ID  ,\
                          VARIO_FIRST_ID, ACCX_FIRST_ID , ACCY_FIRST_ID , ACCZ_FIRST_ID ,\
                          UPLINK_RSSI_1_ID , UPLINK_RSSI_2_ID , UPLINK_LINK_QUALITY_ID , UPLINK_SNR_ID , ACTIVE_ANTENNA_ID, RF_MODE_ID ,\
                          UPLINK_TX_POWER_ID , DOWNLINK_RSSI_ID , DOWNLINK_LINK_QUALITY_ID , DOWNLINK_SNR_ID ,\
                          ALT_FIRST_ID} ;
    #else
    
    uint16_t listFieldsID[SPORT_TYPES_MAX] = {GPS_LONG_LATI_FIRST_ID , GPS_LONG_LATI_FIRST_ID ,GPS_SPEED_FIRST_ID, GPS_COURS_FIRST_ID , GPS_ALT_FIRST_ID , T1_FIRST_ID  ,\
                          CURR_FIRST_ID , VFAS_FIRST_ID , T2_FIRST_ID , FUEL_FIRST_ID  ,\
                          VARIO_FIRST_ID, ACCX_FIRST_ID , ACCY_FIRST_ID , ACCZ_FIRST_ID ,\
                          UPLINK_RSSI_1_ID , UPLINK_RSSI_2_ID , UPLINK_LINK_QUALITY_ID , UPLINK_SNR_ID , ACTIVE_ANTENNA_ID, RF_MODE_ID ,\
                          UPLINK_TX_POWER_ID , DOWNLINK_RSSI_ID , DOWNLINK_LINK_QUALITY_ID , DOWNLINK_SNR_ID ,\
                          ALT_FIRST_ID} ;
    #endif
    uint8_t listdeviceID[SPORT_TYPES_MAX] = {DATA_ID_GPS, DATA_ID_GPS, DATA_ID_GPS, DATA_ID_GPS, DATA_ID_GPS , DATA_ID_RPM ,\
                            DATA_ID_FAS , DATA_ID_FAS , DATA_ID_RPM , DATA_ID_FAS ,\
                            DATA_ID_VARIO , DATA_ID_ACC , DATA_ID_ACC , DATA_ID_ACC ,\
                            DATA_ID_RPM , DATA_ID_RPM , DATA_ID_RPM , DATA_ID_RPM , DATA_ID_RPM , DATA_ID_RPM,\
                            DATA_ID_RPM , DATA_ID_RPM , DATA_ID_RPM , DATA_ID_RPM ,\
                            DATA_ID_VARIO  };
    uint16_t listInterval[SPORT_TYPES_MAX] = { 500, 500 , 500 , 500 , 500 ,500,\
                            500 , 500 , 500 , 500,\
                            300 , 300 , 300 , 300,\
                            500 , 500 , 500 , 500 , 500, 500,\
                            500 , 500 , 500 , 500 ,\
                            300};
    for (uint8_t i = 0 ;  i<sizeof(listdeviceID); i++){
        fields[i].value= 0;
        fields[i].available= false;
        fields[i].nextMillis= 0;
        fields[i].interval= listInterval[i];
        fields[i].deviceId= listdeviceID[i];
        fields[i].fieldId= listFieldsID[i];
    }
}

#define PICO_I2C1_SDA_PIN 14  
#define PICO_I2C1_SCL_PIN 15  




