/*
 * Copyright (C) ExpressLRS_relay
 *
 *
 * License GPLv3: http://www.gnu.org/licenses/gpl-3.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

//#include <Arduino.h>
#include <stdio.h>
#include "pico/stdlib.h"
//#include "pico/multicore.h"
#include "hardware/pio.h"
//#include "hardware/uart.h"
#include "uart_sport_tx_rx.pio.h"
#include "pico/util/queue.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

#include "sport.h"
#include "tools.h"
#include "config.h"
#include "param.h"
#include <math.h>


// one pio and 2 state machines are used to manage the sport in halfduplex
// one state machine (sm) handle the TX and the second the RX
// to receive data, the sm is initialised and use an IRQ handler when rx fifo is not empty
//    in irq, this byte is store in a Rx queue
//    This queue is read in main loop
//    When a byte is received after a 7E, then we stop the sm that receive (it means frsky made a polling)
//    We fill a buffer with the data
//    We set up a dma to transfer the data to the TX fifo of the Tx state machine
//    We also set up a timestamp to stop after some msec the Tx state machine and start again the Rx one   


#define SPORTSYNCREQUEST 0x7E
#define SPORTDEVICEID    0xE4




extern CONFIG config;
extern uint8_t debugTlm;
queue_t sportRxQueue ;

// one pio with 2 state machine is used to manage the inverted hal duplex uart for Sport
PIO sportPio = pio0;
uint sportSmTx = 0; // to send the telemetry to sport
uint sportSmRx = 1; // to get the request from sport
uint sportOffsetTx ; 
uint sportOffsetRx ; 

// dma channel is used to send Sport telemetry without blocking
int sport_dma_chan;
dma_channel_config sportDmaConfig;

uint8_t sportTxBuffer[50];

uint32_t restoreSportPioToReceiveMillis = 0; // when 0, the pio is normally in receive mode,
                                        // otherwise, it is the timestamp when pio transmit has to be restore to receive mode

extern field fields[];  // list of all telemetry fields that are measured


uint16_t sportFieldId[NUMBER_MAX_IDX]; // contains the code to be used in sport to identify the field
uint8_t sportPriority[NUMBER_MAX_IDX]; // contains the list of fields in priority sequence (first = highest) 
uint8_t sportMaxPooling[NUMBER_MAX_IDX]; // contains the max number of polling allowed between 2 transmissions
uint8_t sportMinPooling[NUMBER_MAX_IDX]; // contains the min number of polling allowed between 2 transmissions
uint32_t sportLastPoolingNr[NUMBER_MAX_IDX] = {0}; // contains the last Pooling nr for each field
uint32_t sportPoolingNr= 0; // contains the current Pooling nr
float sportMaxBandwidth = 1; // coeeficient to manage priorities of sport tlm fields (to ensure all fields are sent) 


void setupSportList(){     // table used by sport
    uint8_t temp[] = { // sequence of fields (first = highest priority to sent on sport)
        VSPEED,      // baro       in cm/s    8
        AIRSPEED_COMPENSATED_VSPEED,  // compensated airspeed 8
        LATITUDE,  //  GPS special format     10
        LONGITUDE,    //  GPS special format  10
        GROUNDSPEED , //  GPS cm/s            20
        RELATIVEALT , // baro      in cm      20
        PITCH,       // 20 imu        in degree  20
        ROLL,        // imu           in degree  20 
        AIRSPEED,    //       in cm/s
        HEADING,      //  GPS 0.01 degree        50
        ALTITUDE ,    //  GPS cm                 50
        ACC_X,        // ACC X in g with 3 dec
        ACC_Y,        // ACC Y in g with 3 dec  
        ACC_Z,        // ACC Z in g with 3 dec
        GPS_HOME_BEARING, // GPS degree          50
        GPS_HOME_DISTANCE, // 10 GPS  in m       50
        MVOLT,        // volt1   in mVolt        50
        CURRENT,  // volt2 must be in seq for voltage.cpp in mA (mV)  50
        RESERVE1, // volt3 must be in seq for voltage.cpp in mV       50
        RESERVE2, // volt4 must be in seq for voltage.cpp in mV       50
        TEMP1,       // = Volt3 but saved as temp in degree           50
        TEMP2,       // = Volt4 but saved as temp in degree           50
        RPM ,        // RPM sensor    in Herzt                        50
        YAW ,        // not used to save data  in degree              100
        NUMSAT ,      //  5 GPS no unit                               200 
        GPS_DATE ,    // GPS special format AAMMJJFF                  200
        GPS_TIME ,    // GPS special format HHMMSS00                  200
        GPS_PDOP ,    // GPS no unit                                  200
        CAPACITY,    // based on current (volt2) in mAh                200
        ADS_1_1,      // Voltage provided by ads1115 nr 1 on pin 2    200
        ADS_1_2,      // Voltage provided by ads1115 nr 1 on pin 2    200
        ADS_1_3,      // Voltage provided by ads1115 nr 1 on pin 3    200
        ADS_1_4,      // Voltage provided by ads1115 nr 1 on pin 4    200
        ADS_2_1,      // Voltage provided by ads1115 nr 2 on pin 1    200
        ADS_2_2,      // Voltage provided by ads1115 nr 2 on pin 2    200
        ADS_2_3,      // Voltage provided by ads1115 nr 2 on pin 3    200
        ADS_2_4,     // Voltage provided by ads1115 nr 2 on pin 4      200
        SBUS_HOLD_COUNTER,  // Sbus hold counter
        SBUS_FAILSAFE_COUNTER, // Sbus failsafe counter
        GPS_CUMUL_DIST, // GPS cumulative dist                        200
        RESERVE3,
        RESERVE4,
        RESERVE5,
        RESERVE6,
        RESERVE7,
    };
    for (uint8_t i = 0; i < NUMBER_MAX_IDX ; i++){
        sportPriority[i]= temp[i];
    }
    
    // code used by sport to identify a field in a sport frame
    sportFieldId[LATITUDE] = GPS_LONG_LATI_FIRST_ID;
    sportFieldId[LONGITUDE] = GPS_LONG_LATI_FIRST_ID;
    sportFieldId[GROUNDSPEED] = GPS_SPEED_FIRST_ID;
    sportFieldId[HEADING] = GPS_COURS_FIRST_ID;
    sportFieldId[ALTITUDE] = GPS_ALT_FIRST_ID;
    sportFieldId[NUMSAT] = DIY_GPS_NUM_SAT;
    sportFieldId[GPS_DATE] = GPS_TIME_DATE_FIRST_ID;
    sportFieldId[GPS_TIME] = GPS_TIME_DATE_FIRST_ID;
    sportFieldId[GPS_PDOP] = DIY_GPS_PDOP;
    sportFieldId[GPS_HOME_DISTANCE] = DIY_GPS_HOME_DISTANCE;
    sportFieldId[GPS_HOME_BEARING] = DIY_GPS_HOME_BEARING;
    sportFieldId[MVOLT] = VFAS_FIRST_ID;
    sportFieldId[CURRENT] = CURR_FIRST_ID;
    sportFieldId[RESERVE1] = DIY_VOLT3;
    sportFieldId[RESERVE2] = DIY_VOLT4;
    sportFieldId[CAPACITY] = DIY_CAPACITY;
    sportFieldId[TEMP1] = T1_FIRST_ID; 
    sportFieldId[TEMP2] = T2_FIRST_ID ;
    sportFieldId[VSPEED] = VARIO_FIRST_ID;
    sportFieldId[RELATIVEALT] = ALT_FIRST_ID ;
    sportFieldId[PITCH] = DIY_PITCH;
    sportFieldId[ROLL] = DIY_ROLL;
    //sportFieldId[YAW] = DIY_YAW;
    sportFieldId[RPM] = RPM_FIRST_ID;
    sportFieldId[ADS_1_1] = DIY_ADS_1_1;
    sportFieldId[ADS_1_2] = DIY_ADS_1_2;
    sportFieldId[ADS_1_3] = DIY_ADS_1_3;
    sportFieldId[ADS_1_4] = DIY_ADS_1_4;
    sportFieldId[ADS_2_1] = DIY_ADS_2_1;
    sportFieldId[ADS_2_2] = DIY_ADS_2_2;
    sportFieldId[ADS_2_3] = DIY_ADS_2_3;
    sportFieldId[ADS_2_4] = DIY_ADS_2_4;
    sportFieldId[AIRSPEED] = AIR_SPEED_FIRST_ID;
    sportFieldId[AIRSPEED_COMPENSATED_VSPEED] = VARIO_LAST_ID;
    sportFieldId[SBUS_HOLD_COUNTER] = DIY_SBUS_HOLD_COUNTER;
    sportFieldId[SBUS_FAILSAFE_COUNTER] = DIY_SBUS_FAILSAFE_COUNTER;
    sportFieldId[GPS_CUMUL_DIST] =  DIY_GPS_CUMUL_DISTANCE;
    sportFieldId[ACC_X] = ACCX_FIRST_ID;
    sportFieldId[ACC_Y] = ACCY_FIRST_ID;
    sportFieldId[ACC_Z] = ACCZ_FIRST_ID;
    sportFieldId[RESERVE3] = DIY_RESERVE3;
    sportFieldId[RESERVE4] = DIY_RESERVE4;
    sportFieldId[RESERVE5] = DIY_RESERVE5;
    sportFieldId[RESERVE6] = DIY_RESERVE6;
    sportFieldId[RESERVE7] = DIY_RESERVE7;
     
/*
    sportMaxPooling[LATITUDE] = 50;
    sportMaxPooling[LONGITUDE] = 50;
    sportMaxPooling[GROUNDSPEED] = 50;
    sportMaxPooling[HEADING] = 80;
    sportMaxPooling[ALTITUDE] = 80;
    sportMaxPooling[NUMSAT] = 200;
    sportMaxPooling[GPS_DATE] = 200;
    sportMaxPooling[GPS_TIME] = 200;
    sportMaxPooling[GPS_PDOP] = 200;
    sportMaxPooling[GPS_HOME_BEARING] = 80;
    sportMaxPooling[GPS_HOME_DISTANCE] = 80;
    sportMaxPooling[MVOLT] = 80;
    sportMaxPooling[CURRENT] = 80;
    sportMaxPooling[RESERVE1] = 80;
    sportMaxPooling[RESERVE2] = 80;
    sportMaxPooling[CAPACITY] = 200;
    sportMaxPooling[TEMP1] = 80;
    sportMaxPooling[TEMP2] = 80;
    sportMaxPooling[VSPEED] = 10;
    sportMaxPooling[RELATIVEALT] = 20;
    sportMaxPooling[PITCH] = 30;
    sportMaxPooling[ROLL] = 30;
    sportMaxPooling[YAW] = 200;
    sportMaxPooling[RPM] = 80;
    sportMaxPooling[ADS_1_1] = 200;
    sportMaxPooling[ADS_1_2] = 200;
    sportMaxPooling[ADS_1_3] = 200;
    sportMaxPooling[ADS_1_4] = 200;
    sportMaxPooling[ADS_2_1] = 200;
    sportMaxPooling[ADS_2_2] = 200;
    sportMaxPooling[ADS_2_3] = 200;
    sportMaxPooling[ADS_2_4] = 200;
    sportMaxPooling[AIRSPEED] = 30;
    sportMaxPooling[AIRSPEED_COMPENSATED_VSPEED] = 10;
    sportMaxPooling[SBUS_HOLD_COUNTER] = 100;
    sportMaxPooling[SBUS_FAILSAFE_COUNTER] = 100;
    sportMaxPooling[GPS_CUMUL_DIST] = 200;  

    sportMinPooling[LATITUDE] = 5;
    sportMinPooling[LONGITUDE] = 5;
    sportMinPooling[GROUNDSPEED] = 10;
    sportMinPooling[HEADING] = 30;
    sportMinPooling[ALTITUDE] = 30;
    sportMinPooling[NUMSAT] = 100;
    sportMinPooling[GPS_DATE] = 100;
    sportMinPooling[GPS_TIME] = 100;
    sportMinPooling[GPS_PDOP] = 100;
    sportMinPooling[GPS_HOME_BEARING] = 30;
    sportMinPooling[GPS_HOME_DISTANCE] = 30;
    sportMinPooling[MVOLT] = 30;
    sportMinPooling[CURRENT] = 30;
    sportMinPooling[RESERVE1] = 30;
    sportMinPooling[RESERVE2] = 30;
    sportMinPooling[CAPACITY] = 100;
    sportMinPooling[TEMP1] = 30;
    sportMinPooling[TEMP2] = 30;
    sportMinPooling[VSPEED] = 3;
    sportMinPooling[RELATIVEALT] = 10;
    sportMinPooling[PITCH] = 10;
    sportMinPooling[ROLL] = 10;
    sportMinPooling[YAW] = 50;
    sportMinPooling[RPM] = 30;
    sportMinPooling[ADS_1_1] = 50;
    sportMinPooling[ADS_1_2] = 50;
    sportMinPooling[ADS_1_3] = 50;
    sportMinPooling[ADS_1_4] = 50;
    sportMinPooling[ADS_2_1] = 50;
    sportMinPooling[ADS_2_2] = 50;
    sportMinPooling[ADS_2_3] = 50;
    sportMinPooling[ADS_2_4] = 50;
    sportMinPooling[AIRSPEED] = 10;
    sportMinPooling[AIRSPEED_COMPENSATED_VSPEED] = 3;
    sportMinPooling[SBUS_HOLD_COUNTER] = 50;
    sportMinPooling[SBUS_FAILSAFE_COUNTER] = 50;
    sportMinPooling[GPS_CUMUL_DIST] = 100;
*/
    /*
    fields[LATITUDE].sportDeviceId = SPORT_DEVICEID_P1;
    fields[LONGITUDE].sportDeviceId = SPORT_DEVICEID_P1;
    fields[GROUNDSPEED].sportDeviceId = SPORT_DEVICEID_P2;
    fields[HEADING].sportDeviceId = SPORT_DEVICEID_P2;
    fields[ALTITUDE].sportDeviceId = SPORT_DEVICEID_P2;
    fields[NUMSAT].sportDeviceId = SPORT_DEVICEID_P3;
    fields[GPS_DATE].sportDeviceId = SPORT_DEVICEID_P3;
    fields[GPS_TIME].sportDeviceId = SPORT_DEVICEID_P3;
    fields[GPS_PDOP].sportDeviceId = SPORT_DEVICEID_P3;
    fields[GPS_HOME_DISTANCE].sportDeviceId = SPORT_DEVICEID_P2;
    fields[GPS_HOME_BEARING].sportDeviceId = SPORT_DEVICEID_P2;
    fields[MVOLT].sportDeviceId = SPORT_DEVICEID_P2;
    fields[CURRENT].sportDeviceId = SPORT_DEVICEID_P3;
    fields[RESERVE1].sportDeviceId = SPORT_DEVICEID_P3;
    fields[RESERVE2].sportDeviceId = SPORT_DEVICEID_P3;
    fields[CAPACITY].sportDeviceId = SPORT_DEVICEID_P3;
    fields[TEMP1].sportDeviceId = SPORT_DEVICEID_P3; 
    fields[TEMP2].sportDeviceId =  SPORT_DEVICEID_P3;
    fields[VSPEED].sportDeviceId = SPORT_DEVICEID_P1;
    fields[RELATIVEALT].sportDeviceId =  SPORT_DEVICEID_P2;
    fields[PITCH].sportDeviceId = SPORT_DEVICEID_P2;
    fields[ROLL].sportDeviceId = SPORT_DEVICEID_P2;
    //fields[YAW].sportDeviceId = SPORT_DEVICEID_P3;
    fields[RPM].sportDeviceId = SPORT_DEVICEID_P2;
    
    fields[VSPEED].sportInterval = 100; //msec
    fields[GPS_DATE].sportInterval = 1000; //msec
    fields[GPS_TIME].sportInterval = 1000; //msec
    fields[ALTITUDE].sportInterval = 1000; //msec
    fields[NUMSAT].sportInterval = 1000; //msec
    fields[CAPACITY].sportInterval = 1000; //msec
    fields[GPS_HOME_DISTANCE].sportInterval = 1000; //msec
    fields[GPS_HOME_BEARING].sportInterval = 1000; //msec
    fields[GPS_PDOP].sportInterval = 1000; //msec
    fields[TEMP1].sportInterval = 1000; //msec
    fields[TEMP2].sportInterval = 1000; //msec
    fields[RPM].sportInterval = 1000; //msec
    */

    // add here other fields that should be sent more often
    //for (uint8_t i = 0 ;  i< NUMBER_MAX_IDX ; i++){
    //    printf("deviceId %d = %x\n", i , fields[i].sportDeviceId);
    //}

} 

void setupSport() {
// configure some table to manage priorities and sport fields codes used bu sport    
    setupSportList();
// configure the queue to get the data from Sport in the irq handle
    queue_init (&sportRxQueue, sizeof(uint8_t), 250);

// set up the DMA but do not yet start it to send data to Sport
// Configure a channel to write the same byte (8 bits) repeatedly to PIO0
// SM0's TX FIFO, placed by the data request signal from that peripheral.
    sport_dma_chan = dma_claim_unused_channel(true);
    sportDmaConfig = dma_channel_get_default_config(sport_dma_chan);
    channel_config_set_read_increment(&sportDmaConfig, true);
    channel_config_set_write_increment(&sportDmaConfig, false);
    channel_config_set_dreq(&sportDmaConfig, DREQ_PIO0_TX0);  // use state machine 0 
    channel_config_set_transfer_data_size(&sportDmaConfig, DMA_SIZE_8);
    dma_channel_configure(
        sport_dma_chan,
        &sportDmaConfig,
        &pio0_hw->txf[0], // Write address (only need to set this once)
        &sportTxBuffer[0],   // we use always the same buffer             
        0 , // do not yet provide the number of bytes (DMA cycles)
        false             // Don't start yet
    );
// Set up the state machine for transmit but do not yet start it (it starts only when a request from receiver is received)
    sportOffsetTx = pio_add_program(sportPio, &sport_uart_tx_program);
    sport_uart_tx_program_init(sportPio, sportSmTx, sportOffsetTx, config.pinTlm, 57600 , true); // we use the same pin and baud rate for tx and rx, true means thet UART is inverted 

// set an irq on pio to handle a received byte
    irq_set_exclusive_handler( PIO0_IRQ_0 , sportPioRxHandlerIrq) ;
    irq_set_enabled (PIO0_IRQ_0 , true) ;

// Set up the state machine we're going to use to receive them.
    sportOffsetRx = pio_add_program(sportPio, &sport_uart_rx_program);
    sport_uart_rx_program_init(sportPio, sportSmRx, sportOffsetRx, config.pinTlm, 57600 , true);  
}


void sportPioRxHandlerIrq(){    // when a byte is received on the Sport, read the pio Sport fifo and push the data to a queue (to be processed in the main loop)
  // clear the irq flag
  irq_clear (PIO0_IRQ_0 );
  while (  ! pio_sm_is_rx_fifo_empty (sportPio ,sportSmRx)){ // when some data have been received
     uint8_t c = pio_sm_get (sportPio , sportSmRx) >> 24;         // read the data
     //printf("c%x\n", c);
     queue_try_add (&sportRxQueue, &c);          // push to the queue
    //sportRxMillis = millisRp();                    // save the timestamp.
  }
}


//void processNextSportRxByte( uint8_t c){
//  static uint8_t previous = 0;
//  printf(" %X",c);
//  if ( ( previous ==  SPORTSYNCREQUEST)  && (c == SPORTDEVICEID) ) sendNextSportFrame();
//  previous = c;
//} 

void handleSportRxTx(void){   // main loop : restore receiving mode , wait for tlm request, prepare frame, start pio and dma to transmit it
    static uint8_t previous = 0;
    uint8_t data;
    if (config.pinTlm == 255) return ; // skip when Tlm is not foreseen
    if ( restoreSportPioToReceiveMillis) {            // put sm back in receive mode after some delay
        if (millisRp() > restoreSportPioToReceiveMillis){
            sport_uart_tx_program_stop(sportPio, sportSmTx, config.pinTlm );
            sport_uart_rx_program_restart(sportPio, sportSmRx, config.pinTlm, true);  // true = inverted
            restoreSportPioToReceiveMillis = 0 ;
        }
    } else {                             // when we are in receive mode
        if (! queue_is_empty(&sportRxQueue)) {
            queue_try_remove (&sportRxQueue,&data);
            //printf("%X\n", data);
            if ( ( previous ==  SPORTSYNCREQUEST)  && (data == SPORT_DEVICEID) ){
                     sendNextSportFrame();
            }
            previous = data; 
        }
    }           
}


void sendNextSportFrame(){ // search for the next data to be sent
    // oXs search (in the sequence defined by priority) for a field that has not been sent since more or equal than max
    // if not found, it search for a field that has not been sent since more or equal than min
    // if not found, it sent a frame with all 0
    waitUs(400); // wait a little before replying to a pooling
    if ( dma_channel_is_busy(sport_dma_chan) ) {
        //printf("dma is busy\n");
        return ; // skip if the DMA is still sending data
    }
    //uint32_t _millis = millisRp();
    uint32_t currentPollingNr = ++sportPoolingNr;
    uint8_t _fieldId; 
    // first we search the first field 
    for (uint8_t i = 0 ; i< NUMBER_MAX_IDX ; i++ ){
         _fieldId = sportPriority[i]; // retrieve field ID to be checked
         if ((fields[_fieldId].available) && (sportMaxPooling[_fieldId] > 0))  {
            if (currentPollingNr > (sportLastPoolingNr[_fieldId] + sportMaxPooling[_fieldId])){
                sendOneSport(_fieldId);
                fields[_fieldId].available = false; // flag as sent
                sportLastPoolingNr[_fieldId] = currentPollingNr; // store pooling that has been used 
                return;
            }
         } 
    } // end for
    // repeat base on min 
    for (uint8_t i = 0 ; i< NUMBER_MAX_IDX ; i++ ){
         _fieldId = sportPriority[i]; // retrieve field ID to be checked
         if ((fields[_fieldId].available) && (sportMaxPooling[_fieldId] > 0)) {
            if (currentPollingNr > (sportLastPoolingNr[_fieldId] + sportMinPooling[_fieldId])){
                sendOneSport(_fieldId);
                fields[_fieldId].available = false; // flag as sent
                sportLastPoolingNr[_fieldId] = currentPollingNr; // store pooling that has been used 
                return;
            }
         } 
    } // end for
    // else send a frame with all zero.
    #define NO_SPORT_DATA 0xFF
    sendOneSport(NO_SPORT_DATA); // 0XFF identify the case where we should send a packet with all 0 (meaning no data available) 
}

void sendOneSport(uint8_t idx){  // fill one frame and send it
    //printf("%d\n", idx); // used to get the id and check the sequence in a xls
    // the frame contains 1 byte = type, 2 bytes = value id, 4 byte( or more) for value, 1 byte CRC
    uint8_t counter = 0;
    //uint8_t value[4] = { 0, 1, 2, 3};
    uint32_t uintValue = fields[idx].value ;
    int32_t intValue = fields[idx].value ;
    // change some formats just before sending e.g. for longitude and latitude
    switch (idx) {
        case LONGITUDE:
            uintValue = (( ((((uint32_t)( intValue < 0 ? -intValue : intValue)) /10 ) * 6 ) / 10 ) & 0x3FFFFFFF)  | 0x80000000;  
            if(intValue < 0) uintValue |= 0x40000000;
            break;
        case LATITUDE:
            uintValue = ((  ((((uint32_t)( intValue < 0 ? -intValue : intValue)) / 10 ) * 6 )/ 10 ) & 0x3FFFFFFF ) ;
            if(intValue < 0) uintValue |= 0x40000000;
            break;
        case GROUNDSPEED:  // to do : test for the right value
            //uintValue =  ( ((uint32_t) uintValue) * 36 )  ; // convert cm/s in 1/100 of km/h (factor = 3.6)
            uintValue =  ( ((uint32_t) uintValue) * 700 ) / 36 ; // convert cm/s in 1/1000 of knots (factor = 19.44)
            break;
        case MVOLT:
            uintValue =  ( ((uint32_t) uintValue) /10 ) ;// voltage in mv is divided by 10 because SPORT expect it (volt * 100)
            break;    
        case CURRENT:
            uintValue =  ( ((uint32_t) uintValue) /100 ) ;// voltage in mv is divided by 100 because SPORT expect it (Amp * 10)
            if (intValue < 0) uintValue = 0; 
            break;    
        case AIRSPEED:
            uintValue =  (uint32_t)( ((float) intValue) * 0.194384 ) ;// from cm/s to 0.1kts/h
            if (intValue < 0) uintValue = 0; 
            break;            
    }
    
    uint16_t crc = 0;
    uint8_t tempBuffer[10]= {0};
    if ( idx != NO_SPORT_DATA) {
        tempBuffer[counter++] = 0X10 ; // type of packet : data
        tempBuffer[counter++] = sportFieldId[idx]    ; // 0x0110 = Id for vario data
        tempBuffer[counter++] = sportFieldId[idx] >> 8 ; // 0x0110 = Id for vario data
        tempBuffer[counter++] = uintValue >> 0 ; // value 
        tempBuffer[counter++] = uintValue >> 8 ;  
        tempBuffer[counter++] = uintValue >> 16 ;  
        tempBuffer[counter++] = uintValue >> 24; // value
    }    
    crc = tempBuffer[0] ;
    for (uint8_t i = 1; i<=6;i++){
      crc +=  tempBuffer[i]; //0-1FF
      crc += crc >> 8 ; //0-100
      crc &= 0x00ff ;
    }
    tempBuffer[counter] = 0xFF-crc ;  // CRC in buffer
    // copy and convert bytes
    // Byte in frame has value 0x7E is changed into 2 bytes: 0x7D 0x5E
    // Byte in frame has value 0x7D is changed into 2 bytes: 0x7D 0x5D
    uint8_t sportLength = 0;
    for (uint8_t i = 0 ; i<=7 ; i++){
      if (tempBuffer[i] == 0x7E) {
        sportTxBuffer[sportLength++] = 0x7D;
        sportTxBuffer[sportLength++] = 0x5E;
      } else if (tempBuffer[i] == 0x7D) {
        sportTxBuffer[sportLength++] = 0x7D;
        sportTxBuffer[sportLength++] = 0x5D;
      } else {
      sportTxBuffer[sportLength++]= tempBuffer[i];
      }
    }
    if (debugTlm == 'Y') {
        printf("Sport= ");
        for (uint8_t j = 0 ; j < sportLength ; j++ ){
            printf(" %02X" , sportTxBuffer[j]);
        }
        printf("\n");    
    }
    //sleep_us(100) ;
    sport_uart_rx_program_stop(sportPio, sportSmRx, config.pinTlm); // stop receiving
    sport_uart_tx_program_start(sportPio, sportSmTx, config.pinTlm, true); // prepare to transmit
    // start the DMA channel with the data to transmit
    dma_channel_set_read_addr (sport_dma_chan, &sportTxBuffer[0], false);
    dma_channel_set_trans_count (sport_dma_chan, sportLength, true) ;
    // we need a way to set the pio back in receive mode when all bytes are sent 
    // this will be done in the main loop after some ms (here 6ms)
    restoreSportPioToReceiveMillis = millisRp() + 6;   
}

void calculateSportMaxBandwidth(){
    for (uint8_t i=0; i<NUMBER_MAX_IDX ; i++){
        sportMaxPooling[i]= 0;
        sportMinPooling[i] = 0;
        //sportMaxBandwidth = 10; // dummy value just to be sure it is initialized
    }
    // fill with values from config.h
    sportMaxPooling[LATITUDE] = P_LATITUDE;
    sportMaxPooling[LONGITUDE] = P_LONGITUDE;
    sportMaxPooling[GROUNDSPEED] = P_GROUNDSPEED;
    sportMaxPooling[HEADING] = P_HEADING;
    sportMaxPooling[ALTITUDE] = P_ALTITUDE;
    sportMaxPooling[NUMSAT] = P_NUMSAT;
    sportMaxPooling[GPS_DATE] = P_GPS_DATE;
    sportMaxPooling[GPS_TIME] = P_GPS_TIME;
    sportMaxPooling[GPS_PDOP] = P_GPS_PDOP;
    sportMaxPooling[GPS_HOME_BEARING] = P_GPS_HOME_BEARING;
    sportMaxPooling[GPS_HOME_DISTANCE] = P_GPS_HOME_DISTANCE;
    sportMaxPooling[MVOLT] = P_MVOLT;
    sportMaxPooling[CURRENT] = P_CURRENT;
    sportMaxPooling[RESERVE1] = P_RESERVE1;
    sportMaxPooling[RESERVE2] = P_RESERVE2;
    sportMaxPooling[CAPACITY] = P_CAPACITY;
    sportMaxPooling[TEMP1] = P_TEMP1;
    sportMaxPooling[TEMP2] = P_TEMP2;
    sportMaxPooling[VSPEED] = P_VSPEED;
    sportMaxPooling[RELATIVEALT] = P_RELATIVEALT;
    sportMaxPooling[PITCH] = P_PITCH;
    sportMaxPooling[ROLL] = P_ROLL;
    sportMaxPooling[YAW] = P_YAW;
    sportMaxPooling[RPM] = P_RPM;
    sportMaxPooling[ADS_1_1] = P_ADS_1_1;
    sportMaxPooling[ADS_1_2] = P_ADS_1_2;
    sportMaxPooling[ADS_1_3] = P_ADS_1_3;
    sportMaxPooling[ADS_1_4] = P_ADS_1_4;
    sportMaxPooling[ADS_2_1] = P_ADS_2_1;
    sportMaxPooling[ADS_2_2] = P_ADS_2_2;
    sportMaxPooling[ADS_2_3] = P_ADS_2_3;
    sportMaxPooling[ADS_2_4] = P_ADS_2_4;
    sportMaxPooling[AIRSPEED] = P_AIRSPEED;
    sportMaxPooling[AIRSPEED_COMPENSATED_VSPEED] = P_AIRSPEED_COMPENSATED_VSPEED;
    sportMaxPooling[SBUS_HOLD_COUNTER] = P_SBUS_HOLD_COUNTER;
    sportMaxPooling[SBUS_FAILSAFE_COUNTER] = P_SBUS_FAILSAFE_COUNTER;
    sportMaxPooling[GPS_CUMUL_DIST] = P_GPS_CUMUL_DIST;
    sportMaxPooling[ACC_X] = P_ACC_X ;
    sportMaxPooling[ACC_Y] = P_ACC_Y ;
    sportMaxPooling[ACC_Z] = P_ACC_Z ;
    sportMaxPooling[RESERVE3] = P_RESERVE3 ;
    sportMaxPooling[RESERVE4] = P_RESERVE4 ;
    sportMaxPooling[RESERVE5] = P_RESERVE5 ;
    sportMaxPooling[RESERVE6] = P_RESERVE6 ;
    sportMaxPooling[RESERVE7] = P_RESERVE7 ;
      
    // sum of inverted values only when field is used
    sportMaxBandwidth = 0;
    for (uint8_t i=0; i<NUMBER_MAX_IDX ; i++){
        if ((fields[i].onceAvailable) && (sportMaxPooling[i]>0)) sportMaxBandwidth += 1.0 / sportMaxPooling[i];
    }
    //sportMaxprintf("MaxBW=%f\n",max);
    if ( sportMaxBandwidth == 0) sportMaxBandwidth=1.0;
    // adapt the min and max
    for (uint8_t i=0; i<NUMBER_MAX_IDX ; i++){
        if ((fields[i].onceAvailable) && (sportMaxPooling[i]>0)) {
            sportMaxPooling[i] = ( sportMaxPooling[i] * sportMaxBandwidth) + 1 ;
            sportMinPooling[i] = sportMaxPooling[i] >> 1 ; // min are equal to max / 2
            //printf("%i =  %i\n", (int)i , (int) sportMaxPooling[i] );
        }    
    }
    
}
