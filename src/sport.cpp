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

// to do: 
// if we can reply to several device id, we should keep a table with the last field index used for this deviceid
// when we get a polling for one device id, we should use this table to search from this last field index 


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

field fields[NUMBER_MAX_IDX];  // list of all telemetry fields and parameters used by Sport

void setupListOfFields(){    // codes use to identify each field is defined in tool.h
    for (uint8_t i = 0 ;  i< NUMBER_MAX_IDX ; i++){ 
        fields[i].value= 0;
        fields[i].available= false;
        fields[i].nextMillis= 0;
        fields[i].sportInterval= 300; // default interval in msec
        fields[i].sportDeviceId= 0XFF; // default dummy value = do not reply on this
        fields[i].sportFieldId= DIY_LAST_ID; // default dummy ID
    }
    // overwrite the default setting
    fields[LATITUDE].sportFieldId = GPS_LONG_LATI_FIRST_ID;
    fields[LONGITUDE].sportFieldId = GPS_LONG_LATI_FIRST_ID;
    fields[GROUNDSPEED].sportFieldId = GPS_SPEED_FIRST_ID;
    fields[HEADING].sportFieldId = GPS_COURS_FIRST_ID;
    fields[ALTITUDE].sportFieldId = GPS_ALT_FIRST_ID;
    fields[NUMSAT].sportFieldId = DIY_GPS_NUM_SAT;
    fields[GPS_DATE].sportFieldId = GPS_TIME_DATE_FIRST_ID;
    fields[GPS_TIME].sportFieldId = GPS_TIME_DATE_FIRST_ID;
    fields[GPS_PDOP].sportFieldId = DIY_GPS_PDOP;
    fields[GPS_HOME_DISTANCE].sportFieldId = DIY_GPS_HOME_DISTANCE;
    fields[GPS_HOME_BEARING].sportFieldId = DIY_GPS_HOME_BEARING;
    fields[MVOLT].sportFieldId = VFAS_FIRST_ID;
    fields[CURRENT].sportFieldId = CURR_FIRST_ID;
    fields[RESERVE1].sportFieldId = DIY_VOLT3;
    fields[RESERVE2].sportFieldId = DIY_VOLT4;
    fields[CAPACITY].sportFieldId = FUEL_FIRST_ID;
    fields[TEMP1].sportFieldId = T1_FIRST_ID; 
    fields[TEMP2].sportFieldId = T2_FIRST_ID ;
    fields[VSPEED].sportFieldId = VARIO_FIRST_ID;
    fields[RELATIVEALT].sportFieldId = ALT_FIRST_ID ;
    fields[PITCH].sportFieldId = DIY_PITCH;
    fields[ROLL].sportFieldId = DIY_ROLL;
    //fields[YAW].sportFieldId = DIY_YAW;
    fields[RPM].sportFieldId = RPM_FIRST_ID;
    
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
    

    // add here other fields that should be sent more often
    //for (uint8_t i = 0 ;  i< NUMBER_MAX_IDX ; i++){
    //    printf("deviceId %d = %x\n", i , fields[i].sportDeviceId);
    //}

} 

void setupSport() {
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
     //printf("%x", c);
     queue_try_add (&sportRxQueue, &c);          // push to the queue
    //sportRxMillis = millis();                    // save the timestamp.
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
        if (millis() > restoreSportPioToReceiveMillis){
            sport_uart_tx_program_stop(sportPio, sportSmTx, config.pinTlm );
            sport_uart_rx_program_restart(sportPio, sportSmRx, config.pinTlm, true);  // true = inverted
            restoreSportPioToReceiveMillis = 0 ;
        }
    } else {                             // when we are in receive mode
        if (! queue_is_empty(&sportRxQueue)) {
            queue_try_remove (&sportRxQueue,&data);
            //printf("%X ", data);
            if ( ( previous ==  SPORTSYNCREQUEST)  &&
                ((data == SPORT_DEVICEID_P1) || (data == SPORT_DEVICEID_P2) || (data == SPORT_DEVICEID_P3)  )){
                     sendNextSportFrame(data);
            }
            previous = data; 
        }
    }           
}


void sendNextSportFrame(uint8_t data_id){ // search for the next data to be sent for this device ID
    // search an idx (0,1,2 ) of current deviceId
    static uint8_t last_sport_idx[3] = {0, 0, 0} ;
    uint8_t deviceIndex= 0; // 0 mean P1 device
    if (data_id == SPORT_DEVICEID_P2) deviceIndex=1;
    if (data_id == SPORT_DEVICEID_P3) deviceIndex=2;
    
    //printf("sendNextSportFrame\n");
    waitUs(300); // wait a little before replying to a pooling
    if ( dma_channel_is_busy(sport_dma_chan) ) {
        //printf("dma is busy\n");
        return ; // skip if the DMA is still sending data
    }
    uint32_t _millis = millis();
    for (uint8_t i = 0 ; i< NUMBER_MAX_IDX ; i++ ){
         last_sport_idx[deviceIndex]++;
         if (last_sport_idx[deviceIndex] >= NUMBER_MAX_IDX) last_sport_idx[deviceIndex] = 0 ;
        //printf("last_sport_idx= %d\n", last_sport_idx);
        uint8_t currentFieldIdx = last_sport_idx[deviceIndex];
        if (fields[currentFieldIdx].sportDeviceId == data_id) { // process only fields for requested device id
            if ( (_millis >= fields[currentFieldIdx].nextMillis) && (fields[currentFieldIdx].available)  ) {
                //printf("Sport for device %X  with field %d\n", data_id , currentFieldIdx);
                sendOneSport(currentFieldIdx);
                fields[currentFieldIdx].available = false; // flag as sent
                fields[currentFieldIdx].nextMillis = millis() + fields[currentFieldIdx].sportInterval;
                break;
            }
        }    
    }
}

void sendOneSport(uint8_t idx){  // fill one frame and send it
    //printf("send idx %f\n", (float) idx);
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
    case HEADING:
        uintValue =  intValue / 1000 ; // convert from degree * 100000 to degree * 100
        break;
    case GROUNDSPEED:  // to do : test for the right value
        uintValue =  ( ((uint32_t) uintValue) * 36 )  ; // convert cm/s in 1/100 of km/h (factor = 3.6)
        //uintValue =  ( ((uint32_t) uintValue) * 700 ) / 36 ; // convert cm/s in 1/1000 of knots (factor = 19.44)
        break;
    case MVOLT:
        uintValue =  ( ((uint32_t) uintValue) /10 ) ;// voltage in mv is divided by 10 because SPORT expect it (volt * 100)
        break;    
    case CURRENT:
        uintValue =  ( ((uint32_t) uintValue) /100 ) ;// voltage in mv is divided by 100 because SPORT expect it (Amp * 10)
        break;    
    }
    uint16_t crc ;
    uint8_t tempBuffer[10];
    tempBuffer[counter++] = 0X10 ; // type of packet : data
    tempBuffer[counter++] = fields[idx].sportFieldId    ; // 0x0110 = Id for vario data
    tempBuffer[counter++] = fields[idx].sportFieldId >> 8 ; // 0x0110 = Id for vario data
    tempBuffer[counter++] = uintValue >> 0 ; // value 
    tempBuffer[counter++] = uintValue >> 8 ;  
    tempBuffer[counter++] = uintValue >> 16 ;  
    tempBuffer[counter++] = uintValue >> 24; // value
    
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
        printf("/n");    
    }
    sleep_us(100) ;
    sport_uart_rx_program_stop(sportPio, sportSmRx, config.pinTlm); // stop receiving
    sport_uart_tx_program_start(sportPio, sportSmTx, config.pinTlm, true); // prepare to transmit
    // start the DMA channel with the data to transmit
    dma_channel_set_read_addr (sport_dma_chan, &sportTxBuffer[0], false);
    dma_channel_set_trans_count (sport_dma_chan, sportLength, true) ;
    // we need a way to set the pio back in receive mode when all bytes are sent 
    // this will be done in the main loop after some ms (here 6ms)
    restoreSportPioToReceiveMillis = millis() + 6;   
}

