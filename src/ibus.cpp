#include <stdio.h>
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "uart_ibus_tx_rx.pio.h"
#include "pico/util/queue.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "MS5611.h"
#include "SPL06.h"
#include "BMP280.h"
#include "tools.h"
#include "gps.h"
#include "param.h"
#include <inttypes.h> // used by PRIu32
#include "ibus.h"

// one pio and 2 state machines are used to manage the ibus in halfduplex
// one state machine (sm) handle the TX and the second the RX
// to receive data, the sm is initialised and use an IRQ handler when rx fifo is not empty
//    in irq, this byte is store in a Rx queue
//    This queue is read in main loop
//    When 4  bytes are received, it is a frame with 
//        - first byte = number of bytes = 04
//        - second = a command (+ adress)
//        - 3 and 4 = checksum
//    Depending on the command, there are 3 types of frames to reply
//    We set up a dma to transfer the data to the TX fifo of the Tx state machine
//    We also set up a timestamp to stop after some msec the Tx state machine and start again the Rx one   

// Rx sent on uart (115200 baud 8N1 not inverted) a frame of 4 char 
// When the second char is 0x8y (starting with 0X81) , it means that the RX ask if sensor y exist.
//    When sensor exits, oXs must reply after a delay of 100 usec, sending back the same char
//    Note : when oXs replies to a 0X8y cmd, Rx send only a cmd 0X9y as long it does not get a reply to his 0x9y cmd
//           so the receiver does not skip to next adr or to 0xAy commands 
// When the second char is 0x9y, it means that the sensor is asking the type of sensor.
//    oXs must reply with a frame of 6 bytes:
//           - first  = 0X06 = number of bytes
//           - second = 0X9y = the received byte
//           - 3d =  sensor type (code from iBus) for y sensor 
//           - 4th = size for this type (2 or 4)
//           - 5/6 = checksum
// When the second char is 0xAy, it means that oXs has to send a frame with the value
//    oXs must reply with a frame of 6 or 8 bytes
//           - lentgh of the frame (6 or 8)
//           - 0XAy (as received)
//           - value (in 2 or 4 bytes)
//           - checksum

//List of types (code specified by ibus)



queue_t ibusRxQueue ;

// one pio with 2 state machine is used to manage the inverted hal duplex uart for Sport
PIO ibusPio = pio0;
uint ibusSmTx = 0; // to send the telemetry to sport
uint ibusSmRx = 1; // to get the request from sport
uint ibusOffsetTx ; 
uint ibusOffsetRx ; 

// dma channel is used to send ibus telemetry without blocking
int ibus_dma_chan;
dma_channel_config ibusDmaConfig;

uint8_t ibusTxBuffer[8];

enum IBUSSTATES {
    RECEIVING,
    //WAIT_FOR_SENDING,
    //SENDING,
    WAIT_END_OF_SENDING
};

uint8_t listOfIbusFields[16] ; // list of fields that can be provided (based on sensor being discovered)
uint8_t maxIbusFieldsIdx ;   // max number of fields that can be provided
int32_t ibusValue; // store a value to be transmitted in ibus format    

uint32_t lastIbusReceivedUs = 0;  // used to check the delay between char. 
uint32_t ibusStartWaiting = 0; // timestamp (usec) when we start waiting 
IBUSSTATES ibusState;

extern field fields[];  // list of all telemetry fields and parameters used by Sport
extern MS5611 baro1;
extern SPL06 baro2;
extern BMP280 baro3;
extern GPS gps;
extern CONFIG config;

#define IBUS_BAUDRATE 115200

uint8_t ibusTypes[NUMBER_MAX_IDX] = {  // list of ibus type in the same sequence as fieldId 
      IBUS_SENSOR_TYPE_GPS_LAT,    // GPS LATITUDE ,  //   GPS special format
      IBUS_SENSOR_TYPE_GPS_LON,    //  GPS LONGITUDE,    //   special format
      IBUS_SENSOR_TYPE_GROUND_SPEED, //GROUNDSPEED , //   GPS cm/s
      IBUS_SENSOR_TYPE_COG,          //HEADING,      //    GPS 0.01 degree
      IBUS_SENSOR_TYPE_GPS_ALT,      //ALTITUDE ,    //   GPS cm

      IBUS_SENSOR_TYPE_GPS_STATUS,   //NUMSAT ,      //    GPS no unit   
      IBUS_SENSOR_TYPE_UNKNOWN,      // GPS_DATE ,    //  ?????????????? GPS special format AAMMJJFF
      IBUS_SENSOR_TYPE_UNKNOWN,      // GPS_TIME ,    //  ?????????????? GPS special format HHMMSS00
      IBUS_SENSOR_TYPE_UNKNOWN,      //GPS_PDOP ,    //  ?????????????? GPS no unit
      IBUS_SENSOR_TYPE_CMP_HEAD,     // GPS_HOME_BEARING, //   GPS degree

      IBUS_SENSOR_TYPE_GPS_DIST,     //GPS_HOME_DISTANCE, //   GPS  in m
      IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE,     //MVOLT,             //    volt1   in mVolt
      IBUS_SENSOR_TYPE_BAT_CURR,     //CURRENT,           // IBUS_SENSOR_TYPE_BAT_CURR  volt2 must be in seq for voltage.cpp in mA (mV)
      IBUS_SENSOR_TYPE_UNKNOWN,      //RESERVE1, //  ???? volt3 must be in seq for voltage.cpp in mV
      IBUS_SENSOR_TYPE_UNKNOWN,      //RESERVE2, //  ???? volt4 must be in seq for voltage.cpp in mV
      
      IBUS_SENSOR_TYPE_FUEL,         //CAPACITY,    // IBUS_SENSOR_TYPE_FUEL    based on current (volt2) in mAh
      IBUS_SENSOR_TYPE_TEMPERATURE,  //TEMP1,       //  = Volt3 but saved as temp in degree
      IBUS_SENSOR_TYPE_TEMPERATURE,  //TEMP2,       // IBUS_SENSOR_TYPE_TEMPERATURE =  Volt4 but saved as temp in degree
      IBUS_SENSOR_TYPE_CLIMB_RATE,   //VSPEED,      //     baro       in cm/s
      IBUS_SENSOR_TYPE_ALT,          //RELATIVEALT , //       baro      in cm
      
      IBUS_SENSOR_TYPE_PITCH,        //PITCH,       //      imu        in degree 
      IBUS_SENSOR_TYPE_ROLL,         //ROLL,        //       imu           in degree
      IBUS_SENSOR_TYPE_UNKNOWN,      //YAW ,        // not used to save data  in degree
      IBUS_SENSOR_TYPE_RPM,          //RPM ,        //      RPM sensor    in Herzt
      IBUS_SENSOR_TYPE_UNKNOWN,      //ADS_1_1,      // Voltage provided by ads1115 nr 1 on pin 1

      IBUS_SENSOR_TYPE_UNKNOWN,      //ADS_1_2,      // Voltage provided by ads1115 nr 1 on pin 2    25
      IBUS_SENSOR_TYPE_UNKNOWN,      //ADS_1_3,      // Voltage provided by ads1115 nr 1 on pin 3
      IBUS_SENSOR_TYPE_UNKNOWN,      //ADS_1_4,      // Voltage provided by ads1115 nr 1 on pin 4
      IBUS_SENSOR_TYPE_UNKNOWN,      //ADS_2_1,      // Voltage provided by ads1115 nr 2 on pin 1
      IBUS_SENSOR_TYPE_UNKNOWN,      //ADS_2_2,      // Voltage provided by ads1115 nr 2 on pin 2
      
      IBUS_SENSOR_TYPE_UNKNOWN,      //ADS_2_3,      // Voltage provided by ads1115 nr 2 on pin 3    30
      IBUS_SENSOR_TYPE_UNKNOWN,      //ADS_2_4,      // Voltage provided by ads1115 nr 2 on pin 4 
      IBUS_SENSOR_TYPE_UNKNOWN,      //AIRSPEED,
      IBUS_SENSOR_TYPE_UNKNOWN,      //AIRSPEED_COMPENSATED_VSPEED,
      IBUS_SENSOR_TYPE_UNKNOWN,      //SBUS_HOLD_COUNTER,
      IBUS_SENSOR_TYPE_UNKNOWN,      //SBUS_FAILSAFE_COUNTER,
      IBUS_SENSOR_TYPE_UNKNOWN,      // GPS CUMULATIVE DISTANCE,      
       
};

void setupIbus() {                                                 
// configure the queue to get the data from ibus in the irq handle
    queue_init (&ibusRxQueue, sizeof(uint8_t), 50);

// set up the DMA but do not yet start it to send data to Sport
// Configure a channel to write the same byte (8 bits) repeatedly to PIO0
// SM0's TX FIFO, placed by the data request signal from that peripheral.
    ibus_dma_chan = dma_claim_unused_channel(true);
    ibusDmaConfig = dma_channel_get_default_config(ibus_dma_chan);
    channel_config_set_read_increment(&ibusDmaConfig, true);
    channel_config_set_write_increment(&ibusDmaConfig, false);
    channel_config_set_dreq(&ibusDmaConfig, DREQ_PIO0_TX0);  // use state machine 0 
    channel_config_set_transfer_data_size(&ibusDmaConfig, DMA_SIZE_8);
    dma_channel_configure(
        ibus_dma_chan,
        &ibusDmaConfig,
        &pio0_hw->txf[0], // Write address (only need to set this once)
        &ibusTxBuffer[0],   // we use always the same buffer             
        0 , // do not yet provide the number of bytes (DMA cycles)
        false             // Don't start yet
    );
// Set up the state machine for transmit but do not yet start it (it starts only when a request from receiver is received)
    ibusOffsetTx = pio_add_program(ibusPio, &ibus_uart_tx_program);
    ibus_uart_tx_program_init(ibusPio, ibusSmTx, ibusOffsetTx, config.pinTlm, IBUS_BAUDRATE , false); // we use the same pin and baud rate for tx and rx, true means thet UART is inverted 

// set an irq on pio to handle a received byte
    irq_set_exclusive_handler( PIO0_IRQ_0 , ibusPioRxHandlerIrq) ;
    irq_set_enabled (PIO0_IRQ_0 , true) ;

// Set up the state machine we're going to use to receive them.
    ibusOffsetRx = pio_add_program(ibusPio, &ibus_uart_rx_program);
    ibus_uart_rx_program_init(ibusPio, ibusSmRx, ibusOffsetRx, config.pinTlm, IBUS_BAUDRATE , false); // false = not inverted   
    setupListIbusFieldsToReply();
}

void addToIbus(uint8_t idx){
    if (maxIbusFieldsIdx >= 15) return; // skip if the table is already full
    maxIbusFieldsIdx++;
    listOfIbusFields[maxIbusFieldsIdx] = idx; // store the oXs code for the field in the list.
}

void setupListIbusFieldsToReply() {  // fill an array with the list of fields (field ID) that are defined
    listOfIbusFields[0] = 0 ;
    maxIbusFieldsIdx = 0 ; // 0 
    /*
      LATITUDE =0,  //  IBUS_SENSOR_TYPE_GPS_LAT GPS special format
      LONGITUDE,    //  IBUS_SENSOR_TYPE_GPS_LON GPS special format
      GROUNDSPEED , //  IBUS_SENSOR_TYPE_GROUND_SPEED GPS cm/s
      HEADING,      //  IBUS_SENSOR_TYPE_COG  GPS 0.01 degree
      ALTITUDE ,    //  IBUS_SENSOR_TYPE_GPS_ALT GPS cm

      NUMSAT ,      //  IBUS_SENSOR_TYPE_GPS_STATUS  GPS no unit   
      GPS_DATE ,    //  ?????????????? GPS special format AAMMJJFF
      GPS_TIME ,    //  ?????????????? GPS special format HHMMSS00
      GPS_PDOP ,    //  ?????????????? GPS no unit
      GPS_HOME_BEARING, // IBUS_SENSOR_TYPE_CMP_HEAD  GPS degree

      GPS_HOME_DISTANCE, // IBUS_SENSOR_TYPE_GPS_DIST  GPS  in m
      MVOLT,             // IBUS_SENSOR_TYPE_BAT_CURR   volt1   in mVolt
      CURRENT,           // IBUS_SENSOR_TYPE_BAT_CURR  volt2 must be in seq for voltage.cpp in mA (mV)
      RESERVE1, //  ???? volt3 must be in seq for voltage.cpp in mV
      RESERVE2, //  ???? volt4 must be in seq for voltage.cpp in mV
      
      CAPACITY,    // IBUS_SENSOR_TYPE_FUEL    based on current (volt2) in mAh
      TEMP1,       // IBUS_SENSOR_TYPE_TEMPERATURE = Volt3 but saved as temp in degree
      TEMP2,       // IBUS_SENSOR_TYPE_TEMPERATURE =  Volt4 but saved as temp in degree
      VSPEED,      //  IBUS_SENSOR_TYPE_CLIMB_RATE   baro       in cm/s
      RELATIVEALT , // IBUS_SENSOR_TYPE_ALT      baro      in cm
      
      PITCH,       // IBUS_SENSOR_TYPE_PITCH     imu        in degree 
      ROLL,        // IBUS_SENSOR_TYPE_ROLL      imu           in degree
      YAW ,        // not used to save data  in degree
      RPM ,        // IBUS_SENSOR_TYPE_RPM     RPM sensor    in Herzt
      ADS_1_1,      // Voltage provided by ads1115 nr 1 on pin 1

      ADS_1_2,      // Voltage provided by ads1115 nr 1 on pin 2    25
      ADS_1_3,      // Voltage provided by ads1115 nr 1 on pin 3
      ADS_1_4,      // Voltage provided by ads1115 nr 1 on pin 4
      ADS_2_1,      // Voltage provided by ads1115 nr 2 on pin 1
      ADS_2_2,      // Voltage provided by ads1115 nr 2 on pin 2
      
      ADS_2_3,      // Voltage provided by ads1115 nr 2 on pin 3    30
      ADS_2_4,      // Voltage provided by ads1115 nr 2 on pin 4
    
    */
    if ( config.pinVolt[0] != 255) {
        addToIbus(MVOLT) ;
    }
    if ( config.pinVolt[1] != 255) {
        addToIbus(CURRENT) ;
        addToIbus(CAPACITY) ;
    } 
    if (( config.pinVolt[2] != 255)  && (config.temperature == 1 or  config.temperature == 2) ) {
        addToIbus(TEMP1) ;
    } 
    if (( config.pinVolt[3] != 255)  && (config.temperature == 2) ) {
        addToIbus(TEMP2) ;
    } 
    // here we could add other voltage parameter (current, ...)
    if ( baro1.baroInstalled || baro2.baroInstalled || baro3.baroInstalled) {
        addToIbus(RELATIVEALT) ; 
        addToIbus(VSPEED) ;
    }
    if ( config.pinRpm != 255) {
        addToIbus(RPM) ;
    } 
    if ( config.pinGpsTx != 255 ) {
        addToIbus(LONGITUDE) ;
        addToIbus(LATITUDE) ;
        addToIbus(NUMSAT) ;
        addToIbus(GROUNDSPEED) ;
        addToIbus(HEADING) ;
        addToIbus(ALTITUDE) ;    
    }
    #ifdef DEBUG
    printf("List of ibus fields : ");
    for (uint8_t i = 0; i<= maxIbusFieldsIdx ; i++){
        printf(" %d ", listOfIbusFields[i]);
    }
    printf("\n");
    #endif 
}


void ibusPioRxHandlerIrq(){    // when a byte is received on the ibus bus, read the pio sbus fifo and push the data to a queue (to be processed in the main loop)
  // clear the irq flag
    irq_clear (PIO0_IRQ_0 );
    while (  ! pio_sm_is_rx_fifo_empty (ibusPio ,ibusSmRx)){ // when some data have been received
        uint8_t c = pio_sm_get (ibusPio , ibusSmRx) >> 24;         // read the data
        queue_try_add (&ibusRxQueue, &c);          // push to the queue
        if (c == 0X04) lastIbusReceivedUs = microsRp();    
    }
}

//#define DEBUG_IBUS_WITHOUT_RX
#ifdef DEBUG_IBUS_WITHOUT_RX
uint32_t lastIbusRequest = 0;
#endif
void handleIbusRxTx(void){   // main loop : restore receiving mode , wait for tlm request, prepare frame, start pio and dma to transmit it
    static uint8_t previous = 0;
    static uint8_t data[4];
    static uint8_t pollingSimulation;
    uint8_t ibusCmd;
    uint8_t ibusAdr;
    uint8_t ibusType;
    uint8_t ibusValueLength;
    uint16_t checksum;
    uint16_t checksumCalc;

    if (config.pinTlm == 255) return ; // skip when Tlm is not foreseen
    switch (ibusState) {
        case RECEIVING :
            #ifdef DEBUG_IBUS_WITHOUT_RX  // simulate a RX sending a polling every 7 msec 
            if ( (millisRp() - lastIbusRequest) > 6)  { // to debug simulate a request once per 200msec
                ibusState = WAIT_FOR_SENDING;
                ibusStartWaiting = microsRp();
                data = pollingSimulation++;
                if (pollingSimulation >0X0F) pollingSimulation = 0;
                lastIbusRequest = millisRp();
            }
            break;
            #endif
            while (queue_get_level(&ibusRxQueue) > 4) queue_try_remove (&ibusRxQueue,&data[0]);  
            if ( queue_get_level(&ibusRxQueue) == 4) {
                queue_try_remove (&ibusRxQueue,&data[0]);
                //printf("%X ", data);
                if ( data[0] != 0X04 ) return ;  // first byte of a frame must be 0X04 
                queue_try_remove (&ibusRxQueue,&data[1]);
                queue_try_remove (&ibusRxQueue,&data[2]);
                queue_try_remove (&ibusRxQueue,&data[3]);
                checksum = data[2] | (data[3] << 8);
                checksumCalc = 0xFFFF - (data[0] + data[1]);
                if (checksumCalc != checksum) {
                    printf("ibus pooling with checksum error %x %x %x %x \n", data[0] , data[1], data[2], data[3]);
                    return; // skip if checksum is wrong
                }
                //printf("ibus get %x %x %x %x\n", data[0] , data[1], data[2], data[3]);
                ibusCmd = data[1] >> 4;
                ibusAdr =  data[1] & 0X0F ;
                ibusType = ibusTypes[listOfIbusFields[ibusAdr]]; // type of field being sent
                ibusValueLength = 2;
                if( ibusType >= 0X80 && ibusType <= 0X8F ) ibusValueLength = 4; // length of field    
                switch (ibusCmd) {
                    case 0X08: // request discovering next sensor ; we reply the same value to confirm it exists
                        if ( ibusAdr > (maxIbusFieldsIdx)) {
                            //printf("ibus request for sensor adr %d max=%d\n", ibusAdr, maxIbusFieldsIdx);
                            return;
                        }        
                        ibusTxBuffer[0] = data[0];
                        ibusTxBuffer[1] = data[1];
                        ibusTxBuffer[2] = data[2];
                        ibusTxBuffer[3] = data[3];
                        break;
                    case 0X09:  // request specify the type for an index
                        if (ibusAdr > maxIbusFieldsIdx) {
                            printf("error ibus request 0X09 for sensor adr %d  max=%d\n", ibusAdr, maxIbusFieldsIdx);
                            return;
                        }    
                        if (ibusType == IBUS_SENSOR_TYPE_UNKNOWN ){
                            printf("error ibus request 0X09 for sensor adr %d  type=%x max=%d\n", ibusAdr, ibusType , maxIbusFieldsIdx);
                            return; // skip when this field is not supported // should not happen
                        }
                        ibusTxBuffer[0] = 0x06; // length of the frame
                        ibusTxBuffer[1] = data[1]; // original cmd +adr
                        ibusTxBuffer[2] = ibusType; // type of field being sent
                        ibusTxBuffer[3] = ibusValueLength;
                        checksum = 0xFFFF - (ibusTxBuffer[0] + ibusTxBuffer[1] + ibusTxBuffer[2] + ibusTxBuffer[3]);
                        ibusTxBuffer[4] = checksum;
                        ibusTxBuffer[5] = checksum >> 8;
                        break;
                    case 0x0A:
                        if (ibusAdr > maxIbusFieldsIdx) {
                            printf("error ibus request 0X0A for sensor adr %d\n", ibusAdr);
                            return;
                        }    
                        if (ibusType == IBUS_SENSOR_TYPE_UNKNOWN ) return; // skip when this field is not supported // should not happen
                        ibusTxBuffer[0] = ibusValueLength + 4; // length of the frame
                        ibusTxBuffer[1] = data[1]; // original cmd +adr
                        checksum = 0xFFFF - (ibusTxBuffer[0] + ibusTxBuffer[1]);
                        //if (formatIbusValue(ibusAdr) == false) return ; // skip when value is not available
                        formatIbusValue(ibusAdr); 
                        uint8_t pByte;
                        for (int i = 0; i < ibusValueLength; i++) {  // there can be 2 or 4 bytes in the value
                            pByte     = (ibusValue >> (8 * i)) & 0xFF;
                            checksum -= pByte;
                            ibusTxBuffer[2+i] = pByte ;
                        }
                        ibusTxBuffer[ibusValueLength + 2] = checksum;
                        ibusTxBuffer[ibusValueLength + 3] = checksum >> 8;
                        break;
                    default:
                        return;
                        break;    
                } // end switch about a ibus request
                // at this point, a frame is prepared and can be sent    
                //printf("ibus frame sent:");
                //for (uint8_t i = 0 ; i < ibusTxBuffer[0]; i++){
                //    printf(" %x ", ibusTxBuffer[i]);
                //}
                //printf("\n");
                sleep_us(100);
                ibus_uart_rx_program_stop(ibusPio, ibusSmRx, config.pinTlm); // stop receiving
                ibus_uart_tx_program_start(ibusPio, ibusSmTx, config.pinTlm, false); // prepare to transmit
                // start the DMA channel with the data to transmit
                dma_channel_set_read_addr (ibus_dma_chan, &ibusTxBuffer[0], false);
                dma_channel_set_trans_count (ibus_dma_chan, ibusTxBuffer[0], true) ; // ibusTxBuffer contains the number of bytes
                ibusState = WAIT_END_OF_SENDING;
                ibusStartWaiting = microsRp();
            } // end if        
            break;
        case WAIT_END_OF_SENDING :
            if ( ( microsRp() - ibusStartWaiting) > (1000) ){ // wait that the 8 bytes have been sent ( ???? usec)
                ibus_uart_tx_program_stop(ibusPio, ibusSmTx, config.pinTlm );
                ibus_uart_rx_program_restart(ibusPio, ibusSmRx, config.pinTlm, false );  // false = not inverted
                ibusState = RECEIVING ;
                //printf("end sending\n");
            }
            break;     
    }
     
}

bool formatIbusValue( uint8_t ibusAdr){
    uint8_t fieldId = listOfIbusFields[ibusAdr];
    ibusValue = 0;
    if (fields[fieldId].available == false) return false; // false when the value is not available
    ibusValue= fields[fieldId].value ; // default do not use conversion
    switch (fieldId) {
        case MVOLT:
            ibusValue= int_round(fields[fieldId].value , 10); // from mvolt to 0.01V
            break;
        case CURRENT:
            ibusValue= int_round(fields[fieldId].value , 10); // from mamp to 0.01A
            break;
        case NUMSAT:
            ibusValue= fields[fieldId].value * 256; // it seems that openTx discard the lowest byte
            break;
        case TEMP1:
            ibusValue= (fields[fieldId].value * 10) + 400; // from ° to 0.1°; it seems that openTx uses an offset of 400
            break;
        case TEMP2:
            ibusValue= (fields[fieldId].value * 10) + 400; // from ° to 0.1°; it seems that openTx uses an offset of 400
            break;
        //case CAPACITY:
        //    value= fields[fieldId].value ; // from mah to mah
        //    break;
        //case TEMP1:
        //    value= fields[fieldId].value ; // from ° to °
        //    break;
        //case TEMP2:
        //    value= fields[fieldId].value ; // from ° to °
        //    break;    
        //case VSPEED:
        //    value= fields[fieldId].value ; // from cm/sec to 0.1m/sec
        //    break;
        //case GROUNDSPEED:
        //    value= fields[fieldId].value ; // from cm/sec to cm/sec
        //    break;            
        //case RPM:
        //    value= fields[fieldId].value *60 / 100 ; // from Hz to 100 tr/min???? not sure it is ok
        //    break;            
        //case GPS_HOME_BEARING:
        //    value= fields[fieldId].value  ; // from  deg  to deg
        //    break;
        //case GPS_HOME_DISTANCE:
        //    value= fields[fieldId].value  ; // from  m  to m
        //    break;     
        //case RELATIVEALT:
        //    value= fields[fieldId].value ; // from cm to cm
        //    break;            
    } // end switch 
    // fields[fieldId].available = false;   // reset the flag available // it seems we always have to send a value otherwise Rx try to rediscover the sensor
    return true;    
}


