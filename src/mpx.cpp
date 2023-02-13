#include <stdio.h>
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "uart_mpx_tx_rx.pio.h"
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
#include "mpx.h"

// one pio and 2 state machines are used to manage the mpx in halfduplex
// one state machine (sm) handle the TX and the second the RX
// to receive data, the sm is initialised and use an IRQ handler when rx fifo is not empty
//    in irq, this byte is store in a Rx queue
//    This queue is read in main loop
//    When a byte is received and is on range 0X00/0X0F, then we stop the sm that receive (it means mpx made a polling)
//    We fill a buffer with the data
//    We set up a dma to transfer the data to the TX fifo of the Tx state machine
//    We also set up a timestamp to stop after some msec the Tx state machine and start again the Rx one   

// Rx sent on uart (38400 baud 8N1 not inverted) a polling code 
// when the polling is in the range 0X00/0X0F, the sensor  with the same 4 LSB bits (= address) must reply after 1 msec
// if no data is available but sensor exist, the value is 0X800
// the reply consist of 3 bytes
// first byte = 4bits (MSB) = address + 4 bits (LSB) that gives the type of units (see table below) 
// imagine that value is in 2 bytes 12345678 ABCDEFGH ; this is shift to 2345678A BCDEFGH0
// then second byte = BCDEFGHx where X = 1 for an alarm, 0 for no alarm
// 3d byte   = 2345678A

//List of polling address
#define MPX_VOLT    1	     //tension	                           0.1V          -600 à +600
#define MPX_CURRENT 2        //électricité                        0.1A          -1000 à +1000
#define MPX_VSPEED  3        //monter/descendre                   0,1 m/s       -500 à +500
#define MPX_SPEED   4        //vitesse                            0,1 km/h       0 à +6000
#define MPX_RPM     5        //vitesse rotationnelle         100 tr/min ou 10 tr/min	0 à +500 / -5000
#define MPX_TMP     6        //Température                         0.1°C         -250 à +7000
#define MPX_DIR     7        //Direction                           0,1 degrés     0 à 3600
#define MPX_HEIGHT  8	     //Hauteur                             1m         -500 à +2000
#define MPX_LEVEL   9        //niveau                              1% réservoir    0 à +100
#define MPX_LQ      10       //Indicateur de qualité du lien	IQL 1 %         0 à +100
#define MPX_CONSUMPTION 11   //consommation d'énergie              1mAh        -16000 à +16000
#define MPX_LIQUID  12       //liquides                            1mL         0 à +16000
#define MPX_DIST    13       //distance                            0.1 km      0 à +16000

//list of all telemetry units supported by Multiplex protocol
#define MU_ALT     0x08  // 1m       (-500 2000)
#define MU_VSPD    0x03  // 0,1 m/s (-500 500) 
#define MU_CURR    0x02  // 0,1 A   (-1000 1000)
#define MU_VOLT    0x01  // 0,1 V   (-600 600)
#define MU_TEMP    0x06  // 0,1 C   (-250 7000)
#define MU_RPM     0x05  // 100t/m?? ou 10t/min
#define MU_MAH     0x0B  // 1mAh    (-16000 16000)
#define MU_ASPD    0x04  // 0,1km/h (0-6000)
#define MU_LEVEL   0x09  // 1%      (0-100)
#define MU_DIR     0x07  // 0,1 degrés  (0 3600)
#define MU_LIQUID  0x0C  // 1ml      (0-16000)
#define MU_DIST    0x0D  // 0,1 km   (0-16000)
// End of list of all telemetry units supported by Multiplex protocol  

uint8_t convertMpxAddToUnit[16] = {
0XFF,       // 0XFF means not used
0XFF,       // 0XFF means not used
MU_CURR,   //#define MPX_CURRENT 2        //électricité                        0.1A          -1000 à +1000
MU_VSPD,   //#define MPX_VSPEED  3        //monter/descendre                   0,1 m/s       -500 à +500
MU_ASPD,   //#define MPX_SPEED   4        //vitesse                            0,1 km/h       0 à +6000
MU_RPM,           //#define MPX_RPM     5        //vitesse rotationnelle         100 tr/min ou 10 tr/min	0 à +500 / -5000
MU_TEMP,         //#define MPX_TMP     6        //Température                         0.1°C         -250 à +7000
MU_DIR,       //#define MPX_DIR     7        //Direction                           0,1 degrés     0 à 3600
MU_ALT,   //#define MPX_HEIGHT  8	     //Hauteur                             1m         -500 à +2000
MU_VOLT,    //#define MPX_VOLT    9	     //tension	                           0.1V          -600 à +600
0XFF,          //#define MPX_LQ      10       //Indicateur de qualité du lien	IQL 1 %         0 à +100
MU_MAH,      //#define MPX_CONSUMPTION 11   //consommation d'énergie              1mAh        -16000 à +16000
0XFF,          //#define MPX_LIQUID  12       //liquides                            1mL         0 à +16000
MU_DIST,          //#define MPX_DIST    13       //distance                            0.1 km      0 à +16000
0XFF,         //14
0XFF          //15
};
//0XFF,          //#define MPX_LEVEL   9        //niveau                              1% réservoir    0 à +100


uint8_t convertMpxAddToFieldId[16] = { 
0XFF,          // 0XFF means not used
0XFF,          // 0XFF means not used 1
CURRENT,       //#define MPX_CURRENT 2        //électricité                        0.1A          -1000 à +1000
VSPEED,        //#define MPX_VSPEED  3        //monter/descendre                   0,1 m/s       -500 à +500
GROUNDSPEED,   //#define MPX_SPEED   4        //vitesse                            0,1 km/h       0 à +6000
RPM,           //#define MPX_RPM     5        //vitesse rotationnelle         100 tr/min ou 10 tr/min	0 à +500 / -5000
TEMP1,          //#define MPX_TMP     6        //Température                         0.1°C         -250 à +7000
GPS_HOME_BEARING ,       //#define MPX_DIR     7        //Direction                           0,1 degrés     0 à 3600
RELATIVEALT,   //#define MPX_HEIGHT  8	     //Hauteur                             1m         -500 à +2000
MVOLT,         //#define MPX_VOLT    9	     //tension	                           0.1V          -600 à +600
0XFF,          //#define MPX_LQ      10       //Indicateur de qualité du lien	IQL 1 %         0 à +100
CAPACITY,      //#define MPX_CONSUMPTION 11   //consommation d'énergie              1mAh        -16000 à +16000
0XFF,          //#define MPX_LIQUID  12       //liquides                            1mL         0 à +16000
GPS_HOME_DISTANCE,          //#define MPX_DIST    13       //distance                            0.1 km      0 à +16000
0XFF,         //14
0XFF          //15
};
//0XFF,          //#define MPX_LEVEL   9        //niveau                              1% réservoir    0 à +100

bool mpxFieldsToReply[16] = {false} ; // this table says if oXs has to reply to this index polling (based on hardware installed)

queue_t mpxRxQueue ;

// one pio with 2 state machine is used to manage the inverted hal duplex uart for Sport
PIO mpxPio = pio0;
uint mpxSmTx = 0; // to send the telemetry to sport
uint mpxSmRx = 1; // to get the request from sport
uint mpxOffsetTx ; 
uint mpxOffsetRx ; 

// dma channel is used to send Sport telemetry without blocking
int mpx_dma_chan;
dma_channel_config mpxDmaConfig;

uint8_t mpxTxBuffer[3];

enum MPXSTATES {
    RECEIVING,
    WAIT_FOR_SENDING,
    SENDING,
    WAIT_END_OF_SENDING
};

uint32_t lastMpxReceivedUs = 0;  // used to check the delay between char. 
uint32_t mpxStartWaiting = 0; // timestamp (usec) when we start waiting 
MPXSTATES mpxState;



//uint32_t restoreMpxPioToReceiveMillis = 0; // when 0, the pio is normally in receive mode,
//                                        // otherwise, it is the timestamp when pio transmit has to be restore to receive mode

extern field fields[];  // list of all telemetry fields and parameters used by Sport

extern MS5611 baro1;
extern SPL06 baro2;
extern BMP280 baro3;

extern GPS gps;
extern CONFIG config;

void setupMpx() {                                                 
// configure the queue to get the data from Mpx in the irq handle
    queue_init (&mpxRxQueue, sizeof(uint8_t), 50);

// set up the DMA but do not yet start it to send data to Sport
// Configure a channel to write the same byte (8 bits) repeatedly to PIO0
// SM0's TX FIFO, placed by the data request signal from that peripheral.
    mpx_dma_chan = dma_claim_unused_channel(true);
    mpxDmaConfig = dma_channel_get_default_config(mpx_dma_chan);
    channel_config_set_read_increment(&mpxDmaConfig, true);
    channel_config_set_write_increment(&mpxDmaConfig, false);
    channel_config_set_dreq(&mpxDmaConfig, DREQ_PIO0_TX0);  // use state machine 0 
    channel_config_set_transfer_data_size(&mpxDmaConfig, DMA_SIZE_8);
    dma_channel_configure(
        mpx_dma_chan,
        &mpxDmaConfig,
        &pio0_hw->txf[0], // Write address (only need to set this once)
        &mpxTxBuffer[0],   // we use always the same buffer             
        0 , // do not yet provide the number of bytes (DMA cycles)
        false             // Don't start yet
    );
// Set up the state machine for transmit but do not yet start it (it starts only when a request from receiver is received)
    mpxOffsetTx = pio_add_program(mpxPio, &mpx_uart_tx_program);
    mpx_uart_tx_program_init(mpxPio, mpxSmTx, mpxOffsetTx, config.pinTlm, 38400 , false); // we use the same pin and baud rate for tx and rx, true means thet UART is inverted 

// set an irq on pio to handle a received byte
    irq_set_exclusive_handler( PIO0_IRQ_0 , mpxPioRxHandlerIrq) ;
    irq_set_enabled (PIO0_IRQ_0 , true) ;

// Set up the state machine we're going to use to receive them.
    mpxOffsetRx = pio_add_program(mpxPio, &mpx_uart_rx_program);
    mpx_uart_rx_program_init(mpxPio, mpxSmRx, mpxOffsetRx, config.pinTlm, 38400 , false); // false = not inverted   
    setupListMpxFieldsToReply();
}

void setupListMpxFieldsToReply(){
    uint8_t oXsCode; 
    for (uint8_t i= 0; i<= 15 ; i++) {
        oXsCode = convertMpxAddToFieldId[i];  
        switch (oXsCode) {
            case MVOLT:
                if (config.pinVolt[1] !=255) mpxFieldsToReply[i] = true;
                break;
            case CURRENT:
                if (config.pinVolt[2] !=255) mpxFieldsToReply[i] = true;
                break;
            case CAPACITY:
                if (config.pinVolt[2] !=255) mpxFieldsToReply[i] = true;
                break;
            case TEMP1:
                if ( (config.pinVolt[3] !=255) && ( (config.temperature == 1) || (config.temperature == 2) ) ) mpxFieldsToReply[i] = true;
                break;
            case VSPEED:
                if (baro1.baroInstalled || baro2.baroInstalled || baro3.baroInstalled ) mpxFieldsToReply[i] = true;
                break;
            case GROUNDSPEED:
                if (gps.gpsInstalled) mpxFieldsToReply[i] = true;
                break;            
            case RPM:
                if (config.pinRpm !=255) mpxFieldsToReply[i] = true;
                break;            
            case GPS_HOME_BEARING:
                if (gps.gpsInstalled) mpxFieldsToReply[i] = true;
                break;
            case GPS_HOME_DISTANCE:
                if (gps.gpsInstalled) mpxFieldsToReply[i] = true;
                break;     
            case RELATIVEALT:
                if (baro1.baroInstalled || baro2.baroInstalled || baro3.baroInstalled ) mpxFieldsToReply[i] = true;
                break;            
        } // end switch
    }    // end for
}

void mpxPioRxHandlerIrq(){    // when a byte is received on the mpx bus, read the pio mpx fifo and push the data to a queue (to be processed in the main loop)
  // clear the irq flag
  irq_clear (PIO0_IRQ_0 );
  while (  ! pio_sm_is_rx_fifo_empty (mpxPio ,mpxSmRx)){ // when some data have been received
     uint8_t c = pio_sm_get (mpxPio , mpxSmRx) >> 24;         // read the data
     //printf("%x", c);
     queue_try_add (&mpxRxQueue, &c);          // push to the queue
    //mpxRxMillis = micros();                    // save the timestamp.
  }
}

//#define DEBUG_MPX_WITHOUT_RX
#ifdef DEBUG_MPX_WITHOUT_RX
uint32_t lastMpxRequest = 0;
#endif
void handleMpxRxTx(void){   // main loop : restore receiving mode , wait for tlm request, prepare frame, start pio and dma to transmit it
    static uint8_t previous = 0;
    static uint8_t data;
    static uint8_t pollingSimulation;
    if (config.pinTlm == 255) return ; // skip when Tlm is not foreseen
    switch (mpxState) {
        case RECEIVING :
            #ifdef DEBUG_MPX_WITHOUT_RX  // simulate a RX sending a polling every 6 msec 
            if ( (millis() - lastMpxRequest) > 6)  { // to debug simulate a request once per 200msec
                mpxState = WAIT_FOR_SENDING;
                mpxStartWaiting = micros();
                data = pollingSimulation++;
                if (pollingSimulation >0X0F) pollingSimulation = 0;
                lastMpxRequest = millis();
            }
            break;
            #endif
            if (! queue_is_empty(&mpxRxQueue)) {
                queue_try_remove (&mpxRxQueue,&data);
                //printf("%X ", data);
                if ( data <= 0X0F && ( (micros() - lastMpxReceivedUs)) > 4000) { // when data is less than 16, it is a sensor polling 
                                                                                // check that it is a first char.
                    mpxState = WAIT_FOR_SENDING;
                    mpxStartWaiting = micros();
                    lastMpxReceivedUs = micros(); 
                    //printf("r\n");
                }
                
            }        
            break;
        case WAIT_FOR_SENDING :
            if ( ( micros() - mpxStartWaiting) > 1600 ) {  // wait 1600usec before sending the reply
                if (sendMpxFrame(data) ) { // return true when a frame is really being sent
                    mpxState = WAIT_END_OF_SENDING;
                    mpxStartWaiting = micros();
                    //printf("s\n");
                } else { 
                    mpxState = RECEIVING;
                }   
            }
            break;         
        /*
        case SENDING :   // wait that dma have sent all data (but the last bytes are not yet sent)
            if ( ! dma_channel_is_busy( mpx_dma_chan)	){
                mpxState = WAIT_END_OF_SENDING;
                mpxStartWaiting = micros();
            }
            break;         
        */
        case WAIT_END_OF_SENDING :
            if ( ( micros() - mpxStartWaiting) > (2500) ){ // wait that the 3 bytes have been sent (780 usec)
                mpx_uart_tx_program_stop(mpxPio, mpxSmTx, config.pinTlm );
                mpx_uart_rx_program_restart(mpxPio, mpxSmRx, config.pinTlm, false );  // false = not inverted
                mpxState = RECEIVING ;
                //printf("e\n");
            }
            break;     
    }
     
}


bool sendMpxFrame(uint8_t data_id){ // data_id is the address of the field to transmit
    bool sendingRequired = false;
    uint8_t fieldId = convertMpxAddToFieldId[data_id];
    int16_t mpxValue ;
    
    if ( fieldId == 0XFF) return false; // do not reply if this address is not supported
    if (mpxFieldsToReply[data_id] == false) return false; // do not reply if the hardware does not support it 
    //printf("da id val= %d %d %d %d\n", data_id , fieldId , fields[fieldId].value , fields[fieldId].available );
    
    if ( dma_channel_is_busy(mpx_dma_chan) ) {
        //printf("dma is busy\n");
        return false; // skip if the DMA is still sending data
    }
    //printf("da id= %d %d\n", data_id , fieldId);
    
    mpxTxBuffer[0] = (data_id << 4) | convertMpxAddToUnit[data_id] ; // 2X 4 bits (address and units)
    switch (fieldId) {
        case MVOLT:
            mpxValue= fields[fieldId].value / 100; // from mvolt to 0.1V
            break;
        case CURRENT:
            mpxValue= fields[fieldId].value / 100; // from mamp to 0.1A
            break;
        case CAPACITY:
            mpxValue= fields[fieldId].value ; // from mah to mah
            break;
        case TEMP1:
            mpxValue= fields[fieldId].value * 10; // from ° to 0.1°
            break;
        case VSPEED:
            mpxValue= fields[fieldId].value / 10; // from cm/sec to 0.1m/sec
            break;
        case GROUNDSPEED:
            mpxValue= fields[fieldId].value * 3600/ 100 /100; // from cm/sec to 0.1 km/h
            break;            
        case RPM:
            mpxValue= fields[fieldId].value *60 / 100 ; // from Hz to 100 tr/min???? not sure it is ok
            break;            
        case GPS_HOME_BEARING:
            mpxValue= fields[fieldId].value *10 ; // from  deg  to 0.1 deg
            break;
        case GPS_HOME_DISTANCE:
            mpxValue= fields[fieldId].value /100 ; // from  m  to 0.1km
            break;     
        case RELATIVEALT:
            mpxValue= fields[fieldId].value /100; // from cm to m
            break;            
    }
    mpxValue = mpxValue << 1; // Shift 1 X and do not set an alarm.
    #define MB_NOVALUE		0x8000
    if ( fields[fieldId].available == false) mpxValue = MB_NOVALUE; //overwrite with NO_VALUE as long this field has never been available (no reset in mpx protocol) 
    //printf("id= %d\n", data_id);
    mpxTxBuffer[1] = mpxValue ; //LOWER part
    mpxTxBuffer[2] = mpxValue >> 8; // upper part
    mpx_uart_rx_program_stop(mpxPio, mpxSmRx, config.pinTlm); // stop receiving
    mpx_uart_tx_program_start(mpxPio, mpxSmTx, config.pinTlm, false); // prepare to transmit
    // start the DMA channel with the data to transmit
    dma_channel_set_read_addr (mpx_dma_chan, &mpxTxBuffer[0], false);
    dma_channel_set_trans_count (mpx_dma_chan, 3, true) ;
    //printf("mpx %X %X %X Vs=%d A=%d\n", mpxTxBuffer[0] , mpxTxBuffer[1] ,mpxTxBuffer[2], fields[VSPEED].value , fields[RELATIVEALT].value  );
    return true;    
}


