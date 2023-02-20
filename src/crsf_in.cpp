#include "pico/stdlib.h"
#include <stdlib.h>
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "uart_crsf_tx.pio.h"
#include "crsf_in.h"
#include "config.h"
#include "crc.h"
#include "pico/util/queue.h"
#include "MS5611.h"
#include "SPL06.h"
#include "tools.h"
#include "stdio.h"
#include <string.h> // memcpy
#include "param.h"
#include <inttypes.h>

extern CONFIG config;

#define CRSF_UART_ID uart1 // used by primary receiver
#define CRSF2_UART_ID uart0 // used by secondary receiver

//#define DEBUGPRIM    // print a line when a frame is received from Primary ELRS receiver
//#define DEBUGSEC     // print a line when a frame is received from secondary ELRS receiver 

queue_t crsfRxQueue ; // primary queue uses to push the data from the uart rx to the main loop
queue_t crsf2RxQueue ; // secondary queue uses to push the data from the uart rx to the main loop

GENERIC_CRC8 crsf_crc_in(CRSF_CRC_POLY);

// primary RX interrupt handler
void on_crsf_uart_rx() {
    while (uart_is_readable(CRSF_UART_ID)) {
        uint8_t ch = uart_getc(CRSF_UART_ID);
        //int count = queue_get_level( &primCrsfRxQueue );
        //printf(" level = %i\n", count);
        //printf( "val = %X\n", ch);  // printf in interrupt generates error but can be tested for debugging if some char are received
        if (!queue_try_add ( &crsfRxQueue , &ch)) printf("crsfRxQueue try add error\n");
        //printf("%02x\n", ch);
    }
}

// secondary Rx interrupy handler
void on_crsf2_uart_rx() {
    while (uart_is_readable(CRSF2_UART_ID)) {
        uint8_t ch = uart_getc(CRSF2_UART_ID);
        //int count = queue_get_level( &primCrsfRxQueue );
        //printf(" level = %i\n", count);
        //printf( "val = %X\n", ch);  // printf in interrupt generates error but can be tested for debugging if some char are received
        if (!queue_try_add ( &crsf2RxQueue , &ch)) printf("crsf2RxQueue try add error\n");
        //printf("%x\n", ch);
    }
}


void setupCrsfIn(){
    uint8_t dummy;
    if (config.pinPrimIn == 255) return ; // skip if Prim is not defined 
    queue_init(&crsfRxQueue , 1 ,256) ;// queue for prim crsf uart with 256 elements of 1
    uart_init(CRSF_UART_ID, config.crsfBaudrate);   // setup UART baud rate
    uart_set_hw_flow(CRSF_UART_ID, false, false);// Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_fifo_enabled(CRSF_UART_ID, false);    // Turn off FIFO's - we want to do this character by character
    gpio_set_function( config.pinPrimIn , GPIO_FUNC_UART); // Set the GPIO pin mux to the UART
    //gpio_set_inover(SBUS_RX_PIN , GPIO_OVERRIDE_INVERT);
    //uart_set_format (PRIM_CRSF_UART_ID, 8, 1 ,UART_PARITY_NONE ) ;
    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART1_IRQ, on_crsf_uart_rx);
    irq_set_enabled(UART1_IRQ, true);
    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(CRSF_UART_ID, true, false);
    while (! queue_is_empty (&crsfRxQueue)) queue_try_remove ( &crsfRxQueue , &dummy ) ;
    #ifdef DEBUGPRIM
    printf("Prim is initialized\n");
    #endif   
}

void setupCrsf2In(){
    uint8_t dummy;
    if (config.pinSecIn == 255) return ; // skip when pinSecIn is not defined
    queue_init(&crsf2RxQueue , 1 ,256) ;// queue for prim crsf uart with 256 elements of 1
    uart_init(CRSF2_UART_ID, config.crsfBaudrate);   // setup UART baud rate
    uart_set_hw_flow(CRSF2_UART_ID, false, false);// Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_fifo_enabled(CRSF2_UART_ID, false);    // Turn off FIFO's - we want to do this character by character
    gpio_set_function( config.pinSecIn , GPIO_FUNC_UART); // Set the GPIO pin mux to the UART0
    //gpio_set_inover(SBUS_RX_PIN , GPIO_OVERRIDE_INVERT);
    //uart_set_format (PRIM_CRSF2UART_ID, 8, 1 ,UART_PARITY_NONE ) ;
    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART0_IRQ, on_crsf2_uart_rx);
    irq_set_enabled(UART0_IRQ, true);
    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(CRSF2_UART_ID, true, false);
    while (! queue_is_empty (&crsf2RxQueue)) queue_try_remove ( &crsf2RxQueue , &dummy ) ;    
    #ifdef DEBUGSEC
    printf("Sec is initialized\n");
    #endif   
}

// here the code to read the CRSF frames from the receiver (in order to get the RC channels data)

// a CRSF RC channel frame contains
// 1 byte with CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
// 1 byte with the payload length + 2 (type and crc) RC_PAYLOAD_LENGTH_PLUS2 = 24
// 1 byte with the type : CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
// 22 bytes = 16 channels (11 bits)
// 1 byte with the crc

#define CRSF_ADDRESS_FLIGHT_CONTROLLER    0xC8
#define RC_PAYLOAD_LENGTH                   22 // does not include the Type and CRC
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16

sbusFrame_s sbusFrame;

enum CRSFState{
    NO_FRAME = 0,
    WAIT_PAYLOAD_LENGTH,
    WAIT_FRAMETYPE_RC_CHANNELS_PACKED,
    RECEIVING_RC_CHANNELS,
    WAIT_CRC
} ;

uint32_t lastRcChannels = 0;   // used in crsf.cpp and in sbus_in.cpp to say that we got Rc channels data
uint32_t lastPriChannelsMillis = 0; // used in crsf.cpp and in sbus_in.cpp to say that we got Rc channels data
uint32_t lastSecChannelsMillis = 0; // used in crsf.cpp and in sbus_in.cpp to say that we got Rc channels data


void handleCrsfIn(void){   // called by main loop : receive the CRSF frame
    static uint8_t crsfRxState = NO_FRAME;
    static uint8_t crsfCounter = 0;
    static uint8_t crsfBufferRcChannels[RC_PAYLOAD_LENGTH];
    uint8_t data;
    uint8_t crc = 0; 
    if (config.pinPrimIn == 255) return ; // skip if Prim is not defined 
    
    while (! queue_is_empty(&crsfRxQueue)) {
        queue_try_remove (&crsfRxQueue,&data);
        //printf(" %02x ",data);
        switch ( crsfRxState ) {
            case NO_FRAME:
                if (data == CRSF_ADDRESS_FLIGHT_CONTROLLER) crsfRxState = WAIT_PAYLOAD_LENGTH;
            break;
            case  WAIT_PAYLOAD_LENGTH:
                if (data == (RC_PAYLOAD_LENGTH + 2)){
                    crsfRxState = WAIT_FRAMETYPE_RC_CHANNELS_PACKED;
                } else if (data == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
                    crsfRxState = WAIT_PAYLOAD_LENGTH;
                } else {
                    crsfRxState = NO_FRAME ;
                }
            break;
            case  WAIT_FRAMETYPE_RC_CHANNELS_PACKED:
                if (data == CRSF_FRAMETYPE_RC_CHANNELS_PACKED){
                    crsfRxState = RECEIVING_RC_CHANNELS;
                    crsfCounter = 0;
                } else if (data == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
                    crsfRxState = WAIT_PAYLOAD_LENGTH;
                } else {
                    crsfRxState = NO_FRAME ;
                }
            break;
            case  RECEIVING_RC_CHANNELS:
                crsfBufferRcChannels[crsfCounter++] = data;
                if ( crsfCounter == RC_PAYLOAD_LENGTH ) {
                    crsfRxState = WAIT_CRC ;
                }                   
            break;
            case  WAIT_CRC:
                crc = crsf_crc_in.calc(CRSF_FRAMETYPE_RC_CHANNELS_PACKED); // CRC calculation includes the Type of message
                crc = crsf_crc_in.calc(&crsfBufferRcChannels[0] ,  RC_PAYLOAD_LENGTH , crc);
                if ( crc == data){
                    // we got a good frame; we can save for later use
                    memcpy(&sbusFrame.rcChannelsData, crsfBufferRcChannels , RC_PAYLOAD_LENGTH) ;
                    lastRcChannels = millisRp();
                    lastPriChannelsMillis = lastRcChannels ;
                    #ifdef DEBUGPRIM
                    printf("Prim= ");
                    for (uint8_t i=0 ; i < RC_PAYLOAD_LENGTH; i++) {
                        printf(" %02X ", crsfBufferRcChannels[i]);
                    }
                    printf("\n");    
                    #endif
                    //printf("Good RC received\n");
                } else {
                    //printf("bad CRC received\n");
                }
                crsfRxState = NO_FRAME;
            break;
        }
    }        
}

void handleCrsf2In(void){   // called by main loop : receive the CRSF frame
    static uint8_t crsf2RxState = NO_FRAME;
    static uint8_t crsf2Counter = 0;
    static uint8_t crsf2BufferRcChannels[RC_PAYLOAD_LENGTH];
    uint8_t data;
    uint8_t crc = 0; 
    if (config.pinSecIn == 255) return ; // skip when pinSecIn is not defined
    
    while (! queue_is_empty(&crsf2RxQueue)) {
        queue_try_remove (&crsf2RxQueue,&data);
        //printf("q=  %02x \n",data);
        switch ( crsf2RxState ) {
            case NO_FRAME:
                if (data == CRSF_ADDRESS_FLIGHT_CONTROLLER) crsf2RxState = WAIT_PAYLOAD_LENGTH;
            break;
            case  WAIT_PAYLOAD_LENGTH:
                if (data == (RC_PAYLOAD_LENGTH + 2)){
                    crsf2RxState = WAIT_FRAMETYPE_RC_CHANNELS_PACKED;
                } else if (data == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
                    crsf2RxState = WAIT_PAYLOAD_LENGTH;
                } else {
                    crsf2RxState = NO_FRAME ;
                }
            break;
            case  WAIT_FRAMETYPE_RC_CHANNELS_PACKED:
                if (data == CRSF_FRAMETYPE_RC_CHANNELS_PACKED){
                    crsf2RxState = RECEIVING_RC_CHANNELS;
                    crsf2Counter = 0;
                } else if (data == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
                    crsf2RxState = WAIT_PAYLOAD_LENGTH;
                } else {
                    crsf2RxState = NO_FRAME ;
                }
            break;
            case  RECEIVING_RC_CHANNELS:
                crsf2BufferRcChannels[crsf2Counter++] = data;
                if ( crsf2Counter == RC_PAYLOAD_LENGTH ) {
                    crsf2RxState = WAIT_CRC ;
                }                   
            break;
            case  WAIT_CRC:
                crc = crsf_crc_in.calc(CRSF_FRAMETYPE_RC_CHANNELS_PACKED); // CRC calculation includes the Type of message
                crc = crsf_crc_in.calc(&crsf2BufferRcChannels[0] ,  RC_PAYLOAD_LENGTH , crc);
                if ( crc == data){
                    // we got a good frame; we can save for later use
                    memcpy(&sbusFrame.rcChannelsData, crsf2BufferRcChannels , RC_PAYLOAD_LENGTH) ;
                    lastRcChannels = millisRp();
                    lastSecChannelsMillis = lastRcChannels ; 
                    //printf("S\n");
                    #ifdef DEBUGSEC
                    printf("Sec = ");
                    for (uint8_t i=0 ; i < RC_PAYLOAD_LENGTH; i++) {
                        printf(" %02X ", crsf2BufferRcChannels[i]);
                    }
                    printf("\n");    
                    #endif
                    //printf("Good RC received\n");
                } else {
                    //printf("bad CRC received\n");
                }
                crsf2RxState = NO_FRAME;
            break;
        }
    }        
}

