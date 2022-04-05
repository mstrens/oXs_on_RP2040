#include "pico/stdlib.h"
#include "stdio.h"  // used by printf
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico/util/queue.h"
#include "sbus_in.h"
#include "crsf.h"
#include "tools.h"
#include <string.h> // used by memcpy

#define SBUS_UART_ID uart1
#define SBUS_RX_PIN 9
#define SBUS_BAUDRATE 100000
queue_t sbusQueue ;

enum SBUS_STATE{
  NO_SBUS_FRAME = 0,
  RECEIVING_SBUS 
};

extern sbusFrame_s sbusFrame; // full frame including header and End bytes; To generate PWM , we use only the RcChannels part.

uint8_t runningSbusFrame[24];  // data are accumulated in this buffer and transfered to sbusFrame when the frame is complete and valid
//extern rcFrameStruct crsfRcFrame ; // buffer used by crsf.cpp to fill the RC frame sent to ELRS 
extern uint32_t lastRcChannels ;     // Time stamp of last valid rc channels data

// RX interrupt handler
void on_sbus_uart_rx() {
    while (uart_is_readable(SBUS_UART_ID)) {
        //printf(".\n");
        uint8_t ch = uart_getc(SBUS_UART_ID);
        int count = queue_get_level( &sbusQueue );
        //printf(" level = %i\n", count);
        //printf( "val = %X\n", ch);  // printf in interrupt generates error but can be tested for debugging if some char are received
        if (!queue_try_add ( &sbusQueue , &ch)) printf("queue try add error\n");
        //printf("%x\n", ch);
    }
}


void setupSbusIn(){
    queue_init(&sbusQueue , 1 ,256) ;// queue for sbus uart with 256 elements of 1
    
    uart_init(SBUS_UART_ID, SBUS_BAUDRATE);   // setup UART at 100000 baud
    uart_set_hw_flow(SBUS_UART_ID, false, false);// Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_fifo_enabled(SBUS_UART_ID, false);    // Turn on FIFO's 
    uart_set_format (SBUS_UART_ID, 8, 2 ,UART_PARITY_EVEN ) ;
    
    //gpio_set_function(SBUS_TX_PIN , GPIO_FUNC_UART); // Set the GPIO pin mux to the UART 
    gpio_set_function(SBUS_RX_PIN , GPIO_FUNC_UART);
    gpio_set_inover(SBUS_RX_PIN , GPIO_OVERRIDE_INVERT);
    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART1_IRQ, on_sbus_uart_rx);
    irq_set_enabled(UART1_IRQ, true);
    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(SBUS_UART_ID, true, false);
    uint8_t dummy;
    while (! queue_is_empty (&sbusQueue)) queue_try_remove ( &sbusQueue , &dummy ) ;
}  // end setup sbus
   
void handleSbusIn(){
  static SBUS_STATE sbusState = NO_SBUS_FRAME ;
  static uint8_t sbusCounter = 0;
  static uint32_t lastSbusMillis = 0;
  uint8_t c;
  while (! queue_is_empty (&sbusQueue) ){
    
    if ( (millis() - lastSbusMillis ) > 2 ){
      sbusState = NO_SBUS_FRAME ;
    }
    lastSbusMillis = millis();
    queue_try_remove ( &sbusQueue , &c);
    //printf(" %X",c);
    switch (sbusState) {
      case NO_SBUS_FRAME :
        if (c == 0x0F) {
          sbusCounter = 1;
          sbusState = RECEIVING_SBUS ;
        }
      break;
      case RECEIVING_SBUS :
        runningSbusFrame[sbusCounter++] = c;
        if (sbusCounter == 24 ) {
          if ( (c != 0x00) && (c != 0x04) && (c != 0x14) && (c != 0x24) && (c != 0x34) ) {
            sbusState = NO_SBUS_FRAME;
          } else {
            storeSbusFrame();
            sbusState = NO_SBUS_FRAME;
          }
        }
      break;      
    } 
  }
}


void storeSbusFrame(){
    memcpy(  (uint8_t *) &sbusFrame.rcChannelsData, &runningSbusFrame[1], 22);
    lastRcChannels = millis(); 
    //float rc1 = ((runningSbusFrame[1]   |runningSbusFrame[2]<<8) & 0x07FF);
    //printf("rc1 = %f\n", rc1/2);
    //printf("sbus received\n");

}  
