#include "pico/stdlib.h"
#include "stdio.h"  // used by printf
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico/util/queue.h"
#include "sbus_in.h"
//#include "crsf.h"
#include "tools.h"
#include <string.h> // used by memcpy
#include "param.h"
#include "sbus2_tlm.h"


#define SBUS_UART_ID uart1
#define SBUS2_UART_ID uart0
//#define SBUS_RX_PIN 9
#define SBUS_BAUDRATE 100000
queue_t sbusQueue ;
queue_t sbus2Queue ;

enum SBUS_STATE{
  NO_SBUS_FRAME = 0,
  RECEIVING_SBUS 
};

extern CONFIG config;
extern sbusFrame_s sbusFrame; // full frame including header and End bytes; To generate PWM , we use only the RcChannels part.
extern sbusFrame_s sbus2Frame; // full frame including header and End bytes; To generate PWM , we use only the RcChannels part.


uint8_t runningSbusFrame[25];  // data are accumulated in this buffer and transfered to sbusFrame when the frame is complete and valid
uint8_t runningSbus2Frame[25];  // data are accumulated in this buffer and transfered to sbusFrame when the frame is complete and valid

//extern rcFrameStruct crsfRcFrame ; // buffer used by crsf.cpp to fill the RC frame sent to ELRS 
extern uint32_t lastRcChannels ;     // Time stamp of last valid rc channels data
extern uint32_t lastPriChannelsMillis; // used in crsf.cpp and in sbus_in.cpp to say that we got Rc channels data
extern uint32_t lastSecChannelsMillis; // used in crsf.cpp and in sbus_in.cpp to say that we got Rc channels data

bool sbusPriMissingFlag = true;
bool sbusSecMissingFlag = true;
bool sbusPriFailsafeFlag = true;
bool sbusSecFailsafeFlag = true;

uint32_t sbusHoldCounter = 0;
uint32_t sbusHoldCounterTotal = 0;
uint32_t sbusFailsafeCounter = 0;
uint32_t sbusFrameCounter = 0;
#define SBUS_HOLD_COUNTED_ON_FRAMES 100 // calculate the % every X frames
bool prevFailsafeFlag = false;

// RX interrupt handler on one uart
void on_sbus_uart_rx() {
    while (uart_is_readable(SBUS_UART_ID)) {
        //printf(".\n");
        uint8_t ch = uart_getc(SBUS_UART_ID);
        int count = queue_get_level( &sbusQueue );
        //printf(" level = %i\n", count);
        //printf( "val = %X\n", ch);  // printf in interrupt generates error but can be tested for debugging if some char are received
        if (!queue_try_add ( &sbusQueue , &ch)) printf("sbusQueue try add error\n");
        //printf("%x\n", ch);
    }
}

// RX interrupt handler on one uart
void on_sbus2_uart_rx() {
    while (uart_is_readable(SBUS2_UART_ID)) {
        //printf(".\n");
        uint8_t ch = uart_getc(SBUS2_UART_ID);
        //int count = queue_get_level( &sbus2Queue );
        //printf(" level = %i\n", count);
        //printf( "val = %X\n", ch);  // printf in interrupt generates error but can be tested for debugging if some char are received
        if (!queue_try_add ( &sbus2Queue , &ch)) printf("sbusQueue2 try add error\n");
        //printf("%x\n", ch);
    }
}

#include "hardware/address_mapped.h"
#include "hardware/platform_defs.h"
#include "hardware/uart.h"

#include "hardware/structs/uart.h"
#include "hardware/resets.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "pico/assert.h"
#include "pico.h"


uint uart_init_extended(uart_inst_t *uart, uint baudrate , uint data_bits, uint stop_bits, uart_parity_t parity, bool fifoEnabled) {
    invalid_params_if(UART, uart != uart0 && uart != uart1);

    if (clock_get_hz(clk_peri) == 0)
        return 0;
    reset_block(uart_get_index(uart) ? RESETS_RESET_UART1_BITS : RESETS_RESET_UART0_BITS);
    unreset_block_wait(uart_get_index(uart) ? RESETS_RESET_UART1_BITS : RESETS_RESET_UART0_BITS);
    
#if PICO_UART_ENABLE_CRLF_SUPPORT
    uart_set_translate_crlf(uart, PICO_UART_DEFAULT_CRLF);
#endif
    // Any LCR writes need to take place before enabling the UART
    uint baud = uart_set_baudrate(uart, baudrate);
    uart_set_format(uart, data_bits, stop_bits, parity);
    // Enable/disable FIFOs
    uart_set_fifo_enabled(uart, fifoEnabled);
    // Always enable DREQ signals -- no harm in this if DMA is not listening
    uart_get_hw(uart)->dmacr = UART_UARTDMACR_TXDMAE_BITS | UART_UARTDMACR_RXDMAE_BITS;
    // Enable the UART, both TX and RX
    uart_get_hw(uart)->cr = UART_UARTCR_UARTEN_BITS | UART_UARTCR_TXE_BITS | UART_UARTCR_RXE_BITS;
    
    return baud;
}
/// \end::uart_init[]



void setupSbusIn(){
    uint8_t dummy;
    if (config.pinPrimIn == 255) return ; // skip when pinPrimIn is not defined
    queue_init(&sbusQueue , 1 ,256) ;// queue for sbus uart with 256 elements of 1
    
    uart_init_extended(SBUS_UART_ID, SBUS_BAUDRATE , 8, 2, UART_PARITY_EVEN, false);   // setup UART at 100000 baud, 8 bits, 2 stops, EVEN, no fifo
    //uart_get_hw(SBUS_UART_ID)->cr = 0;    // disable uart    // this seems required when uart is not 8N1 (bug in sdk)
    //sleep_us(100);
    //while ( uart_get_hw(SBUS_UART_ID)->fr & UART_UARTFR_BUSY_BITS) {}; // wait that UART is not busy 
    //uart_set_format(SBUS_UART_ID, 8, 2, UART_PARITY_EVEN);        // format may be changed only when uart is disabled and not busy
    
    //uart_set_hw_flow(SBUS_UART_ID, false, false);// Set UART flow control CTS/RTS, we don't want these, so turn them off
    //uart_set_fifo_enabled(SBUS_UART_ID, false);    // Turn off FIFO's 
    // uart_get_hw(uart1)->cr = UART_UARTCR_UARTEN_BITS | UART_UARTCR_TXE_BITS | UART_UARTCR_RXE_BITS; // this seems required when uart is not 8N1 (bug in sdk)    
   
    //uart_set_format (SBUS_UART_ID, 8, 2 ,UART_PARITY_EVEN ) ;
    
    //gpio_set_function(SBUS_TX_PIN , GPIO_FUNC_UART); // Set the GPIO pin mux to the UART 
    gpio_set_function( config.pinPrimIn , GPIO_FUNC_UART);
    gpio_set_inover( config.pinPrimIn , GPIO_OVERRIDE_INVERT);
    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART1_IRQ, on_sbus_uart_rx);
    irq_set_enabled(UART1_IRQ, true);
    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(SBUS_UART_ID, true, false);
    
    while (! queue_is_empty (&sbusQueue)) queue_try_remove ( &sbusQueue , &dummy ) ;
}
void setupSbus2In(){
    uint8_t dummy;
    if (config.pinSecIn == 255) return ; // skip when pinSecIn is not defined
    queue_init(&sbus2Queue , 1 ,256) ;// queue for sbus uart with 256 elements of 1
    
    uart_init_extended(SBUS2_UART_ID, SBUS_BAUDRATE , 8, 2, UART_PARITY_EVEN, false);   // setup UART at 100000 baud, 8 bits, 2 stops, EVEN, no fifo
    //uart_get_hw(SBUS2_UART_ID)->cr = 0;                              // this seems required when uart is not 8N1 (bug in sdk)
    //while ( uart_get_hw(SBUS2_UART_ID)->fr & UART_UARTFR_BUSY_BITS) {}; // wait that UART is not busy 
    //uart_set_format(SBUS2_UART_ID, 8, 2, UART_PARITY_EVEN);
    //uart_set_hw_flow(SBUS2_UART_ID, false, false);// Set UART flow control CTS/RTS, we don't want these, so turn them off
    //uart_set_fifo_enabled(SBUS2_UART_ID, false);    // Turn off FIFO's 
    //uart_get_hw(SBUS2_UART_ID)->cr = UART_UARTCR_UARTEN_BITS | UART_UARTCR_TXE_BITS | UART_UARTCR_RXE_BITS; // this seems required when uart is not 8N1 (bug in sdk)    
    
    //gpio_set_function(SBUS_TX_PIN , GPIO_FUNC_UART); // Set the GPIO pin mux to the UART 
    gpio_set_function( config.pinSecIn , GPIO_FUNC_UART);
    gpio_set_inover( config.pinSecIn , GPIO_OVERRIDE_INVERT);
    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART0_IRQ, on_sbus2_uart_rx);
    irq_set_enabled(UART0_IRQ, true);
    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(SBUS2_UART_ID, true, false);
    while (! queue_is_empty (&sbus2Queue)) queue_try_remove ( &sbus2Queue , &dummy ) ;
    
}  // end setup sbus
   
void handleSbusIn(){
    static SBUS_STATE sbusState = NO_SBUS_FRAME ;
    static uint8_t sbusCounter = 0;
    static uint32_t lastSbusMillis = 0;
    uint8_t c;
    if (config.pinPrimIn ==255) return ; // skip when pinPrimIn is not defined
    while (! queue_is_empty (&sbusQueue) ){
        if ( (millisRp() - lastSbusMillis ) > 2 ){
        sbusState = NO_SBUS_FRAME ;
        }
        lastSbusMillis = millisRp();
        queue_try_remove ( &sbusQueue , &c);
        //printf(" %X\n",c);
        switch (sbusState) {
        case NO_SBUS_FRAME :
            if (c == 0x0F) {
            sbusCounter = 1;
            sbusState = RECEIVING_SBUS ;
            }
        break;
        case RECEIVING_SBUS :
            runningSbusFrame[sbusCounter++] = c;
            if (sbusCounter == 25 ) {  // 25 because SbusCounter is already pointing to next byte 
            if ( (c != 0x00) && (c != 0x04) && (c != 0x14) && (c != 0x24) && (c != 0x34) ) {
                //printf("fs=%X\n", c);
                sbusState = NO_SBUS_FRAME;
            } else {
                // for Futaba protocol, if we get a Sbus2 frame and if tlm pin is defined we build 8 slots
                if ( (config.protocol == '2') && ( config.pinTlm != 255) &&
                    ((c == 0x04) || (c == 0x14) || (c == 0x24) || (c == 0x34) )) fill8Sbus2Slots(c>>4);
                storeSbusFrame();
                sbusState = NO_SBUS_FRAME;
            }
            }
        break;      
        }
    } // end while
    if ( sbusFrameCounter >= SBUS_HOLD_COUNTED_ON_FRAMES) {
        #ifdef SEND_TOTAL_HOLD
        sent2Core0(SBUS_HOLD_COUNTER , sbusHoldCounterTotal ); // total number of hold
        #else
        sent2Core0(SBUS_HOLD_COUNTER , sbusHoldCounter * 100 / sbusFrameCounter); // * 100 because the value is in %
        #endif
        sbusHoldCounter = 0;             // reset the counter
        sbusFrameCounter = 0;
        sent2Core0(SBUS_FAILSAFE_COUNTER , sbusFailsafeCounter);  // for failsafe, we just count the total number
    }
    #ifdef SIMULATE_SBUS2_ON_PIN
    //printf("h sbus in\n");
    generateSbus2RcPacket();
    #endif     
  }
  

void handleSbus2In(){
  static SBUS_STATE sbus2State = NO_SBUS_FRAME ;
  static uint8_t sbus2Counter = 0;
  static uint32_t lastSbus2Millis = 0;
  uint8_t c;
if (config.pinSecIn == 255) return ; // skip when pinSecIn is not defined
    while (! queue_is_empty (&sbus2Queue) ){
        if ( (millisRp() - lastSbus2Millis ) > 2 ){
        sbus2State = NO_SBUS_FRAME ;
        }
        lastSbus2Millis = millisRp();
        queue_try_remove ( &sbus2Queue , &c);
        //printf(" %X\n",c);
        switch (sbus2State) {
        case NO_SBUS_FRAME :
            if (c == 0x0F) {
            sbus2Counter = 1;
            sbus2State = RECEIVING_SBUS ;
            }
        break;
        case RECEIVING_SBUS :
            runningSbus2Frame[sbus2Counter++] = c;
            if (sbus2Counter == 25 ) {
            if ( (c != 0x00) && (c != 0x04) && (c != 0x14) && (c != 0x24) && (c != 0x34) ) {
                sbus2State = NO_SBUS_FRAME;
            } else {
                storeSbus2Frame();
                sbus2State = NO_SBUS_FRAME;
            }
            }
        break;      
        }
    }     
    if ( sbusFrameCounter >= SBUS_HOLD_COUNTED_ON_FRAMES) {
        sent2Core0(SBUS_HOLD_COUNTER , sbusHoldCounter * 100 / sbusFrameCounter); // * 100 because the value is in %
        sbusHoldCounter = 0;             // reset the counter
        sbusFrameCounter = 0;
        sent2Core0(SBUS_FAILSAFE_COUNTER , sbusFailsafeCounter);  // for failsafe, we just count the total number
    }
}


void storeSbusFrame(){      // running SbusFrame[0] is supposed to be 0X0F, channels are coded from byte [0]
    sbusPriMissingFlag = (runningSbusFrame[23] >> 2) & 0X01;
    sbusPriFailsafeFlag = (runningSbusFrame[23] >> 3) & 0X01;
    if ( ( config.pinSecIn == 255 || ( ( millisRp() - lastSecChannelsMillis )  > 50 )))  {
        sbusFrameCounter++;
        if ( sbusPriMissingFlag ) {
            sbusHoldCounter++;
            sbusHoldCounterTotal++;
        }
        if ( sbusPriFailsafeFlag && (prevFailsafeFlag == false)) sbusFailsafeCounter++;
        prevFailsafeFlag = sbusPriFailsafeFlag; 
    }    
    if ((( sbusPriMissingFlag == false) && (sbusPriFailsafeFlag == false)) || // copy when frame is OK   
        ( sbusSecFailsafeFlag)  ||                                            //   or previous SEC is failsafe
        ( ( millisRp() - lastSecChannelsMillis )  > 50 )) {                     //   or SEC do not exist                   
        memcpy(  (uint8_t *) &sbusFrame.rcChannelsData, &runningSbusFrame[1], 22);
    }
    lastRcChannels = millisRp();
    lastPriChannelsMillis =  lastRcChannels;
    //float rc1 = ((runningSbusFrame[1]   |runningSbusFrame[2]<<8) & 0x07FF);
    //printf("rc1 = %f\n", rc1/2);
    //printf("sbus received\n");
}  

void storeSbus2Frame(){
    sbusSecMissingFlag = (runningSbus2Frame[23] >> 2) & 0X01;
    sbusSecFailsafeFlag = (runningSbus2Frame[23] >> 3) & 0X01;
    if ( ( config.pinPrimIn == 255) || (( millisRp() - lastPriChannelsMillis )  > 50 ) ) {
        sbusFrameCounter++;
        if ( sbusSecMissingFlag ){
            sbusHoldCounter++;
            sbusHoldCounterTotal++;
        } 
        if ( sbusSecFailsafeFlag && (prevFailsafeFlag == false)) sbusFailsafeCounter++;
        prevFailsafeFlag = sbusSecFailsafeFlag;
    } 
    if ((( sbusSecMissingFlag == false) && (sbusSecFailsafeFlag == false))  ||                               // copy when frame is OK   
        (( sbusSecMissingFlag == true) && (sbusSecFailsafeFlag == false) && (sbusPriFailsafeFlag == true)) || // or previous PRI is failsafe and SEC is only missing
        (( millisRp() - lastPriChannelsMillis )  > 50 )) {                                                      // or PRI do not exist           
        memcpy(  (uint8_t *) &sbusFrame.rcChannelsData, &runningSbus2Frame[1], 22);
    }
    lastRcChannels = millisRp();
    lastSecChannelsMillis =  lastRcChannels; 
    //float rc1 = ((runningSbusFrame[1]   |runningSbusFrame[2]<<8) & 0x07FF);
    //printf("rc1 = %f\n", rc1/2);
    //printf("sbus received\n");
}  

