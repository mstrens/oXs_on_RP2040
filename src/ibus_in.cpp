#include "pico/stdlib.h"
#include "stdio.h"  // used by printf
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico/util/queue.h"
#include "ibus_in.h"
//#include "crsf.h"
#include "tools.h"
#include <string.h> // used by memcpy
#include "param.h"
//#include "sbus2_tlm.h"


#define IBUS_UART_ID uart1
#define IBUS2_UART_ID uart0
//#define SBUS_RX_PIN 9
#define IBUS_BAUDRATE 115200
#define IBUS_IN_FREE_MICROS 3000
queue_t ibusInQueue ;
queue_t ibus2InQueue ;

enum IBUS_IN_STATE{
  NO_IBUS_IN_FRAME = 0,
  RECEIVING_IBUS_IN 
};

uint32_t ibusInMicros;

extern CONFIG config;
extern sbusFrame_s sbusFrame; // full frame including header and End bytes; To generate PWM , we use only the RcChannels part.
extern sbusFrame_s sbus2Frame; // full frame including header and End bytes; To generate PWM , we use only the RcChannels part.
extern bool newRcChannelsReceivedForPWM ;  // used to update the PWM data


uint8_t runningIbusFrame[32];  // data are accumulated in this buffer and transfered to sbusFrame when the frame is complete and valid
uint8_t runningIbus2Frame[32];  // data are accumulated in this buffer and transfered to sbusFrame when the frame is complete and valid

//extern rcFrameStruct crsfRcFrame ; // buffer used by crsf.cpp to fill the RC frame sent to ELRS 
extern uint32_t lastRcChannels ;     // Time stamp of last valid rc channels data
extern uint32_t lastPriChannelsMillis; // used in crsf.cpp and in sbus_in.cpp to say that we got Rc channels data
extern uint32_t lastSecChannelsMillis; // used in crsf.cpp and in sbus_in.cpp to say that we got Rc channels data

extern volatile bool isPrinting; // avoid error msg in irq handler filling the queue while printing 

//bool sbusPriMissingFlag = true;
//bool sbusSecMissingFlag = true;
//bool sbusPriFailsafeFlag = true;
//bool sbusSecFailsafeFlag = true;

//uint32_t sbusHoldCounter = 0;
//uint32_t sbusHoldCounterTotal = 0;
//uint32_t sbusFailsafeCounter = 0;
//uint32_t sbusFrameCounter = 0;
//#define SBUS_HOLD_COUNTED_ON_FRAMES 100 // calculate the % every X frames
//bool prevFailsafeFlag = false;

/*
 *  supports max 14 channels in this lib (with messagelength of 0x20 there is room for 14 channels)

  Example set of bytes coming over the iBUS line for setting servos: 
    20 40 DB 5 DC 5 54 5 DC 5 E8 3 D0 7 D2 5 E8 3 DC 5 DC 5 DC 5 DC 5 DC 5 DC 5 DA F3
  Explanation
    Protocol length: 20
    Command code: 40 
    Channel 0: DB 5  -> value 0x5DB
    Channel 1: DC 5  -> value 0x5Dc
    Channel 2: 54 5  -> value 0x554
    Channel 3: DC 5  -> value 0x5DC
    Channel 4: E8 3  -> value 0x3E8
    Channel 5: D0 7  -> value 0x7D0
    Channel 6: D2 5  -> value 0x5D2
    Channel 7: E8 3  -> value 0x3E8
    Channel 8: DC 5  -> value 0x5DC
    Channel 9: DC 5  -> value 0x5DC
    Channel 10: DC 5 -> value 0x5DC
    Channel 11: DC 5 -> value 0x5DC
    Channel 12: DC 5 -> value 0x5DC
    Channel 13: DC 5 -> value 0x5DC
    Checksum: DA F3 -> calculated by adding up all previous bytes, total must be FFFF
 */


// RX interrupt handler on one uart
void on_ibus_uart_rx() {
    uint32_t nowMicros = microsRp();
    while (uart_is_readable(IBUS_UART_ID)) {
        //printf(".\n");
        uint16_t ch = (uint16_t) uart_getc(IBUS_UART_ID);
        if ( ( nowMicros - ibusInMicros) > IBUS_IN_FREE_MICROS ) ch |= 0X8000 ; // set a flag when it is the first char since a delay
          //int count = queue_get_level( &ibusInQueue );
          //printf(" level = %i\n", count);
        //printf( "put= %X\n", ch);  // printf in interrupt generates error but can be tested for debugging if some char are received
        if (!queue_try_add ( &ibusInQueue , &ch)){
            if ( ! isPrinting) printf("ibusInQueue try add error\n"); 
        } 
        ibusInMicros = nowMicros;                    // save the timestamp.    
        //printf("%x\n", ch);
    }
}

// RX interrupt handler on one uart
/*
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
*/
#include "hardware/address_mapped.h"
#include "hardware/platform_defs.h"
#include "hardware/uart.h"

#include "hardware/structs/uart.h"
#include "hardware/resets.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "pico/assert.h"
#include "pico.h"


uint ibus_uart_init_extended(uart_inst_t *uart, uint baudrate , uint data_bits, uint stop_bits, uart_parity_t parity, bool fifoEnabled) {
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



void setupIbusIn(){
    uint16_t dummy;
    if (config.pinPrimIn == 255) return ; // skip when pinPrimIn is not defined
    queue_init(&ibusInQueue , sizeof(uint16_t), 250) ;// queue for sbus uart with 250 elements of 2 bytes
    
    ibus_uart_init_extended(IBUS_UART_ID, IBUS_BAUDRATE , 8, 1, UART_PARITY_NONE, false);   // setup UART at 115200 baud, 8 bits, 1 stops, No parity, no fifo
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
    //gpio_set_inover( config.pinPrimIn , GPIO_OVERRIDE_INVERT);
    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART1_IRQ, on_ibus_uart_rx);
    irq_set_enabled(UART1_IRQ, true);
    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(IBUS_UART_ID, true, false);
    
    while (! queue_is_empty (&ibusInQueue)) queue_try_remove ( &ibusInQueue , &dummy ) ;
}
/*
void setupSbus2In(){
    uint8_t dummy;
    if (config.pinSecIn == 255) return ; // skip when pinSecIn is not defined
    queue_init(&sbus2Queue , 1 ,256) ;// queue for sbus uart with 256 elements of 1
    
    ibus_uart_init_extended(SBUS2_UART_ID, SBUS_BAUDRATE , 8, 2, UART_PARITY_EVEN, false);   // setup UART at 100000 baud, 8 bits, 2 stops, EVEN, no fifo
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
*/ 

void handleIbusIn(){
    static IBUS_IN_STATE ibusInState = NO_IBUS_IN_FRAME ;
    static uint8_t ibusInCounter = 0;
    static uint32_t lastIbusMillis = 0;
    uint16_t c;
    static uint16_t checksum ;
    
    //#define SIMULATE_IBUS_IN
    #ifdef SIMULATE_IBUS_IN
    static uint8_t ibusRcChannelsSimulation[] = {
        0X20,  0X40, 0XDB, 0X05, 0XDC, 0X05, 0X54, 0X05, 0XDC, 0X05, 0XE8, 0X03, 0XD0, 0X07, 0XD2, 0X05, 0XE8,
        0X03, 0XDC, 0X05, 0XDC, 0X05, 0XDC, 0X05, 0XDC, 0X05, 0XDC, 0X05, 0XDC, 0X05, 0XDA, 0XF3
    };
    
    
    static uint32_t ibusLastSimulationMs = 0;
    if ( (millisRp() - ibusLastSimulationMs) > 999 ) { // send a message once every 10 ms
        //printf("simulation fill queue\n");
        ibusLastSimulationMs = millisRp();
        uint16_t c = ibusRcChannelsSimulation[0] | 0X8000;
        queue_try_add (&ibusInQueue, &c);          // push to the queue
        for (uint8_t i = 1; i < sizeof(ibusRcChannelsSimulation); i++){
            c = ibusRcChannelsSimulation[i] ;
            queue_try_add (&ibusInQueue, &c);
        }
    }
    #endif // end of simulation
    

    
    
    if (config.pinPrimIn ==255) return ; // skip when pinPrimIn is not defined
    while (! queue_is_empty (&ibusInQueue) ){
        if ( (millisRp() - lastIbusMillis ) > 2 ){
            ibusInState = NO_IBUS_IN_FRAME ;
        }

        lastIbusMillis = millisRp();
        queue_try_remove ( &ibusInQueue , &c);
        //printf("get= %X\n", c);
        if (c == 0X8020) {          // First byte is length and bit 15 is 1 to say it is first byte ; Rc channel frame has a length of 0X20 (32 bytes)
            ibusInCounter = 0;
            ibusInState = RECEIVING_IBUS_IN ;
            checksum = 0xFFFF  ;
        }
        if ( ibusInState == RECEIVING_IBUS_IN){
            runningIbusFrame[ibusInCounter++] = (uint8_t) c;
                if ( ibusInCounter < 31) checksum -=  c & 0X00FF ; // calculate checksum 
        }    
        if (ibusInCounter == 0X20 ) {  // 32 byte per frame 
            ibusInState = NO_IBUS_IN_FRAME;
            ibusInCounter = 0;
            //printf("Frame= ");
            //for (uint8_t i = 0 ; i < 32 ; i++ ) { // fill a table with values
            //    printf("%x ", runningIbusFrame[i] );
            //}
            //printf("\n");
            uint16_t checksum2 = ((uint16_t)runningIbusFrame[30]) + ((uint16_t) runningIbusFrame[31]<<8);
            //printf("checksum2 = %x  %x  %x\n",checksum2, (uint16_t) runningIbusFrame[30] , ((uint16_t) runningIbusFrame[31])<<8) ;
            if  ( runningIbusFrame[1] = 0X40 ) {   // byte 1 must be 0X40 for a RC channel frame
                //printf("checksum = %x\n",checksum) ;
                if ( checksum == checksum2 ) { // check checksum
                    ibusDecodeRcChannels();
                }    
            }
                  
        }
    } // end while     
}
  
/*
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
*/

void ibusDecodeRcChannels(){             // channels values are coded on 2 bytes. last bit = 1/8 usec
	//printf("exbus decoding Rc channels\n");
    uint8_t sbus[23];
    uint16_t ibusRcChannels[16] = {0X8000};
    float ratioPwmToSbus = (float) (FROM_SBUS_MAX - FROM_SBUS_MIN) / (float) (TO_PWM_MAX - TO_PWM_MIN);
    uint16_t temp;
    //printf("Rc= ");
    for (uint8_t i = 0 ; i < 14 ; i++ ) { // Frame contains 14 channels
        temp = runningIbusFrame[2+ (i<<1)] + (((uint16_t) runningIbusFrame[3+(i<<1)]) << 8) ;
        if ( temp < 885) temp = 885;
        if ( temp > 2160) temp = 2160; 
    //    printf("%i ", (int) temp );
        ibusRcChannels[i] = (uint16_t) ((((float) (temp - TO_PWM_MIN)) * ratioPwmToSbus)+0.5) +  FROM_SBUS_MIN ; // convert in Sbus value 16 bits)
    }
    sbus[0] = ibusRcChannels[0];
    sbus[1] = (ibusRcChannels[0] >> 8) | (ibusRcChannels[1] & 0x00FF)<<3;
    sbus[2] = ibusRcChannels[1]>>5|(ibusRcChannels[2]<<6);
    sbus[3] = (ibusRcChannels[2]>>2)& 0x00ff;
    sbus[4] = ibusRcChannels[2]>>10| (ibusRcChannels[3] & 0x00FF)<<1;
    sbus[5] = ibusRcChannels[3]>>7|  (ibusRcChannels[4] & 0x0FF )<<4;
    sbus[6] = ibusRcChannels[4]>>4| (ibusRcChannels[5] & 0xFF) <<7;
    sbus[7] = (ibusRcChannels[5]>>1)& 0x00ff;
    sbus[8] = ibusRcChannels[5]>>9| (ibusRcChannels[6] & 0xFF)<<2;
    sbus[9] = ibusRcChannels[6]>>6| (ibusRcChannels[7] & 0xFF)<<5;
    sbus[10] = (ibusRcChannels[7]>>3)& 0x00ff;//end
    sbus[11] = (ibusRcChannels[8] & 0XFF);
    sbus[12] = (ibusRcChannels[8]>> 8) | (ibusRcChannels[9] & 0xFF)<<3;
    sbus[13] = ibusRcChannels[9]>>5 | (ibusRcChannels[10]<<6);
    sbus[14] = (ibusRcChannels[10]>>2) & 0xff;
    sbus[15] = ibusRcChannels[10]>>10 | (ibusRcChannels[11] & 0XFF)<<1;
    sbus[16] = ibusRcChannels[11]>>7 | (ibusRcChannels[12] & 0XFF)<<4;
    sbus[17] = ibusRcChannels[12]>>4 | (ibusRcChannels[13] & 0XFF)<<7;
    sbus[18] = (ibusRcChannels[13]>>1)& 0xff;
    sbus[19] = ibusRcChannels[13]>>9 | (ibusRcChannels[14] & 0XFF)<<2;
    sbus[20] = ibusRcChannels[14]>>6 | (ibusRcChannels[15] & 0XFF)<<5;
    sbus[21] = (ibusRcChannels[15]>>3)& 0xff;

    sbus[22] = 0x00;
    //if ( channelOrFailsafe == SRXL2_RC_FAILSAFE) sbus[22] |= (1<<3);//FS activated   
    //    if(missingPackets >= 1) sbus[22] |= (1<<2);//frame lost
    memcpy( (uint8_t *) &sbusFrame.rcChannelsData, &sbus[0], 23) ; // copy the data to the Sbus buffer
    lastRcChannels = millisRp();
    lastPriChannelsMillis =  lastRcChannels;
    newRcChannelsReceivedForPWM = true;  // used to update the PWM data

} 

/*
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
*/

/*
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
*/
