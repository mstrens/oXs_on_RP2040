
#include "hardware/pio.h"
#include "uart_esc_rx.pio.h"
#include "pico/util/queue.h"
#include "hardware/irq.h"
#include "tools.h"
#include "param.h"
#include <string.h>
#include <inttypes.h>
#include "stdio.h"
#include "stdint.h"
#include "config.h"
#include "esc.h"
#include "math.h"

// one pio (0) and 1 state machines is used to get data from ESC
// to receive data, the sm is initialised and use an IRQ handler when rx fifo is not empty
//    in irq, this byte is store in a Rx queue
//    This queue is read in main loop
//    We fill a buffer with the data


//------- Hobbywing V4 
// Esc sent on uart (19200 baud 8N1 not inverted) a frame every 20 msec that contains
// [0] 0x9B
// [4..5] throtle (0...1024)
// [6..7] PWM     (0...1024)
// [8..10] RPM
// [11..12] volt (to convert)
// [13..14] raw current (to convert)
// [15..16] temp FET (to convert)
// [17..18] temp BEC (to convert)

/*  Hobbywing V4
1.Frame with 19bytes = telemetry
2.Frame with 13bytes = signature
Before throttle is raised from 0, signature packets are sent between telemetry packets. This is used to identify the hardware and firmware of the ESC.
Telemetry packets:
Byte | 1 | 2 | 3 | 4 | 5 | 6 |
Value | Package Head (0x9B) | Package Number 1 | Package Number 2 | Package Number 3 | Rx Throttle 1 | Rx Throttle 2 |
7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
Output PWM 1 | Output PWM 2 | RPM 1 | RPM 2 | RPM 3 | Voltage 1 | Voltage 2 | Current 1 | Current 2 | TempFET 1 |
17 | 18 | 19
TempFET 2 | Temp 1 | Temp 2
Signature packets:
Byte | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13
V4LV25/60/80A | 0x9B | 0x9B | 0x03 | 0xE8 | 0x01 | 0x08 | 0x5B | 0x00 | 0x01 | 0x00 | 0x21 | 0x21 | 0xB9
*/


// ------ Kontronix ------------------------
// ESC sent on uart (115200 8E1) a frame every 10 msec that contains
// "KODL" as header
// U32 RPM T/min
// U16 volt 0.01Volt
// I16 amp 0.1A (accu)
// I16 amp 0.1A (motor)
// I16 amp 0.1A (peak)
// u16 mAh (capacity)
// u16 mA  bec current 
// u16 mV  bec volt
// u16 us  pwm in 
// i8  %   gas in
// u8  %  ???
// i8 °C temp fet
// i8 °C temp bec
// other bytes

// this code is based on the code developped here https://github.com/derFliegendeHamburger/msrc/tree/master

// ----- ZTW Mantis --------------------------------------
// ESC sent on uart (115200 8N1) a frame every 50 msec that contains
// 
//Byte0: Packet Header - --- 0xDD
//Byte1: Protocol No ---- 0x01
//Byte2: Data length ----- 0x20
//Byte3: Voltage-H in 0.1V
//Byte4: Voltage-L (10-1V, data range: 0 0x3e8 100V) If the reported voltage is 58.2V, the data will be 0x02, 0x46.
//Byte5: Current-H (in 0.1A)
//Byte6: Current-L (10-1A, data range: 0 0x1388 = 5000 =>500A)
//Byte7: Throttle-PCT (0x01-1%, data range: 0 – 0x64 100% input)
//Byte8: RPM-H
//Byte9: RPM-L (0x01-10RPM, data range: 0 – 0xffff )
//Byte10: Mos-temp (0x01 – 1℃, data range: 0 –0x96 150℃) (so probably an offset of 96)
//Byte11: Motor-temp (0x01 – 1℃, data range: 0 –0x96 150℃)
//Byte12: Throttle-PWM (0x01-1% , data range: 0 0x64 100% output)
//Byte13: State-H
//Byte14: State-L
//Byte15: Mah-used H high value of the used/consumed power
//Byte16: Mah-used L low value of the used/consumed power (unit = mah)
//Byte17: UART-TH serial throttle input
//Byte18: CAN-TH can throttle
//Byte19: BEC voltage (0-25V) so in 0.1V
//Byte20- Byte29: reserved
//Byte30 &31: byte0 byte29 Sum. accumulate & verify
//State-L explanations for statuses
//0x01 Short-circuit protection 0x10 Low-voltage protection
//0x02 motor wire break 0x20 Temperature protection
//0x04 PPM TH loss protection 0x40 Start locked-rotor
//0x08 Power on TH is not zero 0x80 Current protection
//State-H explanations for statuses
//0x01 PPM throttle is not within the regulated range, the PPM throttle is in an abnormal state, and the throttle is not within 700us~2500us.
//0x10: the battery voltage is not within the regulated range.
//0x02 UART Throttle is not within the regulated range, UART throttle is in an abnormal state, the throttle value exceeds 1000.
//0x04 UART throttle loss, UART TH loss
//0x08 CAN throttle loss, CAN TH loss


// BlHeli frame (BLH esc type)
//One transmission will have 10 times 8-bit bytes sent with 115200 baud and 3.6V.
//Byte 0: Temperature
//Byte 1: Voltage high byte
//Byte 2: Voltage low byte
//Byte 3: Current high byte
//Byte 4: Current low byte
//Byte 5: Consumption high byte
//Byte 6: Consumption low byte
//Byte 7: Rpm high byte
//Byte 8: Rpm low byte
//Byte 9: 8-bit CRC


// Hobbywing
#define ESC_HOBBYV3_MAX_FRAME_LEN 19

#define ESC_HOBBYV4_BAUDRATE 19200
#define ESC_HOBBYV4_MAX_FRAME_LEN 19
// interval = 20000 usec but 19 bytes at 19200 = nearly 10000usec 
#define ESC_HOBBYV4_MIN_FREE_TIME_US 8000 // minimum interval without uart signal between 2 frames
#define ESCHW4_NTC_BETA 3950.0
#define ESCHW4_NTC_R1 10000.0
#define ESCHW4_NTC_R_REF 47000.0
#define ESCHW4_V_REF 3.3
#define ESCHW4_ADC_RES 4096.0

//#define POLES 14
//#define V_DIVISOR 210 // for 3_6 S = 110; for 3_8 S = 154; for 5-12 S = 210 ; note: value is multiplied by 10 to avoid a decimal
//#define DIFFAMP_GAIN 100 // note: value is multiplied by 10 to avoid a decimal
//#define HWV4_CURRENT_MAX 250000  // in mA


// kontronik
#define ESC_KONTRONIK_BAUDRATE 115200
#define ESC_KONTRONIK_MAX_FRAME_LEN 35
// 35 byte at 115200 = nearly 3500 usec; there is one frame per 10000usec
#define ESC_KONTRONIK_MIN_FREE_TIME_US 4000 // minimum interval without uart signal between 2 frames

// ZTW Mantis
#define ESC_ZTW1_BAUDRATE 115200
#define ESC_ZTW1_MAX_FRAME_LEN 32
// 32 byte at 115200 = nearly 3500 usec; there is one frame per 10000usec (or 50000)
#define ESC_ZTW1_MIN_FREE_TIME_US 2000 // minimum interval without uart signal between 2 frames

// BlHeli
#define ESC_BLH_BAUDRATE 115200
#define ESC_BLH_MAX_FRAME_LEN 10
// 10 byte at 115200 = nearly 1100 usec; there is one frame per 36000usec (or 50000)
#define ESC_BLH_MIN_FREE_TIME_US 5000 // minimum interval without uart signal between 2 frames

// Jeti ESC
#define ESC_JETI_BAUDRATE 9600
#define ESC_JETI_MAX_FRAME_LEN 69 // Length can vary  but should not exceed 69
#define ESC_JETI_MIN_FREE_TIME_US 2000 // it seems to work with 2000; there is a frame each 20ms


// Len here must be big enough to contain all types of ESC frame
#define ESC_MAX_BUFFER_LEN 100 // jeti seems to have 32+2+29+5? = about 70

queue_t escRxQueue ;

// one pio with 2 state machine is used to manage the inverted hal duplex uart for Sport
PIO escPioRx = pio0;
uint escSmRx = 3; // to get the request from sport
uint escOffsetRx ; 

/*
enum ESC_STATES {
    RECEIVING,
    WAIT_FOR_SENDING,
    SENDING,
    WAIT_END_OF_SENDING
};
*/

uint32_t lastEscReceivedUs = 0;  // used to check the delay between char. 
//MPXSTATES mpxState;

extern field fields[];  // list of all telemetry fields and parameters used by oXs
extern CONFIG config;

uint8_t escRxBuffer[ESC_MAX_BUFFER_LEN];
uint8_t escRxBufferIdx = 0 ;
uint8_t escLen = 0; // length of frame
uint32_t escFreeTimeUs = 0 ; 

uint8_t escMaxFrameLen ;
uint8_t escShift = 24; // for 8N1 uart, we have to shift the 32 bits 24 pos right


void setupEsc(){
    if (config.pinEsc == 255) return;  // skip when there is no ESC
// set up max len of frame
    if ( config.escType == HW4) { 
        escMaxFrameLen = ESC_HOBBYV4_MAX_FRAME_LEN;
        escFreeTimeUs = ESC_HOBBYV4_MIN_FREE_TIME_US;
    } else if ( config.escType == HW3) { 
        escMaxFrameLen = ESC_HOBBYV3_MAX_FRAME_LEN;
        escFreeTimeUs = ESC_HOBBYV4_MIN_FREE_TIME_US;
    } else if ( config.escType == KONTRONIK) { 
        escMaxFrameLen = ESC_KONTRONIK_MAX_FRAME_LEN;
        escFreeTimeUs = ESC_KONTRONIK_MIN_FREE_TIME_US;
        escShift = 23;             // for 8E1 uart, we shift by 23 pos instead of 24 because we get 9 bit instead of 8
    } else if ( config.escType == ZTW1) { 
        escMaxFrameLen = ESC_ZTW1_MAX_FRAME_LEN;
        escFreeTimeUs = ESC_ZTW1_MIN_FREE_TIME_US;
    } else if ( config.escType == BLH) { 
        escMaxFrameLen = ESC_BLH_MAX_FRAME_LEN;
        escFreeTimeUs = ESC_BLH_MIN_FREE_TIME_US;
    } else if ( config.escType == JETI_ESC) { 
        escMaxFrameLen = ESC_JETI_MAX_FRAME_LEN;
        escFreeTimeUs = ESC_JETI_MIN_FREE_TIME_US;
        escShift = 22;             // for 9O1 uart, we shift by 23 pos instead of 24 because we get 9 bit instead of 8

    } 
// configure the queue to get the data from ESC in the irq handle
    queue_init (&escRxQueue, sizeof(uint16_t), 50);

// set an irq on pio to handle a received byte
    irq_set_exclusive_handler( PIO0_IRQ_1 , escPioRxHandlerIrq) ;  // we use pio0 and irq 1
    irq_set_enabled (PIO0_IRQ_1 , true) ;
    if ( ( config.escType == HW3) || ( config.escType == HW4) ) {
    // Set up the state machine we're going to use to receive them.
        escOffsetRx = pio_add_program(escPioRx, &esc_uart_rx_8N1_program);
        esc_uart_rx_8N1_program_init(escPioRx, escSmRx, escOffsetRx, config.pinEsc, ESC_HOBBYV4_BAUDRATE , false); // false = not inverted   
        //setupListMpxFieldsToReply();
    } else if (config.escType == KONTRONIK){
        escOffsetRx = pio_add_program(escPioRx, &esc_uart_rx_8E1_program);
        esc_uart_rx_8E1_program_init(escPioRx, escSmRx, escOffsetRx, config.pinEsc, ESC_KONTRONIK_BAUDRATE , false); // false = not inverted
    } else if (config.escType == ZTW1){
        escOffsetRx = pio_add_program(escPioRx, &esc_uart_rx_8N1_program);
        esc_uart_rx_8N1_program_init(escPioRx, escSmRx, escOffsetRx, config.pinEsc, ESC_ZTW1_BAUDRATE , false); // false = not inverted
    }  else if (config.escType == BLH){
        escOffsetRx = pio_add_program(escPioRx, &esc_uart_rx_8N1_program);
        esc_uart_rx_8N1_program_init(escPioRx, escSmRx, escOffsetRx, config.pinEsc, ESC_BLH_BAUDRATE , false); // false = not inverted
    }  else if (config.escType == JETI_ESC){
        escOffsetRx = pio_add_program(escPioRx, &esc_uart_rx_9O1_program);
        esc_uart_rx_9O1_program_init(escPioRx, escSmRx, escOffsetRx, config.pinEsc, ESC_JETI_BAUDRATE , false); // false = not inverted
    }
    //#define DEBUG_ESC
    #ifdef  DEBUG_ESC
    uart_init(uart0, 115200); // Initialise UART 0 !!!!!!!!!! do not use pin 0 for another purpose
    gpio_set_function(0, GPIO_FUNC_UART); // Set the GPIO pin mux to the UART - 0 is TX, 1 is RX
    uart_set_format	(uart0, 8, 1 , UART_PARITY_EVEN) ;
    #endif     
}

void escPioRxHandlerIrq(){    // when a byte is received on the esc bus, read the pio esc fifo and push the data to a queue (to be processed in the main loop)
  // clear the irq flag
    irq_clear (PIO0_IRQ_1 );
    uint32_t nowMicros = microsRp();
    uint16_t c ;
    while (  ! pio_sm_is_rx_fifo_empty (escPioRx ,escSmRx)){ // when some data have been received
        if ( config.escType == JETI_ESC) {
            c = (pio_sm_get (escPioRx , escSmRx) >> escShift) &0x01FF;      // read the data, shift by 22 and keep 9 bits (not the parity bit)
        } else {
         c = (pio_sm_get (escPioRx , escSmRx) >> escShift) &0x00FF;         // read the data, shift by 24, 23 or 22 and keep 8 bits
        }
        // here we discard the parity bit from Kontronik and ZWT1
         //when previous byte was received more than X usec after the previous, then add 1 in bit 15 
        if ( ( nowMicros - lastEscReceivedUs) > escFreeTimeUs ) c |= 0X8000 ; // add a flag when there is a gap between char
        if (queue_try_add (&escRxQueue, &c) == false) printf("Esc queue is full\n");          // push to the queue
        lastEscReceivedUs = microsRp();    
    }
    
}




void handleEsc(){ 
    uint16_t data;
    static bool frameStarted = false;
    static bool pullupHW = false;
    if (config.pinEsc == 255) return ; // skip when esc is not foreseen
    // for hobbywing ESC, we have to activate the pullup only after 2 sec otherwise ESC do not start
    if (config.escType == HW4) {
        if ((pullupHW == false) and (millisRp() > 3000)) {
            gpio_pull_up(config.pinEsc);
            pullupHW = true;
        }
    }
    while (! queue_is_empty(&escRxQueue)) {
        // we get the value in the queue
        queue_try_remove (&escRxQueue,&data);
        // if bit 15 = 1, it means that the value has been received after x usec from previous and so it must be a synchro 
        // so reset the buffer and process the byte
        if (data & 0X8000) {
            escRxBufferIdx = 0;
        } else if (escRxBufferIdx >= escMaxFrameLen) {
            continue ; // discard the car if buffer is full (should not happen)    
        }
        // to debug the char being received
        if ((config.escType == ZTW1) )printf("%4X\n",data ); 
        if (config.escType == JETI_ESC){
            if (data == 0XFE) { //start of jetibox ascii = 34 bytes including start and end
                //printf("Start\n");
                escRxBufferIdx = 0; // reset the counter
                escRxBuffer[escRxBufferIdx++] = (uint8_t) data; // store the start byte in the buffer            
            } else if (data == 0XFF) { // when end has been detected, process the frame
                //printf("Process\n");
                processJetiboxEscFrame();
                escRxBufferIdx = 0; // reset the counter
            } else if ((escRxBufferIdx > 0) and (escRxBufferIdx < 34)) { // when start has been detected, Idx is > 0
                escRxBuffer[escRxBufferIdx++] = (uint8_t) data; // store the byte in the buffer
            } // otherwise discard the byte (when start has not been detected)   
        } else {                 // not a jeti esc  
            escRxBuffer[escRxBufferIdx++] = (uint8_t) data; // store the byte in the buffer
            if (escRxBufferIdx == escMaxFrameLen) {         // when buffer is full, process it
                processEscFrame(); // process the incoming byte
            }     
        }    
    } // end while
    #ifdef DEBUG_ESC // generate a frame once per 10 msec
    static uint32_t lastZWT1Ms;
    if ((millisRp() - lastZWT1Ms) > 50) {
        lastZWT1Ms = millisRp();
        for (uint8_t i = 1; i<=32 ; i++) {
            uart_putc_raw(uart0, (char) i);
        }    
    }
    #endif 
}           

float escConsumedMah = 0;
uint32_t lastEscConsumedMicros = 0;


int digit(uint8_t i){ // return the digit or 0 when it is not a digit
        if (escRxBuffer[i] >= 0x30 and escRxBuffer[i] <= 0x39) return escRxBuffer[i] - 0x30;
    return 0;
}

void processJetiboxEscFrame(){
    // first byte is start byte
    //                1 6 ,  2  V   
    // then we have e.g. 
    // FE 31 36 2C 30 56 20 42 31 30 30 25 20 32 34 DF 43 20 20 20 20 30 41 20 20 20 20 20 20 30 72 70 6D
    //  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32
    //     1  6  ,  0  V     B  1  0  0  %     2  4  °  C              0  A                    0  r  p  m
    // Note B is sometime replaced by R
    //for (uint8_t i = 0; i<=32; i++){
    //    printf("%2X ", escRxBuffer[i]);
    //}
    //printf("\n");
    if (escRxBuffer[5] == 'V' and escRxBuffer[3] == ',') {
        int32_t volt = digit(1)*10000 + digit(2)*1000 + digit(4)*100;
        if (config.pinVolt[0] == 255) { // when volt1 is defined, we discard voltage from esc
            sent2Core0( MVOLT, volt ) ; 
        }
    }
    if (escRxBuffer[15] == 0xDF and escRxBuffer[16] == 'C') {
        int32_t degree = digit(12)*100 + digit(13)*10 + digit(14);
        if ((config.pinVolt[2] == 255) or ((config.temperature != 1)  and (config.temperature != 2))){ //  we discard temp from esc
            sent2Core0( TEMP1, degree) ;
        }
    }        
    if (escRxBuffer[22] == 'A') {
        int32_t current = digit(18)*1000000 + digit(19) * 100000 + digit(20)*10000 + digit(21) * 1000;
        if (config.pinVolt[1] == 255) { 
            sent2Core0( CURRENT, current) ; 
        }
    }
    if (escRxBuffer[30] == 'r' and escRxBuffer[31] == 'p') {
        int32_t rpm = digit(24)*100000 + digit(25)*10000 + digit(26)*1000 + digit(27) * 100 + digit(28)*10 + digit(29) ;
        if (config.pinRpm == 255) { // when rpm pin is defined, we discard rpm from esc
            sent2Core0( RPM,  (int32_t) ((float) rpm  * config.rpmMultiplicator  )) ; // in rpm
        }
    }        
}

void processEscFrame(){ // process the incoming byte (except the Jeti that is procesed in another function) 
    //To debug the frame
    //#define DEBUG_ESC_FRAME
    #ifdef DEBUG_ESC_FRAME
    for (uint8_t i=0; i< escMaxFrameLen ; i++){
         printf("%2X ",escRxBuffer[i] );   
    }
    printf("\n");
    #endif
    if (config.escType == HW4) { // when frame is received for Hobbywing V4
        processHW4Frame();
    } else if (config.escType == KONTRONIK) {
        processKontronikFrame();
    } else if (config.escType == ZTW1) {
        processZTW1Frame();
    }  else if (config.escType == BLH) {
        processBlhFrame();
    }
}

void processHW4Frame(){
    if (escRxBuffer[0] == 0x9B) {
        int throttle = escRxBuffer[4] << 8 | escRxBuffer[5];  // in range 0...1024
        int pwm = escRxBuffer[6] << 8 | escRxBuffer[7];       // in range 0...1024
        int rpm = escRxBuffer[8] << 16 | escRxBuffer[9] << 8 | escRxBuffer[10];
        int voltage = escRxBuffer[11] << 8 | escRxBuffer[12];
        int current = escRxBuffer[13] << 8 | escRxBuffer[14];
        int tempFet = escRxBuffer[15] << 8 | escRxBuffer[16];
        int tempBec = escRxBuffer[17] << 8 | escRxBuffer[18];
        if (throttle < 1024 &&
            pwm < 1024 &&
            rpm < 200000 &&
            escRxBuffer[11] < 0xF &&
            escRxBuffer[13] < 0xF &&
            escRxBuffer[15] < 0xF &&
            escRxBuffer[17] < 0xF) {
        //if (throttle >= 1024 || pwm >= 1024 || rpm >= 200000 || escRxBuffer[11] & 0xF0 ||\
        //        escRxBuffer[13] & 0xF0 || escRxBuffer[15] & 0xF0 || escRxBuffer[17] & 0xF0 || escRxBuffer[1] == 0x9B)
        //        // escRxBuffer[1] == 0x9B added by mstrens to perhaps avoid info frame; in principe LEN is different and so should already be omitted
        //{
        //}
        //else 
        //{
            if (config.pinRpm == 255) { // when rpm pin is defined, we discard rpm from esc
                sent2Core0( RPM,  (int32_t) ((float) rpm  * config.rpmMultiplicator)) ; //Multiplicator should be 2/number of poles
            }
            // original formule = raw_voltage*(V_REF/ADC_RES)*V_DIV
            //                  =            * 3300 /4096 * V_DIV with V_DIV = e.g. 12
            if (config.pinVolt[0] == 255) { // when volt1 is defined, we discard voltage from esc
                sent2Core0( MVOLT, (int32_t)  (( float) voltage * config.scaleVolt1)) ; 
            }
            if (config.pinVolt[1] == 255) { 
                float currentf = 0;
                currentf = ((float)current) * config.scaleVolt2 - config.offset2;
                if (currentf<0) currentf = 0;
                if (currentf < ESC_MAX_CURRENT) { // discard when current is to high
                    sent2Core0( CURRENT, (int32_t)  currentf) ; 
                    // calculate consumption
                    float interval = (float) (microsRp() - lastEscConsumedMicros);
                    if ((interval >0 ) && (interval < 800000)) {
                        escConsumedMah += currentf * interval / 3600000000.0 ;  // in mah.
                        sent2Core0( CAPACITY, (int32_t) escConsumedMah);
                    }    
                    lastEscConsumedMicros =  microsRp(); 
                }
            }        
            if ((config.pinVolt[2] == 255) or ((config.temperature != 1)  and (config.temperature != 2))){ //  we discard temp from esc
                sent2Core0( TEMP1, calcTemp((float) tempFet)) ;
            }
            if ((config.pinVolt[3] == 255) or (config.temperature != 2)){ //  we discard temp from esc
                sent2Core0( TEMP2, calcTemp((float) tempBec)) ;
            }
            
            //printf("Esc throttle=%i   pwm=%i   Volt=%i  current=%i  consumed=%i  temp1=%i  temp2=%i\n", throttle , pwm , voltage , (int) current, (int) escConsumedMah , (int) tempFet , (int) tempBec );
            
            //throttle += ALPHA*(update_throttle(raw_throttle)-throttle);
            //rpm += ALPHA*(update_rpm(raw_rpm)-rpm);
            //pwm += ALPHA*(update_pwm(raw_pwm)-pwm);
            //voltage += ALPHA*(update_voltage(raw_voltage)-voltage);
            //current += ALPHA*(update_current(raw_current)-current);
            //temperature += ALPHA*(update_temperature(raw_temperature)-temperature);
        } 
    }
}

int32_t calcTemp(float tempRaw){
    float voltage = tempRaw * ESCHW4_V_REF / ESCHW4_ADC_RES;
    float ntcR_Rref = (voltage * ESCHW4_NTC_R1 / (ESCHW4_V_REF - voltage)) / ESCHW4_NTC_R_REF;
    if (ntcR_Rref < 0.001)
        return 0;
    float temperature = 1 / (log(ntcR_Rref) / ESCHW4_NTC_BETA + 1 / 298.15) - 273.15;
    if (temperature < 0)
        return 0;
    return (int32_t) temperature;
}


void processHW3Frame(){

}

void processKontronikFrame(){
    if (escRxBuffer[0] == 0x4B && escRxBuffer[1] == 0x4F && escRxBuffer[2] == 0x44 && escRxBuffer[3] == 0x4C) {
        uint32_t rpm = (uint32_t)escRxBuffer[7] << 24 | (uint32_t)escRxBuffer[6] << 16 | (uint16_t)escRxBuffer[5] << 8 | escRxBuffer[4];
        uint32_t voltage = ((uint16_t)escRxBuffer[9] << 8 | escRxBuffer[8]) * 10;  // convert 0.01V to mv
        float currentf = ((uint16_t)escRxBuffer[11] << 8 | escRxBuffer[10]) * 100;  // convert from 0.1A to ma 
        //float current_bec = ((uint16_t)escRxBuffer[19] << 8 | escRxBuffer[18]) / 1000.0;
        //float voltage_bec = ((uint16_t)escRxBuffer[21] << 8 | escRxBuffer[20]) / 1000.0;
        int32_t tempFet = escRxBuffer[26];
        int32_t tempBec = escRxBuffer[27];
        
        if (config.pinRpm == 255) { // when rpm pin is defined, we discard rpm from esc
            sent2Core0( RPM,  (int32_t) ((float) rpm  * config.rpmMultiplicator  )) ; // rpm
        }
        if (config.pinVolt[0] == 255) { // when volt1 is defined, we discard voltage from esc    
            sent2Core0( MVOLT, (int32_t)  (( float) voltage * config.scaleVolt1)) ; 
        }
        if (config.pinVolt[1] == 255) {
            sent2Core0( CURRENT, (int32_t) ( currentf * config.scaleVolt2 - config.offset2 ) ) ; 
            if (lastEscConsumedMicros) { 
                escConsumedMah += (currentf * (microsRp() - lastEscConsumedMicros)) / 3600000000.0 ;  // in mah.
                sent2Core0( CAPACITY, (int32_t) escConsumedMah);
            }
            lastEscConsumedMicros =  microsRp(); 
        }
        if ((config.pinVolt[2] == 255) or ((config.temperature != 1)  and (config.temperature != 2))){ //  we discard temp from esc    
            sent2Core0( TEMP1, tempFet) ;
        }
        if ((config.pinVolt[3] == 255) or (config.temperature != 2)){ //  we discard temp from esc
            sent2Core0( TEMP2, tempBec) ;
        }
        printf("Esc Volt=%i   current=%i  consumed=%i  temp1=%i  temp2=%i\n", voltage , (int) currentf, (int) escConsumedMah , (int) tempFet , (int) tempBec );
    }    
}

void processZTW1Frame(){
    if (escRxBuffer[0] == 0xDD && escRxBuffer[1] == 0x01 && escRxBuffer[2] == 0x20 ) {
        uint32_t voltage = ( ((uint32_t)escRxBuffer[3] << 8) | ((uint32_t) escRxBuffer[4]) ) * 100;  // convert 0.1V to mv
        float currentf =   ( (uint32_t)escRxBuffer[5] << 8)  | ((uint32_t) escRxBuffer[6] ) * 100;  // convert from 0.1A to ma 
        // byte 7 = throttle ; not used here 
        uint32_t rpm = ((uint32_t)escRxBuffer[8]) << 8 | ((uint32_t) escRxBuffer[9]);
        int32_t tempFet = ((int) escRxBuffer[10]) ;  // It seems that at least when positieve, the value is in degree. I do not understand why the doc refers to a range -96/150
        int32_t tempBec = ((int) escRxBuffer[11]) ;  // 
        // bytes 2/14 not used here 
        uint32_t consumed = ( ((uint32_t)escRxBuffer[15] << 8) | ((uint32_t) escRxBuffer[16]) ); // unit = ???
        if (config.pinRpm == 255) { // when rpm pin is defined, we discard rpm from esc
            sent2Core0( RPM,  (int32_t) ((float) rpm  * config.rpmMultiplicator )) ; // rpm
        }
        if (config.pinVolt[0] == 255) { // when volt1 is defined, we discard voltage from esc    
            sent2Core0( MVOLT, (int32_t)  (( float) voltage * config.scaleVolt1 - config.offset1)) ; 
        }
        if (config.pinVolt[1] == 255) {
            sent2Core0( CURRENT, (int32_t) ( currentf * config.scaleVolt2 - config.offset2 ) ) ; 
            sent2Core0( CAPACITY, (int32_t) consumed);
        }
        if ((config.pinVolt[2] == 255) or ((config.temperature != 1)  and (config.temperature != 2))){ //  we discard temp from esc    
            sent2Core0( TEMP1, tempFet) ;
        }
        if ((config.pinVolt[3] == 255) or (config.temperature != 2)){ //  we discard temp from esc
            sent2Core0( TEMP2, tempBec) ;
        }
        //printf("Esc Volt=%i   current=%i  consumed=%i  temp1=%i  temp2=%i\n", voltage , (int) currentf, (int) consumed , (int) tempFet , (int) tempBec );
    }    
}


//Byte 0: Temperature
//Byte 1: Voltage high byte (volt in 0.01V)
//Byte 2: Voltage low byte
//Byte 3: Current high byte (in 0.01A)
//Byte 4: Current low byte
//Byte 5: Consumption high byte (in mah)
//Byte 6: Consumption low byte
//Byte 7: Rpm high byte (in 100rpm)
//Byte 8: Rpm low byte
//Byte 9: 8-bit CRC

//#define DEBUG_BLHELI
void processBlhFrame(){
    uint8_t crc = get_crc8(escRxBuffer, 9);
    static uint32_t frameCount = 0;
    static uint32_t errorFrameCount = 0;
    frameCount++; 
    if (crc != escRxBuffer[9]) {
        errorFrameCount++;
        printf("Error in CRC from Blheli frame: %i / %i", errorFrameCount , frameCount);
        #ifdef DEBUB_BLHELI
        for (uint8_t i = 0; i<10 ; i++) {
            printf(" %x", escRxBuffer[i]);
        }
        printf("\n");
        #endif
        return;    
    }
    #ifdef DEBUB_BLHELI
    if (( frameCount % 100) == 0) {
        printf("valid Blheli frame:  %i", frameCount);
        for (uint8_t i = 0; i<10 ; i++) {
            printf(" %x", escRxBuffer[i]);
        }
        printf("\n");
    }
    #endif
    int32_t temp = escRxBuffer[0] ;  
    uint32_t voltage = ( ( ((uint32_t)escRxBuffer[1]) << 8) | ((uint32_t) escRxBuffer[2]) ) * 10;  // convert 0.01V to mv
    float currentf =   (float) ( ( ( ((uint32_t)escRxBuffer[3]) << 8)  | ( (uint32_t) escRxBuffer[4] ) ) * 10);  // convert from 0.01A to ma 
    uint32_t consumption =   ( ((uint32_t)escRxBuffer[5]) << 8)  | ((uint32_t) escRxBuffer[6] ) ;  // in mah 
    uint32_t rpm = (( ((uint32_t)escRxBuffer[7])) << 8)  | ((uint32_t) escRxBuffer[8]) ;   // in 100rpm 
    if (config.pinVolt[2] == 255) { //  we discard temp from esc    
        sent2Core0( TEMP1, temp) ;
    }
    if (config.pinVolt[0] == 255) { // when volt1 is defined, we discard voltage from esc    
        sent2Core0( MVOLT, (int32_t)  (( float) voltage * config.scaleVolt1 - config.offset1)) ; 
    }
    if (config.pinVolt[1] == 255) {
        sent2Core0( CURRENT, (int32_t) ( currentf * config.scaleVolt2 - config.offset2 ) ) ; 
        sent2Core0( CAPACITY, consumption);
    }
    if (config.pinRpm == 255) { // when rpm pin is defined, we discard rpm from esc
            sent2Core0( RPM,  (int32_t) ( ((float) rpm)  * config.rpmMultiplicator * 100.0  )) ; // * 100  because we convert to rpm     
    }
    //printf("Esc Volt=%i   current=%i  consumed=%i  temp1=%i   rpm=%i\n", voltage , (int) currentf, consumption , temp  , rpm);

}

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed){
    uint8_t crc_u, i;
    crc_u = crc;
    crc_u ^= crc_seed;
    for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
    return (crc_u);
}
uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen){
    uint8_t crc = 0, i;
    for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
    return (crc);
}


/*
#define POLES 14.0


#define V_REF 33   // in mv is 3300 but V_divisor is here multiplied by 10 to avoid a decimal in 15.4; so V_REF is 330 in stead of 3300
#define V_DIVISOR 21 // for 3_6 S = 11; for 3_8 S = 15.4; for 5-12 S = 21


#define ADC_RESOLUTION 4096
#define PWM_RES 590.0

#define DIFFAMP_GAIN 10.0
#define DIFFAMP_SHUNT 0.25/1000.0

#define NTC_RS 10000.0
#define NTC_RF 47000.0
#define NTC_BETA 3950.0
#define NTC_TR 298.15

#define ALPHA 1.0
#define OMEGA_C 25 
#define DT 20E-3

float ESC::update_throttle(int raw_throttle)
{
    return raw_throttle/PWM_RES;
}

float ESC::update_pwm(int raw_pwm)
{
    return raw_pwm/PWM_RES;
}    

float ESC::update_rpm(int raw_rpm)
{
    return raw_rpm*(2.0/POLES);
}  

float ESC::update_voltage(int raw_voltage)
{
    return raw_voltage*(V_REF/ADC_RES)*V_DIV;
}  

float ESC::update_current(int raw_current)
{
    float curr = raw_current*(V_REF/ADC_RES)/(DIFFAMP_GAIN*DIFFAMP_SHUNT);
    if (curr < 0)
        return 0;
    return curr;
} 

float ESC::update_temperature(int raw_temperature)
{
    float v_out = raw_temperature * (V_REF/ADC_RES);
    float rt = (v_out*NTC_RS/(V_REF-v_out));
    if (rt <= 0)
        return 0;
    float temp = 1/(log(rt/NTC_RF)/NTC_BETA+1/NTC_TR)-273.15;
    if (temp <= 0)
        return 0;
    return temp;
} 
*/