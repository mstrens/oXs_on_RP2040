
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
#include "esc_hobbyV4.h"
#include "math.h"

// one pio (0) and 1 state machines is used to get data from ESC
// to receive data, the sm is initialised and use an IRQ handler when rx fifo is not empty
//    in irq, this byte is store in a Rx queue
//    This queue is read in main loop
//    We fill a buffer with the data

// Esc sent on uart (19200 baud 8N1 not inverted) a frame every 20 msc that contains
// [0] 0x9B
// [4..5] throtle (0...1024)
// [6..7] PWM     (0...1024)
// [8..10] RPM
// [11..12] volt (to convert)
// [13..14] raw current (to convert)
// [15..16] temp FET (to convert)
// [17..18] temp BEC (to convert)


// this code is based on the code developped here https://github.com/derFliegendeHamburger/msrc/tree/master


#define ESC_HOBBYV4_BAUDRATE 19200


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

#define ESC_HOBBYV4_MAX_FRAME_LEN 19
uint8_t escRxBuffer[ESC_HOBBYV4_MAX_FRAME_LEN];
uint8_t escRxBufferIdx = 0 ;
uint8_t escLen = 0; // length of frame



void setupEscHobbyV4(){
    if (config.pinEsc == 255) return;  // skip when there is no ESC
// configure the queue to get the data from ESC in the irq handle
    queue_init (&escRxQueue, sizeof(uint16_t), 50);

// set an irq on pio to handle a received byte
    irq_set_exclusive_handler( PIO0_IRQ_1 , escPioRxHandlerIrq) ;  // we use pio0 and irq 1
    irq_set_enabled (PIO0_IRQ_1 , true) ;

// Set up the state machine we're going to use to receive them.
    escOffsetRx = pio_add_program(escPioRx, &esc_uart_rx_program);
    esc_uart_rx_program_init(escPioRx, escSmRx, escOffsetRx, config.pinEsc, ESC_HOBBYV4_BAUDRATE , false); // false = not inverted   
    //setupListMpxFieldsToReply();
}

void escPioRxHandlerIrq(){    // when a byte is received on the esc bus, read the pio esc fifo and push the data to a queue (to be processed in the main loop)
  // clear the irq flag
    irq_clear (PIO0_IRQ_1 );
    uint32_t nowMicros = microsRp();
    while (  ! pio_sm_is_rx_fifo_empty (escPioRx ,escSmRx)){ // when some data have been received
        uint16_t c = pio_sm_get (escPioRx , escSmRx) >> 24;         // read the data
         //when previous byte was received more than X usec after the previous, then add 1 in bit 15 
        if ( ( nowMicros - lastEscReceivedUs) > 20000 ) c |= 0X8000 ; // add a flag when there is a gap between char
        queue_try_add (&escRxQueue, &c);          // push to the queue
        lastEscReceivedUs = microsRp();    
    }
}




void handleEscHobbyV4(){ 
    uint16_t data;
    if (config.pinEsc == 255) return ; // skip when esc is not foreseen
    while (! queue_is_empty(&escRxQueue)) {
        // we get the value in the queue
        queue_try_remove (&escRxQueue,&data);
        //printf("%x\n", (uint8_t) data);
        
        // if bit 15 = 1, it means that the value has been received after x usec from previous and so it must be a synchro 
        // so reset the buffer and process the byte
        if (data & 0X8000) {
            escRxBufferIdx = 0; 
        } else if (escRxBufferIdx >= ESC_HOBBYV4_MAX_FRAME_LEN) {
            continue ; // discard the car if buffer is full (should not happen)    
        }
        processNextEscInputByte( (uint8_t) data); // process the incoming byte 
    } // end while
}           

#define POLES 14
#define V_DIVISOR 210 // for 3_6 S = 110; for 3_8 S = 154; for 5-12 S = 210 ; note: value is multiplied by 10 to avoid a decimal
#define DIFFAMP_GAIN 100 // note: value is multiplied by 10 to avoid a decimal
#define HWV4_CURRENT_MAX 250000  // in mA

void processNextEscInputByte( uint8_t c){ // process the incoming byte 
    escRxBuffer[escRxBufferIdx++] = c;  // save the received byte
    if (escRxBufferIdx == ESC_HOBBYV4_MAX_FRAME_LEN ){ // when frame is received
        if (escRxBuffer[0] == 0x9B) {
            int throttle = escRxBuffer[4] << 8 | escRxBuffer[5];  // in range 0...1024
            int pwm = escRxBuffer[6] << 8 | escRxBuffer[7];
            int rpm = escRxBuffer[8] << 16 | escRxBuffer[9] << 8 | escRxBuffer[10];
            int voltage = escRxBuffer[11] << 8 | escRxBuffer[12];
            int current = escRxBuffer[13] << 8 | escRxBuffer[14];
            int tempFet = escRxBuffer[15] << 8 | escRxBuffer[16];
            int tempBec = escRxBuffer[17] << 8 | escRxBuffer[18];
            if (throttle > 1024 || pwm > 1024 || rpm > 200000 || escRxBuffer[11] & 0xF0 ||\
                 escRxBuffer[13] & 0xF0 || escRxBuffer[15] & 0xF0 || escRxBuffer[17] & 0xF0 || escRxBuffer[1] == 0x9B)
                 // escRxBuffer[1] == 0x9B added by mstrens to perhaps avoid info frame; in principe LEN is different and so should already be omitted
            {
            }
            else 
            {
                sent2Core0( RPM,  (int32_t) ((float) rpm  * config.rpmMultiplicator)) ; //Multiplicator should be 2/number of poles
                // original formule = raw_voltage*(V_REF/ADC_RES)*V_DIV
                //                  =            * 3300 /4096 * V_DIV with V_DIV = e.g. 12
                sent2Core0( MVOLT, (int32_t)  (( float) voltage * config.scaleVolt1)) ; 

                float currentf = 0;
                if ( throttle > 256) { // current is calculated only when throttle is more than 1/4 of max value
                    // float curr = raw_current*(V_REF/ADC_RES)/(DIFFAMP_GAIN*DIFFAMP_SHUNT) = original formule
                    //                         * 3300 /4096 / (DIFFAMP_GAIN * 0.25 /1000) with DIFFAMP_GAIN = e.g. 10
                    currentf = (current) * config.scaleVolt2 - config.offset2;
                }
                if (currentf<0) currentf = 0;
                sent2Core0( CURRENT, (int32_t)  currentf) ; 
                sent2Core0( TEMP1, calcTemp((float) tempFet)) ;
                sent2Core0( TEMP2, calcTemp((float) tempBec)) ;
                //if (current > HWV4_CURRENT_MAX) { // reset if value is to high
                //    currentf = 0;
                //}     
                
                //printf("Esc Volt=%i   current=%i\n", voltage , current);
                //throttle += ALPHA*(update_throttle(raw_throttle)-throttle);
                //rpm += ALPHA*(update_rpm(raw_rpm)-rpm);
                //pwm += ALPHA*(update_pwm(raw_pwm)-pwm);
                //voltage += ALPHA*(update_voltage(raw_voltage)-voltage);
                //current += ALPHA*(update_current(raw_current)-current);
                //temperature += ALPHA*(update_temperature(raw_temperature)-temperature);
            }
        }
    }
}

#define ESCHW4_NTC_BETA 3950.0
#define ESCHW4_NTC_R1 10000.0
#define ESCHW4_NTC_R_REF 47000.0
#define ESCHW4_V_REF 3.3
#define ESCHW4_ADC_RES 4096.0

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