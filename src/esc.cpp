
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


// Len here must be big enough to contain all types of ESC frame
#define ESC_MAX_BUFFER_LEN 50

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
    }     
}

void escPioRxHandlerIrq(){    // when a byte is received on the esc bus, read the pio esc fifo and push the data to a queue (to be processed in the main loop)
  // clear the irq flag
    irq_clear (PIO0_IRQ_1 );
    uint32_t nowMicros = microsRp();
    while (  ! pio_sm_is_rx_fifo_empty (escPioRx ,escSmRx)){ // when some data have been received
        uint16_t c = (pio_sm_get (escPioRx , escSmRx) >> escShift) &0xFF;         // read the data, shift by 24 or 23 and keep 8 bits
        // here we discard the parity bit from Kontronik
         //when previous byte was received more than X usec after the previous, then add 1 in bit 15 
        if ( ( nowMicros - lastEscReceivedUs) > escFreeTimeUs ) c |= 0X8000 ; // add a flag when there is a gap between char
        queue_try_add (&escRxQueue, &c);          // push to the queue
        lastEscReceivedUs = microsRp();    
    }
}




void handleEsc(){ 
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
        } else if (escRxBufferIdx >= escMaxFrameLen) {
            continue ; // discard the car if buffer is full (should not happen)    
        }
        escRxBuffer[escRxBufferIdx++] = (uint8_t) data; // store the byte in the buffer
        if (escRxBufferIdx == escMaxFrameLen) {         // when buffer is full, process it
            processEscFrame(); // process the incoming byte
        }     
    } // end while
}           

float escConsumedMah = 0;
uint32_t lastEscConsumedMillis = 0;

void processEscFrame(){ // process the incoming byte 
    if (config.escType == HW4) { // when frame is received for Hobbywing V4
        processHW4Frame();
    }
}

void processHW4Frame(){
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
            if ( throttle > ESC_MIN_THROTTLE) { // current is calculated only when throttle is more than 1/4 of max value
                // float curr = raw_current*(V_REF/ADC_RES)/(DIFFAMP_GAIN*DIFFAMP_SHUNT) = original formule
                //                         * 3300 /4096 / (DIFFAMP_GAIN * 0.25 /1000) with DIFFAMP_GAIN = e.g. 10
                currentf = (current) * config.scaleVolt2 - config.offset2;
            }
            if (currentf<0) currentf = 0;
            if (currentf < ESC_MAX_CURRENT) { // discard when current is to high
                sent2Core0( CURRENT, (int32_t)  currentf) ; 
            }
            // calculate consumption
            if (lastEscConsumedMillis) { 
                escConsumedMah += (currentf * (millisRp() - lastEscConsumedMillis)) / 3600000.0 ;  // in mah.
                sent2Core0( CAPACITY, (int32_t) escConsumedMah);
            }
            lastEscConsumedMillis =  millisRp(); 
            
            sent2Core0( TEMP1, calcTemp((float) tempFet)) ;
            sent2Core0( TEMP2, calcTemp((float) tempBec)) ;
            //if (current > HWV4_CURRENT_MAX) { // reset if value is to high
            //    currentf = 0;
            //}     
            
            printf("Esc Volt=%i   current=%i  consumed=%i  temp1=%i  temp2=%i\n", voltage , (int) current, (int) escConsumedMah , (int) tempFet , (int) tempBec );
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
        sent2Core0( RPM,  (int32_t) ((float) rpm  * config.rpmMultiplicator / 60 )) ; // 60 because we convert from t/min in HZ
        sent2Core0( MVOLT, (int32_t)  (( float) voltage * config.scaleVolt1)) ; 
        sent2Core0( CURRENT, (int32_t) ( currentf * config.scaleVolt2 - config.offset2 ) ) ; 
        if (lastEscConsumedMillis) { 
            escConsumedMah += (currentf * (millisRp() - lastEscConsumedMillis)) / 3600000.0 ;  // in mah.
            sent2Core0( CAPACITY, (int32_t) escConsumedMah);
        }
        lastEscConsumedMillis =  millisRp(); 
        
        sent2Core0( TEMP1, tempFet) ;
        sent2Core0( TEMP2, tempBec) ;
        printf("Esc Volt=%i   current=%i  consumed=%i  temp1=%i  temp2=%i\n", voltage , (int) currentf, (int) escConsumedMah , (int) tempFet , (int) tempBec );
    }    
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