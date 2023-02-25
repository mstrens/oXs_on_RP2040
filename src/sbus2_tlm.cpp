#include "pico/stdlib.h"
#include "stdio.h"  // used by printf
#include "config.h"
#include <param.h>
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "sbus2_tlm.h"

//#include "pico/util/queue.h"
//#include "sbus_in.h"
//#include "crsf.h"
#include "tools.h"


// manage the telemetry for futaba
// This uses the same UART as the one used to receive the PRI Sbus signal so it is uart1
// As this needs a uart Tx pin, this pin is the uart1 RX pin - 1 and should be defined as TLM
// It is safe to use a 1K resistor between RX and TX pin and to connect Rx pin to the receiver 

//for each group of sensors (vario, battery, gps) add a setup in config.h to specify the slot being used
// GPS uses slot 8, 16 or 24 (and following)
// Vario uses 2 slots consecutive slots
// Battery uses 3 slots (volt, current, capacity) 
// Temp1 and Temp2 use each 1 slot



// when a full Sbus frame is received, set an alarm 2msec after and fill the first slot to be sent
// when the alarm fires,  sent the frame and prepare a new alarm for next slot if any 

// to do : add a check about config that Prim Rx pin is not used for another purpose when futaba protocol is selected 
// and check that TLM pin = PRI pin -1

/****************************************************************
 * transmit_frame
 * 
 * Implemented as an ISR to fit the strict timing requirements
 * Slot 1 has to be transmitted 2ms after receiving the SBUS2 Frame
 * Slot 2-8 has to be transmitted every 660 mirco seconds
 * When a data is not available the slot is unused (but not reused for another data)
 * Each Slot needs about 320 mircoseconds to be transmitted via uart
 * The delay between the Slots must not exceed 400 mirco seconds
 * 
 * so we wait to receive a RC frame.
 * * when a valid frame is received, we stop listening and we prepare transmitting
 * if byte 24 is 04,14,23 or 34 then ir is a SBUS2 and we can reply
 * we save the 0,1,2,3 (tlm frame counter), set sequence = 0 and start then a timer for 2msec
 * when it fires, we look if sequence is < 8 
 * if < 8, and if the data is available for slot sequence + frame counter *8, we fill the buffer and transmit it
 * if < 8 and data is not available, set a new alarm 660 later
 * In both cases we increment sequence
 * if == 8, we stop transmitting (we start receiving)
 * 
 * Attention:  the RX can probably sent some telemetry frame him self in slot 0
 ****************************************************************/

extern CONFIG config;
extern field fields[];  // list of all telemetry fields and parameters used by Sport


// 32 Slots for telemetrie Data
uint8_t slotId[32] =           {0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3,
                                 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
                                 0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB,
                                 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB};

bool slotAvailable[8] = {false}; // 0 = Not available = do not transmit
uint8_t slotValueByte1[8] = {0};  // value to transmit
uint8_t slotValueByte2[8] = {0};  // value to transmit

uint8_t slotGroup ; // 0/4 depending on the sbus2 frame
uint8_t slotNr;     // 0/7 depending of the slot being sent between 2 Rc frames 

void setupSbus2Tlm(){
    // setup of uart Tx pin : nothing is required ; only Tx pin must be enabled
    // setup of uart for receive and send : will be done while running
    // init list of slot codes used by futaba as first byte (32 values)
    // init list of slot being used (based on the config parameters)
    // GPIO pin Tx as GPIO input
    gpio_init(config.pinTlm);
    gpio_set_dir(config.pinTlm, GPIO_IN);
    gpio_pull_down(config.pinTlm); 
    //gpio_set_function( config.pinTlm,  GPIO_FUNC_UART);
    //gpio_set_inover( config.pinTlm, GPIO_OVERRIDE_INVERT);
    
}

void enableSbus2Transmit(){
    gpio_init(config.pinPrimIn);                      // disconnect pin to avoid Rx interrupt
    gpio_set_function( config.pinTlm, GPIO_FUNC_UART);
    gpio_set_inover( config.pinTlm , GPIO_OVERRIDE_INVERT);
    // !!!!!!!!! here we should also disable the interrupts the Rx UART
}

void disableSbus2Transmit(){
    gpio_init(config.pinTlm);
    gpio_set_dir(config.pinTlm, GPIO_IN);
    gpio_pull_down(config.pinTlm);
    gpio_set_function( config.pinPrimIn, GPIO_FUNC_UART);  // reactive RX
    gpio_set_inover( config.pinPrimIn , GPIO_OVERRIDE_INVERT);

  
    //uart_set_fifo_enabled(uart1 ,  true); // is already enabled
}

void fill8Sbus2Slots (uint8_t slotGroup){
    add_alarm_in_us(2000, sendNextSlot_callback, NULL, false); 
    slotNr= 0;
    uint8_t firstSlot32Idx = slotGroup * 8; 
    for (uint8_t i=0 ; i<8 ; i++){  //reset all flags
        slotAvailable[i] = false;         
    }
    if ((fields[VSPEED].available) && ( slotGroup == (SBUS2_SLOT_VARIO_2 >>3))){
        fillVario( SBUS2_SLOT_VARIO_2 & 0x07); // keep 3 last bits as slot index
    }
    if (( (fields[MVOLT].available) || (fields[CURRENT].available) ) && ( slotGroup == (SBUS2_SLOT_BATTERY_3 >>3))){
        fillBattery( SBUS2_SLOT_BATTERY_3 & 0x07);
    }
    if ((fields[TEMP1].available) && ( slotGroup == (SBUS2_SLOT_TEMP1_1 >>3))){
        fillTemp1(SBUS2_SLOT_TEMP1_1 & 0x07); // keep 3 last bits as slot index
    }
    if ((fields[TEMP2].available) && ( slotGroup == (SBUS2_SLOT_TEMP2_1 >>3))){
        fillTemp2(SBUS2_SLOT_TEMP2_1  & 0x07); // keep 3 last bits as slot index
    }
    if ((fields[RPM].available) && ( slotGroup == (SBUS2_SLOT_RPM_1 >>3))){
        fillRpm(SBUS2_SLOT_RPM_1  & 0x07); // keep 3 last bits as slot index
    }
    if ((fields[LONGITUDE].available) && ( slotGroup == (SBUS2_SLOT_GPS_8 >>3))){
        fillGps(SBUS2_SLOT_GPS_8  & 0x07); // keep 3 last bits as slot index
    }
    enableSbus2Transmit();   
}

int64_t sendNextSlot_callback(alarm_id_t id, void *user_data){ // sent the next callback (or stop sending after 8 slots)
    if (slotNr < 8){
        add_alarm_in_ms(660, sendNextSlot_callback, NULL, false); 
        if (slotAvailable[slotNr] ) {
            uart_putc_raw(uart1 , (char) slotId[slotNr + (slotGroup * 8)] ) ;
            uart_putc_raw(uart1 , (char) slotValueByte1[slotNr] );      /// !!!!!!!!!! perhaps to reverse
            uart_putc_raw(uart1 , (char) slotValueByte2[slotNr] );
            }
        slotNr++;  
    } else {
        disableSbus2Transmit();      
    }
}

void fillTemp1(uint8_t slot8){ // emulate SBS/01T ; Temp in °
    int16_t value=  fields[TEMP1].value;
    value |= 0x8000;
    value = value + 100;
    slotValueByte1[slot8] = value;// >> 8;
    slotValueByte2[slot8] = value >> 8;
}

void fillTemp2(uint8_t slot8){
    int16_t value=  fields[TEMP2].value;
    value |= 0x8000;
    value = value + 100;
    slotValueByte1[slot8] = value;// >> 8;
    slotValueByte2[slot8] = value >> 8;

}

void fillVario(uint8_t slot8){ // emulate F1672 ; Alt from cm to m ; Vspeed from cm/s to 0.1m/s

}

void fillBattery(uint8_t slot8){   // emulate SBS/01C ; Current from mA to 0.1A; volt from mV to 0.1V ; capacity in mAh
    uint16_t value = 0;
    uint32_t local = 0;
   // CURRENT
   local = (uint32_t) fields[CURRENT].value / 100;
   if ( fields[CURRENT].value > 0 ) {
        value = (uint16_t)local;
   } else {
        value = 0;
   }       
   if ( value > 0x3FFF ){ // max current is 163.83
      value = 0x3FFF;
   }  
   slotValueByte1[slot8] = ((value >> 8) | 0x40) & 0x7F;
   slotValueByte2[slot8] = value;// >> 8;

   //VOLTAGE
   local = (uint32_t) fields[MVOLT].value ;
   if ( fields[MVOLT].value > 0 ){
         value = (uint16_t)local;
   } else {
        value = 0;
   }  
   slotValueByte1[slot8 + 1] = value >> 8;
   slotValueByte2[slot8 + 1] = value;
   
   // CAPACITY
   local = (uint32_t) fields[CAPACITY].value ;
   if ( fields[CAPACITY].value > 0 ){
         value = (uint16_t)local;
   } else {
        value = 0;
   }
   slotValueByte1[slot8 + 2] = value >> 8;
   slotValueByte2[slot8 + 2] = value;
}


void fillRpm(uint8_t slot8){ // emulate SBS01RO ; in from Hz to RPM

}
void fillGps(uint8_t slot8){ // emulate SBS01G  ; speed from  to Km/h ; Alt from ??? to m ; vario to m/s

}