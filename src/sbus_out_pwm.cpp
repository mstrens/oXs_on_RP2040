
#include "sbus_out_pwm.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "uart_sbus_tx.pio.h"
#include "hardware/pwm.h"
#include "tools.h"
#include "param.h"
#include <string.h>
#include <inttypes.h>
#include "stdio.h"


PIO pioTxSbus = pio0; // we use pio 0; DMA is hardcoded to use it
uint smTxSbus = 2;  // we use the state machine 2 for Sbus Tx; DMA is harcoded to use it (DREQ) 

#define PIN_TX_SBUS 0
#define SERIAL_BAUD_SBUS 100000
int dma_sbus_chan;
dma_channel_config cSbus;

extern sbusFrame_s sbusFrame;
uint16_t sbusFrame16Bits[25];
extern uint32_t lastRcChannels;
extern CONFIG config;


static const bool ParityTable256[256] = 
{
#   define P2(n) n, n^1, n^1, n
#   define P4(n) P2(n), P2(n^1), P2(n^1), P2(n)
#   define P6(n) P4(n), P4(n^1), P4(n^1), P4(n)
    P6(0), P6(1), P6(1), P6(0)
};


// Set up a PIO state machine to serialise our bits
void setupSbusOutPio(){
    // setup the PIO for TX UART
    uint offsetTx = pio_add_program(pioTxSbus, &uart_sbus_tx_program);
    uart_sbus_tx_program_init(pioTxSbus, smTxSbus, offsetTx, PIN_TX_SBUS, SERIAL_BAUD_SBUS);

    // Configure a channel to write the same word (32 bits) repeatedly to PIO0
    // SM0's TX FIFO, paced by the data request signal from that peripheral.
    dma_sbus_chan = dma_claim_unused_channel(true);
    cSbus = dma_channel_get_default_config(dma_sbus_chan);
    channel_config_set_read_increment(&cSbus, true);
    channel_config_set_write_increment(&cSbus, false);
    channel_config_set_dreq(&cSbus, DREQ_PIO0_TX2);
    channel_config_set_transfer_data_size(&cSbus, DMA_SIZE_16);
    dma_channel_configure(
        dma_sbus_chan,
        &cSbus,
        &pio0_hw->txf[smTxSbus], // Write address (only need to set this once)
        sbusFrame16Bits,   // we use always the same buffer, it has 16 bits because it use a 10n1 format to manage the 8E2 sbus format             
        0 , // do not yet provide the number DMA cycles
        false             // Don't start yet
    );
    // do not set interrupt on DMA. The main loop will check if DMA is busy before sending
    
}



void fillSbusFrame(){
    static uint32_t lastSbusSentMillis = 0;
    if (!lastRcChannels) return;  // do not generate a sbus fram when whe never get a RC chanels frame form crsf
    // we should also check that dma is not busy anymore
    
    if ( (millis() - lastSbusSentMillis) >= 9 ) { // we send a frame once every 9 msec
        lastSbusSentMillis = millis();   
        sbusFrame.synchro = 0x0F ; 
        if ( ( millis()- lastRcChannels) > 500 ) { // if we do not get a RC channels frame, we apply failsafe
            sbusFrame.flag = 0x10; // indicates a failsafe
            if (config.failsafeType == 'C') memcpy( &sbusFrame.rcChannelsData , &config.failsafeChannels, sizeof(config.failsafeChannels));
        } else {
            sbusFrame.flag = 0x00;
        }    
        sbusFrame.endByte = 0x00;
        //printf("ch1= %" PRIu32 "\n",sbusFrame.rcChannelsData.ch0);
        uint8_t * ptr = (uint8_t *) &sbusFrame ;
        for (uint8_t i = 0; i<25 ; i++){ // copy sbusframe to sbusfram16b adding the parity and extra stop byte
            uint8_t c = *ptr;
            uint8_t p = ParityTable256[c];
            sbusFrame16Bits[i] = ( ((uint32_t) c) ) | ( ((uint32_t)p ) << 8) | ((uint32_t) 0x200) ;
            //printf("%x , %x , %X, %X , %f \n", c , p, sbusFrame16Bits[i] >> 2 , sbusFrame16Bits[i] & 0x3 , (float) sizeof(sbusFrame));
            ptr++; 
        }
        dma_channel_set_read_addr (dma_sbus_chan, sbusFrame16Bits, false);
        dma_channel_set_trans_count (dma_sbus_chan, sizeof(sbusFrame), true) ; 
           
    }    
}

#define PIN_PWM1 1
#define PIN_PWM2 2
#define PIN_PWM3 3
#define PIN_PWM4 4
#define PIN_PWM5 5
#define PIN_PWM6 6
#define PIN_PWM7 7
#define PIN_PWM8 8
uint8_t pwmPins[8] = { PIN_PWM1 , PIN_PWM2 , PIN_PWM3 , PIN_PWM4 , PIN_PWM5, PIN_PWM6 , PIN_PWM7 , PIN_PWM8 };
    
#define TOP 20000
#define DIVIDER 133

void setupPwm(){
    // Tell the LED pin that the PWM is in charge of its value.
    for( uint8_t i = 0 ; i < 8 ; i++){
        gpio_set_function(pwmPins[i] , GPIO_FUNC_PWM);
        // Figure out which slice we just connected to the LED pin
        uint slice_num = pwm_gpio_to_slice_num(pwmPins[i]);        
        // Get some sensible defaults for the slice configuration. By default, the
        // counter is allowed to wrap over its maximum range (0 to 2**16-1)
        pwm_config config = pwm_get_default_config();
        // Set divider, reduces counter clock to sysclock/this value
        pwm_config_set_wrap (&config , TOP) ; // set top value for wrapping
        pwm_config_set_clkdiv_int (&config , DIVIDER);
        // Load the configuration into our PWM slice, and set it running.
        pwm_init(slice_num, &config, true);
        pwm_set_gpio_level ( pwmPins[i] , 0) ; // start PWM with 0% duty cycle
    }    
}

void updatePWM(){
    static uint32_t lastPwmMillis = 0 ;
    if ( ! lastRcChannels) return ;
    if ( (millis() - lastPwmMillis) > 5 ){ // we update once every 5 msec ???? perhaps better to update at each new crsf frame in order to reduce the latency
        lastPwmMillis = millis();
        if ( ( millis()- lastRcChannels) > 500 ) { // if we do not get a RC channels frame, apply failsafe value if defined
            if (config.failsafeType == 'C') memcpy( &sbusFrame.rcChannelsData , &config.failsafeChannels, sizeof(config.failsafeChannels));
        }
        for( uint8_t i = 0 ; i < 8 ; i++){
            uint16_t rcChannelValue;
            switch (i) {
                case 0 :
                    rcChannelValue = (uint16_t) sbusFrame.rcChannelsData.ch0 ; 
                break;
                case 1 :
                    rcChannelValue = (uint16_t) sbusFrame.rcChannelsData.ch1 ; 
                break;
                case 2 :
                    rcChannelValue = (uint16_t) sbusFrame.rcChannelsData.ch2 ; 
                break;
                case 3 :
                    rcChannelValue = (uint16_t) sbusFrame.rcChannelsData.ch3 ; 
                break;
                case 4 :
                    rcChannelValue = (uint16_t) sbusFrame.rcChannelsData.ch5 ; 
                break;
                case 5 :
                    rcChannelValue = (uint16_t) sbusFrame.rcChannelsData.ch6 ; 
                break;
                case 6 :
                    rcChannelValue = (uint16_t) sbusFrame.rcChannelsData.ch7 ; 
                break;
                case 7 :
                    rcChannelValue = (uint16_t) sbusFrame.rcChannelsData.ch8 ; 
                break;
            }
            uint16_t pwmValue;
            pwmValue = fmap( rcChannelValue, 172, 1811, 988, 2012 );
            //printf("pwm= %" PRIu16 "\n", pwmValue);
            pwm_set_gpio_level (pwmPins[i], pwmValue) ;
        }    
    }
    
}

uint16_t  fmap(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
    return ((x - in_min) * (out_max - out_min) * 2 / (in_max - in_min) + out_min * 2 + 1) / 2;
}


