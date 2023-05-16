#include "hardware/pio.h"
#include "sbus_out_pwm.h"
#include "hardware/dma.h"
#include "uart_sbus_tx.pio.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "tools.h"
#include "param.h"
#include <string.h>
#include <inttypes.h>
#include "stdio.h"
#include "ws2812.h"
#include "pwm.pio.h"
#include "mpu.h"

// Sbus is 100000 baud, even parity, 8 bits , 2 stops,  inverted
// in order to use the PIO we calculate in the main program the parity bit and the second stop bit

#define FAILSAFE_DELAY 500 // for CRSF protocol, failsafe applies when we do not get frame for more than this delay (millis sec)

PIO pioTxSbus = pio0; // we use pio 0; DMA is hardcoded to use it
uint smTxSbus = 2;  // we use the state machine 2 for Sbus Tx; DMA is harcoded to use it (DREQ) 

//#define PIN_TX_SBUS 0
#define SERIAL_BAUD_SBUS 100000
int dma_sbus_chan;
dma_channel_config cSbus;

extern sbusFrame_s sbusFrame;
uint16_t sbusFrame16Bits[25];
extern uint32_t lastRcChannels;
extern uint32_t lastPriChannelsMillis; // used in crsf.cpp and in sbus_in.cpp to say that we got Rc channels data
extern uint32_t lastSecChannelsMillis; // used in crsf.cpp and in sbus_in.cpp to say that we got Rc channels data

extern bool sbusPriMissingFlag;
extern bool sbusSecMissingFlag;
extern bool sbusPriFailsafeFlag;
extern bool sbusSecFailsafeFlag;

// remapping from sbus value to pwm value
extern uint16_t fromSbusMin;
extern uint16_t toPwmMin; 
extern uint16_t fromSbusMax;
extern uint16_t toPwmMax; 


extern CONFIG config;
extern uint8_t debugSbusOut;

extern MPU mpu;
extern int32_t cameraPitch;
extern int32_t cameraRoll; 

extern field fields[];


static const bool ParityTable256[256] = 
{
#   define P2(n) n, n^1, n^1, n
#   define P4(n) P2(n), P2(n^1), P2(n^1), P2(n)
#   define P6(n) P4(n), P4(n^1), P4(n^1), P4(n)
    P6(0), P6(1), P6(1), P6(0)
};


extern uint8_t ledState;

uint16_t rcSbusOutChannels[16];


// Set up a PIO state machine to serialise our bits
void setupSbusOutPio(){
        // setup the PIO for TX UART
        uint offsetTx = pio_add_program(pioTxSbus, &uart_sbus_tx_program);
        uart_sbus_tx_program_init(pioTxSbus, smTxSbus, offsetTx, config.pinSbusOut , SERIAL_BAUD_SBUS);

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

void setLedState(){
    uint32_t now = millisRp();
    bool priFailsafe = sbusPriFailsafeFlag; // we first try to use the sbus flags
    bool secFailsafe = sbusSecFailsafeFlag;
    if ( (config.protocol == 'C') || (config.protocol == 'E') )     {   // overwrite when protocol is CRSF or EXbus because we have to take care of last receiving timestamp
        priFailsafe = (now - lastPriChannelsMillis) >  FAILSAFE_DELAY;
        secFailsafe = (now - lastSecChannelsMillis) >  FAILSAFE_DELAY;    
    } else {                       // even for Sbus, it could be that we have to ovewrite
        if ( (now - lastPriChannelsMillis) >  FAILSAFE_DELAY ) priFailsafe = true; // if we stop receiving sbus, we force a failsafe
        if ( (now - lastSecChannelsMillis) >  FAILSAFE_DELAY ) secFailsafe = true; // if we stop receiving sbus, we force a failsafe
    }
    if ( config.pinPrimIn == 255) {
        if (config.pinSecIn == 255) {
            ledState = STATE_NO_SIGNAL;
        } else {                   // only SEC defined
            if ( secFailsafe ) {
                ledState = STATE_FAILSAFE;
            } else {
                ledState = STATE_OK;
            }
        }
    } else {
        if (config.pinSecIn == 255) {    // only PRI defined
            if ( priFailsafe ) {
                ledState = STATE_FAILSAFE;
            } else {
                ledState = STATE_OK;
            }
        } else {                         // PRI and SEC defined
            if ( secFailsafe && priFailsafe) {
                ledState = STATE_FAILSAFE;
            } else if ( (secFailsafe == false) && (priFailsafe == false) ) {
                ledState = STATE_OK;
            } else {
                ledState = STATE_PARTLY_OK;
            }
        }
    }
    //printf("%d\n", (int) ledState);
}

void fillSbusFrame(){
    static uint32_t lastSbusSentMillis = 0;
    if (!lastRcChannels) return;  // do not generate a sbus frame when we never get a RC channels frame form crsf
    setLedState();                // set the color of the led   
    if (config.pinSbusOut == 255) return; // skip when pin is not defined
    // normally we should test if previous dma has finished sending all previous sbus frame. 

    if ( (millisRp() - lastSbusSentMillis) >= 9 ) { // we send a frame once every 9 msec
        lastSbusSentMillis = millisRp();   
        sbusFrame.synchro = 0x0F ; 
        if ( ( millisRp()- lastRcChannels) > FAILSAFE_DELAY ) { // if we do not get a RC channels frame, we apply failsafe
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
        if ( debugSbusOut == 'Y' ){
            printf("SbusOut: ");
            for (uint8_t i = 0; i< sizeof(sbusFrame) ; i++) printf( " %03X ", sbusFrame16Bits[i]);
            printf("\n");
        }    
        dma_channel_set_read_addr (dma_sbus_chan, sbusFrame16Bits, false);
        dma_channel_set_trans_count (dma_sbus_chan, sizeof(sbusFrame), true) ; 
           
    }    
}

    
#define TOP 20000
#define DIVIDER 133

bool pwmIsUsed;
float sbusCenter = (FROM_SBUS_MIN + FROM_SBUS_MAX) /2; 
float ratioSbusRange = 400.0 / (float) (FROM_SBUS_MAX - FROM_SBUS_MIN) ; // full range of Sbus should provide a difference of 400 (from -200 up to 200)

void setupPwm(){
    pwmIsUsed = false;
    for (uint8_t i=0 ; i<16 ; i++){
        if ( config.pinChannels[i] != 255) pwmIsUsed = true;
    }
    if ( pwmIsUsed == false) return ; // skip when PWM is not used
    for( uint8_t i = 0 ; i < 16 ; i++){
        if ( config.pinChannels[i] == 255 ) continue ; // skip i when pin is not defined for this channel
        gpio_set_function( config.pinChannels[i] , GPIO_FUNC_PWM);
        // Figure out which slice we just connected to the pin
        uint slice_num = pwm_gpio_to_slice_num(config.pinChannels[i]);        
        // Get some sensible defaults for the slice configuration. By default, the
        // counter is allowed to wrap over its maximum range (0 to 2**16-1)
        pwm_config configPwm = pwm_get_default_config();
        // Set divider, reduces counter clock to sysclock/this value
        pwm_config_set_wrap (&configPwm , TOP) ; // set top value for wrapping
        pwm_config_set_clkdiv_int (&configPwm , DIVIDER);
        // Load the configuration into our PWM slice, and set it running.
        pwm_init(slice_num, &configPwm, true);
        pwm_set_gpio_level ( (uint) config.pinChannels[i] , 0) ; // start PWM with 0% duty cycle
    }    
}


void updatePWM(){
    static uint32_t lastPwmMillis = 0 ;
    uint16_t pwmValue;
    float ratio;
    int16_t _pwmValue;
    int16_t pwmMax;
    int16_t pwmMin;

    if ( pwmIsUsed == false) return ; // skip when PWM is not used
    if ( ! lastRcChannels) return ;   // skip if we do not have last channels
    if ( (millisRp() - lastPwmMillis) > 5 ){ // we update once every 5 msec ???? perhaps better to update at each new crsf frame in order to reduce the latency
        lastPwmMillis = millisRp();
        if ( ( millisRp()- lastRcChannels) > FAILSAFE_DELAY ) { // if we do not get a RC channels frame, apply failsafe value if defined
            if (config.failsafeType == 'C') memcpy( &sbusFrame.rcChannelsData , &config.failsafeChannels, sizeof(config.failsafeChannels));
        }
        rcSbusOutChannels[0] = (uint16_t) sbusFrame.rcChannelsData.ch0 ;
        rcSbusOutChannels[1] = (uint16_t) sbusFrame.rcChannelsData.ch1 ;
        rcSbusOutChannels[2] = (uint16_t) sbusFrame.rcChannelsData.ch2 ;
        rcSbusOutChannels[3] = (uint16_t) sbusFrame.rcChannelsData.ch3 ;
        rcSbusOutChannels[4] = (uint16_t) sbusFrame.rcChannelsData.ch4 ;
        rcSbusOutChannels[5] = (uint16_t) sbusFrame.rcChannelsData.ch5 ;
        rcSbusOutChannels[6] = (uint16_t) sbusFrame.rcChannelsData.ch6 ;
        rcSbusOutChannels[7] = (uint16_t) sbusFrame.rcChannelsData.ch7 ;
        rcSbusOutChannels[8] = (uint16_t) sbusFrame.rcChannelsData.ch8 ;
        rcSbusOutChannels[9] = (uint16_t) sbusFrame.rcChannelsData.ch9 ;
        rcSbusOutChannels[10] = (uint16_t) sbusFrame.rcChannelsData.ch10 ;
        rcSbusOutChannels[11] = (uint16_t) sbusFrame.rcChannelsData.ch11 ;
        rcSbusOutChannels[12] = (uint16_t) sbusFrame.rcChannelsData.ch12 ;
        rcSbusOutChannels[13] = (uint16_t) sbusFrame.rcChannelsData.ch13 ;
        rcSbusOutChannels[14] = (uint16_t) sbusFrame.rcChannelsData.ch14 ;
        rcSbusOutChannels[15] = (uint16_t) sbusFrame.rcChannelsData.ch15 ;
        for( uint8_t i = 0 ; i < 16 ; i++){    
            if ( config.pinChannels[i] == 255) continue ; // skip i when pin is not defined for this channel 
            //pwmValue = fmap( rcSbusOutChannels[i] , 172, 1811, 988, 2012 );
            pwmValue = fmap( rcSbusOutChannels[i]  );
            //printf("chan= %u  pin= %u pwm= %" PRIu16 "\n", i , config.pinChannels[i] , pwmValue);
            #ifdef PITCH_CONTROL_CHANNEL
                if ( (i==(PITCH_CONTROL_CHANNEL-1)) && (mpu.mpuInstalled) && fields[PITCH].onceAvailable) {
                    // here we supposed that a PITCH_RATIO of 100 should provide a displacement of 100% of the servo and 90° of the camera
                    // so compensation of pitch 90° should change PWM value by 512 step
                    // so correction = pitch /90 * 512 * ratio /100 = pitch * ratio * 512 / 9000 = pitch *ratio * 0.0569
                    // here pitch in 0.1 of degree and so we have to multiply by 512/90000 = 0.00569
                    ratio = PITCH_RATIO;
                    #if defined(PITCH_RATIO_CHANNEL) && (PITCH_RATIO_CHANNEL >0) && (PITCH_RATIO_CHANNEL <= 16) 
                    ratio = ( (float) rcSbusOutChannels[PITCH_RATIO_CHANNEL - 1] - sbusCenter) * ratioSbusRange;
                    #endif
                    _pwmValue = ((int16_t) pwmValue) - (int16_t) (cameraPitch * ratio * 0.00569) ; 
                    pwmMax = fmapMinMax(PITCH_MAX);
                    pwmMin = fmapMinMax(PITCH_MIN);
                    if (_pwmValue > pwmMax ) _pwmValue = pwmMax;
                    if (_pwmValue < pwmMin ) _pwmValue = pwmMin;
                    //printf("%i %i %i %f %f\n", (int) cameraPitch , (int) pwmValue , (int) _pwmValue , (float) rcSbusOutChannels[PITCH_RATIO_CHANNEL - 1] , ratio);
                    pwmValue = _pwmValue;
                } 
            #endif
            #ifdef ROLL_CONTROL_CHANNEL
                if ( (i==(ROLL_CONTROL_CHANNEL-1)) && (mpu.mpuInstalled) && fields[ROLL].onceAvailable) {
                    // here we supposed that a PITCH_RATIO of 100 should provide a displacement of 100% of the servo and 90° of the camera
                    // so compensation of pitch 90° should change PWM value by 512 step
                    // so correction = pitch /90 * 512 * ratio /100 = pitch * ratio * 512 / 9000 = pitch *ratio * 0.0569
                    // here pitch in 0.1 of degree and so we have to multiply by 512/90000 = 0.00569
                    ratio = ROLL_RATIO;
                    #if defined(ROLL_RATIO_CHANNEL) && (ROLL_RATIO_CHANNEL >0) && (ROLL_RATIO_CHANNEL <= 16) 
                    ratio = ( (float) rcSbusOutChannels[ROLL_RATIO_CHANNEL - 1] - sbusCenter) * ratioSbusRange;
                    #endif
                    _pwmValue = ((int16_t) pwmValue) - (int16_t) (cameraRoll * ratio * 0.00569) ; 
                    pwmMax = fmapMinMax(ROLL_MAX);
                    pwmMin = fmapMinMax(ROLL_MIN);
                    if (_pwmValue > pwmMax ) _pwmValue = pwmMax;
                    if (_pwmValue < pwmMin ) _pwmValue = pwmMin;
                    //printf("%i %i %i %i\n", (int) cameraRoll , (int) pwmValue , (int) _pwmValue), (int) ratio;
                    pwmValue = _pwmValue;
                } 
            #endif
   
            pwm_set_gpio_level (config.pinChannels[i], pwmValue) ;
            
        }   
    }
    
}

uint16_t  fmap(uint16_t x)
{
    return (uint16_t)(((int)x - (int)fromSbusMin) * (int)(toPwmMax - toPwmMin) * 2 / (int)(fromSbusMax - fromSbusMin) + (int) toPwmMin * 2 + 1) / 2;
}



uint16_t fmapMinMax(int x){
    return (((x + 100) * (int)(toPwmMax - toPwmMin) / 100) + toPwmMin * 2 + 1) / 2;
}

/*

PIO pioPWM = pio1;
int smPWM0 = 0;
int smPWM1 = 1;

uint8_t pwmPioPin0 = 0;
uint8_t pwmPioPin11 = 11;

#define PWM_INTERVAl 20000

uint32_t lastPioPwmMillis = 0;

// Write `period` to the input shift register
void pio_pwm_set_period(PIO pio, uint sm, uint32_t period) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_put_blocking(pio, sm, period);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));
}

void setupPioPwm(){
    uint offset = pio_add_program(pioPWM, &pwm_program);
    if ( config.gpio11 > 0 && config.gpio11 < 17) {
        pwm_program_init(pioPWM, smPWM0, offset, pwmPioPin11);  // frequency is set in order to get about 1usec per cycle
        pio_pwm_set_period(pioPWM, smPWM0, PWM_INTERVAl - 4 ); // for 20 msec PWM cycle, we use a value of 20000 
    }
    if ( config.gpio0 > 0 && config.gpio0 < 17) {
        pwm_program_init(pioPWM, smPWM1, offset, pwmPioPin0);  // frequency is set in order to get about 1usec per cycle
        pio_pwm_set_period(pioPWM, smPWM1, PWM_INTERVAl - 4 ); // for 20 msec PWM cycle, we use a value of 20000 
    }
}

void updatePioPwm(){
    static uint32_t lastPioPwmMillis = 0 ;
    if ( ! lastRcChannels) return ;
    if ( (millisRp() - lastPioPwmMillis) > 20 ){ // we update once every 6 msec to avoid blocking the loop due to a full fifo
        lastPioPwmMillis = millisRp();
        if ( ( millisRp()- lastRcChannels) > 500 ) { // if we do not get a RC channels frame, apply failsafe value if defined
            if (config.failsafeType == 'C') memcpy( &sbusFrame.rcChannelsData , &config.failsafeChannels, sizeof(config.failsafeChannels));
        }
        uint16_t pwmValue;
        if ( config.gpio11 > 0 && config.gpio11 < 17) {
            pwmValue = fmap( rcSbusOutChannels[config.gpio11 - 1] , 172, 1811, 988, 2012 );
            pio_sm_put(pioPWM, smPWM0, (uint32_t) pwmValue); // we use the non blocking put instruction
            //pio_sm_put(pioPWM, smPWM0, 1500); // dummy value for testing
            pio_sm_set_enabled(pioPWM, smPWM0, true);     // start the PWM only when we have a first value
        }
        if (config.gpio0 > 0 && config.gpio0 < 17) {
            pwmValue = fmap( rcSbusOutChannels[config.gpio0 - 1] , 172, 1811, 988, 2012 );
            pio_sm_put(pioPWM, smPWM1, (uint32_t) pwmValue); // we use the non blocking put instruction
            //pio_sm_put(pioPWM, smPWM0, 1500); // dummy value for testing
            pio_sm_set_enabled(pioPWM, smPWM1, true);     // start the PWM only when we have a first value
        } 
    }
}
*/