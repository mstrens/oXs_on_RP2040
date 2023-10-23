#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"
#include "hardware/watchdog.h"
#include "config.h"
#include "param.h"

#include "ws2812.h"

#define IS_RGBW false // no white led are on the RP2040-zero board

extern CONFIG config;

PIO rgbPio = pio1;
uint rgbSm  = 3;
//uint8_t rgbPin = 16;  // RP2040 zero uses a rgb neopixel on pin 16
uint8_t rgbRed;
uint8_t rgbGreen;
uint8_t rgbBlue;
bool rgbOn = false;

extern CONFIG config;

void setupLed(){
    if ((config.pinLed >= 0 ) && (config.pinLed <= 29 ) ){
        rgbOn = false;
        uint offset = pio_add_program(rgbPio, &ws2812_program);
        ws2812_program_init(rgbPio, rgbSm, offset, config.pinLed, 800000, IS_RGBW);
    }
}

void setRgbColor(uint8_t red , uint8_t green , uint8_t blue){
    rgbRed = red;
    rgbGreen = green;
    rgbBlue = blue;
}


void setRgbColorOn(uint8_t red , uint8_t green , uint8_t blue){
    rgbRed = red;
    rgbBlue = blue;
    rgbGreen = green;
    setRgbOn();
}

void setRgbOn(){
    if (config.pinLed <= 29 ) {
        rgbOn = true;
        if (config.ledInverted == 'I') {
            pio_sm_put_blocking(rgbPio, rgbSm ,  (((uint32_t) rgbGreen) <<16) |
                (((uint32_t) rgbRed) << 24) |
                (((uint32_t) rgbBlue) << 8) );
        } else {
            pio_sm_put_blocking(rgbPio, rgbSm ,  (((uint32_t) rgbRed) <<16) |
                (((uint32_t) rgbGreen) << 24) |
                (((uint32_t) rgbBlue) << 8) );
        }
    }    
}

void setRgbOff(){
    if (config.pinLed <= 29 ){
        rgbOn = false;
        pio_sm_put_blocking(rgbPio, rgbSm , 0);
    }    
}

void toggleRgb(){
    if (rgbOn) {
        setRgbOff();
    } else {
        setRgbOn();
    }
    //static uint8_t count;
    //printf("%d\n",(int) count);
    //count++;    
}

void blinkRgb(uint8_t red , uint8_t green , uint8_t blue, uint32_t period , uint32_t count){
    setRgbColor(red,green,blue);
    while(count){
        setRgbOn();
        watchdog_update();
        sleep_ms(period);
        setRgbOff();
        watchdog_update();
        sleep_ms(period);
        count--;
    }       
}

void checkLedColors(){
    while (1){
        blinkRgb(10,0,0, 500 , 10);
        blinkRgb(0,10,0, 1000 , 5);
        blinkRgb(0,0, 10, 5000 , 1);
    }
}