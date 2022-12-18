#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"


#include "ws2812.h"

#define IS_RGBW false

PIO rgbPio = pio1;
uint rgbSm  = 3;
uint8_t rgbPin = 16;  // RP2040 zero uses a rgb neopixel on pin 16
uint8_t rgbRed;
uint8_t rgbGreen;
uint8_t rgbBlue;
bool rgbOn = false;

void setupLed(){
    rgbOn = false;
    uint offset = pio_add_program(rgbPio, &ws2812_program);
    ws2812_program_init(rgbPio, rgbSm, offset, rgbPin, 800000, IS_RGBW);

}

void setRgbColor(uint8_t red , uint8_t green , uint8_t blue){
    rgbRed = red;
    rgbBlue = blue;
    rgbGreen = green;
}


void setRgbColorOn(uint8_t red , uint8_t green , uint8_t blue){
    rgbRed = red;
    rgbBlue = blue;
    rgbGreen = green;
    rgbOn = true;
    pio_sm_put_blocking(rgbPio, rgbSm , ( ((uint32_t) red) <<16) |
             ( ((uint32_t) green) << 24) |
             ((uint32_t) blue) << 8 );
}

void setRgbOn(){
    rgbOn = true;
    pio_sm_put_blocking(rgbPio, rgbSm ,  (((uint32_t) rgbRed) <<16) |
              (((uint32_t) rgbGreen) << 24) |
             (((uint32_t) rgbBlue) << 8) );

}

void setRgbOff(){
    rgbOn = false;
    pio_sm_put_blocking(rgbPio, rgbSm , 0);
}

void toggleRgb(){
    if (rgbOn) {
        setRgbOff();
    } else {
        setRgbOn();
    }    
}
