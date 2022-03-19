# expressLRS_oXs

Version of openXsensor to be used with expressLRS

This project is foreseen to generate telemetry data to a ELRS receiver when a flight controller is not used.

It can provide

    * up to 4 analog voltages
    
    * the altitude and the vertical speed when connected to a pressure sensor (optional)
    
    * GPS data (longitude, latitude, speed, altitude,...) (optional)

It can also generate:

    * 8 PWM signals to drive servos or ESC

    * a Sbus signal to drive Sbus servos or Sbus/PWM converters


This project requires a board with a RP2040 processor like the rapsberry pi pico.

A better alternative is the RP2040-Zero (same processor but smaller board): https://www.waveshare.com/rp2040-zero.htm

This board can be connected to:

* a pressure sensor (GY63 or GY86 board based on MS5611) to get altitude and vertical speed

* a GPS from UBlox (like the beitian bn220)

* some voltage dividers (=2 resistors) when the voltages to measure exceed 3V


This project does not require to compile the program nor a USB/SERIAL converter.
A compiled version of the firmware is available. It can be used to flash the RP2040 using just a USB cable.

To flash it : 
    * Press the "Boot" button when the USB cable in isserted in the PC. This will enter the "bootloader" mode.
    * The PC will then show a new drive named RPI-RP2.
    * Just drag and drop the uf2 file to this drive.
    * It will be automatically uploaded and the RP2040 will restart in "normal mode".
    * The drive RPI-RP2 will disapear and the PC will show a serial (COM) port.
    * Some options of the firmware can then be configured from the PC using a serial terminal emulator (arduino IDE, putty, ...)
    * Set the baudrate of the serial port to 115200.
    * Using the serial terminal, send "Enter" and the firmware will print the current config and the list of available commands.
         
More explanations are given in the config.h file.
