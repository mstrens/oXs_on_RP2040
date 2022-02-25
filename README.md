# expressLRS_oXs

Version of openXsensor to be used with expressLRS

This project is foreseen to generate telemetry data to a ELRS receiver when a flight controller is not used.

It can provide

    * up to 4 analog voltages
    
    * the altitude and the vertical speed when connected to a pressure sensor (optional)
    
    * GPS data (longitude, latitude, speed, altitude,...) (optional)



This project requires a board with a RP2040 processor (like the rapsberry pi pico).

A better alternative is the RP2040-Zero (same processor but smaller board): https://www.waveshare.com/rp2040-zero.htm

This board can be connected to:

* a pressure sensor (GY63 or GY86 board based on MS5611) to get altitude and vertical speed

* a GPS from UBlox (like the beitian bn220)

* some voltage dividers (=2 resistors) when the voltages to measure exceed 3V


More explanations are given in the file config_basic.h
