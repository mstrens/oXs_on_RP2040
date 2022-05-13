# expressLRS_oXs

This project can be interfaced with an ELRS, a FRSKY or a Jeti receiver (protocol has to be selected accordingly).
 
 This project is foreseen to generate telemetry data (e.g. when a flight controller is not used) , PWM and Sbus signals.
 
 For telemetry, it can provide
 
 * up to 4 analog voltages measurement (with scaling and offset)
 * the altitude and the vertical speed when connected to a pressure sensor (optional)
 * GPS data (longitude, latitude, speed, altitude,...) (optional)

It can also provide 10 PWM RC channels from a CRSF (ELRS) or a Sbus (Frsky) signal.

It can also provide a Sbus signal (only from CRSF/ELRS signal; for Frsky Sbus is provided by the Frsky Receiver itself.  

## -------  Hardware -----------------

This project requires a board with a RP2040 processor (like the rapsberry pi pico).

A better alternative is the RP2040-Zero (same processor but smaller board)

This board can be connected to:
* a pressure sensor to get altitude and vertical speed. It can be
   * a GY63 or a GY86 board based on MS5611
   * a SPL06-001 sensor
* a GPS from UBlox (like the beitian bn220) or one that support CASIC messages

       note : a Ublox GPS has to use the default standard config. It will be automatically reconfigure by this firmware  
       
       a CASIC gps has to be configured before use in order to generate only NAV-PV messages at 38400 bauds  
       
       This can be done using a FTDI and the program GnssToolkit3.exe (to download from internet)
* some voltage dividers (=2 resistors) when the voltages to measure exceed 3V

      note : a voltage can be used to measure e.g. a current when some external devices are use to generate an analog voltage 

## --------- Wiring --------------------

* FRSKY/ELRS receiver, MS5611 and GPS must share the same Gnd
* Connect a 5V source to the Vcc pin of RP2040 board  
* When used with a ELRS receiver:
   * Connect gpio 9 from RP2040 (= PIO RX signal) to the TX pin from ELRS receiver (this wire transmit the RC channels)
   * Connect gpio 10 from RP2040 (= PIO TX signal) to the RX pin from ELRS receiver (this wire transmits the telemetry data)
* When used with a FRSKY receiver:
   * Connect gpio 9 from RP2040 (= UART0 RX signal) to the Sbus pin from Frsky receiver (this wire transmit the RC channels)
   * Connect gpio 10 from RP2040 (= PIO TX signal) via a 1k resistor to the Sport pin from Frsky receiver (this wire transmits the telemetry data)  
    
* 9 PWM signals can be generated on gpio 1...8 and gpio 11.
* One more PWM can be generated on gpio 0 when this pin is not used to generate a Sbus signal 

* The config parameters allow:

   * to select the RC channels generated on gpio 1, 5 and 11. Gpio 2..4 (and gpio 6...9) will then generate the following RC channels. 

   * to select if gpio 0 has to generate a Sbus signal or a PWM RC channel.

* Voltages 1...4 are measured on gpio 26...29 

   Take care to use a voltage divider (2 resistances) in order to limit the voltage on those pins to 3V max 

* When a baro sensor is used:

   * Connect the 3V pin from RP2040 board to the 5V pin of GY63/GY86 or the Vcc of SPL06  

   Note: do not connect 5V pin of GY63/GY86 to a 5V source because the SDA and SCL would then be at 5V level and would damage the RP2040          

   * Connect SCL to gpio 15 (I2C1)

   * Connect SDA to gpio 14 (I2C1)

* When a GPS is used:

   * Connect the 3V pin from RP2040 board to the Vin/5V pin from GPS

   * Connect the RX pin from GPS to gpio 12 (UART0-TX) 
   
   * Connect the TX pin from GPS to gpio 13 (UART0-RX)
        
## --------- Software -------------------
This software has been developped using the RP2040 SDK provided by Rapsberry.

It uses as IDE platformio and the WIZIO extension (to be found on internet here : https://github.com/Wiz-IO/wizio-pico )

Developers can compile and flash this software with those tools.

Still if you just want to use it, there is no need to install/use those tools.

On github, in uf2 folder, there is already a compiled version of this software that can be directly uploaded and configured afterwards

To upload this compiled version, the process is the folowing:
* download the file in folder uf2 on your pc
* insert the USB cable in the RP2040 board
* press on the "boot" button on the RP2040 board while you insert the USB cable in your PC.
* this will enter a special bootloader mode and your pc should show a new drive named RPI-RP2
* copy and paste the uf2 file to this new drive
* the file should be automatically picked up by the RP2040 bootloader and flashed
* the RPI_RP2 drive should disapear from the PC and the PC shoud now have a new serial port (COMx on windows)
* you can now use a serial terminal (like putty , the one from arduino IDE, ...) and set it up for 115200 baud 8N1
* while the RP2040 is connected to the pc with the USB cable, connect this serial terminal to the serial port from the RP2040
* when the RP2040 start (or pressing the reset button), it will display the current configuration and the commands to change it.
* if you want to change some parameters, fill in the command and press the enter.
* the RP2040 should then display the new (saved) config.  

Notes:

The RP2040 send the telemetry data to the ELRS receiver at some speed.

This speed (=baud rate) must be the same as the baudrate defined on the receiver

Usually ELRS receiver uses a baudrate of 420000 to transmit the CRSF channels signal to the flight controller and to get the telemetry data.

Still, ELRS receivers can be configured to use another baud rate. In this case, change the baudrate in parameters accordingly

