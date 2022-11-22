# expressLRS_oXs

This project can be interfaced with 1 or 2 ELRS, FRSKY or Jeti receiver(s) (protocol has to be selected accordingly).
 
 This project is foreseen to generate telemetry data (e.g. when a flight controller is not used) , PWM and Sbus signals.
 
 For telemetry, it can provide
 
 * up to 4 analog voltages measurement (with scaling and offset)
 * the altitude and the vertical speed when connected to a pressure sensor (optional)
 * GPS data (longitude, latitude, speed, altitude,...) (optional)
 * RPM

It can also provide up to 16 PWM RC channels from a CRSF (ELRS) or a Sbus (Frsky/Jeti) signal.

When connected to 2 receivers, the generated PWM and Sbus signals will be issued from the last received Rc channels (= diversity).  

## -------  Hardware -----------------

This project requires a board with a RP2040 processor (like the rapsberry pi pico).

A better alternative is the RP2040-Zero (same processor but smaller board)

This board can be connected to:
* a pressure sensor to get altitude and vertical speed. It can be
   * a GY63 or a GY86 board based on MS5611
   * a SPL06-001 sensor
   * a BMP280 sensor
* a GPS from UBlox (like the BEITIAN bn220) or one that support CASIC messages

       note : a Ublox GPS has to use the default standard config. It will be automatically reconfigure by this firmware  
       
       a CASIC gps has to be configured before use in order to generate only NAV-PV messages at 38400 bauds  
       
       This can be done using a FTDI and the program GnssToolkit3.exe (to download from internet)
* some voltage dividers (=2 resistors) when the voltages to measure exceeds 3V

      note : a voltage can be used to measure e.g. a current when some external devices are use to generate an analog voltage 

## --------- Wiring --------------------

* FRSKY/ELRS receiver, MS5611 and GPS must share the same Gnd
* Connect a 5V source to the Vcc pin of RP2040 board (attention max is 5.5Volt)  
* Select the functions and pins being used
* The config parameters allow to select:

   * the pins used to generate PWM channels (Gpio0 up to Gpio15) 

   * a pin (within Gpio5 ,9, 21 or 25) that get the Rc channels and is connected to one receiver ELRS Tx or SBus pin.

   * a pin (within Gpio1, 13 , 17 or29) that get the Rc channels and is connected to a second receiver ELRS Tx or SBus pin.

   * a pin used to generate a Sbus signal (gpio 0...29)

   * a pin used to transmit telemetry data (gpio 0...29) (connected to ELRS Rx/Sport/Jeti Ex)

   * the max 4 pins used to measure voltages (gpio 26...29)
    
   * a pin used to measure RPM (gpio 0...29)
   
   * the 2 pins used for GPS (gpio 0...29)
   
   * the 2 pins connected to baro sensor (SDA=2, 6, 10, 14, 18, 22, 26) (SCL=3, 7, 11, 15, 19, 23, 27)


   Take care to use a voltage divider (2 resistances) in order to limit the voltage on those pins to 3V max 

* When a baro sensor is used:

   * Connect the 3V pin from RP2040 board to the 5V pin of GY63/GY86 or the Vcc of SPL06  

   Note: do not connect 5V pin of GY63/GY86 to a 5V source because the SDA and SCL would then be at 5V level and would damage the RP2040          

* When a GPS is used:

   * Connect the 3V pin from RP2040 board to the Vin/5V pin from GPS

* For more details, look at file named config.h

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
* when the RP2040 start (or pressing the reset button), press the enter key and it will display the current configuration and the commands to change it.
* if you want to change some parameters, fill in the command and press the enter.
* the RP2040 should then display the new (saved) config.  

Notes:

The RP2040 send the telemetry data to the ELRS receiver at some speed.

This speed (=baud rate) must be the same as the baudrate defined on the receiver

Usually ELRS receiver uses a baudrate of 420000 to transmit the CRSF channels signal to the flight controller and to get the telemetry data.

Still, ELRS receivers can be configured to use another baud rate. In this case, change the baudrate in parameters accordingly

