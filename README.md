# openXsensor (oXs) on RP2040 board
## For rc protocols : expressLRS / FRSKY (sport + Fbus) / HOTT / JETI Ex / JETI EXBUS/ MPX / FLYSKY / Futaba (SBUS2) / Spektrum (SRXL2) 

This project can be interfaced with 1 or 2 ELRS, FRSKY , HOTT , MPX, FLYSKY , Futaba, Spektrum or Jeti receiver(s) (protocol has to be selected accordingly).
 
### This project is foreseen to generate:
- telemetry data (e.g. when a flight controller is not used)
- PWM and/or
- Sbus signals
- PWM signals to stabilize a camera on pitch and roll

### For telemetry, it can provide
   - up to 4 analog voltages measurement (with scaling and offset) (optional)
   - one RPM measurement; a scaling (SCALE4) can be used to take care e.g. of number of blades (optional)
   - the altitude and the vertical speed when connected to a pressure sensor (optional)
   - the airspeed when connected to a differential pressure sensor (and a pitot tube) (optional)
   - compensated vertical speed when connected to a baro + a differentil pressure sensor 
   - Pitch and Roll when conncted to a MP6050 sensor (optional); 
   - GPS data (longitude, latitude, speed, altitude,...) (optional)
   Note: vertical speed is improved when baro sensor is combined with MP6050 sensor.
   
### It can also provide up to 16 PWM RC channels from a CRSF/ELRS or from 1 or 2 Sbus signal (e.g Frsky or Jeti).
 
### It can also provide SBUS signal (e.g. from 1 or 2 ELRS receivers). 
 When connected to 2 receivers, the generated PWM and Sbus signals will be issued from the last received Rc channels.
 So this provide a kind of redundancy/diversity.

Each function (telemetry/PWM/SBUS) can be used alone or combined with the others.

### To stabilize a camera, it requires
    - to use a mp6050 device
    - to configure oXs in order to get Rc channels and to generate PWM signals for the servos controling the camera
    - to edit the camera parameters in the config.h file and to compile the edited project.

## -------  Hardware -----------------

This project requires a board with a RP2040 processor (like the rapsberry pi pico).

A better alternative is the RP2040-Zero (same processor but smaller board)

This board can be connected to:
   * a pressure sensor (GY63 or GY86 board based on MS5611, SPL06 or BMP280) to get altitude and vertical speed
   * a MS4525D0_A or a SDP3X (x=1,2,3) or SDP8xx differential pressure sensor to get airspeed (and compensated vertical speed)
   * a MP6050 (acc+gyro e.g. GY86) to improve reaction time of the vario or to get pitch/roll
   * 1 or 2 ADS1115 if you want to measure more than 4 analog voltages
   * a GPS from UBlox (like the beitian bn220) or one that support CASIC messages   
       *  note : a Ublox GPS can be re-configured automatically by oXs ( with own oXs param). It has then to use the default standard ublox config.

          It can also be configured manually (with U-center firmware) externally prior to be connected to oXs. Set up must then be:  
              - 38400 baud (for a M10) or 9600 baud (for a M8)  
              - output on uart1: only 4 UBX messages (no NEMA): UBX-NAV-PVT , UBX-NAV-POSLLH, UBX-NAV-VELNED (when supported) , UBX-NAV-SOL (when supported)
       * a CASIC gps has to be configured before use in order to generate only NAV-PV messages at 38400 bauds  
           This can be done using a FTDI and the program GnssToolkit3.exe (to download from internet)
   * some voltage dividers (=2 resistors) when the voltages to measure exceed 3V  
      note : a voltage can be used to measure e.g. a current (Volt2) or a temperature (Volt3/4) when some external devices are used to generate an analog voltage 

About the SDP31, SDP32, SDP33 , SDP810:
     Those sensors are probably better than MS4525. They do not requires calibration (and reset) and are more accurate at low speed.
     Those sensors exists in 3 versions which differs by the maximum differential pressure (and so the max speed) they can measure
     SDP31 (or SDP810-500) can measure up to 500 Pa = 105 km/h
     SDP32 (or SDP810-125) can measure up to 125 Pa = 52 km/h
     SDP33 can measure up to 1500 Pa = 189 km/h
     The difference between SDP3x and SDP800 series is mainly the size of the sensor.
        SDP3x are very small (5mm) and require soldering on small pin
        SDP810 are bigger (25 mm) and have a 4 pin connector
     Currently oXs code is written for SDP3x serie but using a SDP810 requires only to change the I2C address in the config.h file.


## --------- Wiring --------------------

FRSKY/ELRS/JETI/... receiver, MS5611, GPS and other sensors must share the same Gnd  
Connect a 5V source to the Vcc pin of RP2040 board ( RP2040-zero board does not accept more than 5.5V on Vcc pin !! )  
There is no default affectation of the RP2040 pins so user has to specify it with some parameters after flashing the firmware (see below)  

Depending on the protocol, the pins used for PRIMARY/SECONDARY RC Channels and for Telemetry (TLM) varies
| protocol       | PRI pin is connectected to | SEC pin is connected to | TLM pin is connected to| Comment|
|----------      |----------------------------|-------------------------|------------------------|--------|
| C(ELRS)        |    (TX from Rx1)           |     (TX from Rx2)       | (RX from RX1)          |        |
| S(Frsky sport) |    (Sbus from Rx1)         |     (Sbus from Rx2)     |(Sport from RX1 or Rx2) |   (1)  |
| F(Frsky Fbus)  |    Fbus from Rx1           |     (Sbus from Rx2)     | Not used               |   (2)  |
| J(Jeti ex)     |    (Sbus from Rx1)         |     (Sbus from Rx2)     |  (Ex from Rx1 or Rx2)  |        |
| E(Jeti Exbus)  |    Exbus from Rx1          |     (Sbus from Rx2)     | Not used               |   (2)  |
| H(Hott)        |    (Sbus from Rx1)         |     (Sbus from Rx2)     |???(tlm from RX1 or Rx2)|   (1)  |
| M(Multiplex)   |    (Sbus from Rx1)         |     (Sbus from Rx2)     |???(tlm from RX1 or Rx2)|   (1)  |
| I(Flysky Ibus) |    (Sbus from Rx1)         |     (Sbus from Rx2)     | ( Ibus from RX1 or Rx2)|   (1)  |
| L(Spektrum Srxl2)|  Srxl from Rx1           |     Not used            | Not used               |   (2)  |
| 2(Futaba Sbus2) |   Sbus2 from Rx1          |     (Sbus2 from Rx2)    | Sbus2 from Rx1 via 1Kohm | (3)  |

Note: pins between () means that they are optional.

(1) for safety, insert a 1 kOhm resistor between TLM pin and Rx 

(2) for safety, insert a 1 kOhm resistor between PRI pin and Rx

(3) For Futaba, TLM pin must be equal to PRI pin - 1 and insert 1 kOhm resistor between PRI and TLM


Up to 16 PWM signals can be generated on pin gpio 0...15 (to select in setup parameters). 

Voltages 1, 2, 3, 4 can be measured on gpio 26...29. Take care to use a voltage divider (2 resistances) in order to limit the voltage on those pins to 3V max 

One RPM (Hz) can be measured
* Take care to limit the voltage to the range 0-3V; so if you use capacitor coupling, add diodes and resistor to limit the voltage
* All pulsed are counted (no debouncing); so use a hardware low pass filter (resistor/capitor) to avoid dummy pulses reading

When a MS5611/SPL06/BMP280 (baro sensor) and/or MP6050 is used:
* Connect the 3V pin from RP2040 board to the 5V pin of GY63/GY86 or the Vcc from other sensor   
           Note: do not connect 5V pin of GY63/GY86 to a 5V source because the SDA and SCL would then be at 5V level and would damage the RP2040          
* Connect SCL from baro sensor to the pin selected as SCL in parameter for RP2040
* Connect SDA from baro sensor to the pin selected as SDA in parameter for RP2040

When a differential pressure sensor is used, you should connect SCL/SDA like for a baro sensor.
* Vcc is connected to 5V or 3.3V depending on the chip you selected
* If the module you are using does not have pullup resistors and if you do not use other I2C modules, than you must use pullup resistor (4.7K) connected between SCL/SDA and 3.3V

When a GPS is used:
*  Connect the 3V pin from RP2040 board to the Vin/5V pin from GPS
*  Connect the RX pin from GPS to the RX pin selected in parameter for RP2040 
*  Connect the TX pin from GPS to the TX pin selected in parameter for RP2040
*  So take care that wires TX and RX are not crossed (as usual in Serial connection)  
      
The affectation of the pins has to be defined by the user.  
Here are the command codes and the pins that can be used are:  
Note: pin 16 is reserved for an internal LED on RP2040-zero as so should not be used.  
|Command|used for:|
|----|----|
|C1 = 0/15  ... C16 = 0/15|PWM output|
|GPS_TX = 0/29            |getting GPS data |
|GPS_RX = 0/29            |configuring GPS|
|PRI = 5 ,9, 21 ,25       |primary RC channel input|  
|SEC = 1, 13 , 17 ,29     |secondary RC channel input|  
SBUS_OUT = 0/29           |Sbus output|  
TLM = 0/29                |telemetry data (! for futaba Sbus2, this pin must be equal to PRI pin - 1)|  
VOLT1= 26/29 ... VOLT4 = 26/29 |voltage measurements|  
SDA = 2, 6, 10, 14, 18, 22, 26 | I2C devices (baro, airspeed, MP6050, ADS115, ...)|  
SCL = 3, 7, 11, 15, 19, 23, 27 | I2C devices (baro, airspeed, MP6050, ADS115, ...)|
RPM = 0/29                     | RPM|
LED = 16                       | internal led of RP2040-zero|  

## --------- Software -------------------
This software has been developped using the RP2040 SDK provided by Rapsberry.

If you just want to use it, there is (in most cases) no need to install/use any tool.
* download from github the zip file containing all files and unzip them where you want.
* in your folder, there is a file named oXs.uf2; this is a compiled version of this software that can be directly uploaded and configured afterwards
* insert the USB cable in the RP2040 board
* press on the "boot" button on the RP2040 board while you insert the USB cable in your PC.
* this will enter the RP2040 in a special bootloader mode and your pc should show a new drive named RPI-RP2
* copy and paste (or drag and drop) the oXs.uf2 file to this new drive
* the file should be automatically picked up by the RP2040 bootloader and flashed
* the RPI_RP2 drive should disapear from the PC and the PC shoud now have a new serial port (COMx on windows)
* you can now use a serial terminal (like putty , the one from arduino IDE, ...) and set it up for 115200 baud 8N1
* while the RP2040 is connected to the pc with the USB cable, connect this serial terminal to the serial port from the RP2040
* when the RP2040 start (or pressing the reset button), press Enter and it will display the current configuration and the commands to change it.
* if you want to change some parameters, fill in the command (code=value) and press the enter.
* the RP2040 should then display the new (saved) config.  
   
Developers can change the firmware, compile and flash it with VScode and Rapsberry SDK tools.  
An easy way to install those tools is to follow the tutorials provided by Rapsberry.  
In particular for Windows there is currently an installer. See : https://github.com/raspberrypi/pico-setup-windows/blob/master/docs/tutorial.md

Once the tools are installed, copy all files provided on github on you PC (keeping the same structure).  
Open VScode and then select menu "File" + item "Open Folder". Select the folder where you copied the files.  
In VScode, press CTRL+SHIFT+P and in the input line that appears, enter (select) CMake: Configure + ENTER  
This will create some files needed for the compilation.  
To compile, select the "CMake" icon on the left vertical pannel (rectangle with a triangle inside).  
Move the cursor on the line oXs [oXs.elf]; an icon that look like an open box with some dots apears; click on it.  
Compilation should start. When done a new file oXs.uf2 should be created.  
For more info on VScode and SDK look at tutorials on internet.  

Note :  the file config.h contains some #define that can easily be changed to change some advanced parameters.

Note for ELRS:  
The RP2040 send the telemetry data to the ELRS receiver at some speed.  
This speed (=baud rate) must be the same as the baudrate defined on the receiver.  
Usually ELRS receiver uses a baudrate of 420000 to transmit the CRSF channels signal to the flight controller and to get the telemetry data.  
Still, ELRS receivers can be configured to use another baud rate. In this case, change the baudrate in parameters accordingly.  

You have to compile your self the firmware if you want to change some values in the config.h file in order e.g. to:
* change the setup of the ADS1115
* allocate other slots for Sbus2 in Futaba protocol
* allocate another physical ID for Sport in Sport/Fbus protocols
* avoid or change priorities of some telemetry fields for Sport in Sport/Fbus protocols
* assign another sequence number and/of generate alarms for some telemetry fields in Multiplex protocol
* change the I2C address of some I2C sensors

## ------------ Failsafe---------------
* For ELRS protocol, oXs does not received any RC channels data from the receiver(s) when RF connection is lost. If oXs is connected to 2 receivers (via PRI and SEC), oXs will generate PWM and Sbus signals on the last received data. If oXs does not get any data anymore from receiver(s), it will still continue to generate PWM and/or SBUS signals based on the failsafe setup stored inside oXs.


* For Frsky/Jeti... protocols where Sbus is used, the failsafe values are normally defined inside the receiver and the receiver continue to generate a Sbus signal even if the RF connection is lost. Still, when connection is lost Sbus signal contains some flags that say that some data are missing or that failsafe has been applied. When oXs is connected to 2 different receivers, it gives priority to PRI sbus signal except when SEC signal is OK and PRI is not OK (no signal, missing frame, failsafe). So for Frsky/Jeti, oXs does not have to take care of his own failsafe setup (except if oXs would not get any Sbus signal anymore - e.g due a wiring issue).   

* For failsafe oXs has 3 options:
    * "Hold" = failsafe will be the last Rc channels values known just before connection is lost; to select this option, use the serial interface with the command "FAILSAFE=H"
    * store as failsafe the current RC channels values using the serial interface with command "SETFAILSAFE".
    *  store as failsafe the current RC channels values using the "boot" button on the RP2040. To activate this option, doubble click the button. Led should become fixed blue. In the next 5 seconds, press and hold the "boot" button. Led will become white for 2 seconds confirming that values are saved in the config.
    
For the 2 last options, the handset must be on and generating the channels values that you want to save in oXs.


## --- Telemetry fields being measured and transmitted ---

oXs tries to detect automatically which sensors are connected (based on the parameters being fill in the setup).
It can display on the PC (on a serial terminal getting the messages via usb ) the current setup and the sensors that have been discovered.

oXs measures different fields depending on the sensors being detected.

Please note that the data being transmitted depends also on the protocol being used (Sport, ELRS, ...).

For more information, please look at document "fields per protocol.txt" in folder "doc"

When a baro sensor and an airspeed sensor are both used, oXs calculates 2 vertical speeds: 
* the normal one based only on the baro sensor; this one is always transmitted
* an airspeed compensated Vspeed (=dte) that take care of the variation of airspeed.

You can use a channel to control the way airspeed compensated Vspeed is calculated and/or transmitted.

First you have to send a command ACC (via the PC) to specify the channel being used (1...16).

You must use a protocol/wiring (like Sbus, Fbus, Exbus, ...) that allows the Rx to communicate the RC channel values to oXs.

Then, depending on the value sent by the Tx on the selected channel, oXs manages the airspeed compensated Vspeed in different ways:
* if the value is around the center position, oXs uses a default coefficient (defined in config.h file as 1.15) to calculate compensated Vspeed and transmit it.
* if the value is largely positive, oXs uses it to adapt the coefficient (from 0.9 up to 1.4). Assigning e.g. a slider to this channel allows you adjust the coefficient while flying to find the best value.
* if the value is largely negative, oXs sent the "normal" Vspeed in the field foreseen for compensated Vspeed. So even if vario tone is based on compensated Vspeed telemetry field, you can switch while flying between the 2 Vspeed (with a switch on the TX).

Note: you can use the FV command to know the current coefficient. This allow you to check that your Tx sent a Rc channel value that match the expected goal and indeed required, adjust your Tx settings.
## ------------------ Led -------------------
When a RP2040-Zero is used, the firmware will handle a RGB led (internally connected to gpio16).
* when config is wrong, led is red and ON.
* when config is valid, led is blinking and the color depends on RC channels being received ot not
    * Red = Rc frames have nerver been received, Sbus and/or PWM signals are not generated.
    * Blue = Sbus and/or PWM signals are based on failsafe values. Failsafe values are given by the receiver for Sbus or are configured in oXs for CRSF protocol)
    * Yellow/oranje = Sbus and/or PWM signals are based on a valid RC channels frame received from PRI or SEC source but the orther source does not provided a valid RC channels frame.
    * blinking green = Sbus and/or PWM signals are based on valid RC channels frames (from one source; from both sources if both are foressen in the setup)
* when "Boot" button is used for setting the failsafe values, led becomes blue and white (see above)

Note: some users got a RP2040-zero where red and green colors are inverted.
If you got such a device and want to get the "normal" colors, you can uncomment a line in config.h file.
It is line :

//#define INVERTED_RGB // uncomment if red and green colors are inverted on your board


Please note that other boards do not have a RGB led on gpio16 and so this does not applies.
