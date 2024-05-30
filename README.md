# openXsensor (oXs) on RP2040 board
## For rc protocols : expressLRS / FRSKY (sport + Fbus) / HOTT / JETI Ex / JETI EXBUS/ MPX / FLYSKY / Futaba (SBUS2) / Spektrum (SRXL2) 

This project can be interfaced with 1 or 2 ELRS, FRSKY , HOTT , MPX, FLYSKY , Futaba, Spektrum or Jeti receiver(s) (protocol has to be selected accordingly).
 
### This project is foreseen to generate:
- telemetry data (e.g. when a flight controller is not used)
- PWM servo signals (based on Rc channel)
- Sbus signals
- PWM signals to stabilize a camera on pitch and roll
- different sequences of PWM signals (to control Servo or to generate an analog/digital voltage) based on Rc channel values
- data's (telemetry and/or PWM Rc channels) to be logged on a SD card
- localisation data's on a second Rf link in order to retrieve a lost model (= locator)
### For telemetry, it can provide
   - up to 4 analog voltages measurement (with scaling and offset) (optional); one voltage is normally used to measure a current and 1 or 2 (optionnaly) for temperature(s)
   - one RPM measurement; a scaling (rpmMultiplicator) can be used to take care e.g. of number of blades (optional)
   - the altitude and the vertical speed when connected to a pressure sensor (optional)
   - the airspeed when connected to a differential pressure sensor (and a pitot tube) (optional)
   - compensated vertical speed when connected to a baro + a differentil pressure sensor 
   - Pitch/Roll and accelerations X/Y/Z when conncted to a MP6050 sensor (optional); 
   - GPS data (longitude, latitude, speed, altitude,...) (optional)
   - rpm/volt/temp/current/consumption from some ESC (Hobbywing4, ZTW mantis, Kontronix, BlHeli, Jeti)
   Note: vertical speed is improved when baro sensor is combined with MP6050 sensor.
   
### It can also provide up to 16 PWM RC channels to drive servos from a CRSF/ELRS or from 1 or 2 Sbus/Fbus/Exbus/Ibus/SRXL2 signal (e.g Frsky,Jeti,Flysky,Spektrum). The refresh rate can be set between 50Hz(default) and 333Hz.
 
### It can also provide SBUS signal (e.g. from 1 or 2 ELRS receivers). 
 When connected to 2 receivers, the output signals (e.g. PWM or Sbus) will be issued from the last received Rc channels.
 So this provides a kind of redundancy/diversity.

### It can stabilize the plane (gyro). This requires
    - to use a mp6050 device
    - to configure oXs in order to get Rc channels and to generate PWM signals for the servos controling the camera
    - to get from the handset the "original" (not mixed) sticks positions in 3 additional RC channels
    - to get from the handset the gyro mode and general gain in 1 additional RC channels
   
    
### It can stabilize a camera. This requires
    - to use a mp6050 device
    - to configure oXs in order to get Rc channels and to generate PWM signals for the servos controling the camera
    - to edit the camera parameters in the config.h file and to compile the edited project.

### It can generates some sequences on servos or e.g. led. This requires to configure oXs in order to get Rc channels.


### It can log data's on a SD card. This require to build also another module with another RP2040: see oXs_logger project


### It can transmit some localisation data on a separate rf link to another module in order to retrieve a model that has been lost.

Each function (telemetry/PWM/SBUS/gyro/logger/sequencer/localisation) can be used alone or combined with the others.


Note: when a mpu6050 is used (to improve vario reactivity, stabilize the plane and/or a camera), it is important to calibrate the mp6050 horizontally and vertically (see section below)

[![](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](https://www.paypal.com/donate/?hosted_button_id=4T3KNNJ58J3B4)


## -------  Hardware -----------------

This project requires a board with a RP2040 processor (like the rapsberry pi pico).

A better alternative is the RP2040-Zero or the RP2040-TINY (both have the same processor but smaller board)

This board can be connected to:
   * one current sensor providing an analog voltage depending on the current
   * one or two temperature sensor(s) providing an analog voltage depending on the temperature. The sensor can be an IC like the TMP36 or a CTN (thermistor). In the last case (CTN) you have to enter the specification of the CTN in config.h and to compile yourself.
   * a pressure sensor (GY63 or GY86 board based on MS5611, SPL06 or BMP280) to get altitude and vertical speed
   * a MS4525D0_A or a SDP3X (x=1,2,3) or SDP8xx  or XGZP6897D differential pressure sensor to get airspeed (and compensated vertical speed)
   * a MP6050 (acc+gyro e.g. GY86) to improve reaction time of the vario, to get pitch/roll, to stabilize a camera and/or the model 
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
   * a RPM sensor
   * an ESC from Hobbywing (using V4 telemetry protocol), ZTW mantis, Kontronik, Jeti or from BlHeli. Those ESC provide one voltage, one current (+ current consumption) + RPM + 1 or 2 temperatures.
   * another rp2040 with an SD card to log huge volume of data's
   * a LORA module with a Ebyte E220-900M22S (previous version - about 1.14.10 was with SX1276/RFM95) to transmit the localisation on a long range rf link (see locator section)   

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
Connect a 5V source to the Vcc pin of RP2040 board ( RP2040-zero or RP2040-TINY boards do not accept more than 5.5V on Vcc pin !! )  
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

Voltages 1, 2, 3, 4 can be measured on gpio 26...29. Take care to use a voltage divider (2 resistances) in order to limit the voltage on those pins to 3V max. V2 is normally used to measure a current (based on the analog voltage). V3 and V4 can be used to measure or a voltage or a temperature (based on a voltage provided by a sensor like TMP36 or a CTN/thermistor). For each voltage being measured, you probably have to specify the offset and scale to be applied.

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

When a Hobbywing, ZWT, Kontronik, Jeti or BlHeli ESC is used:
 * Connect the serial pin from ESC to the pin selected in parameter for RP2040 (for ESC_PIN)
 * Connect GND from ESC to RP2040 GND
 * do not define gpio's in RP2040 parameters for V1, V2, RPM and let TEMP parameter on 0. You can use V3 and V4 if you want. Note: SCALE1, SCALE2, OFFSET2 and RPM_MULT have to be defined based on your ESC and your motor.

About sequencers and locator, see below.

The affectation of the pins has to be defined by the user.  
Here are the command codes and the pins that can be used are:  
Note: pin 16 is reserved for an internal LED on RP2040-zero or RP2040-TINY and so should not be used with this board.  
|Command|used for:|
|----|----|
|C1 = 0/15  ... C16 = 0/15|PWM output|
|GPS_TX = 0/29            |getting GPS data |
|GPS_RX = 0/29            |sending configuration to GPS|
|PRI = 5 ,9, 21 ,25       |primary RC channel input|  
|SEC = 1, 13 , 17 ,29     |secondary RC channel input|  
|SBUS_OUT = 0/29           |Sbus output|  
|TLM = 0/29                |telemetry data (! for futaba Sbus2, this pin must be equal to PRI pin - 1)|  
|V1= 26/29 ... V4= 26/29 |voltage (or current/temperatue) measurements |  

|RPM = 0/29                     | RPM|
|LED = 16                       | internal led of RP2040-zero or RP2040-TINY|  
|LOG = 0/29                     | data to be logged |  
|ESC_PIN = 0/29                 | data provided by ESC (rpm, volt, current, temp)|
|SPI_CS  = 0/29                 | Chip Select (=NSS) pin from E220-900M22S (locator)|
|SPI_SCK = 10, 14, 26           | SCK pin from E220-900M22S  (locator)|
|SPI_MOSI = 11, 15, 27          | MOSI pin from E220-900M22S  (locator)|
|SPI_MISO = 8, 12, 24, 28       | MISO pin from E220-900M22S  (locator)| 
|SPI_BUSY = 0/29                | Busy pin from E220-900M22S  (locator)|
|HIGH = 0/29                    | set the voltage level to 3V; can be used as Vcc for some sensors|
|LOW = 0/29                     | set the voltage level to 0V; can be used as Ground for some sensors|


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
* you can now use a serial terminal (like putty , the one from arduino IDE, ...) and set it up for 115200 baud 8N1.
* IMPORTANT NOTE: Set your USB/SERIAL terminal in order to let it send automatically CR/LF (carriage return+line feed) when you press ENTER.
* while the RP2040 is connected to the pc with the USB cable, connect this serial terminal to the serial port from the RP2040
* when the RP2040 start (or pressing the reset button), press just Enter and it will display the current configuration.
* to list all the commands, send ?.
* if you want to change some parameters, fill in the command (code=value) and press the enter.
* you can enter severals commands without repowering the device
* to fill several command at once, use a ";" to separate several commands; press enter at the end. 
* Important note : when you enter usb commands to change parameters, they are not automatically applied. Most of the time, oXs will stop most functionalities. You have to save the changes using the SAVE command and then make a power off/on.
* with the DUMP command, you can get a list of most important parameters (not failsafe, mpu offsets nor sequencers) in a format that allows easy copy/paste/edit. This is an easy way to copy parameters from one oXs device to another one. 


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

You have to compile your self the firmware if you want to change some less usual values in the config.h file in order e.g. to:
* change the setup of the ADS1115
* allocate other slots for Sbus2 in Futaba protocol
* allocate another physical ID for Sport in Sport/Fbus protocols
* avoid or change priorities of some telemetry fields for Sport in Sport/Fbus protocols
* assign another sequence number and/of generate alarms for some telemetry fields in Multiplex protocol
* change the I2C address of some I2C sensors
* use other default paramaters in order to avoid using commands via the USB/serial monitor. 
* change the sensitivity of the XGZP sensor (if different from XGZP6897D001KPDPN) (see #define XGZP_K_FACTOR in config.h)

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

## ---------------- Calibration and orientation of MP6050  ---------------

When MP6050 (=MPU) is used, oXs must know his orientation in the model. There are many possible orientations.

Furthermore, for best result, each MP6050 must be calibrated (accelerometer and gyro). This is just optional when MP650 is used to improve vario reactivity (so not to know roll/pitch, stabilize camera and/or model). 

Best is to start with calibration of the accelerometer, then calibration of the gyro and finaly to set up the orientation of the MPU in the model.

Note: Once the MP6050 calibration process has been done and saved, it is normally not required to do it again.
Take care that, if you change the orientation of the M6050 in a model, you have to update the orientation parameters in oXs (not the calibration).


### 1 Calibration of the accelerometer

To calibrate the MP6050, oXs must be conected to the PC (via usb) and the PC must be running a serial montitor (to enter commands and display the oXs messages). It requires to make several measurements while the sensor remains fix in many different orientations.

* The process starts entering a USB command MPUCAL=A.
* Then the sensor is put in one orientation and while staying fix you press ENTER. oXs performs a measurement. It is is valid, oXs displays the accelerations X,Y,Z in this position. If the measurement is not valid, oXs displays a message saying that the measure was to noisy (then press ENTER again to take the mesures again in the same position).
* When done in one position, change slightly the orientation of the sensor and press ENTER again to make a new mesurement.
* Repeat the process in many different orientations (more than 20, less than 200).
* When done, enter the command MPUCLA=E to end the process.
* oXs specifies then the number of measurements, the scale being used and a list of all measurements.
* Make a copy/paste of this list and save it in a txt file.
* Run the program "magneto12.exe" that is present in the folder doc. In this program fill the box "Norm" with the scale and open the saved txt file that you created with the measurements. Click the button "Calibrate". This provides 12 parameters ( 3 values in "(b)" and 3 rows of 3 values in "(A)").
* Enter those 12 parameters (space delimited) in oXs with one  command MPUACC=xx.xxxxx yy.yyyyy ....
* Then enter SAVE command to save the parameters.

In the doc folder, you can also find word document explaining again the process and a link to a video about accelerometer calibration. It shows the principle even if it is not done with an oXs device.


### 2 Calibration of the gyro

The gyro calibration is quite easy because it only requires that the sensor does not move at all.

It is normally done automatically at each power on but it requires that the model stay still the first 2 sec. If oXs detects that the sensor moves during the first 2 sec, the process will fail and oXs will reuse parameters that have been saved.

Note: It is possible to disable this automatic calibration with a parameter in file config.h. 

So, it is not bad practice to save gyro calibration offsets. To do so, oXs must be conected to the PC (via usb) and the PC must be running a serial montitor (to enter commands and display the oXs messages).

* Keep the model absolutely still (orientation does not matter) and enter a USB command MPUCAL=G
* After a short time, oXs says if calibration is OK or if the measurement is not valid (to noisy, model is moving)
* If measurement is not valid, reenter the command MPUCAL=G
* Then enter SAVE command to save the parameters.

### 3 Set up the gyro orientation.

The mpu must be installed in the model in such a way that one axis of MPU6050 is vertical and that another axis is aligned with main axis of the model (nose-queue). There are 24 possible orientations to match this. Most commercial gyro's require that the user declares the orientation of the MP6050 in the model. oXs does it automatically.

There are 2 ways to let oXs know the MPU orientation:
* using USB commands (MPUORI=H and MPUORI+V); this is the easiest way to do it when the MP6050 is not used to stabilize the model (gyro function). The process is explained here below. 
* using the gyro learning process with the handset; this process is used when the MP6050 is used to stabilize the model (gyro function) because it is combined with the learning of the servo mixers defined on the handset. It is done from the handset (even if messages can also be displayed on the PC via USB). It is explained in the gyro section (see learning process). 

#### Process with USB command:
* Set the plane horizontally (like when it flies and roll/pitch are both 0) and enter the usb command MPUORI=H. The result is displayed.
* Set the plane vertically with the nose up and enter the usb command MPUORI=V. The result is displayed.
* Then enter SAVE command to save the parameters.


Notes:
* The current configuration is displayed (like other parameters) with the ENTER command.
* When the MP6050 is used to stabilize the plane (gyro function), you have also to perform the gyro learning process to let oXs know the  gyro mixer calibration. See gyro section.  

## ------------------ Gyro ------------------

Important note: at this stage, this is still experimental. It has not been intensively tested. So used it at you own risk.

### Principle.
- When oXs get the Rc channels and has a MPU6050 (accelerometer/gyro), oXs can automatically apply corrections on the PWM/sbus signals in order to stabilize the model on 3 axis
- Gyro has 4 modes (off/Normal/Hold/stabilize); user selects the active mode (between 3) on the Tx with a 3 positions switch.
- This switch must allow the handset to generate on a Rc channel a signal that is or negative (Normal mode), or null (gyro off) or positive (gyro in hold or stabilize mode depending on a oXs parameter).
    - In "Normal" mode, the gyro tries to compensate for external perturbations (wind,...). The sticks allow to control the model.
    - In "OFF" mode, oXs just transmit the channels provided by the handset without any gyro correction
    - In "Hold" mode, oXS tries to keep the model in the current orientation when sticks are centered. Moving the sticks allows to change the orientation of the model.
    - In "Stabilize" mode, oXs tries to keep the model horizontal when the sticks are centered. The sticks allow to control the model. 
- Respectively the positive and the negative values from this channel allow also to select the general gain of the gyro (for each mode separately) 
- oXs can apply Gyro corrections on as many servos as needed (e.g. for a wing with 4 aileron servos, on 2 elevator servos and/or 2 rudder servos, on Vtail stab,... )
- On the opposite to many commercial gyros, the mixers and servos limits are defined only in the handset (just like when no gyro is used). oXs detect automatically the mixers and limits applied on the servos concerned by gyro corrections during a special setup phase (= gyro learning phase = mixer calibration).
- To be able to apply corrections using the same mixers as the handset, oXs must also know the position of the 3 sticks aileron, elevator and rudder. This requires that the handset transmit those positions on top of all channels that control the servos. So this requires 3 more channels + 1 channel to select the gyro mode/gain than when no gyro is used.
- several parameters (see below) have to be defined with Usb commands (no compilation/reflash required) to set up the oXs gyro. 

### Setup on the handset.
- first make your setup just like there would be no gyro (mixers, servo directions, limits, expo, differential, ...).
- on top of your normal setup, add a channel controlled by a 2 or 3 positions switch that will let you select the gyro mode. When rc channel provides 0%, the gyro will be OFF. When the channel provides a negative value (between -100% and 0%), the gyro will be in "Normal" mode (=rate mode). When the channel provides a positive value (between 0% and 100%), the gyro will be in "Hold" or "Stabilize" mode depending on an oXs parameter. The gain of the gyro can (should) be different in each mode. So the negative value can (should) not be just the opposite of the positive. When the absolute value is 100%, the gyro gain is at max. Best is to use some global variables and/or slider if the handset allows it.
- let the handset transmit 3 additional channels giving the stick positions (aileron, elevator and rudder). Range must be -100%/100%. Best is to avoid to include trim, expo, differential, ... so that those additional channels represent just the raw positions of the sticks.


### Required oXs parameters to set up the gyro.
oXs has to know: 
- gpio's and channels used for servos (just like when no gyro is used) with commands like C2=4 (meaning channel 2 is generated on gpio 4)
- the channel used to select the gyro mode and the general gyro gain. This is specified with the command GMG=xx (Gyro Mode Gain; xx = the rc channel between 1 and 16).  
- the 3 additional channels providing "original" stick positions. Those channels are specified with the commands GSA (Gyro Stick Aileron), GSE(gyro Stick Elevator), GSR (gyro Stick Rudder)

### Optional oXs parameters to fine tune the settings: to get the list of commands to use, enter the USB command ? (=help)
- 3 gains (one per axis roll/pitch/yaw). This allows to fine tune the gain per axis. Note: the sign of the gain define the direction of the gyro corrections.
- 1 parameter to select the stick range around center where corrections apply (full throw , 1/2 , 1/4)
- 1 parameter max rotate rate in hold mode (very low, low, medium , high)
- 1 parameter to enable (or not) max rotate rate in normal mode too.
- 1 parameter to select if oXs must apply Hold mode or Stabilize mode.
- PID parameters (Kp,Ki,Kd) per axis and per mode (Normal/Hold/stabilize). This allows a fine tuning of the gyro. The values depends on the model.

Note: as usual with oXs:
- the list of all usb commands and the allowed values can be displayed entering "?" command.
- the current setting is displayed just entering ENTER
- do not forget to enter SAVE command to keep you changes after a power off. After a SAVE command you must, most of the time, make a shutdown/reset to really activate the changes.


### Gyro learning process = mixer calibration = discovering the orientation/mixers and limits

The general principle is to let the mixers + servo centers/min/max being defined only on the handset.
Still oXs must take care of those mixers and limits when it applies gyro corrections.
To achieve this, oXs has to capture the positions of all Rc channels when sticks are in several specific positions. That is the reason why you had to add 3 channels on the handset (see above).

The process to let oXs discover the mixers/limits to apply is named the "gyro learning" process (also named "mixer calibration"). It is mandatory. This process is not the same as the calibration of the MP6050.

This process also let oXs find automatically the orientation of the MP6050 in the model. That is the reason why you don't need to use usb commands (MPUORI=..) to setup the orirentation of the MP6050 when gyro is used.

Important note: the learning process must be done again if the orientation of the MPU in the model change and/or if the mixers/servo directions change on the handset.

The learning process consist of several steps.

#### 1 Starting the process

To start the mixer calibration, the user has to put the model HORIZONTAL and, on the handset, simultaneously:
- put AIL and RUD sticks FULL to the right
- put ELV stick FULL in the direction that makes the model go up (so normally the stick full down)
- move the switch used to control the gyro mode more than 4 X within 5 sec.

When oXs detect this situation, It will try to find the MP6050 axis that measures gravity.
If oXs can't, it stops moving the servos and sending telemetry. The user has to make a power off. So it is clear that the process did not ended properly.

If "horizontal" orientation is found, oXs will set the led on RED and register the "horizontal" orientation and the 3 stick positions.
It then goes to the next step (discovering the mixers)

#### 2 First Phase : discovering the mixers.

oXs will then analyse the positions of sticks expecting to detect 7 cases:
- 1: AIL, RUD and ELV sticks simultaneously centered.
- 2/7: one of the 3 sticks (AIL,ELV,RUD) is full in a direction while the 2 others are centered. This should be done in all 6 possible cases (AIL in RIGHT corner, AIL in LEFT corner, RUD in RIGHT corner, RUD in LEFT corner, ELV in UP corner , ELV in DOWN corner).

Each time a case is detected, oXs will register the positions of all Rc channels.
This wil help to apply the gyro corrections with the right proportions on the rigth servos.

During this phase, to avoid side effect on the discovered mixers, it is VERY IMPORTANT that :
* Throttle does not change (so best use the safety switch to lock the transmitted RC channel to e.g. -100%)
* switches, sliders do not change.

The 7 cases can be done in any order and any number of times.

When all cases have been detected at least once, LED will become BLUE (saying that it allowed to switch to next step).

The user can still continue to move the sticks as previous as long as he want.

#### 3 Switching to phase 2:

When all cases have been detected (led is blue),next step must be activated.

To do so, SET THE MODEL VERTICAL with the NOSE UP and then change (1X is enough) the position of the gyro switch.

Note: switch changes during the first 5 sec of phase 1 are just discarded (so it does not matter if user changed more than 5 X the switch when it was starting the process).

oXs tries to detect the new orientation (vertical nose up) and checks that it is different from the "Horizontal".

In case of error (wrong orientation or at least one of the 7 cases not detected -LED still RED), oXs stops moving the servos and sending telemetry. The user has to make a power off. So it is clear that the process did not ended properly.

If gyro switch is valid, LED becomes GREEN (= second phase (discovering the limits) is running).

#### 4 Second phase: dicovering the limits

User can now move all sticks, sliders, switches(except gyro switch) simultaneously in all positions in order to let each servo reaches his min and max allowed positions.

oXs registers those limits. They will be used to limit the movements when oXs applies gyro corrections on top of the Rc channel received from the handset.
User can make this step as long he want (but must be at least 2sec).
During this phase, orientation of the model does not matter (it can be set e.g. horizontally in order to easily move the stick).

#### Ending the learning process

To close the mixer calibration process, user has to change once more the gyro switch.

oXs saves then all parameters in flash so the calibration does not have to be repeated (except if mixers/mechanical limits are modified on the handset).

Note: switch changes during the first 2 sec of phase 2 are just discarded (so it does not matter if user changed to often the switch when it was switching to phase 2).

### Notes:
At each power on, oXs uploads saved parameters and uses them.

Outside of the learning process process, end points of each servo (=min/max limits) are automatically updated based on the Rc channel values received from the receiver (so before gyro corrections).
This allows oXs to apply gyro corrections that exceed the limits registered during the cabration but without exceeding the limits defined in the handset.
The drawback of skipping step 2 of learning process is that some gyro corrections could be more restricted than really required in the first minutes after a power on.

### Checks.

when the learning process has been done, it is important to check that every thing is OK.
So without moving the model, first check that the servos move as expected.

## ---------------- Sequencers ---------------
With oXs, one single channel on the handset can control one or several SERVOS in sequences defined by the user (e.g. for landing gears with doors and wheels).

It can also generate one or several ANALOG signals in sequences (e.g. to blink leds or to start/stop motors).

One output is controlled by only one Rc channel.


You can use several Rc channels; each channel controls one or serveral outputs (SERVO and/or ANALOG).


oXs uses 3 concepts : sequencer, sequence and step.

* oXs uses ONE "sequencer" per GPIO to be controlled. There can be up to 16 sequencers (one per gpio 0...15).
* Each "sequencer" has several (min 2, max 21) "sequences". Each sequence is activated by a specific RC channel value (-100%,- 90%, ... 90%,100%) 
* Each "sequence" has one or several steps. Each step correspond to one action (move servo to position X, set led on power Y)
* oXs can repeat continously one sequence or just stay on the last step waiting for a new Rc channel value to start a new sequence.
* Each "sequencer" is defined by 7 parameters:
    * The GPIO on which a PWM signal is generated ( must be in range 0/15); the same GPIO may not be used for another purpose or by another sequencer
    * The type of PWM signal
        * 0 = SERVO =  PWM signal to control a servo (every xx msec a pulse in range 1ms/2ms is generated)
        * 1 = ANALOG = PWM signal to control a LED or analog voltage ( every xx msec, a pulse in range 0/xx ms is generated)
    * The duration of one "clock" in msec (must be greater than 20); this defines the base unit of the "smooth" and the "keep" delays used in steps definition (see below) 
    * The Rc channel that control this sequencer (must be in range 1/16); the same Rc channel may be used in several sequencers 
    * The default PWM value (to apply when no Rc channel has yet been received) (must be in range -100/100 for SERVO, 0/100 for ANALOG outputs)
    * The min PWM value (must be in same range as default PWM value); if a step requests a lower PWM value, the min PWM will be used
    * The max PWM value (must be in same range as default PWM value); if a step requests a greater PWM value, the max PWM will be used.
    * note: Min and Max defined at sequencer level can be usefull in order to define the end points of servo travel; they can avoid having to change many values at step level while using servos.
* Each "sequence" is defined by 5 parameters
    * The Rc channel value that activates this sequence. The value must be a multiple of 10 and in range -100...100 (so like -100, -90, -80... 0, 10, 20,...100);so there a 21 valid values. Note: the rc channel value sent by the handset can slightly differ from those values because oXs applies a tolerance of +/- 4%. So, if handset sent e.g. a value equal to -86, it will be handeld like -90.
    * an optional flag ("R" = Repeat) to say if the sequence must be automatically repeated after the last step; by default, sequence is not repeated
    * an optional flag ("U" = Uninterrupted) to say that the sequence may be interrupted before end of the last step; by default sequence may be interrupted
    * an optional flag ("O" = Only interrupted by priority sequence) to say that the sequence may be interrupted but only by a "priority" sequence
    * an optional flag ("P" = priority) to say that the sequence is a "priority" sequence (so it may interrupt a sequence with flag "O")  
* Each "step" is defined by 3 parameters
    * The number of clocks (=delay) for a smooth transition from current PWM value up to the PWM value from this step (must be in range 0/255)
    * The PWM value to apply in this step at the end of the transition (in range -125/125 for SERVO, 0/100 for ANALOG pwm); a value 127 is also possible and has a special function: it means that oXs must keep the current value. This can be useful only when a running sequence must be interrupted and the position must remain unchanged.
    * The number of clocks the PWM value must be kept before applying next step (if any) or going back to the first step of this sequence. Must be in range 0/255 
* For each sequencer, when the handset sent a different Rc value that matches the value of a sequence, oXs starts "playing" all steps of the related sequence. If this happens while oXs is already playing a sequence, oXs will continue or stop playing the current sequence depending on the flags U, O and P (see above). If the current step may not be interrupted, oXs, will delay the new requested sequence up to the end of the current sequence. When the current sequence reaches his end, oXs will or play the delayed sequence (if any), or repeat the current sequence (if flag = "R") or keep the last PWM output.
* For each sequencer, if the handset sent a Rc value that does not match a defined value (taking care of tolerance), the change of Rc value has no effect; oXs continues to "play" the current sequence.
* Sequencers, sequence and steps are defined sending a command via the USB port using a serial terminal. So there is no need to compile/flash to change some parameters.
* All sequencers are defined by only one command:  SEQ=[...] (...) {...} {...} (...) {...} etc...
    * each [...] contains the 7 parameters of one sequencer (space delimited)
    * each (...) contains the parameters (Rc value + 4 optional flags) of one sequence (space delimited)
    * each {...} contains the 3 parameters of of one step (space delimited)
    * each [...] must be followed by (...) to specify the first sequence of this sequencer 
    * each (...) must be followed by one or several {...} to specify the steps of this sequence
    * a sequencer must contain a least 2 sequences.
    * sequences of one sequencer must be in ascending order of RC channel values      
    * e.g. SEQ=[3 0 100 15 -100 -100 +90] (-100 R O) {0 50 3} {10 100 2} (100 P) {0 -100 10} 
               [4 1 500 16 0 0 100 ] (-100) {0 0 20} (-30) {0 100 1} (70 R) {0 100 20} {5 20 10}
    * this defines 2 sequencers:        
        * Sequencer nr 1 [3 0 100 15 -100 -100 +90] has PWM output on gpio 3, for a servo , 100ms/clock, controlled by channel 15, PWM = -100 (default), -100 (min) and 90(max); it contains 2 sequences:
            * First sequence (-100 R O) is activated when Rc channel changes to -100%, may be repeated automatically and may be interrupted only by a priority sequence; it contains 2 steps:
                * first step {0 50 3} says that PWM must be set immediately(smooth=0) on 50%, stay on 50% for 3 clocks and then switch to step 2 
                * second step {10 100 2} says that PWM must increase gradualy up to 100% over a timelaps of 10 clocks and then stay on 100% for 2 clocks before repeating the sequence
            * Second sequence (100 P) is activated when Rc channel change to 100% and is a priority sequence (so it may interrupt sequence 1)
                * step {0 -100 10} says that PWM must be set immediately (smooth =0) on -100% and stay on this value for at least 10 clock units
        * Sequencer nr 2 [4 1 500 16 0 0 100 ] has PWM output on gpio 4, for analog voltage, 500ms/clock, controlled by channel 16, PWM = 0 (default), 0 (min) and 100(max = Vcc); it contains 3 sequences
            * First sequence (-100) is activated when Rc channel change to -100% (does not repeat and may be interrupted) 
                * step {0 0 20} says that PWM must be set immediately (smooth=0) on 0 and stay on 0; note: as the sequence may not be repeated automatically but may be interrupted by any new sequence, the delay of 20 clock units has in practice no effect  
            * Second sequence (-30) is activated when Rc channel change to -30%  (does not repeat and may be interrupted); it contains 2 steps: 
                * step {0 100 1} says PWM must be set immediately (smooth=0) on 100% and stay on this value for 1 clock. note: as the sequence may not be repeated automatically but may be interrupted by any new sequence, the delay of 1 clock units has in practice no effect
            * Third sequence (70 R) is activated when Rc channel change to 70 and may be repeated automatically; it contains 2 steps;
                * step one {0 100 20} says that PWM must be set immediately(smooth=0) on 100 and stay on 100 for 20 clock and then next step is applied.
                * step two {0 20 10} says that PWM must be set smoothly (over 5 clock units) to 20 and stay so for 10 clock units before repeating the sequence.
* To delete all sequencers, enter SEQ=DEL
* Processing of SEQ commands
    * most controls on SEQ commands are performed before saving the parameters. In case of error, the command is just discarded and not saved
    * commands that fit those controls are saved in flash memory and oXs is rebooted. It can be that you have to make a manual reset (or power off+on)
    * After reset, oXs performs some more controls that could lead to an invalid config (e.g. if a gpio is used for several functionalities)

* Sequencers parameters are displayed as all other parameters when you press only ENTER. It is possible to make a complete copy/paste of the displayed SEQ to the input aera of the serial terminal in order to easily edit some parameters. 

## ------------------ Logging -------------------
If the LOG Gpio is defined, all telemetry data and all PWM Rc channel values (usec) captured by oXs are transmitted on the LOG pin in a compressed format.


Each time a set of data is ready, oXs generates a packet (with only the newly generated data). 


The packet is generated with an UART 8N1 (8 bits, no parity, 1 stop bit) at the defined logger baudrate.


Each packet starts with 0X7E (= synchro byte) followed by 4 bytes (number of milli sec since RP2040 start up) and by  
* For telemetry :  one or several data blocks; each one contains
    * one byte to identify the type of data (max 63 types e.g. Vspeed, Altitude, ...). The 2 most significant bits gives the number of "0" bytes that should be added to the data value in order to get a in32_t (code on 4 bytes)
    * 1, 2, 3 or 4 bytes with the value
* For Rc channels : 
    * one byte equal to "41" (= type for Rc channels)
    * 32 bytes = 16 X 2 bytes; each 2 bytes (=uint16_t) represent the PWM values in microsec (1500 usec = neutral) of one Rc channel.

There is some stuffing mecanism (like in Frsky Sport protocol) in order to ensure that the value 0X7E can only be present at the begining of a packet.
\
\
This format allows to compress the data transmitted via the (quite slow) UART to the logger.
\
\
The logger will remove the stuff bytes, uncompress the data, combine the new data with previous one to create an "actual" set of data's, convert it in CSV format and finally store it on a SD card. 

## ------------------ Model Locator ----------------------------------------
oXs can be used to locate a lost model (if you add a LORA module Ebyte E220-900M22S ).
 
The model is normally connected to the handset but when the model is on the ground, the range is quite limitted. 
So if a model is lost at more than a few hundreed meters, the handset will not get any telemetry data anymore. 
oXs allows to use a separate connection (with LORA modules) in order to have an extended range and have a chance to find back a lost model.
This is possible because those modules use a lower frequency, a lower transmitting speed and a special protocol for long range.
The LORA modules are E220-900M22S that are small, cheap and easily available (e.g. Aliexpress, ebay, amazon)
\
\
The principle is the following:
* You have to build 2 devices: 
    * an oXs device with the sensors you want (ideally a GPS and optionally e.g. vario, voltages, current, ...) and a E220-900M22S module
    * a "locator receiver" device with:
        * a Wemos D1 mini (or clone) board
        * a second E220-900M22S module
        * optionally a display 0.96 pouces OLED 128X64 I2C SSD1306. It is small and is available for about 2€.
        * optionally a push button to activate the Wifi 

* Normally:
    * the locator receiver is not in use.
    * oXs is installed in the model and transmits the sensor data's over the normal RC Rx/Tx link. The E220-900M22S module in oXs is in listening mode (it does not tranmit) 
* When a model is lost:
    * the locator receiver" is powered on. It starts sending requests to oXs.    
    * When the E220-900M22S module in oXs receives a request, it replies with a small message containing the GPS coordinates and some data over the quality of the request signal.
    * the display on the locator receiver shows those data's (longitude/latitude) as wel as the quality of the signal received and the time enlapsed since the last received message.
    * the same informations can be displayed on a GSM when the wifi is activated (button has been pushed). This requires connect the GSM to the Locator Wifi, to open a web browser (e.g. Chrome) and to enter an IP address. More details are given in the oXs locator receiver project.


Note: the range of communication between two Ebyte modules is normally several time bigger then the common RC 2.4G link.   
If oXs and locator receiver are both on the ground, it can be that there are to far away to communicate with each other.
But there are 2 ways to extend the range:
* use a directional antena on the locator receiver. The advantage of this solution is that, if you get a communication, you can use the system as a goniometer (looking at the quality of the signal) to know the direction of the lost model. This even works if you have no GPS connected to oXs. The drawback is that a directional antenna is not as small as a simple wire.
* put the locator receiver (which is still a small device) on another model and fly over expected lost aera. In this case, the range can be more than 5 km and the chance is very high that a communication can be achieved between the 2 modules. Even if the communication is broken when the model used for searching goes back on the ground, you will know the location of the lost model because the display will still display the last received GPS coordinates.




An oXs device with an Ebyte does not perturb the 2.4G link (it uses another frequency range - default 868mHz) and consumes only a few milliAmp because it remains normally in listening mode and when sending it is just a few % of the time. So, in order to increase the reliability of the system, it is possible to power oXs with a separate 1S lipo battery of e.g. 200/500 mAh. This should allow the system to work for several hours.


Cabling : The Ebyte module must be connected to the Rp2040 in the following way
* rp2040 SPI_CS    <=> NSS from module
* rp2040 SPI_MOSI  <=> MOSI from module
* rp2040 SPI_MISO  <=> MISO from module
* rp2040 SPI_SCK   <=> SCK from module
* rp2040 GRND      <=> GRND from module
* rp2040 SPI_BUSY  <=> Busy from module
* external (or rp2040 ) 3.3V   <=> 3.3V from module (!!! module does not support 5 Volt).
* Futhermore, the pins "TX" and "DI02" of the module must be connected toegether. 


To build the locator receiver, please check this link https://github.com/mstrens/oXs_locator_receiver_on_ESP8266 


Note: oXs (=transmitter) stay in sleep mode most of the time. Once every X (see config.h) sec, it starts listening to the receiver for e.g. 5 sec. If the receiver is not powered on, oXs never get a request and so never sent data.
When powered on, the receiver sent a request every 1 sec. At least X sec later (when entering listening mode), oXs should get this request and then reply immediately. oXs will then reply to each new request (so every 1 sec). oXs goes back to sleep mode for X sec if it does not get a request within 60 sec.

## ------------------ Led -------------------
When a RP2040-Zero or RP2040-TINY is used, the firmware will handle a RGB led (internally connected to gpio16).
* when config is wrong, led is red and always ON.
* when config is valid, led is blinking and the color depends on RC channels being received ot not
    * Red = Rc frames have nerver been received, Sbus and/or PWM signals are not generated.
    * Blue = Sbus and/or PWM signals are based on failsafe values. Failsafe values are given by the receiver for Sbus or are configured in oXs for CRSF protocol
    * Yellow/oranje = Sbus and/or PWM signals are based on a valid RC channels frame received from PRI or SEC source but the orther source does not provided a valid RC channels frame.
    * blinking green = Sbus and/or PWM signals are based on valid RC channels frames (from one source; from both sources if both are foressen in the setup)
* when "Boot" button is used for setting the failsafe values, led becomes blue and white (see above)

Note: some users got a RP2040-zero or RP2040-TINY where red and green colors are inverted.
If you got such a device and want to get the "normal" colors, you can enter a command LED=I to invert the 2 colors.


Please note that other boards do not have a RGB led on gpio16 and so this does not applies.
