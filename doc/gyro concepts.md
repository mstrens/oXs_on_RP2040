This document explain the way gyro corrections are done in oXs.

-------------- Principles ------------
- When oXs get the Rc channels and has a MPU6050 (accelerometer/gyro), oXs can automatically apply corrections to stabilize on 3 axis the PWM signals that drive servos (and later on also on Sbus out)
- Gyro has 3 modes (off/Normal/Hold); user selects the active mode with a channel on the Tx with a 3 positions switch
- The same channel allows to change the general gain of the gyro for each mode separately; 
- Corrections can be applied on as many servos as wanted (e.g. on 4 ail servos per wing )
- The mixers and servos limits are defined only in the handset (as usually when no gyro is used). oXs detect automatically the mixers and limits applied on the servos concerned by gyro correction during a special setup phase (= mixer calibration)
- all other parameters (see below) can be defined with Usb commands (no compilation/reflash required) except some advanced settings that require to edit config.h file 

-------------- parameters to set up the gyro ----------------------
Required:
- 1 channel to select the gyro mode/gain:  value can be betwwen -100% and 100%; negatieve values => Normal mode, 0% => OFF , positieve => Hold; gain varies with the Rc value (bom 0% to 100% or -100%) 
- 3 channels providing "original" stick positions (so before any mixer/limit apply by handset !!!)
- gpio's and channels used for servos (just like when no gyro is used) 

Optionnally to fine tune the settings
- 3 gains (one per axis roll/pitch/yaw) ; the sign of the gain define the direction of the the gyro corrections.
- 1 parameter to select the stick range around center where corrections apply (full throw , 1/2 , 1/4)
- 1 parameter max rotate rate in hold mode (very low, low, medium , high)
- 1 parameter to enable (or not) max rotate rate in normal mode too.
Note: as usual with oXs:
- the list of all usb commands and the allowed values can be displayed entering "?" command.
- the current setting is displayed just entering ENTER

PID parameters (Kp,Ki,Kd) per axis and for Normal/Hold modes can be changes but it requires to edit the config.h and to compile/flash

--------------- learning process = mixer calibration -----------------------
Before using the gyro or when mixers are changed on the handset, oXs has to capture the positions of all Rc channels when sticks are in several specific positions:

The general principle is to let the mixers+ servo centers/min/max being defined only on the handset.
Still oXs has to apply similar mixers on the gyro corrections.


To do so, oXs has to receive from the handset not only the Rc channel values for each servo (ail1, ail2, ...) but also the positions of the 3 sticks Ail, Elv, Rud before the handset applies any mixer/curve/limit on them.
This requires that those positions are transmitted in "unused" Rc channels on top of all other Rc channels controlling servos, ESC, sequencer, ...
The 3 Rc channels used to sent the 3 stick positions (Ail, Elv, Rud) are defined in the parameters of oXs.

To let oXs know the mixers to apply on gyro corrections and the limits to respect when gyro corrections are applied, a "learning" process also name "mixer calibration" is required. This process is not the same as the calibration of the mpu6050 (gyro sensor).

To start the mixer calibration, the user has to simultaneously to:
- put AIL and RUD sticks in right corner
- put ELV stick in up corner
- move the switch used to control the gyro mode more than 4 X within 5 sec.

When oXs detect this situation, it will set the led on RED (and register the 3 stick positions).
oXs will then analyse the positions of sticks expecting to detect 7 cases:
- 1: AIL, RUD and ELV sticks simultaneously centered.
- 2/7: one of the 3 sticks (AIL,ELV,RUD) is in one corner while the 2 others are centered. This should be done in all 6 possible cases (AIL in RIGHT corner, AIL in LEFT corner, RUD in RIGHT corner, RUD in LEFT corner, ELV in UP corner , ELV in DOWN corner).

Each time a case is detected, oXs will register the positions of all Rc channels.
This wil help to apply the gyro corrections with the right proportions on the rigth servos.

During this phase, to avoid side effect on the discovered mixers, it is very important that :
# Throttle does not change (so best use the safety switch to lock the transmitted RC channel to e.g. -100%)
# switches, sliders do not change.

The 7 cases can be done in any order and any number of times.

When all cases have been detected at least once, LED will become BLUE.

The user can continue to move the sticks as previous as long as he want.

At some time, user must change the position of the gyro switch to instruct oXs that the first step is done.
Note: switch changes during the first 5 sec are just discarded (so it does not matter if user changed more than 5 X the switch to start the process).


If at least one case has not yet been detected (LED is still RED) while the gyro switch (=end of first step) changes, oXs considers that config is not valid and stops moving the servos and sending telemetry. The user has to make a power off. So it is clear that the process did not ended properly.

When the gyro switch is activated when LED is BLUE, LED becomes GREEN (= start second step).
User can now move all sticks, sliders, switches(except gyro switch) simultaneously in all positions in order to let each servo reaches his min and max allowed positions.

oXs registers those limits. They will be used to limit the movements when oXs applies gyro corrections on top of the Rc channel received from the handset.
User can make this step as short or as long he want.

To close the mixer calibration process, user has to change once more the gyro switch.
oXs saves then all parameters in flash so the calibration does not have to be repeated (except if mixers/mechanical limits are modified on the handset).

At each power on, oXs uploads saved parameters and uses them.

Outside of the calibration process, end points of each servo (=min/max limits) are automatically updated based on the Rc channel values received from the receiver (so before gyro corrections).
This allows oXs to apply gyro corrections that exceed the limits registered during the cabration but without exceeding the limits defined in the handset.
The drawback of skipping step 2 of learning process is that some gyro corrections could be more restricted than really required in the first minutes after a power on.


