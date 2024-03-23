This document explain the way gyro corrections are done in oXs.

-------------- Principles ------------
- When oXs get the Rc channels and has a MPU6050 (accelerometer/gyro), oXs can automatically apply corrections to stabilize on 3 axis the PWM signals that drive servos (and later on also on Sbus out)
- Gyro has 4 modes (off/Normal/Hold/stabilize); user selects the active mode (between 3) on the Tx with a 3 positions switch.
- This switch must allow the handset to generate on a Rc channel a signal that is or negative (Normal mode), or null (gyro off) or positive (gyro in hold or stabilize mode depending on a oXs parameter).
    - In "Normal" mode, the gyro tries to compensate for external perturbation (wind,...). The sticks allow to control the model.
    - In "OFF" mode, oXs just transmit the PWM signals provided by the handset without any gyro correction
    - In "Hold" mode, oXS tries to keep the model in the current orientation when sticks are centered. Moving the sticks allows change the orientation of the model.
    - In "Stabilize" mode, oXs tries to keep the model horizontal when the sticks are centered. The sticks allow to control the model. 
- Respectively the positive and negative value from this channel allows also to select the general gain of the gyro for each mode separately
 
- Gyro corrections can be applied on as many servos as needed (e.g. on 4 ail servos per wing, on 2 elevator servos and/or 2 rudder servos, on Vtail stab,... )
- On the opposite to many gyro, the mixers and servos limits are defined only in the handset (just like when no gyro is used). oXs detect automatically the mixers and limits applied on the servos concerned by gyro correction during a special setup phase (= mixer calibration)
- several parameters (see below) can be defined with Usb commands (no compilation/reflash required) to set up the oXs gyro. 

-------------- parameters to set up the gyro ----------------------
Required:
- 1 channel to select the gyro mode and the general gyro gain:  this is specified with the command GMG=xx (Gyro Mode Gain; xx = the rc channel between 1 and 16). This Rc channel will provide a value betwwen -100% and 100%; negatieve values => Normal mode, 0% => OFF , positieve => Hold or Stabilize; gain varies with the Rc value (from 0% to 100% or -100%) 
- 3 channels providing "original" stick positions (so without mixer including trim, expo/limit/subtrim...!!!!). Those channels are specified with the commands GSA (Gyro Stick Aileron), GSE(gyro Stick Elevator), GSR (gyro Stick Rudder)
- gpio's and channels used for servos (just like when no gyro is used) with commands like C2=4 (meaning channel 2 is generated on gpio 4)

Optionnally to fine tune the settings
- 3 gains (one per axis roll/pitch/yaw) ; the sign of the gain define the direction of the the gyro corrections.
- 1 parameter to select the stick range around center where corrections apply (full throw , 1/2 , 1/4)
- 1 parameter max rotate rate in hold mode (very low, low, medium , high)
- 1 parameter to enable (or not) max rotate rate in normal mode too.
- PID parameters (Kp,Ki,Kd) per axis and for Normal/Hold/stabilize modes can be changes.

Note: as usual with oXs:
- the list of all usb commands and the allowed values can be displayed entering "?" command.
- the current setting is displayed just entering ENTER
- do not forget to enter SAVE command to keep you changes after a power off. After a SAVE command you must most of the time make a shutdown/reset to really activate the changes.


--------------- learning process = gyro calibration = discovering the mixers -----------------------

The general principle is to let the mixers+ servo centers/min/max being defined only on the handset.
Still oXs has to apply similar mixers on the gyro corrections.

Before using the gyro or when mixers are changed on the handset, oXs has to capture the positions of all Rc channels when sticks are in several specific positions:

To do so, oXs has to receive from the handset not only the Rc channel values for each servo (ail1, ail2, ...) but also the positions of the 3 sticks Ail, Elv, Rud before the handset applies any mixer/curve/limit on them.
This requires that those positions are transmitted in "unused" Rc channels on top of all other Rc channels controlling servos, ESC, sequencer, ...
The 3 Rc channels used to sent the 3 stick positions (Ail, Elv, Rud) are defined in the parameters of oXs (with GSA, GSE,GSR commands).
On the handset, it is important that the 3 channels that gives the stick positions produce a signal of -100%/0%/+100% when stick are full one one side, centered and full on the other side.
So in the mixers that generate those 3 signals, you must use a set up that discards trim, expo, ...

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


