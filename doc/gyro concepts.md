This document explain the way gyro compensation is done.

---------------Purpose ------------
- use oXs as gyro : based on the MPU6050, oXs can apply automatically corrections on 3 axis to PWM signals that drive servos (and later on on Sbus out)
- gyro has 3 modes (off/Normal/hold); user select the active mode with a channel on the Tx (3 pos switch)
- the same channel allows to change the general gain.
- all other parameters (see below) can be defined in config.h and with Usb commands
- corrections can be applied on as many servos as wanted (e.g. on 4 ail servos per wing )
- It is not required to define the type of wing and elevator; in OFF mode, oXs just applies the incomming Rc values; when active oXs uses the mixer discovered during a setup phase (mixer calibration)

-------------- parameters in config ----------------------
- channel controlling the gyro (mode and general gain)
- 3 channels providing "original" stick positions (so before any mixer apply by handset !!!)
- gains per axis (roll/pitch/yaw)
- per axis, PID parameters (Kp,Ki,Kd) for Normal and for Hold mode 
- one parameter to select the stick range around center where corrections apply (full, 1/2 , 1/4) 

--------------- learning process = mixer calibration -----------------------
Before using the gyro or when mixers are changed on the handset, oXs has to capture the positions of all Rc channels when sticks are in several specific positions:

The general principle is to let the mixers+ servo centers/min/max being defined only on the handset.
Still oXs has to apply similar mixers on the gyro corrections.
To do so, oXs has to receive from the handset not only the Rc channel values for each servo (ail1, ail2, ...) but also the positions of the 3 sticks Ail, Elv, Rud before the handset applies any mixer/curve/limit on it.
This requires that those positions are transmitted in "unused" Rc channels on top of all other Rc channels controlling servos, ESC, sequencer, ...
The 3 Rc channels used to sent the 3 stick positions (Ail, Elv, Rud) are defined in the config of oXs.

To let oXs know the mixers to apply on gyro corrections and the limits to respect when gyro corrections are applied, a "learning" process also name "mixer calibration" is required. This process is not the same as the calibration of the mpu6050 (gyro sensor).

To start the learning process, the user has to simultaneously to:
- put AIL and RUD sticks in right corner
- put ELV stick in up corner
- move the switch used to control the gyro mode more than 4 X within 5 sec.

When oXs detect this situation, it will set the led on RED (and register the 3 stick positions).
oXs will then analyse the positions of sticks expecting to detect 7 cases:
- 1: AIL, RUD and ELV sticks simultaneously centered.
- 2/7: one of the 3 sticks (AIL,ELV,RUD) is in one corner while the 2 others are centered. This should be done in all 6 possible cases (AIL in RIGHT corner, AIL in LEFT corner, RUD in RIGHT corner, RUD in LEFT corner, ELV in UP corner , ELV in DOWN corner).

Each time a case is detected, oXs will register the positions of all Rc channels.
This wil help to apply the gyro corrections with the right proportions on the rigth servos.

During this learning phase, to avoid side effect on the discovered mixers, it is very important that :
# Throttle does not change (so best use the safety switch to lock the transmitted RC channel to e.g. -100%)
# switches, sliders do not change.

The 7 cases can be done in any order and any number of times.

When all cases have been detected at least once, LED will become BLUE.

The user can continue to move the sticks as previous as long as he want.

At some time, user must change the position of the gyro switch to instruct oXs that the first step is done.
Note: witch changes during the first 5 sec are just discarded (so it does not matter if user changed more than 5 X the switch to start the process).


If at least one case has not yet been detected (LED is still RED) while the gyro switch (=end of first step) changes, oXs considers that config is not valid and stops moving the servos and sending telemetry. The user has to make a power off. So it is clear that the process did not ended properly.

If the gyro switch is activated when LED is BLUE, LED becomes GREEN (= start second step).
User can now move all sticks, sliders, switches(except gyro switch) simultaneously in all positions in order to let each servo reaches his min and max allowed positions.

oXs registers those limits. They will be used to limit the movements when oXs applies gyro corrections on top of the Rc channel received from the handset.
User can make this step as short or as long he want.

To close the learning process, user has to change once more the gyro switch.
oXs saves then all parameters in flash so the learning process does not have to be repeated (except if mixers/mechanical limits are modified on the handset).

At each power on, oXs uploads saved parameters and uses them.

Outside of the learning process, end points of each servo (=min/max limits) are automatically updated based on the Rc channel values received from the receiver (but not stored in flash).
This allows oXs to apply gyro corrections that exceed the limits registered in the learning process but without exceeding the limits defined in the handset.
The drawback of skipping step 2 of learning process is that some gyro corrections could be more restricted than really required in the first minutes after a power on.
