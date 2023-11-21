#include "pico/stdlib.h"
#include "gyro.h"
#include "param.h"
#include "tools.h"
#include "stdlib.h"
#include "mpu.h"
#include "sbus_in.h"
#include "sbus_out_pwm.h"
#include "crsf_in.h"
#include "hardware/watchdog.h"
#include "hardware/pio.h"  


// gyroX already take care of the gyro orientation and is the axis that point to the nose; so it is the roll axis;
//    positive values means left wing goes up, right wing goes down; Roll changes in the same way
// gyroY = axis to the left wing; it is the pitch axis
//    positive values means that nose goes down; pitch changes in the opposite way
// gyroZ = vertical axis = yaw axis
//    positive values means that plane turns to the left (conter clockwise); yaw changes in the opposite way
// on openTx, when Ail or Rudd stick goes to the right, pwm increases goes to 2000us (+512)
//            when Elv stick gives nose up, pwm goes to 1000us (-512)  
// imagine that when stick ail goes to right, one channel for one Ail servo increases (e.g. goes from 1500 to 1800 us)
//         that when stick Elv goes nose up, the channel  for Elv servo decreases     (e.g goes from 1500 to 1200 us)
//         that when stick Rud goes to right, the channel for Rud servo decreases     (e.g goes from 1500 to 1000 us)
// so after calibration the value in the mixer will be 
//          for the roll right = +300 (1800-1500)     and roll left = -400 (opposite side)
//                  pitch up    = -300 (1200-1500)        pitch down = +300   
//                  rud right = -500 (1000-1500)         Rud left = +500  

// In Rate mode: oXs uses directectly the values from the gyro to calculate the compensation.                                              
// imagine that due to wind, the left wing goes up (= plane goes to right). So gyroY becomes positive.
// so PID set point remain 0 and pid input is > 0; pid error > 0 (input-setpoint) ; pid output > 0 (error * k)
// so correction > 0. If stick is centered (OSP= 0) ESP will be positive (ESP=OSP+correction)
// then correction on positive side will be positieve (= correction) and correction on negative side will be 0
// finally, the PWM will be the original value + (correction positive side * rateRollright). As rateRollRight= 300, PWN will increase
// this means that this is equivalent to setting the stick to the rigth.
// this is wrong because the servo should move in the other direction.
// so PID correction has to be INVERTED to keep PID/master/stick gains positive
// this is done inverting pid input!!!!!!
//
// imagine that due to wind, the nose goes up. So gyroY becomes negative (gyro and pitch goes in opposite way).
// so PID set point remain 0 and pid input is <0; pid error < 0 (input-setpoint) ; pid output < 0 (error * k)
// so correction < 0. If stick is centered (OSP= 0) ESP will be negative (ESP=OSP+correction)
// then correction on positive side = 0 and correction on negative side will be negative (= correction)
// finally, the PWM will be the original value - (correction negative side * ratePitchDown). As ratepitchDown= +300, PWN will increase
// this means that this is equivalent to setting the stick nose down.
// this is OK.
// so PID correction on pitch has NOT to be inverted to keep PID/master/stick gains positive
//
// imagine that due to wind, the plane turn to right on yaw axis. So gyroZ becomes negative.
// so PID set point remain 0 and pid input is < 0; pid error < 0 (input-setpoint) ; pid output < 0 (error * k)
// so correction < 0. If stick is centered (OSP= 0) ESP will be negative (ESP=OSP+correction)
// then correction on positive side = 0 and correction on negative side < 0 (= correction)
// finally, the PWM will be the original value - (correction negative side * rateRudderRLeft). As rateRudleft= 500, PWN will increase
// this means that this is equivalent to setting the stick to the left.
// this is OK
// so PID correction has NOT to be inverted to keep PID/master/stick gains positive
//
// in stabilized mode : oXs uses roll and pitch that heve been calculated by the IMU (mahony algo)
// imagine that the left wing is up (= plane goes to right). So roll is positive.
// so PID set point remain 0 and pid input is > 0; pid error > 0 (input-setpoint) ; pid output > 0 (error * k)
// so correction > 0. If stick is centered (OSP= 0) ESP will be positive (ESP=OSP+correction)
// then correction on positive side will be positieve (= correction) and correction on negative side will be 0
// finally, the PWM will be the original value + (correction positive side * rateRollright). As rateRollRight= 300, PWN will increase
// this means that this is equivalent to setting the stick to the rigth.
// this is wrong because the servo should move in the other direction.
// so PID correction has to be INVERTED to keep PID/master/stick gains positive
// this is done inverting pid input!!!!!!
//
// imagine that the nose is down. So pitch is negative .
// so PID set point remain 0 and pid input is <0; pid error < 0 (input-setpoint) ; pid output < 0 (error * k)
// so correction < 0. If stick is centered (OSP= 0) ESP will be negative (ESP=OSP+correction)
// then correction on positive side = 0 and correction on negative side will be negative (= correction)
// finally, the PWM will be the original value - (correction negative side * ratePitchDown). As ratepitchDown= +300, PWN us will increase
// this means that this is equivalent to setting the stick nose down.
// this is NOT OK.
// so PID correction on pitch has to be inverted to keep PID/master/stick gains positive
// this is done inverting pid input!!!!!!
//




//enum LEARNING_STATE {LEARNING_OFF, LEARNING_INIT,LEARNING_WAIT, LEARNING_MIXERS, LEARNING_LIMITS, LEARNING_ERROR, LEARNING_OK};
enum LEARNING_STATE learningState = LEARNING_OFF;

    #define MARGIN 30
    #define CENTER_LOW 1500-MARGIN
    #define CENTER_HIGH 1500+MARGIN
    #define END_LOW 1010  // Normal 988
    #define END_HIGH 1990  // normal 2012


extern CONFIG config;
extern MPU mpu;
extern uint16_t rcChannelsUs[16];  // Rc channels values provided by the receiver in Us units 
uint16_t rcChannelsUsCorr[16];  // Rc channels values with gyro corrections applied (in Us)

extern int16_t gyroX; // data provided by mpu, sent to core0 and with some filtering
extern int16_t gyroY;
extern int16_t gyroZ;

extern bool configIsValid;
extern uint8_t ledState;

extern int32_t cameraPitch;
extern int32_t cameraRoll;


//bool autolevel = false;

struct gyroMixer_t gyroMixer ; // contains the parameters provided by the learning process for each of the 16 Rc channel

bool gyroIsInstalled = false;  // becomes true when config.gyroChanControl is defined (not 255) and MPU6050 installed


extern const char* mpuOrientationNames[8] ;

/***************************************************************************************************************
 * PID
 ***************************************************************************************************************/

#define PID_PERIOD 10000
// relative exponential weights kp:ki:kd
#define PID_KP_SHIFT 3 
#define PID_KI_SHIFT 6
#define PID_KD_SHIFT 2

struct _pid_state {
  // setpoint, input [-8192, 8191]
  int16_t setpoint[3];
  int16_t input[3];
  int16_t last_err[3];
  int32_t sum_err[3];
  int32_t i_limit[3];
  int16_t output[3];
};

struct _pid_state pid_state; // pid engine state

int32_t constrain(int32_t x , int32_t min, int32_t max ){
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

int16_t min(const int16_t a, const int16_t b)
{
    return (b < a) ? b : a;
}

void compute_pid(struct _pid_state *ppid_state, struct _pid_param *ppid_param) 
{
  for (int8_t i=0; i<3; i++) {
    int32_t err, sum_err, diff_err, pterm, iterm, dterm;
    err = ppid_state->input[i] - ppid_state->setpoint[i];
    // accumulate the error up to an i_limit threshold
    sum_err = ppid_state->sum_err[i] = constrain(ppid_state->sum_err[i] + err, -ppid_state->i_limit[i], ppid_state->i_limit[i]);
    diff_err = err - ppid_state->last_err[i]; // difference the error
    ppid_state->last_err[i] = err;    
        
    pterm = (ppid_param->kp[i] * err) >> PID_KP_SHIFT;
    iterm = (ppid_param->ki[i] * sum_err) >> PID_KI_SHIFT;
    dterm = (ppid_param->kd[i] * diff_err) >> PID_KD_SHIFT;
    ppid_state->output[i] = (pterm + iterm + dterm) >> ppid_param->output_shift;

#if defined(SERIAL_DEBUG) && 0
    if (i == 2) {
      Serial.print(ppid_state->input[i]); Serial.print('\t');
      Serial.print(err); Serial.print('\t');
      Serial.print(diff_err); Serial.print('\t');
      Serial.print(pterm); Serial.print('\t');
      Serial.print(iterm); Serial.print('\t');
      Serial.print(ppid_state->sum_err[i]); Serial.print('\t');
      Serial.print(dterm); Serial.print('\t');
      Serial.println(ppid_state->output[i]);
    }
#endif
  }
}


// ----------- compensation from gyro ----------------------
uint32_t last_pid_time = 0;

  
enum STAB_MODE stabMode = STAB_RATE;
// rx
#define RX_GAIN_HYSTERESIS 25
#define RX_MODE_HYSTERESIS 25
#define RX_WIDTH_MIN 900
#define RX_WIDTH_LOW_FULL 1000
#define RX_WIDTH_LOW_NORM 1100
#define RX_WIDTH_LOW_TRACK 1250
#define RX_WIDTH_MID 1500
#define RX_WIDTH_MODE_MID  1500 //it was on 1550	, changed by mstrens to 1500// Move all hysteresis to Hold Mode side so 1500-1520 will always force Rate Mode
#define RX_WIDTH_HIGH_TRACK 1750
#define RX_WIDTH_HIGH_NORM 1900
#define RX_WIDTH_HIGH_FULL 2000
#define RX_WIDTH_MAX 2100

int16_t stickAilUs_offset, stickElvUs_offset, stickRudUs_offset; 
const int16_t stick_gain_max = 400; // [1100-1500] or [1900-1500] => [0-STICK_GAIN_MAX]
const int16_t master_gain_max = 400; // [1500-1100] or [1500-1900] => [0-MASTER_GAIN_MAX]

int16_t correction[3] = {0, 0, 0};
int16_t correctionPosSide[3] = {0, 0, 0};
int16_t correctionNegSide[3] = {0, 0, 0};

uint32_t lastStabModeUs = 0;
int8_t stabModeCount;

// -------------------------------   calculate corrections ---------------
void calculateCorrectionsToApply(){ 
                                // it is called from applyGyroCorrections only if gyroIsInstalled applying PWM on the outputs
                              // when gyro is not yet calibrated, corrections are not calculated but we still detect stabMode changes
                              //       this is required to enter the learning calibration process and the reset of center pos
                              // it calculates the corrections and split them in positive and negatieve part (set to 0 if not calibrated)
                              // at this step, we do not take care of the mixers/ratio/.. defined on the handset and being part of mixer calibration 

    uint32_t t = microsRp(); 
    if ((int32_t)(t - last_pid_time) < PID_PERIOD) return;

    // just to make code easier to write
    int16_t stickAilUs, stickElvUs, stickRudUs, controlUs ;    
    int16_t stick_gain[3];
    int16_t master_gain;
    uint8_t i;

    // ------ get stick positions and switch
    stickAilUs = rcChannelsUs[config.gyroChan[0]-1]; // get original position of the 3 sticks
    stickElvUs = rcChannelsUs[config.gyroChan[1]-1];
    stickRudUs = rcChannelsUs[config.gyroChan[2]-1];
    controlUs = rcChannelsUs[config.gyroChanControl-1]; // get original position of the control channel
    // -----  determine how much sticks are off center (from neutral) taking care of gyroMixer calibration 
    stickAilUs_offset = stickAilUs - gyroMixer.neutralUs[config.gyroChan[0]-1]; 
    stickElvUs_offset = stickElvUs - gyroMixer.neutralUs[config.gyroChan[1]-1];
    stickRudUs_offset = stickRudUs - gyroMixer.neutralUs[config.gyroChan[2]-1];
    

    // stabilization mode
    //STAB_RATE when 988us = switch haut
    //STAB_HOLD when 2012us = switch bas
    enum STAB_MODE stabMode2 = 
      (stabMode == STAB_HOLD && controlUs <= RX_WIDTH_MODE_MID - RX_MODE_HYSTERESIS) ? STAB_RATE : 
      (stabMode == STAB_RATE && controlUs >= RX_WIDTH_MODE_MID + RX_MODE_HYSTERESIS) ? STAB_HOLD : stabMode; // hysteresis, all now in Hold Mode region

    //----------detect switch changes from up to down and the opposite (not the center position)
    if (stabMode2 != stabMode) {
        stabMode = stabMode2;
        //printf("stabMode changed in updateGyroCorrections() to %i\n", stabMode);
        // reset attitude error when and i_limit threshold when mode change from Hold or rate or vice versa (also reset when gain is 0)
        pid_state.sum_err[0] = 0; pid_state.sum_err[1] = 0; pid_state.sum_err[2] = 0;
        pid_state.i_limit[0] = 0; pid_state.i_limit[1] = 0; pid_state.i_limit[2] = 0;
                
        // check for inflight rx calibration; when swith mode change 3X (ex RATE/HOLD/RATE) with no more that 0.5 sec between each change    
        //if (cfg.inflight_calibrate == INFLIGHT_CALIBRATE_ENABLE) {
        if ((int32_t)(t - lastStabModeUs) > 500000L) {  // so 2X per sec
            stabModeCount = 0;
        }
        lastStabModeUs = t;
        if (++stabModeCount >= 3) {
            if ( (stickAilUs > 1400) and (stickAilUs < 1600) and (stickElvUs > 1400) and (stickElvUs < 1600) and (stickRudUs > 1400) and (stickRudUs < 1600) ){ 
                gyroMixer.neutralUs[config.gyroChan[0]-1] = stickAilUs; // save the new neutral positions for the 3 original sticks
                gyroMixer.neutralUs[config.gyroChan[1]-1] = stickElvUs;
                gyroMixer.neutralUs[config.gyroChan[2]-1] = stickRudUs;
                printf("New center position detected for raw sticks:   Ail=%-4i%%    Elv=%-4i%%    Rud=%-4i%%\n",\
                pc(stickAilUs-1500) , pc(stickElvUs-1500) , pc(stickRudUs-1500) );
            }    
        }
        
    } // end of stabmode change
    
    // ------------   calculate stick priority (based on stick offset and stick_gain_throw)
    // see enum STICK_GAIN_THROW. shift=0 => FULL, shift=1 => HALF, shift=2 => QUARTER
    int8_t shift = config.stick_gain_throw - 1;
        // stick_gain[] [1100, <ail*|ele|rud>_mid, 1900] => [0%, 100%, 0%] = [0, STICK_GAIN_MAX, 0]
    stick_gain[0] = stick_gain_max - min(abs(stickAilUs_offset) << shift, stick_gain_max);
    stick_gain[1] = stick_gain_max - min(abs(stickElvUs_offset) << shift, stick_gain_max);
    stick_gain[2] = stick_gain_max - min(abs(stickRudUs_offset) << shift, stick_gain_max);    
    // adapt pid I limit (in hold mode)
    if (stabMode == STAB_HOLD) {
        // max attitude error (bounding box)    
        for (i=0; i<3; i++) {
            // 2000 deg/s == 32768 units, so 1 deg/(PID_PERIOD=10ms) == 32768/20 units
            #define STAB_HOLD_RATIO (30 * 32768 / 2 / PID_PERIOD / 1000)
            pid_state.i_limit[i] = ( (int32_t) stick_gain[i] * STAB_HOLD_RATIO ) >> 9;  // >>9 is div by 512
        }
    }
    
    // -------  calculate master gain.
    // master gain [Rate = 1475-1075] or [Hold = 1575-1975] => [0, MASTER_GAIN_MAX] 
    if (controlUs < (RX_WIDTH_MID - RX_GAIN_HYSTERESIS))  {
        // Handle Rate Mode Gain Offset, gain = 1 when hysteresis area is exited
        // Previously gain was the value of RX_GAIN_HYSTERESIS at first exit
        master_gain = constrain(((RX_WIDTH_MID - RX_GAIN_HYSTERESIS) - controlUs) , 0, master_gain_max);
    } else {
        if (controlUs > (RX_WIDTH_MODE_MID + RX_MODE_HYSTERESIS))  {
        // Handle Hold Mode Gain Offset, gain = 1 when both hysteresis areas are exited
        // Previously gain was the value of RX_MODE_HYSTERESIS at first exit
        master_gain = constrain(controlUs - (RX_WIDTH_MID + RX_MODE_HYSTERESIS), 0, master_gain_max);		
        } else  {   // Force Gain to 0 while in either of the Hysteresis areas
            master_gain = 0; // Force deadband
            // reset attitude error when and i_limit threshold when gain is 0 (dead band)
            pid_state.sum_err[0] = 0; pid_state.sum_err[1] = 0; pid_state.sum_err[2] = 0;
            pid_state.i_limit[0] = 0; pid_state.i_limit[1] = 0; pid_state.i_limit[2] = 0;
        }    
    }	  	    
    
    // ---------- set setpoint of PID depending on the mode
    if (config.gyroAutolevel and stabMode == STAB_HOLD){  // reuse Hold position for autolevel
            pid_state.setpoint[0] = 0; pid_state.setpoint[1] = 0; pid_state.setpoint[2] = 0; // set target = 0 
    } 
    // commanded angular rate (could be from [ail|ele|rud], note direction/sign)
    else if (stabMode == STAB_HOLD || 
        (stabMode == STAB_RATE && config.rate_mode_stick_rotate == RATE_MODE_STICK_ROTATE_ENABLE)) {
        // stick controlled roll rate
        // cfg.max_rotate shift = [1, 4]
        // eg. max stick == 400, cfg.max_rotate == 4. then 400 << 4 = 6400 => 6400/32768*2000 = 391deg/s (32768 == 2000deg/s)
        int16_t sp[3];
        sp[0] = stickAilUs_offset << config.max_rotate;
        sp[1] = stickElvUs_offset << config.max_rotate;
        sp[2] = stickRudUs_offset << config.max_rotate;
        for (i=0; i<3; i++)
            //pid_state.setpoint[i] = vr_gain[i] < 0 ? sp[i] : -sp[i];
            pid_state.setpoint[i] = config.vr_gain[i] < 0 ? sp[i] : -sp[i];    
    } else { // Stab mode = RATE
        pid_state.setpoint[0] = 0; pid_state.setpoint[1] = 0; pid_state.setpoint[2] = 0;
    }
            
    //-----------set input ---------------------------------
    // gyroX and roll change in the same way
    // gyroY and pitch change in opposite way
    // gyroZ and yaw change in opposite way
    if (config.gyroAutolevel and stabMode == STAB_HOLD){  // reuse Hold position for autolevel
        if ( (abs(cameraRoll)>600) or (abs(cameraPitch)>600) ) {  // roll and pitch are not reliable when values are hight; so discard gyro
            pid_state.input[0] = 0;     // see text on top of this file to justify the - sign 
            pid_state.input[1] = 0;
            pid_state.input[2] = 0;
            pid_state.sum_err[0] = 0; pid_state.sum_err[1] = 0; pid_state.sum_err[2] = 0;    
        } else {
            // camera roll is in 0.1 deg so varies -900/900 (not totally true because once can be 180°)
            // gyroZ varies from -32768/32768 (int16) 
            // See Xls sheet to justify the << 2 (=*2)
            pid_state.input[0] = ((int32_t) -cameraRoll) << 1;     // see text on top of this file to justify the - sign 
            pid_state.input[1] = ((int32_t) -cameraPitch) << 1;
            pid_state.input[2] = gyroZ;
        }            
    } else {
    // measured angular rate (from the gyro and apply calibration offset but no scaling)
        pid_state.input[0] = - gyroX;  // gyroX,Y,Z max value is +/-32768 = +/-2000°/sec // see top of the file to explain the "-" for gyroX/roll axis 
        pid_state.input[1] = gyroY; // flightstab used a constrain -8192/8191 but I do not seee the reason
        pid_state.input[2] = gyroZ;        
    }
    // ----------  apply PID control depending on the mode
    compute_pid(&pid_state, (stabMode == STAB_RATE) ? &config.pid_param_rate : (config.gyroAutolevel) ?  &config.pid_param_stab : &config.pid_param_hold);
    
    //  Only to debug
    //#define DEBUG_PID_CORRECTIONS
    #ifdef  DEBUG_PID_CORRECTIONS
    static int16_t maxOutput[3] = { 0,0,0};
    static int16_t setpoint[3];
    static int16_t input[3];
    for (i=0; i<3; i++) {
            if ( (abs(pid_state.output[i])) > maxOutput[i] ) {
                maxOutput[i] = pid_state.output[i];
                setpoint[i] = pid_state.setpoint[i];
                input[i]    = pid_state.input[i];
            }
        }
    if(msgEverySec(1)){
         printf("Ail in=%i  set=%i   out=%i\n", input[0] , setpoint[0] , maxOutput[0]);
         printf("Elv in=%i  set=%i   out=%i\n", input[1] , setpoint[1] , maxOutput[1]);   
         printf("Rud in=%i  set=%i   out=%i\n\n", input[2] , setpoint[2] , maxOutput[2]); 
         maxOutput[0] = 0 ; maxOutput[1] = 0 ; maxOutput[2] = 0;    // reset max
    }
    #endif


    // ------- apply vr_gain, stick_gain and master_gain and split correction in positive and negative parts
    for (i=0; i<3; i++) {
        // vr_gain [-128,0,127]/128, stick_gain [0,400,0]/512, master_gain [400,0,400]/512
        //correction[i] = ((((int32_t)pid_state.output[i] * vr_gain[i] >> 7) * stick_gain[i]) >> 9) * master_gain >> 9;
        if (gyroMixer.isCalibrated == false) {  // set corrections to 0 when gyroMixer is not calibrated
            correctionNegSide[i] = correctionPosSide[i] = 0;
        } else {                                // when gyroMixer is calibrated, calculate correction
            correction[i] = ((((int32_t)pid_state.output[i] * config.vr_gain[i] >> 7) * stick_gain[i]) >> 9) * master_gain >> 9;
        }
        // split correction in positieve and negative parts (because limits can be different)
        uint16_t OSP = rcChannelsUs[config.gyroChan[i]-1];  // orginal stick position
        uint16_t ESP = OSP + correction[i];                // expected stick position
        if (OSP >= 1500 ){
            if (ESP >= 1500){
            correctionPosSide[i] = correction[i];
    
            } else {
                correctionPosSide[i] = (1500 - OSP) ; // corr is neg, so Positive part must be negative     
            }
        } else {
            if (ESP >= 1500){
            correctionPosSide[i] = ESP -1500;  // corr is positive, part 1 and 2 must be pos too   
            } else {
                correctionPosSide[i] = 0 ;   
            }
        }
        correctionNegSide[i] = correction[i] - correctionPosSide[i] ;       
    }
    
    // -------------- just to debug ---------------------
    //#define DEBUG_GYRO_CORRECTIONS
    #ifdef DEBUG_GYRO_CORRECTIONS
    if (msgEverySec(2) and gyroMixer.isCalibrated) { printf("Mode=%-5i  gx=%-5i  gy=%-5i  gz=%-5i  cAilT=%-5i  cAilP=%-5i  cAilN=%-5i  "  ,\
                 stabMode , pid_state.input[0] , pid_state.input[1],pid_state.input[2]\
                            ,correction[0] , correctionPosSide[0] , correctionNegSide[0]\
    );}
    #endif
    last_pid_time = t;
    // end of calculateCorrectionsToApply()
}

void applyGyroCorrections(){
    //This should be called only when new Rc values have been received (called by updatePwm)
    // This function is called only when gyro is used (checked before calling this function) and calibrated
    // corrections are calculated and added in rcChannelsUsCorr[]

    // register min and max Values in order to use them as servo limits.    
    for (uint8_t i=0; i<16;i++){
        if ( rcChannelsUs[i] > gyroMixer.maxUs[i]) {   // automatically update min and max (to adapt the limits from power on - values are not saved in flash)
            gyroMixer.maxUs[i] = rcChannelsUs[i] ;
        } else if (rcChannelsUs[i] < gyroMixer.minUs[i]) {
            gyroMixer.minUs[i] = rcChannelsUs[i] ;
        }    
    }
    calculateCorrectionsToApply(); // recalculate gyro corrections at regular interval but without taking care of the mixers;
                                   // corrections are set to 0 when gyromixer is not calibrated
    // from here, we know the corrections to apply on the pos and neg sides
    // apply corrections
    for (uint8_t i=0 ; i<16 ; i++){  // for each Rc channel
        if ( gyroMixer.used[i] ) { // when this channel uses gyro correction.
            // adapt the corrections Pos and Neg for each axis based on the ranges stored in gyroMixer.
            // to do : check that those are the correct matching between Pos/neg and Left/right
            //if ((msgEverySec(3)) and (i == 1)) {
            //    printf(" cp=%-5i * rr=%-5i  cn=%-5i * rl=%-5i  ", correctionPosSide[0] , gyroMixer.rateRollRightUs[i],\
            //       correctionNegSide[0] , gyroMixer.rateRollLeftUs[i]);
            //}
            //imagine that after learning process : RollRigth is negative (mean PWM goes to min), so then RollLeft is positive (always the opposite)
            // imagine that a correction to the right is required and that gives a positive corr applied from neutral.
            //      so  Pos part of corr is positieve and neg part = 0
            // when applied on the servo pwm must changed by pos part (+)* rollright (-) => result is negative (and roll goes to right => OK)
            //
            // imagine a correction to the left is required and so a negative corr is applied from neutral.
            // Pos part is = 0 and neg part is negatieve
            // when applied on the servo pwm must changed by pos part (+)* - rollleft (-) => result is positieve (and roll goes to the Left => OK)
            //
            // imagine that a correction to the right is required and that gives a positive corr but applied from stick at already at X% to the left.
            //      so before correction pwm is e.g. at 1600us.
            //      so  Pos part of corr is positieve and neg part = 0
            // when applied on the servo pwm must changed by pos part (+)* rollright (-) => result is negative (and roll goes to right => OK)
            //
            // imagine a correction to the left is required and that gives a negatieve corr but applied from stick at already at X% to the left.
            //     so before correction pwm is e.g. at 1600us.
            //     so Pos part is = negatieve and neg part is negatieve
            // when applied on the servo pwm must changed 
            //             by pos part (-) *  rollright (-) => result is positieve (and roll goes to the Left => OK)
            //             by neg part (-) * - rollleft (+) => result is positieve (and roll goes to the Left => OK)
            // so as rateRollRightUs and rateRollLeftUs have opposite signs, whe have to reverse the sign when applying the correction.
            rcChannelsUsCorr[i] += ((int32_t) correctionPosSide[0] * (int32_t) gyroMixer.rateRollRightUs[i])  >> 9 ; // division by 512 because full range is about 500.
            rcChannelsUsCorr[i] -= ((int32_t) correctionNegSide[0] * (int32_t) gyroMixer.rateRollLeftUs[i])  >> 9 ; // division by 512 because full range is about 500.
            rcChannelsUsCorr[i] += ((int32_t) correctionPosSide[1] * (int32_t) gyroMixer.ratePitchUpUs[i])  >> 9 ; // division by 512 because full range is about 500.
            rcChannelsUsCorr[i] -= ((int32_t) correctionNegSide[1] * (int32_t) gyroMixer.ratePitchDownUs[i])  >> 9 ; // division by 512 because full range is about 500.
            rcChannelsUsCorr[i] += ((int32_t) correctionPosSide[2] * (int32_t) gyroMixer.rateYawRightUs[i])  >> 9 ; // division by 512 because full range is about 500.
            rcChannelsUsCorr[i] -= ((int32_t) correctionNegSide[2] * (int32_t) gyroMixer.rateYawLeftUs[i])  >> 9 ; // division by 512 because full range is about 500.
            // stay within the limits 
            if (rcChannelsUsCorr[i] < gyroMixer.minUs[i]) rcChannelsUsCorr[i] = gyroMixer.minUs[i] ;
            if (rcChannelsUsCorr[i] > gyroMixer.maxUs[i]) rcChannelsUsCorr[i] = gyroMixer.maxUs[i] ;    
        }
    }
    // here all PWM have been recalculated and can be used for PWM output
}

    

bool checkForLearning(){ // return true when learning process can start
    // We consider that there are 5 changes within 5sec when the sum of 5 intervals between 2 changes is less than 5 sec.
    // we store the last 5 intervals
    // each time an change occurs, we loose the oldiest   
    static enum STAB_MODE prevStabMode = STAB_RATE;
    static uint32_t lastChangeMs = 0 ;
    static uint32_t intervals[5] = {10000 , 10000 ,10000 ,10000 ,10000}; 
    static uint8_t idx = 0;
    static uint32_t sumIntervals = 50000;
    
    
    if (stabMode == prevStabMode)  {
        return false;
    }
    //printf("stabMode changed to %i\n", stabMode);
    prevStabMode = stabMode;
    if ((rcChannelsUs[config.gyroChan[0]-1] <= END_LOW) or (rcChannelsUs[config.gyroChan[0]-1] >= END_HIGH) &&\
        (rcChannelsUs[config.gyroChan[1]-1] <= END_LOW) or (rcChannelsUs[config.gyroChan[1]-1] >= END_HIGH) &&\
        (rcChannelsUs[config.gyroChan[2]-1] <= END_LOW) or (rcChannelsUs[config.gyroChan[2]-1] >= END_HIGH)){ 
        uint32_t t = millisRp();
        uint32_t interval = t - lastChangeMs;
        lastChangeMs = t;
        sumIntervals -= intervals[idx];
        sumIntervals += interval;
        intervals[idx] = interval;
        idx++;
        if (idx >= 5) idx =0; 
        if (sumIntervals < 5000) {
            return true; 
        }
    }
    return false; 
}

void calibrateGyroMixers(){
    static bool centerFlag;
    static uint16_t centerCh[16];  // rc channels when 3 sticks are centered
    //static bool dirFlag ;
    static uint16_t dirCh[3];  // stick positions when all 3 are in the corner = AIL and RUD Rigth and ELV UP
    static bool rightUpFlags[3] ;   // register if we already know the 16 Rc channels for only one of the 3 sticks in low corner 
    static uint16_t rightUpUs[3][16];  // register the 16 Rc channels values 
    static bool leftDownFlags[3] ;    // idem for 3 sticks in opposite corner
    static uint16_t leftDownUs[3][16];
    static enum STAB_MODE prevStabMode ;  // used to detect when gyro switch change
    static uint32_t startMs ;             // used to check that we stay in step 1 for at least 5 sec  
    static uint16_t minLimitsUs[16]; // min limits of the 16 servos during the learning process
    static uint16_t maxLimitsUs[16]; // idem for max limits
    static uint16_t prevStickPosUs[3]; // use to control that the stick are in a stable position.

    if (learningState == LEARNING_OFF){
        if (checkForLearning() == false){  // function return true when conditions to start the learning process are OK  
            return;
        }
        // learning process may start
        // save the current position of sticks
        //dirFlag = true;
        //dirFlag = false;
        dirCh[0] = rcChannelsUs[config.gyroChan[0]-1]; // save the stick values for Ail Right  
        dirCh[1] = rcChannelsUs[config.gyroChan[1]-1]; // save the values for ELV Up
        dirCh[2] = rcChannelsUs[config.gyroChan[2]-1]; // save the values for Rud Right  
        centerFlag = rightUpFlags[0] = rightUpFlags[1] = rightUpFlags[2] = leftDownFlags[0] = leftDownFlags[1] = leftDownFlags[2] = false;
        for (uint8_t i=0; i<16; i++) {  // reset the min and max RC channel limits
            minLimitsUs[i] = 2012; // lower Rc channel values will be discoverd during the learning process
            maxLimitsUs[i] = 988;  // idem for upper values
        }
        //centerCh[16]; dirCh[16]; rightUpUs[3][16]; leftDownUs[3][16]; will be filled with the flags
        startMs = millisRp();         // used to check that there is 5 msec before switching to second step
        ledState = STATE_GYRO_CAL_MIXER_NOT_DONE;
        printf("\nThe learning process started\n");
        learningState = LEARNING_MIXERS;   
    }
    // once learning process has started, we save always the min and max servo positions
    for (uint8_t i=0; i<16;i++){
        if ( rcChannelsUs[i] > maxLimitsUs[i]) {
            maxLimitsUs[i] = rcChannelsUs[i];
        } else if (rcChannelsUs[i] < minLimitsUs[i]) {
            minLimitsUs[i] = rcChannelsUs[i] ; 
        }    
    }
    // when process starts, change led color to red (state= LEARNING_MIXERS)  
    // if state = LEARNING_MIXERS
    // if all 3 sticks are centered (+/-2), set flag and save 16 channels
    // if one axis is > 98% and the 2 others = 0 (+/-2) and if axis is more than previous, save a flag, save the 16 channels
    // if one axis is < 98% and the 2 others = 0 (+/-2) and if axis is less than previous, save a flag, save the 16 channels
    // when all 7 cases have been discoverd, state => LEARNING_MIXERS_DONE and change LED color to blue. 
    // if stabMode change (other than OFF) after 5 sec
    //    if state is not LEARNING _MIXERS_DONE, this is an error => set config in error (this will block everything)
    //    if ok (all flags set), set state to LEARNING_LIMITS
    // Wait for a new change before saving the parameters and change state to LEARNING_OFF
    // LED color is managed is main loop based on the state. 

    // this part just help to have clearer and shorter code here after
    int16_t stickPosUs[3];  // 3 original stick positions
    bool inCenter[3] = {false, false,false};     // true when stick is centered (with some tolerance)
    bool inCorner[3] = {false, false,false};        // true when stick is in a corner (with some tolerance)
    uint8_t i;
    bool sticksMaintained = true;
    stickPosUs[0] = (int16_t) rcChannelsUs[config.gyroChan[0]-1]; 
    stickPosUs[1] = (int16_t) rcChannelsUs[config.gyroChan[1]-1]; 
    stickPosUs[2] = (int16_t) rcChannelsUs[config.gyroChan[2]-1];
    for (i=0;i<3;i++){                // detect when sticks are centered or in a corner
        if ((stickPosUs[i] >= CENTER_LOW) and (stickPosUs[i] <= CENTER_HIGH)) inCenter[i] = true;
        if ((stickPosUs[i] <= END_LOW) or (stickPosUs[i] >= END_HIGH)) inCorner[i] = true;
        if (stickPosUs[i] != prevStickPosUs[i]) sticksMaintained = false;
        prevStickPosUs[i] = stickPosUs[i];
    }
    // then process depend on the state 
    if ((learningState == LEARNING_MIXERS) or (learningState == LEARNING_MIXERS_DONE)){
        // if all 3 sticks are centered, set flag and save 16 channels in centerCh
        if ((inCenter[0] && inCenter[1] && inCenter[2] ) and sticksMaintained){
            if (centerFlag == false){
                printf("centered\n");
            }
            centerFlag = true;
            memcpy(centerCh,rcChannelsUs, sizeof(rcChannelsUs) );
            
        } else if (inCorner[0] && inCenter[1] && inCenter[2] && sticksMaintained){ // Ail at end
            i=0;  // Aileron
            if ((abs(stickPosUs[i] - dirCh[i]) < 100) ) { //stick is in the right or up corner
                if ((( stickPosUs[i] < 1500) and ( stickPosUs[i] < rightUpUs[i][config.gyroChan[i]-1])) or\
                    (( stickPosUs[i] > 1500) and ( stickPosUs[i] > rightUpUs[i][config.gyroChan[i]-1])) or\
                    (rightUpFlags[i] == false)){
                    memcpy(rightUpUs[i],rcChannelsUs, sizeof(rcChannelsUs) );
                    rightUpFlags[i] = true;    // save when new pos is lower than previous
                    printf("Ail right Ch1=%i   Ch2=%i    Ch3=%i    Ch4=%i\n", rightUpUs[i][0] , rightUpUs[i][1] , rightUpUs[i][2] ,rightUpUs[i][3]); // to remove
                }
                //if (rightUpFlags[i] == false) {
                //    printf("Ail right Ch1=%i   Ch2=%i    Ch3=%i    Ch4=%i\n", rightUpUs[i][0] , rightUpUs[i][1] , rightUpUs[i][2] ,rightUpUs[i][3]); // to remove
                //}
                
            } else {  // stick is in the left or down corner
                if ((( stickPosUs[i] < 1500) and ( stickPosUs[i] < leftDownUs[i][config.gyroChan[i]-1])) or\
                    (( stickPosUs[i] > 1500) and ( stickPosUs[i] > leftDownUs[i][config.gyroChan[i]-1])) or\
                    (leftDownFlags[i] == false)){
                    memcpy(leftDownUs[i],rcChannelsUs, sizeof(rcChannelsUs) );
                    leftDownFlags[i] = true;    // save when new pos is lower than previous
                    printf("Ail left Ch1=%i   Ch2=%i    Ch3=%i    Ch4=%i\n", leftDownUs[i][0] , leftDownUs[i][1] , leftDownUs[i][2] , leftDownUs[i][3]); // to remove
                }
                //if (leftDownFlags[i] == false) {
                //    printf("Ail left Ch1=%i   Ch2=%i    Ch3=%i    Ch4=%i\n", leftDownUs[i][0] , leftDownUs[i][1] , leftDownUs[i][2] , leftDownUs[i][3]); // to remove
                //}
            }    
        } else if (inCorner[1] && inCenter[0] && inCenter[2] && sticksMaintained){ // ELV at end
            i=1;  // Elv
            if ((abs(stickPosUs[i] - dirCh[i]) < 100)) { //stick is in the right or up corner
                if ((( stickPosUs[i] < 1500) and ( stickPosUs[i] < rightUpUs[i][config.gyroChan[i]-1])) or\
                    (( stickPosUs[i] > 1500) and ( stickPosUs[i] > rightUpUs[i][config.gyroChan[i]-1])) or\
                    (rightUpFlags[i] == false)){
                    rightUpFlags[i] = true;    // save when new pos is lower than previous
                    memcpy(rightUpUs[i],rcChannelsUs, sizeof(rcChannelsUs) );
                    printf("Elv up Ch1=%i   Ch2=%i    Ch3=%i    Ch4=%i\n", rightUpUs[i][0] , rightUpUs[i][1] , rightUpUs[i][2] ,rightUpUs[i][3]); // to remove
                }
                //if (rightUpFlags[i] == false) {
                //    printf("Elv up Ch1=%i   Ch2=%i    Ch3=%i    Ch4=%i\n", rightUpUs[i][0] , rightUpUs[i][1] , rightUpUs[i][2] ,rightUpUs[i][3]); // to remove
                //}
            } else {  // stick is in the left or down corner
                if ((( stickPosUs[i] < 1500) and ( stickPosUs[i] < leftDownUs[i][config.gyroChan[i]-1])) or\
                    (( stickPosUs[i] > 1500) and ( stickPosUs[i] > leftDownUs[i][config.gyroChan[i]-1])) or\
                    (leftDownFlags[i] == false)){
                    leftDownFlags[i] = true;    // save when new pos is lower than previous
                    memcpy(leftDownUs[i],rcChannelsUs, sizeof(rcChannelsUs) );
                    printf("Elv down Ch1=%i   Ch2=%i    Ch3=%i    Ch4=%i\n", leftDownUs[i][0] , leftDownUs[i][1] , leftDownUs[i][2] , leftDownUs[i][3]); // to remove
                }
                //if (leftDownFlags[i] == false) {
                //    printf("Elv down Ch1=%i   Ch2=%i    Ch3=%i    Ch4=%i\n", leftDownUs[i][0] , leftDownUs[i][1] , leftDownUs[i][2] , leftDownUs[i][3]); // to remove
                //}
                
            }    
        } else if (inCorner[2] && inCenter[0] && inCenter[1] && sticksMaintained) { // Rud at end
            i=2; // 
            if ((abs(stickPosUs[i] - dirCh[i]) < 100)) { //stick is in the right or up corner
                if ((( stickPosUs[i] < 1500) and ( stickPosUs[i] < rightUpUs[i][config.gyroChan[i]-1])) or\
                    (( stickPosUs[i] > 1500) and ( stickPosUs[i] > rightUpUs[i][config.gyroChan[i]-1])) or\
                    (rightUpFlags[i] == false)){
                    rightUpFlags[i] = true;    // save when new pos is lower than previous
                    memcpy(rightUpUs[i],rcChannelsUs, sizeof(rcChannelsUs) );
                    printf("Rud right Ch1=%i   Ch2=%i    Ch3=%i    Ch4=%i\n", rightUpUs[i][0] , rightUpUs[i][1] , rightUpUs[i][2] ,rightUpUs[i][3]); // to remove
                }
                //if (rightUpFlags[i] == false) {
                //    printf("Rud right Ch1=%i   Ch2=%i    Ch3=%i    Ch4=%i\n", rightUpUs[i][0] , rightUpUs[i][1] , rightUpUs[i][2] ,rightUpUs[i][3]); // to remove
                //}
            } else {  // stick is in the left or down corner
                if ((( stickPosUs[i] < 1500) and ( stickPosUs[i] < leftDownUs[i][config.gyroChan[i]-1])) or\
                    (( stickPosUs[i] > 1500) and ( stickPosUs[i] > leftDownUs[i][config.gyroChan[i]-1])) or\
                    (leftDownFlags[i] == false)){
                    leftDownFlags[i] = true;    // save when new pos is lower than previous
                    memcpy(leftDownUs[i],rcChannelsUs, sizeof(rcChannelsUs) );
                    printf("Rud left Ch1=%i   Ch2=%i    Ch3=%i    Ch4=%i\n", leftDownUs[i][0] , leftDownUs[i][1] , leftDownUs[i][2] , leftDownUs[i][3]); // to remove
                }
                //if (leftDownFlags[i] == false) {
                //    printf("Rud left Ch1=%i   Ch2=%i    Ch3=%i    Ch4=%i\n", leftDownUs[i][0] , leftDownUs[i][1] , leftDownUs[i][2] , leftDownUs[i][3]); // to remove
                //}
                
            }    
        }
        // detect when all cases have been performed at least once (all flags are true)
        if ((centerFlag==true) && (rightUpFlags[0]==true) && (leftDownFlags[0]==true)\
             && (rightUpFlags[1]==true) && (leftDownFlags[1]==true) && (rightUpFlags[2]==true) && (leftDownFlags[2]==true)){
            ledState = STATE_GYRO_CAL_MIXER_DONE;
            learningState = LEARNING_MIXERS_DONE; // this will change the led color 
            static uint32_t prevPrintMs = 0;
            if ((millisRp() - prevPrintMs) > 1000){
                //printf("Ail=%i  Elv=%i    Rud=%i\n", stickPosUs[0] , stickPosUs[1] , stickPosUs[2] );
                printf("Cal mixer is done; switch may be changed\n");
                prevPrintMs=millisRp();
            }
    
        } 
        // check for a switch change
        // discard the changes done in the first 5 sec
        // if change occurs after 5 secin case of switch change when   check for error        
        if ( stabMode != prevStabMode)  { 
            prevStabMode = stabMode;
            if (( millisRp() - startMs) > 5000) { // when change, occurs after 5 sec of starting the learning process
                if (learningState == LEARNING_MIXERS) {  // error
                    if ( centerFlag == false) {printf("Error in Gyro setup: missing all 3 sticks centered\n");}
                    if ( ( rightUpFlags[0] == false) or ( leftDownFlags[0] == false) ) {printf("Error in Gyro setup: missing one corner position for aileron alone\n");}  
                    if ( ( rightUpFlags[1] == false) or ( leftDownFlags[1] == false) ) {printf("Error in Gyro setup: missing one corner position for elevator alone\n");}  
                    if ( ( rightUpFlags[2] == false) or ( leftDownFlags[2] == false) ) {printf("Error in Gyro setup: missing one corner position for rudder alone\n");}  
                    printf("Error :  oXs will stop handling receiver and sensors; power off/on is required !!!!!!");
                    watchdog_update();
                    ledState = STATE_NO_SIGNAL;
                    learningState = LEARNING_OFF;    
                    // to do : stop all interrupts to avoid error messages because queues are full
                    // disable interrupt to avoid having error msg about queue being full because part of main loop is not executed
                    uart_set_irq_enables(CRSF_UART_ID, false, false);
                    uart_set_irq_enables(CRSF2_UART_ID, false, false);
                    uart_set_irq_enables(SBUS_UART_ID, false, false);
                    uart_set_irq_enables(SBUS2_UART_ID, false, false);
                    pio_set_irq0_source_enabled(pio0 ,  pis_sm1_rx_fifo_not_empty , false ); // pio/sm for Exbus, Hott, ibus,Mpx,sport, srxl2
                    pio_set_irq1_source_enabled(pio0 ,  pis_sm3_rx_fifo_not_empty , false ); // pio/sm for ESC
                    pio_set_irq0_source_enabled(pio1 ,  pis_sm0_rx_fifo_not_empty , false ); // pio/sm for GPS
                    
    
                    configIsValid = false;
                } else {
                    ledState = STATE_GYRO_CAL_LIMIT;
                    printf("\nSecond step of learning process started\n");
                    learningState = LEARNING_LIMITS;
                }    
            }    
        }        
    } else if (learningState == LEARNING_LIMITS){
        // wait for a new change of switch to save and close
        if ( stabMode != prevStabMode) { // when change, 
            //build the parameters and save them
            struct gyroMixer_t temp; // use a temporary structure
            temp.version = GYROMIXER_VERSION;
            temp.isCalibrated = true; 
            for (i=0;i<16;i++) {
                #define MM 50    // MM = MIXER MARGIN
                // look at the channels that have to be controled by the gyro.
                // channel is used if there are differences and channel is not a stick
                temp.used[i] = false;
                if ( ((i != (config.gyroChan[0]-1) ) and (i != (config.gyroChan[1]-1) ) and (i != (config.gyroChan[2]-1))  and ( i!= (config.gyroChanControl-1) ))\
                 and ((abs(rightUpUs[0][i]-leftDownUs[0][i])>MM) or (abs(rightUpUs[0][i]-centerCh[i])>MM) or (abs(leftDownUs[0][i]-centerCh[i])>MM)\
                    or(abs(rightUpUs[1][i]-leftDownUs[1][i])>MM) or (abs(rightUpUs[1][i]-centerCh[i])>MM) or (abs(leftDownUs[1][i]-centerCh[i])>MM)\
                    or(abs(rightUpUs[2][i]-leftDownUs[2][i])>MM) or (abs(rightUpUs[2][i]-centerCh[i])>MM) or (abs(leftDownUs[2][i]-centerCh[i])>MM))) {
                    temp.used[i] = true; 
                }
                temp.neutralUs[i] = centerCh[i];  // servo pos when 3 stricks are centered
                
                temp.rateRollRightUs[i] =  (int16_t) rightUpUs[0][i] - (int16_t) centerCh[i] ;//   then right rate
                temp.rateRollLeftUs[i] =  (int16_t) leftDownUs[0][i] - (int16_t) centerCh[i] ;//   then LEFT rate 
                temp.ratePitchUpUs[i] =    (int16_t) rightUpUs[1][i] - (int16_t) centerCh[i] ;//   then right rate
                temp.ratePitchDownUs[i] = (int16_t) leftDownUs[1][i] - (int16_t) centerCh[i] ;//   then LEFT rate 
                temp.rateYawRightUs[i] =   (int16_t) rightUpUs[2][i] - (int16_t) centerCh[i] ;//   then right rate
                temp.rateYawLeftUs[i]  =  (int16_t) leftDownUs[2][i] - (int16_t) centerCh[i] ;//   then LEFT rate 
                
                temp.minUs[i] = minLimitsUs[i];
                temp.maxUs[i] = maxLimitsUs[i]; 
            }  // end for (all axis processed)
            ledState = STATE_NO_SIGNAL;
            learningState = LEARNING_OFF;
            printf("\nGyro calibration:\n");
            printf("Stick Ail_Right on channel %i at %-5i%%\n", config.gyroChan[0], pc(dirCh[0] - 1500));
            printf("Stick Elv Up    on channel %i at %-5i%%\n", config.gyroChan[1], pc(dirCh[1] - 1500));
            printf("Stick Rud_Right on channel %i at %-5i%%\n", config.gyroChan[2], pc(dirCh[2] - 1500));
            // update gyroMixer from temp.
            memcpy(&gyroMixer , &temp, sizeof(temp));
            printGyroMixer(); 
            printf("\nEnd of learning process; result will be saved in flash\n");
            saveGyroMixer(); //   save the gyro mixer from the learning process.                     
        } // end switch change
    }    
} 

int pc(int16_t val){ // convert a Rc value (-512/512) in %
    return ((int) val * 100 )>>9;  //divide by 512 = 100% 
}