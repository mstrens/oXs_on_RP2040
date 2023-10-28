#include "pico/stdlib.h"
#include "gyro.h"
#include "param.h"
#include "tools.h"
#include "stdlib.h"
#include "mpu.h"
#include "sbus_out_pwm.h"

extern CONFIG config;
extern MPU mpu;
extern uint16_t rcSbusOutChannels[16];  // Rc channels values provided by the receiver in Sbus units (not in PWM us).[172/1811]
uint16_t rcPwmChannels[16];             // remap of original rcSbusOutChannels in Pwm Us values [988/2012]
uint16_t rcPwmChannelsComp[16];        // Pwm us taking care of gyro corrections 

extern int16_t gyroX; // data provided by mpu, sent to core0 and with some filtering
extern int16_t gyroY;
extern int16_t gyroZ;

extern bool configIsValid;
extern uint8_t ledState;

struct gyroMixer_t gyroMixer ; // contains the parameters provided by the learning process for each of the 16 Rc channel

bool gyroIsInstalled = false;  // becomes true when config.gyroChanControl is defined (not 255) and MPU6050 installed

void initGyroMixer(){
    // this is a temprary solution to get gyro mixer without having yet the learning process (and the saving process)
    
    #define AIL1_CHANNEL 0  //here the ID of the output channels where compensation apply
    #define AIL2_CHANNEL 1
    #define ELV1_Channel 2
    #define ELV2_Channel 3
    #define RUD_Channel 4
    for (uint8_t i=0; i<16 ; i++){ // set all mixer as unsued (for gyro)
        gyroMixer.used[i] = false;
        gyroMixer.neutralUs[i] = 1500 ;
        gyroMixer.minUs[i] = 1000 ;
        gyroMixer.maxUs[i] = 2000 ;
        gyroMixer.rateRollLeftUs[i] = 0 ; // 0 means that when stick is at the end pos, this output RC channel is not impacted 
        gyroMixer.rateRollRightUs[i] = 0 ; 
        gyroMixer.ratePitchUpUs[i] = 0 ;  
        gyroMixer.ratePitchDownUs[i] = 0 ;
        gyroMixer.rateYawLeftUs[i] = 0  ;
        gyroMixer.rateYawRightUs[i] = 0 ;   
    }
    // then fill the parameters with your value // normally this should be uploaded from flash.
    gyroMixer.isCalibrated = true ;
    uint8_t i;
    i = AIL1_CHANNEL;    
    gyroMixer.used[i] = true ;
    gyroMixer.neutralUs[i] = 1600;
    gyroMixer.minUs[i] = 1100 ;
    gyroMixer.maxUs[i] = 1900 ;
    gyroMixer.rateRollLeftUs[i] = 300 ; // 300 means that when stick Ail (for Roll) is in the Left corner, RC channel (here AIL1_CHANNEL) is changed by 300 usec 
    gyroMixer.rateRollRightUs[i] = -400 ; // here for stick Ail Right
    // in this set up only AIL_CHANNEL get gyro corrections and only on roll axis
    
    // add here other setup for other output Rc channels    
    // e.g. for a second Ail
    //i = AIL2_CHANNEL;    
    //gyroMixer[i].used = true ;
    //gyroMixer.used[i] = true ;
    //gyroMixer.neutralUs[i] = 1700;
    //gyroMixer.minUs[i] = 1000 ;
    //gyroMixer.maxUs[i] = 2000 ;
    //gyroMixer.rateRollLeftUs[i] = 500 ; // 500 means that when stick Ail (for Roll) is in the Left corner, RC channel (here AIL1_CHANNEL) is changed by 500 usec 
    //gyroMixer.rateRollRightUs[i] = -500 ; // here for stick Ail Right
    
    // for e.g. a V stab you have to define values for 2 servos ELV1 and ELV2 and for each 4 rates.
    
}

// temporary solution waiting to allow changes in usb commands
void initGyroConfig(){
    #define GYRO_CHANNEL_CONTROL 9  // 0 means channel 1  
    #define GYRO_CHAN_AIL 10 // 0 means channel 1
    #define GYRO_CHAN_ELV 11 // 0 means channel 1
    #define GYRO_CHAN_RUD 12 // 0 means channel 1
     
    
    #define IDX_AIL 0
    #define IDX_ELV 1
    #define IDX_RUD 2
    // this is a temporaty function to define the gyro parameters waiting to be able to fill them with usb command
    config.gyroChanControl = GYRO_CHANNEL_CONTROL; // Rc channel used to say if gyro is implemented or not and to select the mode and the general gain. Value must be in range 1/16 or 255 (no gyro)
    config.gyroChan[IDX_AIL] = GYRO_CHAN_AIL ;    // Rc channel used to transmit original Ail, Elv, Rud stick position ; Value must be in range 1/16 when gyroControlChannel is not 255
    config.gyroChan[IDX_ELV] = GYRO_CHAN_ELV ;    // Rc channel used to transmit original Ail, Elv, Rud stick position ; Value must be in range 1/16 when gyroControlChannel is not 255
    config.gyroChan[IDX_RUD] = GYRO_CHAN_RUD ;    // Rc channel used to transmit original Ail, Elv, Rud stick position ; Value must be in range 1/16 when gyroControlChannel is not 255
    for (uint8_t i = 0; i<3; i++){
        config.pid_param_rate.kp[i] = 500; // default PID param
        config.pid_param_rate.ki[i] = 0; // default PID param
        config.pid_param_rate.kd[i] = 500; // default PID param
        config.pid_param_hold.kp[i] = 500; // default PID param
        config.pid_param_hold.ki[i] = 500; // default PID param
        config.pid_param_hold.kd[i] = 500; // default PID param
    }
    
    config.vr_gain[IDX_AIL] = 127;          // store the gain per axis, max value is 128 (to combine with global gain provided by gyroChanControl)
    config.vr_gain[IDX_ELV] = 127;          // store the gain per axis, max value is 128 (to combine with global gain provided by gyroChanControl)
    config.vr_gain[IDX_RUD] = 127;          // store the gain per axis, max value is 128 (to combine with global gain provided by gyroChanControl)
    config.stick_gain_throw = STICK_GAIN_THROW_FULL ;  //STICK_GAIN_THROW_FULL=1, STICK_GAIN_THROW_HALF=2, STICK_GAIN_THROW_QUARTER=3 
} 

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

  
enum STAB_MODE stabMode = STAB_OFF;
// rx
#define RX_GAIN_HYSTERESIS 25
#define RX_MODE_HYSTERESIS 25
#define RX_WIDTH_MIN 900
#define RX_WIDTH_LOW_FULL 1000
#define RX_WIDTH_LOW_NORM 1100
#define RX_WIDTH_LOW_TRACK 1250
#define RX_WIDTH_MID 1500
#define RX_WIDTH_MODE_MID 1550	// Move all hysteresis to Hold Mode side so 1500-1520 will always force Rate Mode
#define RX_WIDTH_HIGH_TRACK 1750
#define RX_WIDTH_HIGH_NORM 1900
#define RX_WIDTH_HIGH_FULL 2000
#define RX_WIDTH_MAX 2100



int16_t ail_in2_offset, ele_in2_offset, rud_in2_offset; //, flp_in2_offset; // difference from *_in2_mid (stick position)
const int16_t stick_gain_max = 400; // [1100-1500] or [1900-1500] => [0-STICK_GAIN_MAX]
const int16_t master_gain_max = 400; // [1500-1100] or [1500-1900] => [0-MASTER_GAIN_MAX]

int16_t correction[3] = {0, 0, 0};
int16_t correctionPosSide[3] = {0, 0, 0};
int16_t correctionNegSide[3] = {0, 0, 0};


int16_t min(const int16_t a, const int16_t b)
{
    return (b < a) ? b : a;
}


void updateGyroCorrections(){
    if ((config.gyroChanControl == 0) or (config.gyroChanControl > 16) or (mpu.mpuInstalled == false)) return;
    
    uint32_t t = microsRp(); 
    if ((int32_t)(t - last_pid_time) < PID_PERIOD) return;

    // to do rename those field for clarity
    int16_t ail_in2, ailr_in2, ele_in2, rud_in2, aux_in2, aux2_in2, thr_in2, flp_in2;
    ail_in2 = rcPwmChannels[config.gyroChan[0]]; // get original position of the 3 sticks
    ele_in2 = rcPwmChannels[config.gyroChan[1]];
    rud_in2 = rcPwmChannels[config.gyroChan[2]];
    aux_in2 = rcPwmChannels[config.gyroChanControl];

    int16_t stick_gain[3];
    int16_t master_gain;
    uint8_t i;

    // stabilization mode
    enum STAB_MODE stabMode2 = 
      (stabMode == STAB_HOLD && aux_in2 <= RX_WIDTH_MODE_MID - RX_MODE_HYSTERESIS) ? STAB_RATE : 
      (stabMode == STAB_RATE && aux_in2 >= RX_WIDTH_MODE_MID + RX_MODE_HYSTERESIS) ? STAB_HOLD : stabMode; // hysteresis, all now in Hold Mode region

    if (stabMode2 != stabMode) {
        stabMode = stabMode2;
        // set_led_msg(1, (stabMode == STAB_RATE) ? 0 : 4, LED_SHORT);  To do later on (led management)

        // reset attitude error and i_limit threshold on mode change
        for (i=0; i<3; i++) {
            pid_state.sum_err[i] = 0;
            pid_state.i_limit[i] = 0;
        }

        /* to do : Undestand if this code is useful or not. I expect not if the original rc channel ail,Elc,Rud are provided without trim.
        // check for inflight rx calibration
        if (cfg.inflight_calibrate == INFLIGHT_CALIBRATE_ENABLE) {
            if ((int32_t)(t - last_stabMode_time) > 500000L) {
            stabMode_count = 0;
            }
            last_stabMode_time = t;

            if (++stabMode_count >= 3) {
            ail_in2_mid = ail_in2;
            ele_in2_mid = ele_in2;
            rud_in2_mid = rud_in2;
            ailr_in2_mid = ailr_in2;
            flp_in2_mid = flp_in2;
            }
        }
        */        
    }
    
    // determine how much sticks are off center (from neutral)
    //ail_in2_offset = ((ail_in2 - ail_in2_mid) + (ailr_in2 - ailr_in2_mid)) >> 1; // changed by ms
    //ele_in2_offset = ele_in2 - ele_in2_mid;
    //rud_in2_offset = rud_in2 - rud_in2_mid;
    //flp_in2_offset = flp_in2 - flp_in2_mid;
    ail_in2_offset = ail_in2 - 1500; // here we suppose that Tx send 1500 us at mid point; otherwise we could use neutral from learning process
    ele_in2_offset = ele_in2 - 1500;
    rud_in2_offset = rud_in2 - 1500;
    
    // vr_gain[] [-128, 128] from VRs or config ; in mstrens code, it is supposed to come only from config 
    
    // see enum STICK_GAIN_THROW. shift=0 => FULL, shift=1 => HALF, shift=2 => QUARTER
    //int8_t shift = cfg.stick_gain_throw - 1;
    int8_t shift = config.stick_gain_throw - 1;
    
    // stick_gain[] [1100, <ail*|ele|rud>_in2_mid, 1900] => [0%, 100%, 0%] = [0, STICK_GAIN_MAX, 0]
    stick_gain[0] = stick_gain_max - min(abs(ail_in2_offset) << shift, stick_gain_max);
    stick_gain[1] = stick_gain_max - min(abs(ele_in2_offset) << shift, stick_gain_max);
    stick_gain[2] = stick_gain_max - min(abs(rud_in2_offset) << shift, stick_gain_max);    
    
    // master gain [Rate = 1475-1075] or [Hold = 1575-1975] => [0, MASTER_GAIN_MAX] 
    if (aux_in2 < (RX_WIDTH_MID - RX_GAIN_HYSTERESIS))  {
    	// Handle Rate Mode Gain Offset, gain = 1 when hysteresis area is exited
    	// Previously gain was the value of RX_GAIN_HYSTERESIS at first exit
    	master_gain = constrain(((RX_WIDTH_MID - RX_GAIN_HYSTERESIS) - aux_in2) , 0, master_gain_max);
    } else {
    	if (aux_in2 > (RX_WIDTH_MODE_MID + RX_MODE_HYSTERESIS))  {
    	   // Handle Hold Mode Gain Offset, gain = 1 when both hysteresis areas are exited
    	   // Previously gain was the value of RX_MODE_HYSTERESIS at first exit
    	   master_gain = constrain(aux_in2 - (RX_WIDTH_MID + RX_MODE_HYSTERESIS), 0, master_gain_max);		
    	} 
    	  // Force Gain to 0 while in either of the Hysteresis areas    
    	  else  master_gain = 0; // Force deadband
    }	  	
  
    // commanded angular rate (could be from [ail|ele|rud]_in2, note direction/sign)
    if (stabMode == STAB_HOLD || 
        (stabMode == STAB_RATE && config.rate_mode_stick_rotate == RATE_MODE_STICK_ROTATE_ENABLE)) {
        // stick controlled roll rate
        // cfg.max_rotate shift = [1, 5]
        // eg. max stick == 400, cfg.max_rotate == 4. then 400 << 4 = 6400 => 6400/32768*2000 = 391deg/s (32768 == 2000deg/s)
        int16_t sp[3];
        sp[0] = ail_in2_offset << config.max_rotate;
        sp[1] = ele_in2_offset << config.max_rotate;
        sp[2] = rud_in2_offset << config.max_rotate;
        for (i=0; i<3; i++)
            //pid_state.setpoint[i] = vr_gain[i] < 0 ? sp[i] : -sp[i];
            pid_state.setpoint[i] = config.vr_gain[i] < 0 ? sp[i] : -sp[i];      
    } else {
        // zero roll rate, stabilization only
        for (i=0; i<3; i++) 
            pid_state.setpoint[i] = 0;
    }
        
    if (stabMode == STAB_HOLD) {
        // max attitude error (bounding box)    
        for (i=0; i<3; i++) {
            // 2000 deg/s == 32768 units, so 1 deg/(PID_PERIOD=10ms) == 32768/20 units
            pid_state.i_limit[i] = ((int32_t)30 * (32768 / 2 / (PID_PERIOD / 1000)) * stick_gain[i]) >> 9; 
        }
    }
    
    // measured angular rate (from the gyro and apply calibration offset)
    //pid_state.input[0] = constrain(gyro[0] - gyro0[0], -8192, 8191);
    //pid_state.input[1] = constrain(gyro[1] - gyro0[1], -8192, 8191);
    //pid_state.input[2] = constrain(gyro[2] - gyro0[2], -8192, 8191);        
    pid_state.input[0] = constrain(gyroX, -8192, 8191);  // changed by Mstrens ; to do check if this is OK
    pid_state.input[1] = constrain(gyroY, -8192, 8191);
    pid_state.input[2] = constrain(gyroZ, -8192, 8191);        
    
    // apply PID control
    compute_pid(&pid_state, (stabMode == STAB_RATE) ? &config.pid_param_rate : &config.pid_param_hold);

    // apply vr_gain, stick_gain and master_gain
    for (i=0; i<3; i++) {
        // vr_gain [-128,0,127]/128, stick_gain [0,400,0]/512, master_gain [400,0,400]/512
        //correction[i] = ((((int32_t)pid_state.output[i] * vr_gain[i] >> 7) * stick_gain[i]) >> 9) * master_gain >> 9;
        correction[i] = ((((int32_t)pid_state.output[i] * config.vr_gain[i] >> 7) * stick_gain[i]) >> 9) * master_gain >> 9;
        uint16_t OSP = rcPwmChannels[config.gyroChan[i]];  // orginal stick position
        uint16_t ESP = OSP + correction[i];                // expected stick position
        if (OSP >= 1500 ){
            if (ESP >= 1500){
            correctionPosSide[i] = correction[i];
            correctionNegSide[i] = 0;   
            } else {
                correctionPosSide[i] = (1500 - OSP) ; // corr is neg, part 1 and 2 must be neg  
                correctionNegSide[i] = (ESP - 1500) ;   
            }
        } else {
            if (ESP >= 1500){
            correctionPosSide[i] = ESP -1500;  // corr is positive, part 1 and 2 must be pos too
            correctionNegSide[i] = 1500 -OSP;   
            } else {
                correctionPosSide[i] = 0 ;
                correctionNegSide[i] = correction[i] ;   
            }
        }      
    }


    /*
    // to do : I need to understand what is calibration_wag 
    // calibration wag on all surfaces if needed
    if (calibration_wag_count > 0) {
        if ((int32_t)(t - last_calibration_wag_time) > 200000L) {
            calibration_wag_count--;
            last_calibration_wag_time = t;
        }
        for (i=0; i<3; i++)
            correction[i] += (calibration_wag_count & 1) ? 150 : -150;    
    }    
    */
#if defined(SERIAL_DEBUG) && 0
    Serial.print(correction[0]); Serial.print('\t');
    Serial.print(correction[1]); Serial.print('\t');
    Serial.print(correction[2]); Serial.println('\t');
#endif
    last_pid_time = t;
    // End of PID process
}

void applyGyroCorrection(){
    //This should be called only when new Rc values have been received
    // This function is called only when gyro is used (checked before calling this function)
    // map the channels from Sbus units to pwm Us and copy them.
    for (uint8_t i=0; i<16;i++){
        rcPwmChannelsComp[i] = rcPwmChannels[i] = fmap( rcSbusOutChannels[i]) ;
        if ( rcPwmChannels[i] > gyroMixer.maxUs[i]) {
            gyroMixer.maxUs[i] = rcPwmChannels[i] ;
        } else if (rcPwmChannels[i] < gyroMixer.minUs[i]) {
            gyroMixer.minUs[i] = rcPwmChannels[i] ;
        }    
    }
    //updateGyroCompensation(); // update gyro compensation (only every xx msec) should be called in main loop at his own 
    // from here, we know the compensations to apply on the pos and neg sided
    // apply compensation
    for (uint8_t i=0 ; i<16 ; i++){  // for each Rc channel
        if ( gyroMixer.used[i] ) { // when this channel uses gyro compensation.
            // adapt the corrections Pos and Neg for each axis based on the ranges stored in gyroMixer.
            // to do : check that those are the correct matching between Pos/neg and Left/right
            rcPwmChannelsComp[i] += ((int32_t) correctionPosSide[0] * (int32_t) gyroMixer.rateRollLeftUs[config.gyroChan[i]])  >> 9 ; // division by 512 because full range is about 500.
            rcPwmChannelsComp[i] += ((int32_t) correctionNegSide[0] * (int32_t) gyroMixer.rateRollRightUs[config.gyroChan[i]])  >> 9 ; // division by 512 because full range is about 500.
            rcPwmChannelsComp[i] += ((int32_t) correctionPosSide[1] * (int32_t) gyroMixer.ratePitchUpUs[config.gyroChan[i]])  >> 9 ; // division by 512 because full range is about 500.
            rcPwmChannelsComp[i] += ((int32_t) correctionNegSide[1] * (int32_t) gyroMixer.ratePitchDownUs[config.gyroChan[i]])  >> 9 ; // division by 512 because full range is about 500.
            rcPwmChannelsComp[i] += ((int32_t) correctionPosSide[2] * (int32_t) gyroMixer.rateYawLeftUs[config.gyroChan[i]])  >> 9 ; // division by 512 because full range is about 500.
            rcPwmChannelsComp[i] += ((int32_t) correctionNegSide[2] * (int32_t) gyroMixer.rateYawRightUs[config.gyroChan[i]])  >> 9 ; // division by 512 because full range is about 500.
            // stay within the limits 
            if (rcPwmChannelsComp[i] < gyroMixer.minUs[config.gyroChan[i]]) rcPwmChannelsComp[i] = gyroMixer.minUs[config.gyroChan[i]] ;
            if (rcPwmChannelsComp[i] > gyroMixer.maxUs[config.gyroChan[i]]) rcPwmChannelsComp[i] = gyroMixer.maxUs[config.gyroChan[i]] ;    
        }
    }
    // here all PWM have been recalculated and can be used for PWM output
}

//enum LEARNING_STATE {LEARNING_OFF, LEARNING_INIT,LEARNING_WAIT, LEARNING_MIXERS, LEARNING_LIMITS, LEARNING_ERROR, LEARNING_OK};

enum LEARNING_STATE learningState = LEARNING_OFF;

#define MARGIN 3
    #define CENTER_LOW 1500-MARGIN
    #define CENTER_HIGH 1500+MARGIN
    #define END_LOW 1000  // Normal 988
    #define END_HIGH 2000  // normal 2012
    

bool checkForLearning(){ // return true when learning process can start
    // We consider that there are 5 changes within 5sec when the sum of 5 intervals between 2 changes is less than 5 sec.
    // we store the last 5 intervals
    // each time an change occurs, we loose the oldiest   
    static enum STAB_MODE prevStabMode = STAB_OFF;
    static uint32_t lastChangeMs = 0 ;
    static uint32_t intervals[5] = {10000 , 10000 ,10000 ,10000 ,10000}; 
    static uint8_t idx = 0;
    static uint32_t sumIntervals = 50000;
    
    if ( (stabMode == prevStabMode) or (stabMode == STAB_OFF)) {
        return false;
    }
    prevStabMode = stabMode;
    if ((rcPwmChannels[config.gyroChan[0]] <= END_LOW) or (rcPwmChannels[config.gyroChan[0]] >= END_HIGH) &&\
        (rcPwmChannels[config.gyroChan[1]] <= END_LOW) or (rcPwmChannels[config.gyroChan[1]] >= END_HIGH) &&\
        (rcPwmChannels[config.gyroChan[2]] <= END_LOW) or (rcPwmChannels[config.gyroChan[2]] >= END_HIGH)){ 
        uint32_t t = millisRp();
        uint32_t interval = t - lastChangeMs;
        lastChangeMs = t;
        sumIntervals -= intervals[idx];
        intervals[idx] = interval;
        idx++;
        if (sumIntervals < 5000) {
            return true; 
        }
    }
    return false; 
}

void calibrateGyroMixers(){
    static bool centerFlag;
    static int16_t centerCh[16];  // rc channels when 3 sticks are centered
    //static bool dirFlag ;
    static int16_t dirCh[3];  // stick positions when all 3 are in the corner = AIL and RUD Rigth and ELV UP
    static bool rightUpFlags[3] ;   // register if we already know the 16 Rc channels for only one of the 3 sticks in low corner 
    static int16_t rightUpUs[3][16];  // register the 16 Rc channels values 
    static bool leftDownFlags[3] ;    // idem for 3 sticks in opposite corner
    static int16_t leftDownUs[3][16];
    static enum STAB_MODE prevStabMode ;  // used to detect when gyro switch change
    static uint32_t startMs ;             // used to check that we stay in step 1 for at least 5 sec  
    static uint16_t minLimitsUs[16]; // min limits of the 16 servos during the learning process
    static uint16_t maxLimitsUs[16]; // idem for max limits
    

    if (learningState == LEARNING_OFF){
        if (checkForLearning() == false){  // function return true when conditions to start the learning process are OK  
            return;
        }
        // learning process may start
        // save the current position of sticks
        //dirFlag = true;
        //dirFlag = false;
        dirCh[0] = rcPwmChannels[config.gyroChan[0]]; // save the stick values for Ail Right  
        dirCh[1] = rcPwmChannels[config.gyroChan[1]]; // save the values for ELV Up
        dirCh[2] = rcPwmChannels[config.gyroChan[2]]; // save the values for Rud Right  
        centerFlag = rightUpFlags[0] = rightUpFlags[1] = rightUpFlags[2] = leftDownFlags[0] = leftDownFlags[1] = leftDownFlags[2] = false;
        for (uint8_t i=0; i<16; i++) {  // reset the min and max RC channel limits
            minLimitsUs[i] = 2012; // lower Rc channel values will be discoverd during the learning process
            maxLimitsUs[i] = 988;  // idem for upper values
        }
        //centerCh[16]; dirCh[16]; rightUpUs[3][16]; leftDownUs[3][16]; will be filled with the flags
        startMs = millisRp();         // used to check that there is 5 msec before switching to second step
        ledState = STATE_GYRO_CAL_MIXER_NOT_DONE;
        learningState = LEARNING_MIXERS;   
    }
    // once learning process has started, we save always the min and max servo positions
    for (uint8_t i=0; i<16;i++){
        if ( rcPwmChannels[i] > maxLimitsUs[i]) {
            maxLimitsUs[i] = rcPwmChannels[i];
        } else if (rcPwmChannels[i] < minLimitsUs[i]) {
            minLimitsUs[i] = rcPwmChannels[i] ; 
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
    stickPosUs[0] = (int16_t) rcPwmChannels[config.gyroChan[0]]; 
    stickPosUs[1] = (int16_t) rcPwmChannels[config.gyroChan[1]]; 
    stickPosUs[2] = (int16_t) rcPwmChannels[config.gyroChan[2]];
    for (i=0;i>3;i++){                // detect when sticks are centered or in a corner
        if ((stickPosUs[i] >= CENTER_LOW) and (stickPosUs[i] <= CENTER_HIGH)) inCenter[i] = true;
        if ((stickPosUs[i] <= END_LOW) or (stickPosUs[i] >= END_HIGH)) inCorner[i] = true;
    }
    // then process depend on the state
    if ((learningState == LEARNING_MIXERS) or (learningState == LEARNING_MIXERS_DONE)){
        // if all 3 sticks are centered, set flag and save 16 channels in centerCh
        if (inCenter[0] && inCenter[1] && inCenter[2] ){
            centerFlag = true;
            memcpy(centerCh,rcPwmChannels, sizeof(rcPwmChannels) );
        } else if (inCorner[0] && inCenter[1] && inCenter[2]){ // Ail at end
            i=0;  // Aileron
            if ((abs(stickPosUs[i] - dirCh[i]) < 100)) { //stick is in the right or up corner
                rightUpFlags[i] = true;    // save when new pos is lower than previous
                if (( stickPosUs[i] < 1500) and ( stickPosUs[i] < rightUpUs[i][config.gyroChan[i]]) or\
                    ( stickPosUs[i] > 1500) and ( stickPosUs[i] > rightUpUs[i][config.gyroChan[i]]) ){\
                    memcpy(rightUpUs[i],rcPwmChannels, sizeof(rcPwmChannels) );
                }
            } else {  // stick is in the left or down corner
                leftDownFlags[i] = true;    // save when new pos is higher than previous
                if (( stickPosUs[i] < 1500) and ( stickPosUs[i] < leftDownUs[i][config.gyroChan[i]]) or\
                    ( stickPosUs[i] > 1500) and ( stickPosUs[i] > leftDownUs[i][config.gyroChan[i]]) ){\
                    memcpy(leftDownUs[i],rcPwmChannels, sizeof(rcPwmChannels) );
                }
            }    
        } else if (inCorner[1] && inCenter[0] && inCenter[2]){ // ELV at end
            i=1;  // Elv
            if ((abs(stickPosUs[i] - dirCh[i]) < 100)) { //stick is in the right or up corner
                rightUpFlags[i] = true;    // save when new pos is lower than previous
                if (( stickPosUs[i] < 1500) and ( stickPosUs[i] < rightUpUs[i][config.gyroChan[i]]) or\
                    ( stickPosUs[i] > 1500) and ( stickPosUs[i] > rightUpUs[i][config.gyroChan[i]]) ){\
                    memcpy(rightUpUs[i],rcPwmChannels, sizeof(rcPwmChannels) );
                }
            } else {  // stick is in the left or down corner
                leftDownFlags[i] = true;    // save when new pos is higher than previous
                if (( stickPosUs[i] < 1500) and ( stickPosUs[i] < leftDownUs[i][config.gyroChan[i]]) or\
                    ( stickPosUs[i] > 1500) and ( stickPosUs[i] > leftDownUs[i][config.gyroChan[i]]) ){\
                    memcpy(leftDownUs[i],rcPwmChannels, sizeof(rcPwmChannels) );
                }
            }    
        } else if (inCorner[2] && inCenter[0] && inCenter[1]){ // Rud at end
            i=2; // 
            if ((abs(stickPosUs[i] - dirCh[i]) < 100)) { //stick is in the right or up corner
                rightUpFlags[i] = true;    // save when new pos is lower than previous
                if (( stickPosUs[i] < 1500) and ( stickPosUs[i] < rightUpUs[i][config.gyroChan[i]]) or\
                    ( stickPosUs[i] > 1500) and ( stickPosUs[i] > rightUpUs[i][config.gyroChan[i]]) ){\
                    memcpy(rightUpUs[i],rcPwmChannels, sizeof(rcPwmChannels) );
                }
            } else {  // stick is in the left or down corner
                leftDownFlags[i] = true;    // save when new pos is higher than previous
                if (( stickPosUs[i] < 1500) and ( stickPosUs[i] < leftDownUs[i][config.gyroChan[i]]) or\
                    ( stickPosUs[i] > 1500) and ( stickPosUs[i] > leftDownUs[i][config.gyroChan[i]]) ){\
                    memcpy(leftDownUs[i],rcPwmChannels, sizeof(rcPwmChannels) );
                }
            }    
        }
        // detect when all cases have been performed at least once (all flags are true)
        if ((centerFlag==true) && (rightUpFlags[0]==true) && (leftDownFlags[0]) && (rightUpFlags[1]==true) && (leftDownFlags[1]) && (rightUpFlags[2]==true) && (leftDownFlags[20])){
            ledState = STATE_GYRO_CAL_MIXER_DONE;
            learningState = LEARNING_MIXERS_DONE; // this will change the led color 
        } 
        // check for a switch change
        // discard the changes done in the first 5 sec
        // if change occurs after 5 secin case of switch change when   check for error        
        if (( stabMode != prevStabMode) and ( stabMode != STAB_OFF) ) { 
            prevStabMode = stabMode;
            if (( millisRp() - startMs) > 5000) { // when change, occurs after 5 sec of starting the learning process
                if (learningState == LEARNING_MIXERS) {  // error
                    if ( centerFlag == false) {printf("Error in Gyro setup: missing all 3 sticks centered\n");}
                    if ( ( rightUpFlags[0] == false) or ( leftDownFlags[0] == false) ) {printf("Error in Gyro setup: missing one corner position for aileron alone\n");}  
                    if ( ( rightUpFlags[1] == false) or ( leftDownFlags[1] == false) ) {printf("Error in Gyro setup: missing one corner position for elevator alone\n");}  
                    if ( ( rightUpFlags[2] == false) or ( leftDownFlags[2] == false) ) {printf("Error in Gyro setup: missing one corner position for rudder alone\n");}  
                    ledState = STATE_NO_SIGNAL;
                    learningState = LEARNING_OFF;    
                    configIsValid = false;
                } else {
                    ledState = STATE_GYRO_CAL_LIMIT;
                    learningState = LEARNING_LIMITS;
                }    
            }    
        }        
    } else if (learningState == LEARNING_LIMITS){
        // wait fo a new change of switch to save and close
        if (( stabMode != prevStabMode) and ( stabMode != STAB_OFF) ){ // when change, 
            //build the parameters and save them
            struct gyroMixer_t temp; // use a temporary structure
            temp.version = GYRO_VERSION;
            temp.isCalibrated = true; 
            for (i=0;i<16;i++) {
                #define MM 5    // MM = MIXER MARGIN
                // look at the channels that have to be controled by the gyro.
                // channel is used if there are differences and channel is not a stick
                temp.used[i] = false;
                if ( ((i != config.gyroChan[0] ) and (i != config.gyroChan[1] ) and (i != config.gyroChan[2] )) and (\
                    (abs(rightUpUs[0][i]-leftDownUs[0][i])>MM) or (abs(rightUpUs[0][i]-centerCh[i])>MM) or (abs(leftDownUs[0][i]-centerCh[i])>MM)\
                    or(abs(rightUpUs[1][i]-leftDownUs[1][i])>MM) or (abs(rightUpUs[1][i]-centerCh[i])>MM) or (abs(leftDownUs[1][i]-centerCh[i])>MM)\
                    or(abs(rightUpUs[2][i]-leftDownUs[2][i])>MM) or (abs(rightUpUs[2][i]-centerCh[i])>MM) or (abs(leftDownUs[2][i]-centerCh[i])>MM))) {
                    temp.used[i] = true; 
                }
                temp.neutralUs[i] = centerCh[i];  // servo pos when 3 stricks are centered
                
                temp.rateRollRightUs[i] =  rightUpUs[0][i] - centerCh[i] ;//   then right rate
                temp.rateRollLeftUs[i] =  leftDownUs[0][i] - centerCh[i] ;//   then LEFT rate 
                temp.ratePitchUpUs[i] =    rightUpUs[1][i] - centerCh[i] ;//   then right rate
                temp.ratePitchDownUs[i] = leftDownUs[1][i] - centerCh[i] ;//   then LEFT rate 
                temp.rateYawRightUs[i] =   rightUpUs[2][i] - centerCh[i] ;//   then right rate
                temp.rateYawLeftUs[i]  =  leftDownUs[2][i] - centerCh[i] ;//   then LEFT rate 
                
                temp.minUs[i] = minLimitsUs[i];
                temp.maxUs[i] = maxLimitsUs[i]; 
            }  // end for (all axis processed)
            ledState = STATE_NO_SIGNAL;
            learningState = LEARNING_OFF;
            printf("\nGyro calibration:\n");
            printf("Stick Ail_Right on channel %i at %-5i\n", config.gyroChan[0]+1, pc(dirCh[0] - 1500));
            printf("Stick Elv Up    on channel %i at %-5i\n", config.gyroChan[1]+1, pc(dirCh[1] - 1500));
            printf("Stick Rud_Right on channel %i at %-5i\n", config.gyroChan[2]+1, pc(dirCh[2] - 1500));
            printf("Gyro corrections (from center pos in %%) on:      \n");
            for (uint8_t i = 0; i<16;i++) {
                if ( temp.used[i]) {
                    printf("Channel %-2i center=%-4i rollRight=%-4i rollLeft=%-4i pitchUp%-4i pitchDown=%-4i yawRight=%-4i yawLeft=%-4i min=%-4i max=%-4i\n",\
                    i+1 , pc(temp.neutralUs[i]-1500) , pc(temp.rateRollRightUs[i]), pc(temp.rateRollLeftUs[i]), \
                    pc(temp.ratePitchUpUs[i]) , pc(temp.ratePitchDownUs[i]),\
                    pc(temp.rateYawRightUs[i]), pc(temp.rateYawLeftUs[i]),\
                    pc(temp.minUs[i]-1500) , pc(temp.maxUs[i]-1500) ) ;
                }
            }
            // update gyroMixer from temp.
            memcpy(&gyroMixer , &temp, sizeof(temp));
            // to do  ------ here add the saving to flash                     
        } // end switch change
    }    
} 

int pc(int16_t val){ // convert a Rc value (-512/512) in %
    return ((int) val * 100 )>>9;  //divide by 512 = 100% 
}