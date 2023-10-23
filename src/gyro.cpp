#include "pico/stdlib.h"
#include "gyro.h"
#include "param.h"
#include "tools.h"
#include "stdlib.h"

extern CONFIG config;

struct gyroMixer_t gyroMixer[16] ; // contains the parameters provided by the learning process for each of the 16 Rc channel

void initGyroMixer(){
    // this is a temprary solution to get gyro mixer without having yet the learning process (and the saving process)
    
    #define AIL1_CHANNEL 0  //here the ID of the output channels where compensation apply
    #define AIL2_CHANNEL 1
    #define ELV1_Channel 2
    #define ELV2_Channel 3
    #define RUD_Channel 4
    for (uint8_t i=0; i<16 ; i++){ // set all mixer as unsued (for gyro)
        gyroMixer[i].used = false;
        gyroMixer[i].neutralUs = 1500 ;
        gyroMixer[i].minUs = 1000 ;
        gyroMixer[i].maxUs = 2000 ;
        gyroMixer[i].rateRollLeftUs = 0 ; // 0 means that when strick is at the end pos, this output RC channel is not impacted 
        gyroMixer[i].rateRollRightUs = 0 ; 
        gyroMixer[i].ratePitchUpUs = 0 ;  
        gyroMixer[i].ratePitchDownUs = 0 ;
        gyroMixer[i].rateYawLeftUs = 0  ;
        gyroMixer[i].rateYawRightUs = 0 ;   
    }
    uint8_t i;
    i = AIL1_CHANNEL;    
    gyroMixer[i].used = true ;
    gyroMixer[i].neutralUs = 1600;
    gyroMixer[i].minUs = 1100 ;
    gyroMixer[i].maxUs = 1900 ;
    gyroMixer[i].rateRollLeftUs = 300 ; // 300 means that when stick is at the end pos, this output RC channel is changed by 300 
    gyroMixer[i].rateRollRightUs = -400 ; 
    // add here other setup for other output Rc channels    
    //i = AIL2_CHANNEL;    
    //gyroMixer[i].used = true ;
    //...
    
}

void initGyroConfig(){
    #define GYRO_CHANNEL_CONTROL 9
    #define IDX_AIL 0
    #define IDX_ELV 1
    #define IDX_RUD 2
    #define GYRO_CHAN_AIL 10
    #define GYRO_CHAN_ELV 11
    #define GYRO_CHAN_RUD 12
     
    
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
    
    config.vr_gain[IDX_AIL] = 128;          // store the gain per axis, max value is 128 (to combine with global gain provided by gyroChanControl)
    config.vr_gain[IDX_ELV] = 128;          // store the gain per axis, max value is 128 (to combine with global gain provided by gyroChanControl)
    config.vr_gain[IDX_RUD] = 128;          // store the gain per axis, max value is 128 (to combine with global gain provided by gyroChanControl)
} 

/***************************************************************************************************************
 * PID
 ***************************************************************************************************************/

#define PID_PERIOD 10000
// relative exponential weights kp:ki:kd
#define PID_KP_SHIFT 3 
#define PID_KI_SHIFT 6
#define PID_KD_SHIFT 2

// struct _pid_param defined in FlightStab.h

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

  
// stabilization mode
enum STAB_MODE {STAB_RATE, STAB_HOLD};
enum STAB_MODE stab_mode = STAB_RATE;
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



extern uint16_t rcSbusOutChannels[16];  // Rc channels values provided by the receiver.

int16_t ail_in2_offset, ele_in2_offset, rud_in2_offset; //, flp_in2_offset; // difference from *_in2_mid (stick position)
const int16_t stick_gain_max = 400; // [1100-1500] or [1900-1500] => [0-STICK_GAIN_MAX]
const int16_t master_gain_max = 400; // [1500-1100] or [1500-1900] => [0-MASTER_GAIN_MAX]

//to do : check that is the same as what is provided by the oxs code for imu.
//int16_t gyro0[3] = {0, 0, 0}; // calibration sets zero-movement-measurement offsets
//int16_t gyro[3] = {0, 0, 0}; // full scale = 16b-signed = +/-2000 deg/sec
extern int16_t gx, gy,gz;

int16_t correction[3] = {0, 0, 0};


int16_t min(const int16_t a, const int16_t b)
{
    return (b < a) ? b : a;
}

void handleGyroCompensation(){
    if ((config.gyroChanControl > 16) or (config.gyroChanControl > 16)) return;
    
    uint32_t t = microsRp(); 
    if ((int32_t)(t - last_pid_time) < PID_PERIOD) return;

    // to do rename those field for clarity
    int16_t ail_in2, ailr_in2, ele_in2, rud_in2, aux_in2, aux2_in2, thr_in2, flp_in2;
    ail_in2 = rcSbusOutChannels[config.gyroChan[0]];
    ele_in2 = rcSbusOutChannels[config.gyroChan[1]];
    rud_in2 = rcSbusOutChannels[config.gyroChan[2]];

    int16_t stick_gain[3];
    int16_t master_gain;
    uint8_t i;

    // stabilization mode
    enum STAB_MODE stab_mode2 = 
      (stab_mode == STAB_HOLD && aux_in2 <= RX_WIDTH_MODE_MID - RX_MODE_HYSTERESIS) ? STAB_RATE : 
      (stab_mode == STAB_RATE && aux_in2 >= RX_WIDTH_MODE_MID + RX_MODE_HYSTERESIS) ? STAB_HOLD : stab_mode; // hysteresis, all now in Hold Mode region

    if (stab_mode2 != stab_mode) {
        stab_mode = stab_mode2;
        // set_led_msg(1, (stab_mode == STAB_RATE) ? 0 : 4, LED_SHORT);  To do later on (led management)

        // reset attitude error and i_limit threshold on mode change
        for (i=0; i<3; i++) {
            pid_state.sum_err[i] = 0;
            pid_state.i_limit[i] = 0;
        }

        /* to do : Undestand if this code is useful or not. I expect not if the original rc channel ail,Elc,Rud are provided without trim.
        // check for inflight rx calibration
        if (cfg.inflight_calibrate == INFLIGHT_CALIBRATE_ENABLE) {
            if ((int32_t)(t - last_stab_mode_time) > 500000L) {
            stab_mode_count = 0;
            }
            last_stab_mode_time = t;

            if (++stab_mode_count >= 3) {
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
    if (stab_mode == STAB_HOLD || 
        (stab_mode == STAB_RATE && config.rate_mode_stick_rotate == RATE_MODE_STICK_ROTATE_ENABLE)) {
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
        
    if (stab_mode == STAB_HOLD) {
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
    pid_state.input[0] = constrain(gx, -8192, 8191);  // changed by Mstrens ; to do check if this is OK
    pid_state.input[1] = constrain(gy, -8192, 8191);
    pid_state.input[2] = constrain(gz, -8192, 8191);        
    
    // apply PID control
    compute_pid(&pid_state, (stab_mode == STAB_RATE) ? &config.pid_param_rate : &config.pid_param_hold);

    // apply vr_gain, stick_gain and master_gain
    for (i=0; i<3; i++) {
        // vr_gain [-128,0,127]/128, stick_gain [0,400,0]/512, master_gain [400,0,400]/512
        //correction[i] = ((((int32_t)pid_state.output[i] * vr_gain[i] >> 7) * stick_gain[i]) >> 9) * master_gain >> 9;
        correction[i] = ((((int32_t)pid_state.output[i] * config.vr_gain[i] >> 7) * stick_gain[i]) >> 9) * master_gain >> 9;
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
    // to do : check how to manage the fact that Rc channels can be provided by the Rx at a different rate than the PID frequency
    // here we still have to apply the correction to all the rc channels having the flag "used" = true and to stored the values in other variables
}