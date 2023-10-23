#include "gyro.h"
#include "param.h"


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