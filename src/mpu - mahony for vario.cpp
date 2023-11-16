#ifdef MAHONY_USED_INITIALLY_FOR_VARIO
// kalmanfilter4D algorithm is based on the code from https://github.com/har-in-air/ESP32_IMU_BARO_GPS_VARIO

//#include "hardware/watchdog.h"
#include "param.h"
#include "tools.h"
#include "KalmanFilter.h"
#include "KalmanFilter4D.h"
#include "MS5611.h"
#include "vario.h"
#include "config.h"
#include "I2Cdev.h"
#include "helper_3dmath.h"
#include "mpu.h"
#include "pico/util/queue.h"


extern CONFIG config;
//extern MS5611 baro1;    // class to handle MS5611; adress = 0x77 or 0x76
extern VARIO vario1;

extern queue_t qSendCmdToCore1;

float zTrack ;
float vTrack ;
float prevVTrack; // use to check hysteresis.
float sumAccZ = 0; // used to calculate an average of AccZ
uint32_t countAccZ=0;

uint32_t lastKfUs = microsRp(); 
uint32_t kfUs ;

MPU::MPU(int a)
{
}


#define PI 3.1416
#define RAD_TO_DEGREE 57.296 //180 / 3.1416

//float A_cal[6] = {335.0, 79.0, 1132.0, 1.0, 1.000, 1.0}; // 0..2 offset xyz, 3..5 scale xyz
//float G_off[3] = { 70.0, -13.0, -9.0}; //raw offsets, determined for gyro at rest
float gscale = ((250./32768.0)*(PI/180.0));   //gyro default 250 LSB per d/s -> rad/s
uint8_t accScaleCode ;
float accScale1G ;  

// ^^^^^^^^^^^^^^^^^^^ VERY VERY IMPORTANT ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


// GLOBALLY DECLARED, required for Mahony filter
// vector to hold quaternion
float qh[4] = {1.0, 0.0, 0.0, 0.0};

// Free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
float Kp = 30.0; // in github.com/har-in-air/ESP32_IMU_BARO_GPS_VARIO.blob/master it is set on 10
float Ki = 0.0;  // on same site, it is set on 0


Quaternion qq;                // quaternion
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorFloat gravity;          // added by mstrens to calculate Z world acc
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements

void MPU::begin()  // initialise MPU6050 
{
    if ( config.pinScl == 255 or config.pinSda == 255) return; // skip if pins are not defined
    #ifdef DEBUG  
    printf("Trying to detect MPU6050 sensor at I2C Addr=%X\n", MPU6050_DEFAULT_ADDRESS);
    #endif

    #ifndef ACC_MAX_SCALE_G
    #error "ACC_MAX_SCALE_G must be defined in config.h"
    #endif
    #if (ACC_MAX_SCALE_G == 2)
    accScaleCode = MPU6050_ACCEL_FS_2;
    accScale1G = 16384.0;
    #elif (ACC_MAX_SCALE_G == 4)
    accScaleCode = MPU6050_ACCEL_FS_4;
    accScale1G = 16384.0/2.0;
    #elif (ACC_MAX_SCALE_G == 8)
    accScaleCode = MPU6050_ACCEL_FS_8;
    accScale1G = 16384.0/4.0;
    #elif (ACC_MAX_SCALE_G == 16)
    accScaleCode = MPU6050_ACCEL_FS_16;
    accScale1G = 16384.0/8.0;
    #else
    #error "ACC_MAX_SCALE_G must be equal to 2,4,8 or 16"
    #endif

    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    if ( i2c_write_timeout_us(i2c1, MPU6050_DEFAULT_ADDRESS , buf, 2, false, 1000) <0) {
        printf("Write error for msp6050 on reset\n");
        return ;
    }
    sleep_us(100);
    mpu6050.initialize();
    
    mpu6050.setDLPFMode(MPU6050_DLPF_BW_188);
    //mpu6050.setDLPFMode(MPU6050_DLPF_BW_10);
    //printf("dlpfMode= %d\n", mpu6050.getDLPFMode() );
    //printf("rate= %d\n", mpu6050.getRate() );
     
    
    //mpu6050.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    //mpu6050.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    //mpu6050.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    //mpu6050.setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!
    
    //printf("Acc & gyro before calibration\n");
    //mpu6050.PrintActiveOffsets() ;
    //mpu6050.CalibrateGyro(6);
    //mpu6050.CalibrateAccel(6);
    //printf("Acc & gyro after calibration\n");
    //mpu6050.PrintActiveOffsets() ;
    
    mpuInstalled =  true;
    //printf("MPU MAP initialized\n");
    kalmanFilter4d_configure(1000.0f*(float)KF_ACCEL_VARIANCE_DEFAULT, KF_ADAPT, 0.0f, 0.0f, 0.0f);

}


void MPU::testDevicesOffsetX(){
    sleep_ms(1000);
    //int16_t gyroX[16];
    printf("with existing offset = %i\n", mpu6050.getXGyroOffset());
    for (uint8_t i=0;i<16;i++)  printf("%d\n", mpu6050.getRotationX()); // get 16 values
    mpu6050.setXGyroOffset(0);
    printf("with offset = 0\n");
    for (uint8_t i=0;i<16;i++)  printf("%d\n", mpu6050.getRotationX()); // get 16 values
    mpu6050.setXGyroOffset(66);
    printf("with offset = 66\n");    
    for (uint8_t i=0;i<16;i++) printf("%d\n", mpu6050.getRotationX()); // get 16 values
}

void MPU::calibrationExecute()  // 
{
    //sleep_ms(2000); // delay to allow core0 to print the config.
    //printf("Acc & gyro before calibration\n");
    //printConfigOffsets() ;
    uint8_t buf[] = {0x6B, 0x00};
    if ( i2c_write_timeout_us(i2c1, MPU6050_DEFAULT_ADDRESS , buf, 2, false,1000) <0) {
        printf("Write error for msp6050 on begin of calibration\n");
        return ;
    }
    sleep_us(100);
    mpu6050.initialize();
    
    //sleep_us(100);
    // not used anymore with new calibration
    //mpu6050.CalibrateGyro(6);
    //mpu6050.CalibrateAccel(6);
    // put the offsets in config.h
    //config.accOffsetX = mpu6050.getXAccelOffset();
    //config.accOffsetY = mpu6050.getYAccelOffset();
    //config.accOffsetZ = mpu6050.getZAccelOffset();
    //config.gyroOffsetX = mpu6050.getXGyroOffset();
    //config.gyroOffsetY = mpu6050.getYGyroOffset();
    //config.gyroOffsetZ = mpu6050.getZGyroOffset();
    
    //printf("Acc & gyro after old calibration\n");
    //printConfigOffsets() ;
    if (calibrateAccelGyro() ) { // fill config offsets with the retrieved values
        //printf("Acc & gyro after new calibration\n");
        //printConfigOffsets() ;
        
        //printf("Calibration done: parameters will be saved\n");
        //sleep_ms(3000);
        sent2Core0(0XFF, 0XFFFFFFFF); // use a dummy type to give a command; here a cmd to save the config
    } else {
        printf("error while calibrating mp6050\n");
    }
    //mpu6050.PrintActiveOffsets() ;
    //testDevicesOffsetX();

}


//--------------------------------------------------------------------------------------------------
// Mahony scheme uses proportional and integral filtering on
// the error between estimated reference vector (gravity) and measured one.
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date      Author      Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
// last update 07/09/2020 SJR minor edits
//--------------------------------------------------------------------------------------------------
// IMU algorithm update

void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez;  //error terms
    float qa, qb, qc;
    static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
    float tmp;
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    tmp = ax * ax + ay * ay + az * az;
    if (tmp > 0.0) {
        // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
        recipNorm = 1.0 / sqrt(tmp);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        // Estimated direction of gravity in the body frame (factor of two divided out)
        vx = qh[1] * qh[3] - qh[0] * qh[2];
        vy = qh[0] * qh[1] + qh[2] * qh[3];
        vz = qh[0] * qh[0] - 0.5f + qh[3] * qh[3];
        // Error is cross product between estimated and measured direction of gravity in body frame
        // (half the actual magnitude)
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);
        // Compute and apply to gyro term the integral feedback, if enabled
        if (Ki > 0.0f) {
        ix += Ki * ex * deltat;  // integral error scaled by Ki
        iy += Ki * ey * deltat;
        iz += Ki * ez * deltat;
        gx += ix;  // apply integral feedback
        gy += iy;
        gz += iz;
        }
        // Apply proportional feedback to gyro term
        gx += Kp * ex;
        gy += Kp * ey;
        gz += Kp * ez;
    }
    // Integrate rate of change of quaternion, q cross gyro term
    deltat = 0.5 * deltat;
    gx *= deltat;   // pre-multiply common factors
    gy *= deltat;
    gz *= deltat;
    qa = qh[0];
    qb = qh[1];
    qc = qh[2];
    qh[0] += (-qb * gx - qc * gy - qh[3] * gz);
    qh[1] += (qa * gx + qc * gz - qh[3] * gy);
    qh[2] += (qa * gy - qb * gz + qh[3] * gx);
    qh[3] += (qa * gz + qb * gy - qc * gx);
    // renormalise quaternion
    tmp = qh[0] * qh[0] + qh[1] * qh[1] + qh[2] * qh[2] + qh[3] * qh[3];
    if ( tmp == 0 ) return;
    recipNorm = 1.0 / sqrt(tmp);
    qh[0] = qh[0] * recipNorm;
    qh[1] = qh[1] * recipNorm;
    qh[2] = qh[2] * recipNorm;
    qh[3] = qh[3] * recipNorm;
}

void orientateMpu(int16_t &ax, int16_t &ay, int16_t &az , int16_t &gx ,int16_t &gy, int16_t &gz){
    int16_t temp;
    switch (config.mpuOrientation){  
        case DOWN_FRONT: // case 0 : normal case; Y point forward, X to the right and Z up
            break;
        case UP_FRONT: //= 4: Y point forward, X point to the left reverse), Z point down (reverse)
            ax=-ax;
            az=-az;
            gx=-gy;
            gz=-gz;
            break;
        default:
            break;
    }
}

bool MPU::getAccZWorld(){ // return true when a value is available ; read the IMU and calculate the acc on Z axis (world)
    if (!mpuInstalled) {
        return false;
    }
    uint8_t buffer[14];
    int16_t ax, ay, az; // accel from MPU (raw values)
    int16_t gx, gy, gz; // gyro from MPU (raw values)
    
    static float deltat = 0;  //loop time in seconds
    static unsigned long now = 0, last = 0; //microsRp() timers
    static int32_t sumAx;
    static int32_t sumAy;
    static int32_t sumAz;
    static int32_t countSumAcc;

    static uint32_t now_ms = 0;
    static uint32_t lastRollPitchMs = 0; //millisRp() timers
    static uint32_t lastAccXYZMs = 0;

    //static float sumAz;
    static float azWorldAverage;
    //static float azAverage; 
    //static float vSpeedAcc = 0;
    //static float vSpeedAccHighPass;
    //static float vSpeedAccLow ;
    //static float vSpeedAcc2;
    //static float accLow;
    //static float accHigh;
    //static float vSpeedAcc3;
    //static float lowPass = 0;        //initialization of EMA S
    static uint32_t lastMpuUs;
    if ( ( microsRp() - lastMpuUs) < 2000) return false ; // perform calculation only every 2 msec 
    lastMpuUs = microsRp();
 
    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    if (i2c_write_timeout_us(i2c1, MPU6050_DEFAULT_ADDRESS, &val, 1, true, 1000)< 0) { // true to keep master control of bus
        printf("Write error for MPU6050\n");
        return false;
    }
    if ( i2c_read_timeout_us(i2c1, MPU6050_DEFAULT_ADDRESS, buffer, 14, false, 2500) <0){
        printf("Read error for MPU6050\n");
        return false;
    }     
    ax = ((int16_t) (buffer[0] << 8 | buffer[1])) - config.accOffsetX ;
    ay = ((int16_t) (buffer[2] << 8 | buffer[3])) - config.accOffsetY ;
    az = ((int16_t) (buffer[4] << 8 | buffer[5])) - config.accOffsetZ ;
    //printf("az=%.0f\n", (float) az ); 
    gx = ((int16_t) (buffer[8] << 8 | buffer[9])) - config.gyroOffsetX ;
    gy = ((int16_t) (buffer[10] << 8 | buffer[11])) - config.gyroOffsetY ;
    gz = ((int16_t) (buffer[12] << 8 | buffer[13])) - config.gyroOffsetZ ;

    //orientateMpu(ax, ay, az , gx ,gy, gz);

    sumAx += ax;  // prepare calculation of averages
    sumAy += ay;
    sumAz += az;
    countSumAcc++;

    if ((config.gyroChanControl > 0) and (config.gyroChanControl <= 16) ) { // when gyro is used
        sent2Core0( GYRO_X_ID , (int32_t) gx) ;
        sent2Core0( GYRO_Y_ID , (int32_t) gy) ;
        sent2Core0( GYRO_Z_ID , (int32_t) gz) ;
    }
    now = microsRp();
    deltat = ((float)(now - last))* 1.0e-6; //seconds since last update
    last = now;
    Mahony_update( (float) ax, (float) ay, (float) az , ((float) gx) * gscale, ((float) gy) * gscale , ((float) gz) * gscale, deltat);
    // this formula to calculate vertical accz comes from
    // https://github.com/har-in-air/ESP32_IMU_BARO_GPS_VARIO/blob/b0157b91902fe4d2de13a3198160438307d71662/src/sensor/imu.cpp
    // in imu_gravityCompensatedAccel()
    // and gives the same result as previous formula
    float accVert = 2.0*(qh[1]*qh[3] - qh[0]*qh[2])*( (float) ax) + 2.0f*(qh[0]*qh[1] + qh[2]*qh[3])*((float)ay)
     + (qh[0]*qh[0] - qh[1]*qh[1] - qh[2]*qh[2] + qh[3]*qh[3])*((float)az) - accScale1G; // scale =16384 for 1g when max is 2g (to be substracted)
    sumAccZ += accVert;
    //printf("aaw=%.0f acc=%.0f  az=%.0f\n", (float) aaWorld.z , accVert, (float) az);
    //acc *= 0.9807f; // in cm/s/s, assuming ax, ay, az are in milli-Gs
    countAccZ++;
        //printf("az azworld %d %d %d\n", az +accScale1G, aaWorld.z, (int32_t) deltat *1000000 ); 
    // here above is executed nearly once per millisec 
    // roll and pitch formula are taken from this link https://www.youtube.com/watch?v=k5i-vE5rZR0 (at minute 27:00)
    // here Y axis is for roll (Y point forward) and X axis for pitch (X point to right); then for MPU6050 Z point to up.
    // this gives wrong results (e.g. does not get up to 90° on roll)
    //roll  = RAD_TO_DEGREE * atan2(2*(qh[0] * qh[2] - 2* qh[1] * qh[3]), (qh[1] * qh[1] + qh[2] * qh[2] + qh[3] * qh[3] + qh[0] * qh[0]));
    //pitch = RAD_TO_DEGREE * asin(2.0 * ( qh[2] * qh[3] + qh[0] * qh[1]));
    
    // there is also this site :https://github.com/nickrehm/dRehmFlight
    //roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
    //pitch_IMU = -asin(-2.0f * (q1*q3 - q0*q2))*57.29577951; //degrees
    roll  = RAD_TO_DEGREE * atan2((qh[0] * qh[1] + qh[2] * qh[3]), 0.5 - (qh[1] * qh[1] + qh[2] * qh[2]));
    pitch = -(RAD_TO_DEGREE * asin(2.0 * ( qh[0] * qh[2] - qh[1] * qh[3])));
    

    // originally it was
    //roll  = RAD_TO_DEGREE * atan2((qh[0] * qh[1] + qh[2] * qh[3]), 0.5 - (qh[1] * qh[1] + qh[2] * qh[2]));
    //pitch = RAD_TO_DEGREE * asin(2.0 * ( qh[0] * qh[2] - qh[1] * qh[3]));
    sent2Core0( CAMERA_PITCH_ID , (int32_t) (pitch * 10.0)) ;
    sent2Core0( CAMERA_ROLL_ID , (int32_t) (roll * 10.0)) ;
    if (vario1.newClimbRateAvailableForMpu){   // here once per about 20 msec
        vario1.newClimbRateAvailableForMpu = false; // reset the flag that says a new relative alt is available
        azWorldAverage = (sumAccZ/countAccZ);
        kfUs = microsRp(); 
        kalmanFilter4d_predict( ((float) (kfUs-lastKfUs )) /1000000.0f);
        lastKfUs = kfUs;  
        kalmanFilter4d_update( (float) vario1.rawRelAltitudeCm , (float) azWorldAverage /accScale1G * 981.0 , (float*) &zTrack , (float*)&vTrack);
        //printf("Vv4 Vk4  %d %d 50 -50\n", (int32_t) vario1.climbRateFloat ,  (int32_t) (float) vTrack);
        //printf("Va4 Va4  %d %d 50 -50\n", (int32_t) vario1.relativeAlt ,  (int32_t) (float) zTrack);
        sumAccZ= 0;
        countAccZ=0;
        //sumAz = 0;
        if ( abs((vTrack - prevVTrack) ) >  VARIOHYSTERESIS ) {
            prevVTrack = vTrack ;
        }
        vario1.compensatedVpseed =  (int32_t) prevVTrack ; // we save it here first, so we can reuse this field for compensated Vspeed when it is disabled  
        sent2Core0( VSPEED , (int32_t) vario1.compensatedVpseed) ;     
    }
    now_ms = millisRp(); //time to send roll and pitch
    if (now_ms - lastRollPitchMs >= 500) {
        lastRollPitchMs = now_ms;
        sent2Core0( PITCH , (int32_t) pitch * 100 ) ; 
        sent2Core0( ROLL , (int32_t) roll * 100 ) ; 
        // print angles for serial plotter...
        #ifdef DEBUG
        //printf("pitch, roll, acc: %6.0f %6.0f %6.0f %6.0f\n", pitch, roll, vTrack, vario1.climbRateFloat);//  Serial.print("ypr ");
        #endif
    }
    if (now_ms - lastAccXYZMs >= 200) {
        lastAccXYZMs = now_ms;
        // * 1000 because sport is in mg; mpu is 16 bits = + or - 32768; when ACC max is +/-2g, it gives 16384 steps / g 
        sent2Core0( ACC_X , (int32_t) (sumAx / countSumAcc * 1000 / (int) accScale1G) ) ;  
        sent2Core0( ACC_Y , (int32_t) (sumAy / countSumAcc * 1000 / (int) accScale1G) ) ;  
        sent2Core0( ACC_Z , (int32_t) (sumAz / countSumAcc * 1000 / (int) accScale1G) ) ;
        sumAx  = 0;
        sumAy  = 0;
        sumAz  = 0;
        countSumAcc = 0; 
    }

    return false; 
}


void MPU::printOffsets() {
    printf("acc = %d    %d   %d\n", config.accOffsetX , config.accOffsetY , config.accOffsetZ);
    printf("gyro= %d    %d   %d\n", config.gyroOffsetX , config.gyroOffsetY , config.gyroOffsetZ);
}


#define ACCEL_NUM_AVG_SAMPLES	50
bool MPU::calibrateAccelGyro(void){
	int16_t ax,ay,az, az1g ;
    int16_t gx,gy,gz ;
	int32_t axAccum, ayAccum, azAccum;
	axAccum = ayAccum = azAccum = 0;
    int32_t gxAccum, gyAccum, gzAccum;
	gxAccum = gyAccum = gzAccum = 0;
	// use a lower dlpf
    uint8_t buffer[2] = {MPU6050_RA_CONFIG , MPU6050_DLPF_BW_20};
    if( i2c_write_timeout_us(i2c1, MPU6050_DEFAULT_ADDRESS, &buffer[0], 2, false,3000)<0){ // true to keep master control of bus
        printf("Write error for MPU6050 calibration DLPF\n");
        return false;
    }   
    sleep_ms(10);
    for (int inx = 0; inx < ACCEL_NUM_AVG_SAMPLES; inx++){
        sleep_ms(10); // take 8 msec with dlpf = 20 
        // Start reading acceleration registers from register 0x3B for 14 bytes (acc, temp, gyro)
        uint8_t val = 0x3B;
        uint8_t buffer[14]; 
        if (i2c_write_timeout_us(i2c1, MPU6050_DEFAULT_ADDRESS, &val, 1, true,1000)<0) { // true to keep master control of bus
            printf("Write error for MPU6050 calibration at 0X3B\n");
            return false;
        }   
        if ( i2c_read_timeout_us(i2c1, MPU6050_DEFAULT_ADDRESS, buffer, 14, false, 3500) <0){
            printf("Read error for MPU6050 calibration\n");
            return false;
        }     
        ax= (buffer[0] << 8 | buffer[1]);
        ay= (buffer[2] << 8 | buffer[3]);
        az= (buffer[4] << 8 | buffer[5]);
        //printf("az=%.0f\n", (float) az ); 
        gx= (buffer[8] << 8 | buffer[9]);
        gy= (buffer[10] << 8 | buffer[11]);
        gz= (buffer[12] << 8 | buffer[13]);
        axAccum += (int32_t) ax;
        ayAccum += (int32_t) ay;
        azAccum += (int32_t) az;
        //printf("az=%.0f\n", (float) az ); 
        gxAccum += gx;
        gyAccum += gy;
        gzAccum += gz;
    }
	config.accOffsetX = (int16_t)(axAccum / ACCEL_NUM_AVG_SAMPLES);
	config.accOffsetY = (int16_t)(ayAccum / ACCEL_NUM_AVG_SAMPLES);
	az1g = (int16_t)(azAccum / ACCEL_NUM_AVG_SAMPLES);
    config.accOffsetZ = az1g > 0 ? az1g - (int16_t) accScale1G : az1g + (int16_t) accScale1G ; // 16384 = 1G when max is 2g
    config.gyroOffsetX = (int16_t)(gxAccum / ACCEL_NUM_AVG_SAMPLES);
	config.gyroOffsetY = (int16_t)(gyAccum / ACCEL_NUM_AVG_SAMPLES);
	config.gyroOffsetZ = (int16_t)(gzAccum / ACCEL_NUM_AVG_SAMPLES);
    // restore dlpf
    uint8_t buffer2[2] = {MPU6050_RA_CONFIG , MPU6050_DLPF_BW_188};
    if( i2c_write_timeout_us(i2c1, MPU6050_DEFAULT_ADDRESS, &buffer2[0], 2, false,30000)<0){ // true to keep master control of bus
        printf("Write error for MPU6050 calibration DLPF\n");
        return false;
    }   
    sleep_ms(10);
    return true;
}
#endif