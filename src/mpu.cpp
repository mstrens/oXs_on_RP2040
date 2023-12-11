
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

bool calibrateImuGyro ; // recalibrate the gyro or not at reset (avoid it after a watchdog reset)

float zTrack ;
float vTrack ;
float prevVTrack; // use to check hysteresis.
float sumAccZ = 0; // used to calculate an average of AccZ
uint32_t countAccZ=0;

uint32_t lastKfUs = microsRp(); 
uint32_t kfUs ;

uint8_t orientationX; // contain the index 0,1, 2 of aRaw[] and gRaw[] to be moved in oax and ogx
uint8_t orientationY; // idem for oay and ogy
uint8_t orientationZ; // idem for oaz and ogz
int8_t signX;         // contains the sign (1 or -1 ) to apply to oax and ogx
int8_t signY;         // idem for oay and ogy
int8_t signZ;         // idem for oaz and ogz
bool orientationIsWrong;  // flag to sat that orientation is wrong and so avoid any process od raw data

const char* mpuOrientationNames[8] = {\
    "FRONT(X+)", "BACK(X-)", "LEFT(Y+)", "RIGHT(Y-)", "UP(Z+)", "DOWN(Z-)", "WRONG", "WRONG"};

int8_t orientationList[36][6] = {
{3,3,3,0,0,0}, {3,3,3,0,0,0}, {1,2,0,1,1,1}, {1,2,0,-1,-1,1}, {2,1,0,1,-1,1}, {2,1,0,-1,1,1},\

{3,3,3,0,0,0}, {3,3,3,0,0,0}, {1,2,0,1,-1,-1}, {1,2,0,-1,1,-1}, {2,1,0,1,1,-1}, {2,1,0,-1,-1,-1},\

{0,2,1,1,-1,1}, {0,2,1,-1,1,1}, {3,3,3,0,0,0}, {3,3,3,0,0,0}, {2,0,1,1,1,1}, {2,0,1,-1,-1,1},\

{0,2,1,1,1,-1}, {0,2,1,-1,-1,-1}, {3,3,3,0,0,0}, {3,3,3,0,0,0}, {2,0,1,1,-1,-1}, {2,0,1,-1,1,-1},\

{0,1,2,1,1,1}, {0,1,2,-1,-1,1}, {1,0,2,1,-1,1}, {1,0,2,-1,1,1}, {3,3,3,0,0,0}, {3,3,3,0,0,0},\

{0,1,2,1,-1,-1}, {0,1,2,-1,1,-1}, {1,0,2,1,1,-1}, {1,0,2,-1,-1,-1}, {3,3,3,0,0,0}, {3,3,3,0,0,0}};


#define PI 3.1416
#define RAD_TO_DEGREE 57.296 //180 / 3.1416

//float A_cal[6] = {335.0, 79.0, 1132.0, 1.0, 1.000, 1.0}; // 0..2 offset xyz, 3..5 scale xyz
//float G_off[3] = { 70.0, -13.0, -9.0}; //raw offsets, determined for gyro at rest
//float gscale = ((250./32768.0)*(PI/180.0));   //gyro default 250 LSB per d/s -> rad/s
uint8_t accScaleCode ;
float accScale1G ;  //divide by this to get number of g 
uint8_t gyroScaleCode ; 
float gyroScaleDegree;  //   multiply adc by this to get °/s
float gyroScaleRad;     //   multiply adc by this to get rad/s

// ^^^^^^^^^^^^^^^^^^^ VERY VERY IMPORTANT ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


// GLOBALLY DECLARED, required for Mahony filter
// vector to hold quaternion ; qh[0]=scalar, qh[1]=x, ...
float qh[4] = {1.0, 0.0, 0.0, 0.0};

// Free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
float Kp = 5.0; // in github.com/har-in-air/ESP32_IMU_BARO_GPS_VARIO.blob/master it is set on 10
float Ki = 0.0;  // on same site, it is set on 0


Quaternion qq;                // quaternion
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorFloat gravity;          // added by mstrens to calculate Z world acc
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements

MPU::MPU(int a) {}

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

    #if (GYRO_MAX_SCALE_DEGREE == 250)
    gyroScaleCode = MPU6050_GYRO_FS_250; //0
    gyroScaleRad = 250.0/ 32768.0 / 180 * PI ;  //   multiply adc by this to get rad°/s 
    #elif (GYRO_MAX_SCALE_DEGREE == 500)
    gyroScaleCode = MPU6050_GYRO_FS_500;  // 1
    gyroScaleRad = 500.0/32768.0 / 180 * PI ;  //   multiply adc by this to get rad°/s
    #elif (GYRO_MAX_SCALE_DEGREE == 1000)
    gyroScaleCode = MPU6050_GYRO_FS_1000;  //2
    gyroScaleRad = 1000.0 / 32768.0 / 180 * PI ;  //   multiply adc by this to get rad°/s
    #elif (GYRO_MAX_SCALE_DEGREE == 2000)
    gyroScaleCode = MPU6050_GYRO_FS_2000;  //3
    gyroScaleRad = 2000.0 / 32768.0 / 180 * PI;  //   multiply adc by this to get rad°/s
    #else
    #error "GYRO_MAX_SCALE_DEGREE must be equal to 250, 500, 1000, 2000"
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
    setupOrientation(); // based on config.mpuOrientation fill orientationX,Y,Z and signX, Y, Z and orientationIsWrong
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

void MPU::calibrationHorizontalExecute()  // 
{
    printf("Before calibration:");
    printConfigOffsets();
    //            sleep_ms(1000); // wait that message is printed  
       
    //uint8_t buf[] = {0x6B, 0x00};
    //if ( i2c_write_timeout_us(i2c1, MPU6050_DEFAULT_ADDRESS , buf, 2, false,1000) <0) {
    //    printf("Write error for msp6050 on begin of calibration\n");
    //    return ;
    //}
    //sleep_us(100);
    //mpu6050.initialize();
    #define ACCEL_NUM_AVG_SAMPLES	1000
	
    int16_t ax,ay,az ;
    int16_t gx,gy,gz ;
	int32_t axAccum, ayAccum, azAccum , axMin , axMax , ayMin , ayMax , azMin , azMax;
	axAccum = ayAccum = azAccum = 0;
    int32_t gxAccum, gyAccum, gzAccum , gxMin , gxMax , gyMin , gyMax , gzMin , gzMax;
	gxAccum = gyAccum = gzAccum = 0;
    axMin = ayMin = azMin = gxMin = gyMin = gzMin = 60000;
    axMax = ayMax = azMax = gxMax = gyMax = gzMax = -60000;  
	// use a lower dlpf
    //uint8_t buffer[2] = {MPU6050_RA_CONFIG , MPU6050_DLPF_BW_20};
    //if( i2c_write_timeout_us(i2c1, MPU6050_DEFAULT_ADDRESS, &buffer[0], 2, false,3000)<0){ // true to keep master control of bus
    //    printf("Write error for MPU6050 calibration DLPF\n");
    //    return false;
    //}   
    //sleep_ms(10);
    for (int inx = 0; inx < ACCEL_NUM_AVG_SAMPLES; inx++){
        sleep_us(2000); // take 8 msec with dlpf = 20 ; 1900us when BW = 188
        // Start reading acceleration registers from register 0x3B for 14 bytes (acc, temp, gyro)
        uint8_t val = 0x3B;
        uint8_t buffer[14]; 
        if (i2c_write_timeout_us(i2c1, MPU6050_DEFAULT_ADDRESS, &val, 1, true,1000)<0) { // true to keep master control of bus
            printf("Write error for MPU6050 calibration at 0X3B command\n");
            return;
        }
        if ( i2c_read_timeout_us(i2c1, MPU6050_DEFAULT_ADDRESS, buffer, 14, false, 3500) <0){
            printf("Read error for MPU6050 calibration\n");
            return;
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
        gxAccum += (int32_t) gx;
        gyAccum += (int32_t) gy;
        gzAccum += (int32_t) gz;
        if (ax < axMin) axMin = ax;
        if (ay < ayMin) ayMin = ay;
        if (az < azMin) azMin = az;
        if (ax > axMax) axMax = ax;
        if (ay > ayMax) ayMax = ay;
        if (az > azMax) azMax = az;
        if (gx < gxMin) gxMin = gx;
        if (gy < gyMin) gyMin = gy;
        if (gz < gzMin) gzMin = gz;
        if (gx > gxMax) gxMax = gx;
        if (gy > gyMax) gyMax = gy;
        if (gz > gzMax) gzMax = gz;
    }
    // here we know the offsets but still have to identify the gravity, set mpuOrientationH and take gravity out of offset 
    #define MAX_ACC_DIFF 500
    if (((axMax - axMin) > MAX_ACC_DIFF) or ((ayMax - ayMin) > MAX_ACC_DIFF) or ((azMax - azMin) > MAX_ACC_DIFF)) {
        printf ("Error in IMU calibration: to much variations in the acceleration values\n");
        return;
    }
    #define MAX_GYRO_DIFF 200
    if (((gxMax - gxMin) > MAX_GYRO_DIFF) or ((gyMax - gyMin) > MAX_GYRO_DIFF) or ((gzMax - gzMin) > MAX_GYRO_DIFF)) {
        printf ("Error in IMU calibration: to much variations in the gyro rates values\n");
        return;
    }
    axAccum /= (ACCEL_NUM_AVG_SAMPLES);
    ayAccum /= (ACCEL_NUM_AVG_SAMPLES);
    azAccum /= (ACCEL_NUM_AVG_SAMPLES);
    gxAccum /= (ACCEL_NUM_AVG_SAMPLES);
	gyAccum /= (ACCEL_NUM_AVG_SAMPLES);
	gzAccum /= (ACCEL_NUM_AVG_SAMPLES);
    uint8_t idx = 6; 
    findGravity( axAccum , ayAccum , azAccum , idx);
	switch (idx) {
        case 0 :
            axAccum -= (int32_t) accScale1G;
            break;
        case 1 :
            axAccum += (int32_t) accScale1G;
            break;
        case 2 :
            ayAccum -= (int32_t) accScale1G;
            break;
        case 3 :
            ayAccum += (int32_t) accScale1G;
            break;
        case 4 :
            azAccum -= (int32_t) accScale1G;
            break;
        case 5 :
            azAccum += (int32_t) accScale1G;
            break; 
        case 6 :
            printf("Error during calibration : gravity direction not found based on Accelerometer\n");
            return;
            break; 
    }
    printf("Upper face is "); printf(mpuOrientationNames[idx]);
    config.accOffsetX = (int16_t)(axAccum);
	config.accOffsetY = (int16_t)(ayAccum);
	config.accOffsetZ = (int16_t)(azAccum);
    config.gyroOffsetX = (int16_t)(gxAccum);
	config.gyroOffsetY = (int16_t)(gyAccum);
	config.gyroOffsetZ = (int16_t)(gzAccum);
    config.mpuOrientationH =  idx ; // save the orientationH
    printf("Acc & gyro after new calibration\n");
    printConfigOffsets() ;
    printf("Horizontal calibration done: use SAVE command to save the config!!\n");
    setupOrientation();  // refresh the parameters linked to the orientation        
    //sent2Core0(0XFF, 0XFFFFFFFF); // use a dummy type to give a command; here a cmd to save the config
    
    // restore dlpf
    //uint8_t buffer2[2] = {MPU6050_RA_CONFIG , MPU6050_DLPF_BW_188};
    //if( i2c_write_timeout_us(i2c1, MPU6050_DEFAULT_ADDRESS, &buffer2[0], 2, false,30000)<0){ // true to keep master control of bus
    //    printf("Write error for MPU6050 calibration DLPF\n");
    //    return false;
    //}   
    //sleep_ms(10);
}

void MPU::calibrationVerticalExecute() {
    // read the Acc and detect which face is on the upper side
    int16_t ax,ay,az ;
    int16_t gx,gy,gz ;
	    // Start reading acceleration registers from register 0x3B for 14 bytes (acc, temp, gyro)
        uint8_t val = 0x3B;
        uint8_t buffer[14]; 
        if (i2c_write_timeout_us(i2c1, MPU6050_DEFAULT_ADDRESS, &val, 1, true,1000)<0) { // true to keep master control of bus
            printf("Write error for MPU6050 calibration at 0X3B\n");
            return;
        }
        if ( i2c_read_timeout_us(i2c1, MPU6050_DEFAULT_ADDRESS, buffer, 14, false, 3500) <0){
            printf("Read error for MPU6050 calibration\n");
            return;
        }     
        ax= (buffer[0] << 8 | buffer[1]);
        ay= (buffer[2] << 8 | buffer[3]);
        az= (buffer[4] << 8 | buffer[5]);
    uint8_t idx = 6; 
    findGravity( ax , ay , az , idx);
	if (idx > 5){
         printf("Error during vertical calibration: direction of gravity has not been found\n");
         return;
    }
    printf("Upper face with nose is "); printf(mpuOrientationNames[idx]);
    config.mpuOrientationV =  idx ; // save the orientationH
    printf("Vertical calibration done: use SAVE command to save it\n");
    setupOrientation();  // refresh the parameters linked to the orientation        
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



void setupOrientation(){
    uint8_t idx;
    orientationIsWrong = false;
    if ( config.mpuOrientationH>5 || config.mpuOrientationV>5 ) {
        orientationIsWrong = true;
        return;
    }
    idx = config.mpuOrientationH *6 + config.mpuOrientationV;  // into a number in range 0/35
    if (orientationList[idx][3] == 0){   // check that combination H and V is valid
        orientationIsWrong = true;
        return;
    }
    orientationX = orientationList[idx][0]; // orientation list contains e.g. 3,2,1,-1,1,1 (first are the index to map, last 3 the sign)
    orientationY = orientationList[idx][1];
    orientationZ = orientationList[idx][2];
    signX = orientationList[idx][3];
    signY = orientationList[idx][4];
    signZ = orientationList[idx][5];
}

void findGravity(int32_t ax, int32_t ay, int32_t az, uint8_t &idx ){
    // find the index and sign of gravity idx=0 is X, 1=Y, 2=Z; sign 1=gravity is the opposite (normally Z axis is up and give 1) 
    if ((float) ax >  (accScale1G*0.7)) { idx = 0 ;
    } else if ((float) ax < -(accScale1G*0.7)) { idx = 1 ;
    } else if ((float) ay >  (accScale1G*0.7)) { idx = 2 ;
    } else if ((float) ay < -(accScale1G*0.7)) { idx = 3 ;
    } else if ((float) az >  (accScale1G*0.7)) { idx = 4 ;
    } else if ((float) az < -(accScale1G*0.7)) { idx = 5 ;
    } else { idx= 6; };
    
    //printf("ax=%i  ay=%i  az=%i  yawIdx=%i   yawSign=%i  scale=%f\n", ax , ay ,  az, idx , sign , accScale1G);
}        

bool MPU::getAccZWorld(){ // return true when a value is available ; read the IMU and calculate the acc on Z axis (world)
    if (!mpuInstalled) {
        return false;
    }
    uint8_t buffer[14];
    int32_t aRaw[3]; // accel from MPU (raw values)
    int32_t gRaw[3]; // gyro from MPU (raw values)
    
    int16_t oax, oay, oaz; // accel from MPU (with offset and orientation)
    int16_t ogx, ogy, ogz; // gyro from MPU (with offset and orientation)
    
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
    if ( ( microsRp() - lastMpuUs) < 2000) return false ; // perform calculation only every 2 usec =  500 Hz 
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
    
    //ax = ((int16_t) (buffer[0] << 8 | buffer[1])) - config.accOffsetX ;
    //ay = ((int16_t) (buffer[2] << 8 | buffer[3])) - config.accOffsetY ;
    //az = ((int16_t) (buffer[4] << 8 | buffer[5])) - config.accOffsetZ ;
    //printf("az=%.0f\n", (float) az ); 
    //gx = ((int16_t) (buffer[8] << 8 | buffer[9])) - config.gyroOffsetX ;
    //gy = ((int16_t) (buffer[10] << 8 | buffer[11])) - config.gyroOffsetY ;
    //gz = ((int16_t) (buffer[12] << 8 | buffer[13])) - config.gyroOffsetZ ;

    aRaw[0] = ((int16_t) (buffer[0] << 8 | buffer[1])) ;
    aRaw[1] = ((int16_t) (buffer[2] << 8 | buffer[3])) ;
    aRaw[2] = ((int16_t) (buffer[4] << 8 | buffer[5])) ;
    //printf("az=%.0f\n", (float) az ); 
    gRaw[0] = ((int16_t) (buffer[8] << 8 | buffer[9])) ;
    gRaw[1] = ((int16_t) (buffer[10] << 8 | buffer[11])) ;
    gRaw[2] = ((int16_t) (buffer[12] << 8 | buffer[13])) ;

    
    #ifdef CALIBRATE_GYRO_ON_RESET
    #define NUMBER_ITER_CALIB 1000
    static uint32_t count = 0;
    static int32_t gSum[3] = {0, 0, 0};
    static int32_t gMin[3] = {60000 , 60000, 60000};
    static int32_t gMax[3] = {-60000 , -60000, -60000};
    
    if (calibrateImuGyro) {
        if (count < NUMBER_ITER_CALIB){
            count++;
            gSum[0] += gRaw[0]; gSum[1] += gRaw[1]; gSum[2] += gRaw[2];
            if (gRaw[0] < gMin[0]) gMin[0] = gRaw[0];
            if (gRaw[1] < gMin[1]) gMin[1] = gRaw[1];
            if (gRaw[2] < gMin[2]) gMin[2] = gRaw[2]; 
            if (gRaw[0] > gMax[0]) gMax[0] = gRaw[0];
            if (gRaw[1] > gMax[1]) gMax[1] = gRaw[1];
            if (gRaw[2] > gMax[2]) gMax[2] = gRaw[2]; 
            return false ; // skip rest of calcul
        } else if (count == NUMBER_ITER_CALIB){
            calibrateImuGyro = false; // avoid calibration (it is done)
            if ( ((gMax[0]-gMin[0]) > MAX_GYRO_DIFF) or ((gMax[1]-gMin[1]) > MAX_GYRO_DIFF) or((gMax[2]-gMin[2]) > MAX_GYRO_DIFF) ){
                printf("Automatic gyro calibration failed; oXs uses gyro offsets saved during horizontal calibration\n");
            } else {
                config.gyroOffsetX= (int16_t)(gSum[0] / 2000);  config.gyroOffsetY= (int16_t)(gSum[1]/2000); config.gyroOffsetZ= (int16_t)(gSum[2]/2000);
            //printf(" offset gx=%i  gy=%i   gz=%i\n",  (int)config.gyroOffsetX ,(int)config.gyroOffsetY,(int)config.gyroOffsetZ);
            }
            return false ; // skip rest of calcul
        }
    }    
    #endif

    if (orientationIsWrong){
        return false; // do not process when orientation is wrong
    }
    // we still have to apply offset and afterward the orientation parameters
    aRaw[0] -= config.accOffsetX ; aRaw[1] -= config.accOffsetY ; aRaw[2] -= config.accOffsetZ;
    gRaw[0] -= config.gyroOffsetX; gRaw[1] -= config.gyroOffsetY; gRaw[2] -= config.gyroOffsetZ;

    oax = aRaw[orientationX] * signX;
    oay = aRaw[orientationY] * signY;
    oaz = aRaw[orientationZ] * signZ;
    ogx = gRaw[orientationX] * signX;
    ogy = gRaw[orientationY] * signY;
    ogz = gRaw[orientationZ] * signZ;
    
    sumAx += oax;  // prepare calculation of averages
    sumAy += oay;
    sumAz += oaz;
    countSumAcc++;

    if ((config.gyroChanControl > 0) and (config.gyroChanControl <= 16) ) { // when gyro is used, send data to core0 to be used by gyro code
        sent2Core0( GYRO_X_ID , (int32_t) ogx >> (3-gyroScaleCode)) ;    // gyroScaleCode = 3 when 2000°/sec, 2=1000°/s , 1=500°/s, 0 when 250°/sec
        sent2Core0( GYRO_Y_ID , (int32_t) ogy >> (3-gyroScaleCode)) ;    // division is to get the same kind of unit (32768 = 2000°/sec) 
        sent2Core0( GYRO_Z_ID , (int32_t) ogz >> (3-gyroScaleCode)) ;
    }
    now = microsRp();
    deltat = ((float)(now - last))* 1.0e-6; //seconds since last update
    last = now;
    Mahony_update( (float) oax, (float) oay, (float) oaz , ((float) ogx) * gyroScaleRad, ((float) ogy) * gyroScaleRad , ((float) ogz) * gyroScaleRad, deltat);
    // this formula to calculate vertical accz comes from
    // https://github.com/har-in-air/ESP32_IMU_BARO_GPS_VARIO/blob/b0157b91902fe4d2de13a3198160438307d71662/src/sensor/imu.cpp
    // in imu_gravityCompensatedAccel()
    // and gives the same result as previous formula
    float accVert = 2.0*(qh[1]*qh[3] - qh[0]*qh[2])*( (float) oax) + 2.0f*(qh[0]*qh[1] + qh[2]*qh[3])*((float)oay)
     + (qh[0]*qh[0] - qh[1]*qh[1] - qh[2]*qh[2] + qh[3]*qh[3])*((float)oaz) - accScale1G; // scale =16384 for 1g when max is 2g (to be substracted)
    sumAccZ += accVert;
    countAccZ++;
    roll  = RAD_TO_DEGREE * atan2((qh[0] * qh[1] + qh[2] * qh[3]), 0.5 - (qh[1] * qh[1] + qh[2] * qh[2]));
    pitch = -RAD_TO_DEGREE * asin(2.0 * ( qh[0] * qh[2] - qh[1] * qh[3]));
    //if (msgEverySec(0)){printf("GOH=%i GOV=%i ax=%i  ay=%i   az=%i  roll=%.0f     pitch=%.0f\n",\
    //            config.mpuOrientationH, config.mpuOrientationV, oax, oay ,oaz , roll , pitch);}
    
    //float a1 , a2, a3;
    //#define F 0.7071068
    //static uint8_t cas = 1;
    //float qt[4]= { 0.7132504, 0,0, 0};
    //qt[cas] = 0.7009093;
    //cas++;
    //if (cas > 3) cas=1;
    //qt[0]=qh[0];qt[1]=qh[1];qt[2]=qh[2];qt[3]=qh[3];
    //int8_t iq[12] = {1, 2, 1, 3, 2, 3, 1, 1, 2, 2 ,3 ,3}; 
    //int8_t jq[12] = {2, 1, 3, 1, 3, 2, 2, 3, 3, 1, 1, 2};
    //int8_t kq[12] = {1, 2, 1, 3, 2, 3, 3, 2, 1, 3, 2, 1};
     
    //float a1, a2, a3;
    //quaternionToAngle( qt, 1, 3 , 2, a1, a2,  a3);
    //float r1 , p1 ,y1;
    //r1 = (atan2(qt[1]+qt[3], qt[0]- qt[2]) - atan2(qt[3] - qt[1] , qt[2] +qt[0]))*RAD_TO_DEGREE;
    //p1 = (acos( (qt[0]-qt[2])*(qt[0]-qt[2]) + (qt[1]+qt[3])*(qt[1]+qt[3]) + -1 ) - PI/2) *RAD_TO_DEGREE;
    //y1 = (atan2(qt[1]+qt[3], qt[0]- qt[2]) + atan2(qt[3] - qt[1] , qt[2] +qt[0]))*RAD_TO_DEGREE;
    /*
    if (msgEverySec(0)){
        printf("q0=%f    q1=%f   q2=%f   q3=%f    roll=%f   pitch=%f\n", qh[0] ,qh[1],qh[2],qh[3] , roll , pitch);
        for (uint8_t i = 0; i<12;i++){
            quaternionToAngle( qh, iq[i], jq[i] , kq[i], a1, a2,  a3);
            printf("     i=%i    j=%i   k=%i   idx=%i   a1=%f   a2=%f   a3=%f\n",  iq[i] , jq[i] ,kq[i] , i , a1 , a2 ,a3 ); 
        }
        printf("\n");    
    } 
    */
    //rollPitch(ax, ay , az,  gx ,  gy , gz);
    //ardupilot(ax, ay , az,  gx ,  gy , gz);
    //mylogic(ax, ay , az,  gx ,  gy , gz);
    //Madgwick6DOF(-ax, ay , az,  gx ,  -gy , -gz) ; // sign are from https://github.com/nickrehm/dRehmFlight/blob/master/Versions/dRehmFlight_Teensy_BETA_1.3/dRehmFlight_Teensy_BETA_1.3.ino

    
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

/*
#define ACCEL_NUM_AVG_SAMPLES	1000
bool MPU::calibrateAccelGyro(void){
	int16_t ax,ay,az, az1g ;
    int16_t gx,gy,gz ;
	int32_t axAccum, ayAccum, azAccum;
	axAccum = ayAccum = azAccum = 0;
    int32_t gxAccum, gyAccum, gzAccum;
	gxAccum = gyAccum = gzAccum = 0;
	// use a lower dlpf
    //uint8_t buffer[2] = {MPU6050_RA_CONFIG , MPU6050_DLPF_BW_20};
    //if( i2c_write_timeout_us(i2c1, MPU6050_DEFAULT_ADDRESS, &buffer[0], 2, false,3000)<0){ // true to keep master control of bus
    //    printf("Write error for MPU6050 calibration DLPF\n");
    //    return false;
    //}   
    //sleep_ms(10);
    for (int inx = 0; inx < ACCEL_NUM_AVG_SAMPLES; inx++){
        sleep_us(2000); // take 8 msec with dlpf = 20 ; 1900us when BW = 188
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
        gxAccum += (int32_t) gx;
        gyAccum += (int32_t) gy;
        gzAccum += (int32_t) gz;
    }

    axAccum /= (ACCEL_NUM_AVG_SAMPLES);
    ayAccum /= (ACCEL_NUM_AVG_SAMPLES);
    azAccum /= (ACCEL_NUM_AVG_SAMPLES);
    gxAccum /= (ACCEL_NUM_AVG_SAMPLES);
	gyAccum /= (ACCEL_NUM_AVG_SAMPLES);
	gzAccum /= (ACCEL_NUM_AVG_SAMPLES);
    uint8_t idx = 6; 
    findGravity( axAccum , ayAccum , azAccum , idx);
	switch (idx) {
        case 0 :
            axAccum -= (int32_t) accScale1G;
            break;
        case 1 :
            axAccum += (int32_t) accScale1G;
            break;
        case 2 :
            ayAccum -= (int32_t) accScale1G;
            break;
        case 3 :
            ayAccum += (int32_t) accScale1G;
            break;
        case 4 :
            azAccum -= (int32_t) accScale1G;
            break;
        case 5 :
            azAccum += (int32_t) accScale1G;
            break; 
    }
    config.accOffsetX = (int16_t)(axAccum);
	config.accOffsetY = (int16_t)(ayAccum);
	config.accOffsetZ = (int16_t)(azAccum);
    config.gyroOffsetX = (int16_t)(gxAccum);
	config.gyroOffsetY = (int16_t)(gyAccum);
	config.gyroOffsetZ = (int16_t)(gzAccum);
    config.mpuOrientation =  (idx << 4)  | (config.mpuOrientation &0x0F) ; // save the second part; replace the first one

    // restore dlpf
    //uint8_t buffer2[2] = {MPU6050_RA_CONFIG , MPU6050_DLPF_BW_188};
    //if( i2c_write_timeout_us(i2c1, MPU6050_DEFAULT_ADDRESS, &buffer2[0], 2, false,30000)<0){ // true to keep master control of bus
    //    printf("Write error for MPU6050 calibration DLPF\n");
    //    return false;
    //}   
    //sleep_ms(10);
    return true;
}
*/
/*
void quaternionToAngle(float q[4], int8_t i, int8_t j , int8_t k, float &a1, float &a2,  float &a3){
    //Input: q∈R4, and i, j, k ∈ {1, 2, 3}, where i ≠ j, j ≠ k; Q[0]= scalar; Q[1]=qx,...
    //Output: θ1, θ2, θ3
    bool not_proper;
    if (i == k){
        not_proper = false;
        k = 6 - i - j; // because i + j + k = 1 + 2 + 3 = 6
    } else {
        not_proper = true;
    }
    float e = (float)((i - j) * (j - k) * (k - i))/2; // equivalent to the Levi-Civita symbol
    float a, b , c, d ;
    if (not_proper) {
        a = q[0] - q[j];
        b = q[i] + q[k] * e;
        c = q[j] + q[0];
        d = q[k] * e - q[i];
    } else {
        a = q[0];
        b = q[i];
        c = q[j];
        d = q[k] * e;
    }
    a2=acos(2*(a*a+b*b)/(a*a+b*b+c*c+d*d)-1);
    float aplus = atan2(b, a);
    float aminus = atan2(d, c);
    if ( a2 == 0 ) {
            a1 = 0; // For simplicity, we are setting θˆ1=0
            a3 = 2 * aplus -a1;
    } else if ( a2 == PI/2 ) {
            a1 = 0;
            a3 = 2 * aminus + a1;
    } else {
            a1 = aplus -aminus;
            a3 = aplus + aminus;

    }
    if (not_proper) {
        a3 = e * a3;
        a2 = a2 - PI/2;
    }
    a1 = a1*RAD_TO_DEGREE;
    a2 = a2*RAD_TO_DEGREE;
    a3 = a3*RAD_TO_DEGREE;
}

    double gyro_pitch=0;  // values provided by gyro 
    double gyro_roll= 0;
    double gyro_yaw = 0;
    float gyro_roll_input=0;
    float gyro_pitch_input=0;
    float gyro_yaw_input =0;
    float angle_roll_acc =0;
    float angle_pitch_acc =0;
    float angle_pitch = 0;
    float angle_roll= 0;

    long acc_x, acc_y, acc_z, acc_total_vector;


void rollPitch(int16_t ax, int16_t ay ,int16_t az, int16_t gx , int16_t gy , int16_t gz) {

    long acc_x, acc_y, acc_z, acc_total_vector;
    
    static uint32_t count = 0;
    static int32_t axSum = 0;static int32_t aySum = 0;static int32_t azSum = 0;static int32_t gxSum = 0;static int32_t gySum = 0;static int32_t gzSum = 0;
    if (count < 2000){
        count++;
        gxSum += gx;gySum += gy;gzSum += gz;
        return;
    } else if (count == 2000){
        gxSum /= 2000;gySum /= 2000;gzSum /= 2000;
        count++;
        return;
    }
    gx -= gxSum; gy -= gySum; gz -= gzSum;  
    gyro_pitch = gx; gyro_roll=gy; gyro_yaw=gz;
    acc_x=ax; acc_y=ay ; acc_z=az;


  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
    // 131 for 250°/s
  #define GYRO_ADC_TO_DEGREE  131.0
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / GYRO_ADC_TO_DEGREE) * 0.3);   // filtered gyro rates in °/sec
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / GYRO_ADC_TO_DEGREE) * 0.3);
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / GYRO_ADC_TO_DEGREE) * 0.3);     


  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  // here 1/200/131 = 0.0000381679389312977
  #define GYRO_TRAVEL 1.0/500.0/131.0
  angle_pitch += gyro_pitch * GYRO_TRAVEL;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyro_roll * GYRO_TRAVEL;                                      
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  // here 
  #define GYRO_TRAVEL_RAD GYRO_TRAVEL *3.1416 /180
  angle_pitch -= angle_roll * sin(gyro_yaw * GYRO_TRAVEL_RAD);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyro_yaw * GYRO_TRAVEL_RAD);                  

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.
  
  if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;         
  }
  if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          
  }
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
  angle_roll_acc -= 0.0;                                                    
  
  angle_pitch = angle_pitch * 0.996 + angle_pitch_acc * 0.004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.996 + angle_roll_acc * 0.004;               

  //pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
  //roll_level_adjust = angle_roll * 15;                                      
if (msgEverySec(1)) printf("pitch= %f   roll=%f   pitchAcc=%f     rollAcc=%f\n", angle_pitch , angle_roll, angle_pitch_acc , angle_roll_acc)   ;
}



struct QuaternionA 
        {
            float w, x, y, z;
        };
struct Vector 
        {
            float x, y, z;
        };


// Computes the magnitude of quaternion q
float norm(const QuaternionA &q) 
{
    return sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

// Computes the magnitude of vector v
float norm(const Vector &v) 
{
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

// computes then returns the product of two quaternions
QuaternionA product(const QuaternionA &p, const QuaternionA &q) 
{
    QuaternionA result;

    result.w = p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z;
    result.x = p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y;
    result.y = p.w * q.y - p.x * q.z + p.y * q.w + p.z * q.x;
    result.z = p.w * q.z + p.x * q.y - p.y * q.x + p.z * q.w;

    // adjust dimensions to ensure the norm is 1 for noise reduction
    float l = norm(result);
    result.w /= l;
    result.x /= l;
    result.y /= l;
    result.x /= l;

    return result;
}

QuaternionA orientation;

float x_angle, y_angle, z_angle;

float x_angle_accel, y_angle_accel;


// this is based on ardupilot : https://github.com/dfvella/autopilot/blob/85b31461b61e4c7242a1b9fd9f2ddc41a8d49e6b/autopilot/imu.cpp
void ardupilot(int16_t ax, int16_t ay ,int16_t az, int16_t gx , int16_t gy , int16_t gz) {
    static uint32_t count = 0;
    static int32_t axSum = 0;static int32_t aySum = 0;static int32_t azSum = 0;static int32_t gxSum = 0;static int32_t gySum = 0;static int32_t gzSum = 0;
    if (count < 2000){
        count++;
        gxSum += gx;gySum += gy;gzSum += gz;
        return;
    } else if (count == 2000){
        gxSum /= 2000;gySum /= 2000;gzSum /= 2000;
        count++;
        return;
    }
    gx -= gxSum; gy -= gySum; gz -= gzSum;  
    gyro_pitch = gx; gyro_roll=gy; gyro_yaw=gz;
    acc_x=ax; acc_y=ay ; acc_z=az;
    static bool start = true;
    static uint32_t timer;
    if (start) 
    {
        start = false;
        timer = microsRp();
        orientation.w = 1;
        orientation.x = 0;
        orientation.y = 0;
        orientation.z = 0;
    }
    else 
    {
        float t_delta = (microsRp() - timer) / 1000000.0; // t_delta units: seconds
        timer = microsRp();

        Vector w;
        //w.x = (mpu.get(GYROX) - x_zero) * Mpu6050::TICKS_PER_DEGREE * RADIANS_PER_DEGREE;
        //w.y = (mpu.get(GYROY) - y_zero) * Mpu6050::TICKS_PER_DEGREE * RADIANS_PER_DEGREE;
        //w.z = (mpu.get(GYROZ) - z_zero) * Mpu6050::TICKS_PER_DEGREE * RADIANS_PER_DEGREE;
        #define TICKS_PER_DEGREE 0.007633588
        #define RADIANS_PER_DEGREE 0.01745329

        w.x = (float) ( gx - gxSum) * TICKS_PER_DEGREE * RADIANS_PER_DEGREE;
        w.y = (float) ( gy - gySum) * TICKS_PER_DEGREE * RADIANS_PER_DEGREE;
        w.z = (float) ( gz - gzSum) * TICKS_PER_DEGREE * RADIANS_PER_DEGREE;
        float w_norm = norm(w);

        QuaternionA rotation;
        rotation.w = cos((t_delta * w_norm) / 2);
        rotation.x = (sin((t_delta * w_norm) / 2) * w.x) / w_norm;
        rotation.y = (sin((t_delta * w_norm) / 2) * w.y) / w_norm;
        rotation.z = (sin((t_delta * w_norm) / 2) * w.z) / w_norm;

        orientation = product(orientation, rotation);

        x_angle = atan2(2 * orientation.x * orientation.w - 2 * orientation.y * orientation.z, 
                1 - 2 * orientation.x * orientation.x - 2 * orientation.z * orientation.z);

        y_angle = asin(2 * orientation.x * orientation.y + 2 * orientation.z * orientation.w);

        //z_angle = atan2(2 * orientation.y * orientation.w - 2 * orientation.x * orientation.z, 
        //        1 - 2 * orientation.y * orientation.y - 2 * orientation.z * orientation.z);

        x_angle /= RADIANS_PER_DEGREE;
        y_angle /= RADIANS_PER_DEGREE;
        //z_angle /= RADIANS_PER_DEGREE;

        float xAngleGyro = x_angle;
        float yAngleGyro = y_angle;
        //float zAngleGyro = z_angle;
        
        if (x_angle < -90) 
            x_angle += 270;
        else 
            x_angle -= 90;

        // construct a 3 dimensional vector representing the net acceration on the aircraft
        Vector net_accel; 
        net_accel.x = (float)ax;// - ACCELX_LEVEL_READING;
        net_accel.y = (float)ay;// - ACCELY_LEVEL_READING;
        net_accel.z = (float)az;// - ACCELZ_LEVEL_READING;

        // compute the magnitude of the net acceration
        float accel_norm = norm(net_accel);

        // compute the roll and pitch angles using the net acceleration vector
        float x_angle_accel_current = asin(net_accel.y / accel_norm) * (1 / RADIANS_PER_DEGREE);
        float y_angle_accel_current = asin(net_accel.x / accel_norm) * (1 / RADIANS_PER_DEGREE) * -1;

        // ********** FILTER GAINS **********
        #define ACCEL_FILTER_GAIN 0.1
        #define DRIFT_FILTER_GAIN 0.98
        // Apply an IIR filter to the accelerometer angles for noise reduction 
        x_angle_accel = x_angle_accel * ACCEL_FILTER_GAIN + 
                        x_angle_accel_current * (1 - ACCEL_FILTER_GAIN);
        y_angle_accel = y_angle_accel * ACCEL_FILTER_GAIN + 
                        y_angle_accel_current * (1 - ACCEL_FILTER_GAIN);

        // Apply the antidrift filter by computring a weighted average between the angle
        // given by the gyroscope data and the angle given by the accelerometer data.
        x_angle = x_angle * DRIFT_FILTER_GAIN + x_angle_accel * (1 - DRIFT_FILTER_GAIN);
        y_angle = y_angle * DRIFT_FILTER_GAIN + y_angle_accel * (1 - DRIFT_FILTER_GAIN);

        
        if (msgEverySec(1)) printf("pitch= %f   roll=%f pitchgyro=%f    rollgyro=%f pitchAcc=%f    rollAcc=%f  \n",\
             y_angle , x_angle , yAngleGyro , xAngleGyro, y_angle_accel_current,x_angle_accel_current  )   ;

    }
}    

void mylogic(int16_t ax, int16_t ay ,int16_t az, int16_t gx , int16_t gy , int16_t gz) {
    static uint32_t count = 0;
    static int32_t axSum = 0;static int32_t aySum = 0;static int32_t azSum = 0;static int32_t gxSum = 0;static int32_t gySum = 0;static int32_t gzSum = 0;
    if (count < 2000){
        count++;
        gxSum += gx;gySum += gy;gzSum += gz;
        axSum += ax;aySum += ay;azSum += az;
        return;
    } else if (count == 2000){
        gxSum /= 2000;gySum /= 2000;gzSum /= 2000;
        axSum /= 2000;aySum /= 2000;azSum /= 2000;
        azSum = azSum - 16384; // we expect 16384 = 1g
        count++;
        return;
    }
    gx -= gxSum; gy -= gySum; gz -= gzSum;
    ax -= axSum; ay -= aySum; az -= azSum;

            // construct a 3 dimensional vector representing the net acceration on the aircraft
//        Vector net_accel; 
//        net_accel.x = (float)ax;// - ACCELX_LEVEL_READING;
//        net_accel.y = (float)ay;// - ACCELY_LEVEL_READING;
//        net_accel.z = (float)az;// - ACCELZ_LEVEL_READING;

        // compute the magnitude of the net acceration
        //float accel_norm = norm(net_accel);



    // Y point to the nose; so is supposed to be roll axis
    // X point to the right wing; so is the pitch axis
    static uint32_t timer = 0;
    static float roll=0;static float pitch=0;
    static float rollGyro=0;static float pitchGyro=0;
    static float rollGyroZ=0;static float pitchGyroZ=0;
    float t_delta;
    if (timer == 0) 
    {
        timer = microsRp();
        return;
    } else {
        t_delta = (microsRp() - timer) / 1000000.0; // t_delta units: seconds
        timer = microsRp();
    }
        
    float rollAcc = atan2((float) -ax, (float) az) * RAD_TO_DEGREE;
    float pitchAcc = atan2((float) ay, (float) az) * RAD_TO_DEGREE;
    float deltaAnglex = (float) gx * gyroScaleDegree * t_delta; // pitch variation in degree
    float deltaAngley = (float) gy * gyroScaleDegree * t_delta; // roll variation
    
    roll +=deltaAngley;
    pitch +=deltaAnglex;
    #define hpf 0.99
    uint8_t update = 0;
    if ((ax > -12000) && (ax < 12000 ) && (ay > -12000) && (ay < 12000)) {
        roll = hpf * ( roll ) + (1-hpf) * rollAcc;
        pitch= hpf * ( pitch ) + (1-hpf) * pitchAcc;
        update = 1; 
    }
    

    if (abs(rollAcc) < 5 and abs(pitchAcc) < 5){  // reset gyro cumul when horizontal ; to do remove later on 
        rollGyro  = 0; pitchGyro  =0;
        rollGyroZ  =0; pitchGyroZ =0;
    } 
    // only base on gyro 1D
    rollGyro += deltaAngley;
    pitchGyro += deltaAnglex;
    
    // based on gyro Z also
    rollGyroZ += deltaAngley;
    pitchGyroZ += deltaAnglex;
    float deltaAngleZRad = (float) gz * gyroScaleRad * t_delta; // in radian
    float sinGyroZ = sin(deltaAngleZRad);  // angle Z in radian
    rollGyroZ += pitchGyroZ * sinGyroZ;
    pitchGyroZ -=  rollGyroZ * sinGyroZ;  
    //ax=%i  ay=%i   az=%i   gx=%i   gy=%i  gz=%i  ax ,ay ,az, gx ,gy, gz, 
    if ( msgEverySec(1))printf("%i  rollAcc=%.0f  rollGyro=%.0f  rollGyroZ=%.0f   roll=%.0f       pitchAcc=%.0f  pitchGyro=%.0f  pitchGyro=%.0f   pitch=%.0f\n",\
         update , rollAcc , rollGyro, rollGyroZ, roll , pitchAcc,  pitchGyro , pitchGyroZ , pitch);
}

float invSqrt(float x) {
  //Fast inverse sqrt for madgwick filter
  //float halfx = 0.5f * x;
  //float y = x;
  //long i = *(long*)&y;
  //i = 0x5f3759df - (i>>1);
  //y = *(float*)&i;
  //y = y * (1.5f - (halfx * y * y));
  //y = y * (1.5f - (halfx * y * y));
  //return y;
  
  
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
  
  return 1.0/sqrtf(x); //Teensy is fast enough to just take the compute penalty lol suck it arduino nano
}


void Madgwick6DOF(int16_t axr, int16_t ayr ,int16_t azr, int16_t gxr , int16_t gyr , int16_t gzr) { //invSampFreq = time interval
                // gyro are in °/sec
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  
  // * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
  // * available (for example when using the recommended MPU6050 IMU for the default setup).
  

static uint32_t count = 0;
    static int32_t axSum = 0;static int32_t aySum = 0;static int32_t azSum = 0;static int32_t gxSum = 0;static int32_t gySum = 0;static int32_t gzSum = 0;
    if (count < 2000){
        count++;
        gxSum += gxr;gySum += gyr;gzSum += gzr;
        axSum += axr;aySum += ayr;azSum += azr;
        return;
    } else if (count == 2000){
        gxSum /= 2000;gySum /= 2000;gzSum /= 2000;
        axSum /= 2000;aySum /= 2000;azSum /= 2000;
        azSum = azSum - 16384; // we expect 16384 = 1g
        count++;
        return;
    }
    gxr -= gxSum; gyr -= gySum; gzr -= gzSum;
    axr -= axSum; ayr -= aySum; azr -= azSum;

            // construct a 3 dimensional vector representing the net acceration on the aircraft
//        Vector net_accel; 
//        net_accel.x = (float)ax;// - ACCELX_LEVEL_READING;
//        net_accel.y = (float)ay;// - ACCELY_LEVEL_READING;
//        net_accel.z = (float)az;// - ACCELZ_LEVEL_READING;

        // compute the magnitude of the net acceration
        //float accel_norm = norm(net_accel);



    // Y point to the nose; so is supposed to be roll axis
    // X point to the right wing; so is the pitch axis
    static uint32_t timer = 0;
    float invSampleFreq;
    if (timer == 0) 
    {
        timer = microsRp();
        return;
    } else {
        invSampleFreq = (microsRp() - timer) / 1000000.0; // t_delta units: seconds
        timer = microsRp();
    }
    float gx=(float) gxr * gyroScaleDegree ;
    float gy=(float) gyr * gyroScaleDegree ;
    float gz=(float) gzr * gyroScaleDegree ;
    float ax = (float) axr;
    float ay = (float) ayr;
    float az = (float) azr;

    static float roll_IMU, pitch_IMU, yaw_IMU;
    static float roll_IMU_prev, pitch_IMU_prev;
    static float q0 = 1.0f; //Initialize quaternion for madgwick filter
    static float q1 = 0.0f;
    static float q2 = 0.0f;
    static float q3 = 0.0f;

  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    //Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
    static float B_madgwick = 0.04;  //Madgwick filter parameter
    static float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
    static float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
    static float B_mag = 1.0;        //Magnetometer LP filter parameter


  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Compute angles
  roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  pitch_IMU = -asin(-2.0f * (q1*q3 - q0*q2))*57.29577951; //degrees
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees

    if (msgEverySec(1)) printf("pitch= %f   roll=%f \n",\
              roll_IMU  ,pitch_IMU )   ;
}
*/
