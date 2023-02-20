#ifdef USE_MPU_BU
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

//Kalman
//KalmanFilter kalman1 ;
//KalmanFilter kalman2 ;

//float zTrack1 ;
//float vTrack1 ;
float zTrack ;
float vTrack ;
float prevVTrack; // use to check hysteresis.
float sumAccZ = 0; // used to calculate an average of AccZ
uint32_t countAccZ=0;

// for Kalman4D
//float zTrack4 ;
//float vTrack4 ;
//float KFAltitudeCm; // heigh calculated by kalman filter
//float KFClimbrateCps ; // Vspeed calculated by kalman filter
//float KFAltitudeCm2; // heigh calculated by kalman filter
//float KFClimbrateCps2 ; // Vspeed calculated by kalman filter
uint32_t lastKfUs = microsRp(); 
uint32_t kfUs ;
    


#define USE_MAH //USE_MAK //USE_DMP  // USE_MAH
MPU::MPU(int a)
{
}

#ifdef USE_DMP


void MPU::begin()  // initialise MPU6050 and dmp; mpuInstalled is true when MPU6050 exist
{
    //mpuInstalled = mpu6050.testConnection(); // true when a MPU6050 is installed
    if ( config.pinScl == 255 or config.pinSda == 255) return; // skip if pins are not defined
    //printf("start MPU initialisation\n");
    mpu6050.initialize();
    devStatus = mpu6050.dmpInitialize();  
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        printf("MPU+DMP initialized\n");
        /* --- if you have calibration data then set the sensor offsets here --- */
        mpu6050.setXAccelOffset(335);
        mpu6050.setYAccelOffset(79);
        mpu6050.setZAccelOffset(1132);
        mpu6050.setXGyroOffset(70);
        mpu6050.setYGyroOffset(-13);
        mpu6050.setZGyroOffset(-9);
        //
        
        
        //mpu6050.CalibrateAccel(6);
        //mpu6050.CalibrateGyro(6);

        //mpu6050.PrintActiveOffsets();
        
        mpu6050.setDMPEnabled(true);                // turn on the DMP, now that it's ready
        mpuIntStatus = mpu6050.getIntStatus();
        dmpReady = true;                        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        packetSize = mpu6050.dmpGetFIFOPacketSize();      // get expected DMP packet size for later comparison
        mpuInstalled =  true;
    } 
    else 
    {                                          // ERROR!        1 = initial memory load failed         2 = DMP configuration updates failed        (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", devStatus);
        mpuInstalled =  false;
    }
    yaw = 0.0;
    pitch = 0.0;
    roll = 0.0;      
}

bool MPU::getAccZWorld(){ // return true when a value is available ; ead the IMU and calculate the acc on Z axis (world)
    if (!mpuInstalled) {
        //printf("no mpu\n");
        return false;
    }
    fifoCount = mpu6050.getFIFOCount(); 
    if (fifoCount < packetSize) return false;
    //enlapsedTime(0);
    
    mpu6050.getFIFOBytes(fifoBuffer, packetSize);                             // read a packet from FIFO
    
    mpu6050.dmpGetQuaternion(&q, fifoBuffer);
    mpu6050.dmpGetAccel(&aa, fifoBuffer);
    //printf("aa: %d,\t %d,\t %d\n", aa.x, aa.y, aa.z);
    mpu6050.dmpGetGravity(&gravity, &q);
    //printf("gravity: %d,\t %d,\t %d\n", gravity.x, gravity.y, gravity.z);
    mpu6050.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    //printf("aaReal: %d,\t %d,\t %d\n", aaReal.x, aaReal.y, aaReal.z);
    mpu6050.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    sumAccZ += aaWorld.z;
    countAccZ++;
    if (vario1.newClimbRateAvailable){
        kalman2.Update((float) baro1.altitude/100  , (sumAccZ/countAccZ) /16384.0 * 981.0 ,  &zTrack2, &vTrack2);  // Altitude and acceleration are in cm
        sumAccZ= 0;
        countAccZ=0;
        sent2Core0( VSPEED , (int32_t) vTrack2) ; 
    //    kalman2.Update((float) baro1.altitude/100  , ((float) (aaWorld.z - 1000)) /16384.0 * 981.0 ,  &zTrack2, &vTrack2);  // Altitude and acceleration are in cm
//    kalman2.Update((float) baro1.altitude/100  , 0,  &zTrack2, &vTrack2);  // Altitude and acceleration are in cm
    }


    //printf("aworld: %d,\t %d,\t %d\n", aaWorld.x, aaWorld.y, aaWorld.z);
    //printf("AccZ=%d\n", aaWorld.z);
    
    return false; 
}
#endif
#ifdef USE_MAK //  // do not use dmp
//*****************************************************
#define PI 3.1416

float qf[4] = {1.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion
uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0;  // used to control display output rate
// parameters for 6 DoF sensor fusion calculations
float GyroMeasError ;     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta ;  // compute beta
float GyroMeasDrift ;      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float deltat = 0.0f;                              // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
uint32_t Now = 0;                                 // used to calculate integration interval

Quaternion qq;                // quaternion
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorFloat gravity;          // added by mstrens to calculate Z world acc
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements


void MPU::begin()  // initialise MPU6050 and dmp; mpuInstalled is true when MPU6050 exist
{
    if ( config.pinScl == 255 or config.pinSda == 255) return; // skip if pins are not defined
    
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c1, MPU6050_DEFAULT_ADDRESS , buf, 2, false);
    sleep_us(100);
        // set offsets
        /*
        mpu6050.setXAccelOffset(335);
        mpu6050.setYAccelOffset(79);
        mpu6050.setZAccelOffset(1132);
        mpu6050.setXGyroOffset(70);
        mpu6050.setYGyroOffset(-13);
        mpu6050.setZGyroOffset(-9);
        */
    
    //GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
    GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
    
    beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
    GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
    mpuInstalled =  true;
    printf("MPU initialized\n");

}

    int16_t acceleration[3], gyro[3], temp;

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[14];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c1, MPU6050_DEFAULT_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c1, MPU6050_DEFAULT_ADDRESS, buffer, 14, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
    for (int i = 4; i < 7; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gyrox, float gyroy, float gyroz)
{
    float axn; // added by ms to avoid changes of ax,ay,az
    float ayn;
    float azn;
    float q1 = qf[0], q2 = qf[1], q3 = qf[2], q4 = qf[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    axn = ax * norm;
    ayn = ay * norm;
    azn = az * norm;
    
    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - axn;
    f2 = _2q1 * q2 + _2q3 * q4 - ayn;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - azn;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;
    
    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;
    
    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;
    
    // Compute estimated gyroscope biases
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
    
    // Compute and remove gyroscope biases
    gbiasx += gerrx * deltat * zeta;
    gbiasy += gerry * deltat * zeta;
    gbiasz += gerrz * deltat * zeta;
    gyrox -= gbiasx;
    gyroy -= gbiasy;
    gyroz -= gbiasz;
    
    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gyrox - _halfq3 * gyroy - _halfq4 * gyroz;
    qDot2 =  _halfq1 * gyrox + _halfq3 * gyroz - _halfq4 * gyroy;
    qDot3 =  _halfq1 * gyroy - _halfq2 * gyroz + _halfq4 * gyrox;
    qDot4 =  _halfq1 * gyroz + _halfq2 * gyroy - _halfq3 * gyrox;

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 -(beta * hatDot1)) * deltat;
    q2 += (qDot2 -(beta * hatDot2)) * deltat;
    q3 += (qDot3 -(beta * hatDot3)) * deltat;
    q4 += (qDot4 -(beta * hatDot4)) * deltat;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    qf[0] = q1 * norm;
    qf[1] = q2 * norm;
    qf[2] = q3 * norm;
    qf[3] = q4 * norm;
}

void GetGravity(VectorFloat *v, Quaternion *q) {
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
}

void GetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
    // Mstrens think that it is not 8192 but 16384
    // get rid of the gravity component (+1g = +8192 in standard DMP FIFO packet, sensitivity is 2g)
    //v -> x = vRaw -> x - gravity -> x*8192;
    //v -> y = vRaw -> y - gravity -> y*8192;
    //v -> z = vRaw -> z - gravity -> z*8192;
    v -> x = vRaw -> x - (int16_t) (gravity -> x*16384.0f);
    v -> y = vRaw -> y - (int16_t) (gravity -> y*16384.0f);
    v -> z = vRaw -> z - (int16_t) (gravity -> z*16384.0f);
}

void GetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) {
    // rotate measured 3D acceleration vector into original state
    // frame of reference based on orientation quaternion
    memcpy(v, vReal, sizeof(VectorInt16));
    v -> rotate(q);
}


bool MPU::getAccZWorld(){ // return true when a value is available ; ead the IMU and calculate the acc on Z axis (world)
    if (!mpuInstalled) {
        //printf("no mpu\n");
        return false;
    }
    
    //printf("mpu\n"); 
    mpu6050_read_raw(acceleration, gyro);
    Now = microsRp();
    deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;
  //    if(lastUpdate - firstUpdate > 10000000uL) {
  //      beta = 0.041; // decrease filter gain after stabilized
  //      zeta = 0.015; // increase gyro bias drift gain after stabilized
  //    }
  // Pass gyro rate as rad/s
  
  //MadgwickQuaternionUpdate(ax, ay, az, gyrox * PI / 180.0f, gyroy * PI / 180.0f, gyroz * PI / 180.0f);
    MadgwickQuaternionUpdate( (float) acceleration[0], (float) acceleration[1],(float) acceleration[2],
        (float) gyro[0]  * PI / 180.0f, (float) gyro[1]  * PI / 180.0f, (float) gyro[2]  * PI / 180.0f) ;     
    
    // at this stage, quaternion qf is updated based on acceleration and gyro
    qq.w = qf[0];
    qq.x = qf[1];
    qq.y = qf[2];
    qq.z = qf[3];
    GetGravity(&gravity, &qq);
    aa.x = acceleration[0];
    aa.y = acceleration[1];
    aa.z = acceleration[2];
    GetLinearAccel(&aaReal, &aa, &gravity);
    //printf("aaReal: %d,\t %d,\t %d\n", aaReal.x, aaReal.y, aaReal.z);
    GetLinearAccelInWorld(&aaWorld, &aaReal, &qq);
    //printf("aworld: %d,\t %d,\t %d\n", aaWorld.x, aaWorld.y, aaWorld.z);
 
    // at this stage, quaternion q is updated based on acceleration and gyro
    qq.w = qf[0];
    qq.x = qf[1];
    qq.y = qf[2];
    qq.z = qf[3];
    GetGravity(&gravity, &qq);
    aa.x = acceleration[0];
    aa.y = acceleration[1];
    aa.z = acceleration[2];
    GetLinearAccel(&aaReal, &aa, &gravity);
    //printf("aaReal: %d,\t %d,\t %d\n", aaReal.x, aaReal.y, aaReal.z);
    GetLinearAccelInWorld(&aaWorld, &aaReal, &qq);
    sumAccZ += aaWorld.z;
    countAccZ++;
    if (vario1.newClimbRateAvailable){
        kalman2.Update((float) baro1.altitude/100  , (sumAccZ/countAccZ) /16384.0 * 981.0 ,  &zTrack2, &vTrack2);  // Altitude and acceleration are in cm
        sumAccZ= 0;
        countAccZ=0;
        sent2Core0( VSPEED , (int32_t) vTrack2) ; 
    //    kalman2.Update((float) baro1.altitude/100  , ((float) (aaWorld.z - 1000)) /16384.0 * 981.0 ,  &zTrack2, &vTrack2);  // Altitude and acceleration are in cm
//    kalman2.Update((float) baro1.altitude/100  , 0,  &zTrack2, &vTrack2);  // Altitude and acceleration are in cm
    }


  // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millisRp() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate
        //printf("aworld: %d,\t %d,\t %d, \t %d  \t %f  \t %f\n", aaWorld.x, aaWorld.y, aaWorld.z, -(acceleration[2]+16384), zTrack, vTrack);
        //printf("aworldz : %d\n", aaWorld.z);
        
        //printf(" %d,\t %2.0f,\t %2.0f,\t %2.0f, \t %5.2f  \n", aaWorld.z-1000, vTrack1 , vTrack2 , vario1.climbRateFloat, (float) baro1.altitude* 0.0001 );
        
        yaw   = atan2(2.0f * (qf[1] * qf[2] + qf[0] * qf[3]), qf[0] * qf[0] + qf[1] * qf[1] - qf[2] * qf[2] - qf[3] * qf[3]);
        pitch = -asin(2.0f * (qf[1] * qf[3] - qf[0] * qf[2]));
        roll  = atan2(2.0f * (qf[0] * qf[1] + qf[2] * qf[3]), qf[0] * qf[0] - qf[1] * qf[1] - qf[2] * qf[2] + qf[3] * qf[3]);

        pitch *= 180.0f / PI;
        yaw   *= 180.0f / PI;
        roll  *= 180.0f / PI;

        //printf("Yaw, Pitch, Roll: %5.0f %5.0f %5.0f\n", yaw, pitch, roll); 
        printf("pitch, roll, acc: %6.0f %6.0f %6.0f %6.0f\n", pitch, roll, vTrack2, vario1.climbRateFloat);//  Serial.print("ypr ");
        count = millisRp();
    }
    return false; 
}
#endif

// **************************************************************

#ifdef USE_MAH

#define PI 3.1416
float A_cal[6] = {335.0, 79.0, 1132.0, 1.0, 1.000, 1.0}; // 0..2 offset xyz, 3..5 scale xyz
float G_off[3] = { 70.0, -13.0, -9.0}; //raw offsets, determined for gyro at rest
float gscale = ((250./32768.0)*(PI/180.0));   //gyro default 250 LSB per d/s -> rad/s

// ^^^^^^^^^^^^^^^^^^^ VERY VERY IMPORTANT ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

int16_t ax, ay, az;
int16_t gxh, gyh, gzh;
  //scaled data as vector
float Axyz[3];
float Gxyz[3];

static float deltat = 0;  //loop time in seconds
static unsigned long now = 0, last = 0; //microsRp() timers


// GLOBALLY DECLARED, required for Mahony filter
// vector to hold quaternion
float qh[4] = {1.0, 0.0, 0.0, 0.0};

// Free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
float Kp = 30.0; // in github.com/har-in-air/ESP32_IMU_BARO_GPS_VARIO.blob/master it is set on 10
float Ki = 0.0;  // on same site, it is set on 2

unsigned long now_ms, last_ms = 0; //millisRp() timers

float yaw, pitch, roll; //Euler angle output

Quaternion qq;                // quaternion
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorFloat gravity;          // added by mstrens to calculate Z world acc
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements

void MPU::begin()  // initialise MPU6050 and dmp; mpuInstalled is true when MPU6050 exist
{
    if ( config.pinScl == 255 or config.pinSda == 255) return; // skip if pins are not defined
    

    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    if ( i2c_write_blocking(i2c1, MPU6050_DEFAULT_ADDRESS , buf, 2, false) == PICO_ERROR_GENERIC) return ;
    sleep_us(100);
    mpu6050.initialize();
    // set offsets with values saved in config
    mpu6050.setXAccelOffset(config.accOffsetX);
    mpu6050.setYAccelOffset(config.accOffsetY);
    mpu6050.setZAccelOffset(config.accOffsetZ);
    mpu6050.setXGyroOffset(config.gyroOffsetX);
    mpu6050.setYGyroOffset(config.gyroOffsetY);
    mpu6050.setZGyroOffset(config.gyroOffsetZ);
    mpu6050.setDLPFMode(MPU6050_DLPF_BW_188);
    //mpu6050.setDLPFMode(MPU6050_DLPF_BW_10);
    //printf("dlpfMode= %d\n", mpu6050.getDLPFMode() );
    //printf("rate= %d\n", mpu6050.getRate() );
     
    /*
    mpu6050.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    mpu6050.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu6050.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu6050.setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!
    
    printf("Acc & gyro before calibration\n");
    mpu6050.PrintActiveOffsets() ;
    mpu6050.CalibrateGyro(6);
    mpu6050.CalibrateAccel(6);
    printf("Acc & gyro after calibration\n");
    mpu6050.PrintActiveOffsets() ;
    */
    mpuInstalled =  true;
    //printf("MPU MAP initialized\n");
    kalmanFilter4d_configure(1000.0f*(float)KF_ACCEL_VARIANCE_DEFAULT, KF_ADAPT, 0.0f, 0.0f, 0.0f);

}

/*   Moved to param.cpp
void MPU::calibrationRequest()  // 
{
    if (!mpuInstalled) {
        printf("Calibration not done: no MP6050 installed\n");
        return ;
    }
    uint8_t data = 0X01; // 0X01 = execute calibration
    queue_try_add(&qSendCmdToCore1 , &data);
}    

void MPU::printConfigOffsets(){
    printf("\nOffset Values in config:\n");
	printf("Acc. X = %d, Y = %d, Z = %d\n", config.accOffsetX , config.accOffsetY, config.accOffsetZ);    
    printf("Gyro. X = %d, Y = %d, Z = %d\n", config.gyroOffsetX , config.gyroOffsetY, config.gyroOffsetZ);
}
*/

void MPU::testDevicesOffsetX(){
    sleep_ms(1000);
    int16_t gyroX[16];
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
    if ( i2c_write_blocking(i2c1, MPU6050_DEFAULT_ADDRESS , buf, 2, false) == PICO_ERROR_GENERIC) return ;
    sleep_us(100);
    mpu6050.initialize();
    
    mpu6050.CalibrateGyro(6);
    mpu6050.CalibrateAccel(6);
    // put the offsets in config.h
    config.accOffsetX = mpu6050.getXAccelOffset();
    config.accOffsetY = mpu6050.getYAccelOffset();
    config.accOffsetZ = mpu6050.getZAccelOffset();
    config.gyroOffsetX = mpu6050.getXGyroOffset();
    config.gyroOffsetY = mpu6050.getYGyroOffset();
    config.gyroOffsetZ = mpu6050.getZGyroOffset();
    //printf("Acc & gyro after calibration\n");
    //printConfigOffsets() ;
    //printf("Calibration done: parameters will be saved\n");
    //sleep_ms(3000);
    sent2Core0(0XFF, 0XFFFFFFFF); // use a dummy type to give a command; here a cmd to save the config
    //mpu6050.PrintActiveOffsets() ;
    //testDevicesOffsetX();
}

void GetGravity(VectorFloat *v, Quaternion *q) {
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
}

void GetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
    // Mstrens think that it is not 8192 but 16384
    // get rid of the gravity component (+1g = +8192 in standard DMP FIFO packet, sensitivity is 2g)
    //v -> x = vRaw -> x - gravity -> x*8192;
    //v -> y = vRaw -> y - gravity -> y*8192;
    //v -> z = vRaw -> z - gravity -> z*8192;
    v -> x = vRaw -> x - (int16_t) (gravity -> x*16384.0f);
    v -> y = vRaw -> y - (int16_t) (gravity -> y*16384.0f);
    v -> z = vRaw -> z - (int16_t) (gravity -> z*16384.0f);
}

void GetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) {
    // rotate measured 3D acceleration vector into original state
    // frame of reference based on orientation quaternion
    memcpy(v, vReal, sizeof(VectorInt16));
    v -> rotate(q);
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
 if (tmp > 0.0)
  {

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
  if ( (qh[0] * qh[0] + qh[1] * qh[1] + qh[2] * qh[2] + qh[3] * qh[3]) == 0 ) return;
  recipNorm = 1.0 / sqrt(qh[0] * qh[0] + qh[1] * qh[1] + qh[2] * qh[2] + qh[3] * qh[3]);
  qh[0] = qh[0] * recipNorm;
  qh[1] = qh[1] * recipNorm;
  qh[2] = qh[2] * recipNorm;
  qh[3] = qh[3] * recipNorm;
}

bool MPU::getAccZWorld(){ // return true when a value is available ; read the IMU and calculate the acc on Z axis (world)
    if (!mpuInstalled) {
        //printf("no mpu\n");
        return false;
    }
    //return false; // to remove after testing

    //printf("mpu\n"); 
    uint8_t buffer[14];
    static float sumAz;
    static float azWorldAverage;
    static float azAverage; 
    static float vSpeedAcc = 0;
    static float vSpeedAccHighPass;
    static float vSpeedAccLow ;
    static float vSpeedAcc2;
    static float accLow;
    static float accHigh;
    static float vSpeedAcc3;
    static float lowPass = 0;        //initialization of EMA S
    static uint32_t lastMpuUs;
    if ( ( microsRp() - lastMpuUs) < 2000) return false ; // perfor calculation only every 2 msec 
    lastMpuUs = microsRp();
 
 


    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c1, MPU6050_DEFAULT_ADDRESS, &val, 1, true); // true to keep master control of bus
    if ( i2c_read_timeout_us(i2c1, MPU6050_DEFAULT_ADDRESS, buffer, 14, false, 2500) == PICO_ERROR_TIMEOUT){
        printf("Read error for MPU6050\n");
        return false;
    }     
    ax = (buffer[0] << 8 | buffer[1]);
    ay = (buffer[2] << 8 | buffer[3]);
    az = (buffer[4] << 8 | buffer[5]);
    //printf("az=%.0f\n", (float) az ); 
    gxh = (buffer[8] << 8 | buffer[9]);
    gyh = (buffer[10] << 8 | buffer[11]);
    gzh = (buffer[12] << 8 | buffer[13]);
  //scaled data as vector
  float Axyz[3];
  float Gxyz[3];
  Axyz[0] = (float) ax;
  Axyz[1] = (float) ay;
  Axyz[2] = (float) az;

  //  printf("AX Gx = %i %i\n", ax , gxh);
  //apply offsets and scale factors from Magneto
  //for (int i = 0; i < 3; i++) Axyz[i] = (Axyz[i] - A_cal[i]) * A_cal[i + 3];

  //Gxyz[0] = ((float) gxh - G_off[0]) * gscale; //250 LSB(d/s) default to radians/s
  //Gxyz[1] = ((float) gyh - G_off[1]) * gscale;
  //Gxyz[2] = ((float) gzh - G_off[2]) * gscale;
    Gxyz[0] = ((float) gxh) * gscale; //250 LSB(d/s) default to radians/s
    Gxyz[1] = ((float) gyh) * gscale;
    Gxyz[2] = ((float) gzh) * gscale;


  //  snprintf(s,sizeof(s),"mpu raw %d,%d,%d,%d,%d,%d",ax,ay,az,gx,gy,gz);
  //  Serial.println(s);

  now = microsRp();
  deltat = ((float)(now - last))* 1.0e-6; //seconds since last update
  last = now;

  Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);
    
    // at this stage, quaternion q is updated based on acceleration and gyro
    qq.w = qh[0];
    qq.x = qh[1];
    qq.y = qh[2];
    qq.z = qh[3];
    GetGravity(&gravity, &qq);
    aa.x = ax;
    aa.y = ay;
    aa.z = az;
    GetLinearAccel(&aaReal, &aa, &gravity);
    //printf("%d,\t %d\n", aa.z - 16384 , aaReal.z);
    
    GetLinearAccelInWorld(&aaWorld, &aaReal, &qq);
    //printf("%d,\t %d\n", ((int32_t) az) - 16384 ,  aaWorld.z );
    
//    kalman2.Update((float) baro1.altitude/100  , ((float) (aaWorld.z)) /16384.0 * 981.0 ,  &zTrack2, &vTrack2);  // Altitude and acceleration are in cm
    //sumAz += (float) az;
    sumAccZ += (float) aaWorld.z;
    countAccZ++;
        //printf("az azworld %d %d %d\n", az +16384, aaWorld.z, (int32_t) deltat *1000000 ); 
    // here above is executed nearly once per millisec 
    if (vario1.newClimbRateAvailable){   // here once per about 20 msec
        //enlapsedTime(0);
        vario1.newClimbRateAvailable = false; // reset the flag that says a new relative alt is available
        azWorldAverage = (sumAccZ/countAccZ);
        //azAverage = (sumAz / countAccZ);
        //accLow = 0.999 * accLow + (1-0.999) * ((float) az); 
        //accLow = 0.999 * accLow + (1-0.999) * ((float) azWorldAverage); 
        
        //accHigh = ((float) azWorldAverage) - accLow;
        //accHigh =  az;
        //vSpeedAcc3 = accHigh / 16384.0 * 981.0 * 0.02;
        //vSpeedAcc += ( ((float) aaWorld.z)  / 16384.0 * 981.0 * 0.02) ;
        //vSpeedAccLow = 0.9 * vSpeedAccLow + (1-0.9)* vSpeedAcc3;
        //vSpeedAcc2 = vSpeedAcc3 - vSpeedAccLow;
    //printf("Vb Va %d ,  %d, %d , 50, -50\n", (int32_t) vario1.climbRateFloat ,   (int32_t) vSpeedAcc3, (int32_t) vSpeedAcc2);  
    //printf("aza wza %0.f %0.f\n", azAverage / 16384.0 * 981.0, azWorldAverage / 16384.0 * 981.0 );




        //kalman2.Update((float) vario1.rawRelAltitudeCm , azWorldAverage /16384.0 * 981.0 ,  &zTrack2, &vTrack2);
        kfUs = microsRp(); 
        kalmanFilter4d_predict( ((float) (kfUs-lastKfUs )) /1000000.0f);
        lastKfUs = kfUs;  
        kalmanFilter4d_update( (float) vario1.rawRelAltitudeCm , (float) azWorldAverage /16384.0 * 981.0 , (float*) &zTrack , (float*)&vTrack);
        //printf("Vv4 Vk4  %d %d 50 -50\n", (int32_t) vario1.climbRateFloat ,  (int32_t) (float) vTrack);
        //printf("Va4 Va4  %d %d 50 -50\n", (int32_t) vario1.relativeAlt ,  (int32_t) (float) zTrack);
            
        
        sumAccZ= 0;
        countAccZ=0;
        sumAz = 0;
        if ( abs((vTrack - prevVTrack) ) >  VARIOHYSTERESIS ) {
            prevVTrack = vTrack ;
        }    
        sent2Core0( VSPEED , (int32_t) prevVTrack) ; 
    }
    now_ms = millisRp(); //time to print?
    if (now_ms - last_ms >= 500) {
        last_ms = now_ms;
        roll  = atan2((qh[0] * qh[1] + qh[2] * qh[3]), 0.5 - (qh[1] * qh[1] + qh[2] * qh[2]));
        pitch = asin(2.0 * (qh[0] * qh[2] - qh[1] * qh[3]));
        //conventional yaw increases clockwise from North. Not that the MPU-6050 knows where North is.
        //yaw   = -atan2((qh[1] * qh[2] + qh[0] * qh[3]), 0.5 - (qh[2] * qh[2] + qh[3] * qh[3]));
        // to degrees
        //yaw   *= 180.0 / PI;
        //if (yaw < 0) yaw += 360.0; //compass circle
        pitch *= 180.0 / PI;
        roll *= 180.0 / PI;
        sent2Core0( PITCH , (int32_t) pitch ) ; 
        sent2Core0( ROLL , (int32_t) roll ) ; 
    
        // print angles for serial plotter...
        #ifdef DEBUG
        //printf("pitch, roll, acc: %6.0f %6.0f %6.0f %6.0f\n", pitch, roll, vTrack, vario1.climbRateFloat);//  Serial.print("ypr ");
        #endif
    }
    return false; 
}

/*
void MPU::printOffsets() {
    printf("acc = %d    %d   %d\n", mpu6050.getXAccelOffset() , mpu6050.getYAccelOffset() , mpu6050.getZAccelOffset());
    printf("gyro= %d    %d   %d\n", mpu6050.getXGyroOffset() , mpu6050.getYGyroOffset() ,mpu6050.getZGyroOffset());
}
*/



#endif

#endif