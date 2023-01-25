#include "mpu.h"
#include "hardware/watchdog.h"
#include "param.h"
#include "tools.h"

extern CONFIG config;

MPU::MPU(int a)
{
}

void MPU::begin()  // initialise MPU6050 and dmp; mpuInstalled is true when MPU6050 exist
{
    //mpuInstalled = mpu6050.testConnection(); // true when a MPU6050 is installed
    if ( config.pinScl == 255 or config.pinSda == 255) return; // skip if pins are not defined
    //printf("start MPU initialisation\n");
    watchdog_enable(0x15000, false);
    mpu6050.initialize();
    watchdog_enable(0x15000, false);
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
        
        
        watchdog_enable(0x1500, false);
        //mpu6050.CalibrateAccel(6);
        watchdog_enable(0x1500, false);
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
    
    //printf("aworld: %d,\t %d,\t %d\n", aaWorld.x, aaWorld.y, aaWorld.z);
    //printf("%d\n", aaWorld.z);
    
    return false; 
}

