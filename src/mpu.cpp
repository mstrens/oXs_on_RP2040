#include "mpu.h"


MPU::MPU(int a)
{
}

void MPU::begin()  // initialise MPU6050 and dmp; mpuInstalled is true when MPU6050 exist
{
    //mpuInstalled = mpu6050.testConnection(); // true when a MPU6050 is installed
    mpu6050.initialize();
    devStatus = mpu6050.dmpInitialize();  
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        printf("DMP initialized\n");
        mpu6050.CalibrateAccel(6);
        mpu6050.CalibrateGyro(6);
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
    printf("mpu\n");
    if (!mpuInstalled) {
        //printf("no mpu\n");
        return false;
    }
    fifoCount = mpu6050.getFIFOCount(); 
    printf("fifo: %d\n",fifoCount);
    if (fifoCount < packetSize) return false;
    mpu6050.getFIFOBytes(fifoBuffer, packetSize);                             // read a packet from FIFO
    mpu6050.dmpGetQuaternion(&q, fifoBuffer);
    mpu6050.dmpGetAccel(&aa, fifoBuffer);
    mpu6050.dmpGetGravity(&gravity, &q);
    mpu6050.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu6050.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    printf("aworld: %d,\t %d,\t %d\n", aaWorld.x, aaWorld.y, aaWorld.z);
}

