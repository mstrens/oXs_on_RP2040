#pragma once


#include "stdint.h"
#include "MPU6050.h"
#include "helper_3dmath.h"

class MPU
{
public:
    explicit MPU(int a);  
    bool mpuInstalled = false; 
    MPU6050 mpu6050 ; 
    bool calibAccRunning = false;
    int16_t calibAccResults[200][3] = {0};
    uint8_t calibAccCount = 0;

    void begin();
    bool getAccZWorld();
    void usbOrientationHorizontalExecute();
    void usbOrientationVerticalExecute();
    void nextAccCalibrationExecute();
    void orientationExecute(bool horizontal); // true for horizontal, false for vertical
    void gyroCalibrationExecute();
    
    uint8_t readAndGetGravity();    
    void testDevicesOffsetX();
    void printOffsets();
    bool calibrateAccelGyro(void);




    /*
    void readRegister(uint8_t register, uint8_t * buffer , uint8_t number);  
    uint8_t getDeviceID();
    int8_t readWords(uint8_t regAddr, uint8_t length, uint16_t *data);
    bool writeWords(uint8_t regAddr, uint8_t length, uint16_t* data);
    void readBytes(uint8_t regAddr , uint8_t length , uint8_t *data);
    uint8_t readByte(uint8_t regAddr );
    void printOffsets();
    int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);
    void calibrateGyro(uint8_t Loops );
    void calibrateAccel(uint8_t Loops );
    void PID(uint8_t ReadAddress, float kP,float kI, uint8_t Loops);
    */
private:
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    float yaw, pitch, roll;
    
};


void setupOrientation();
void findGravity(int32_t ax, int32_t ay, int32_t az, uint8_t &idx );



//void quaternionToAngle(float q[4], int8_t i, int8_t j , int8_t k, float &a1, float &a2,  float &a3);
//void rollPitch(int16_t ax, int16_t ay ,int16_t az, int16_t gx , int16_t gy , int16_t gz) ;
//void ardupilot(int16_t ax, int16_t ay ,int16_t az, int16_t gx , int16_t gy , int16_t gz) ;
//void mylogic(int16_t ax, int16_t ay ,int16_t az, int16_t gx , int16_t gy , int16_t gz) ;
//void Madgwick6DOF(int16_t ax, int16_t ay ,int16_t az, int16_t gx , int16_t gy , int16_t gz);


// -- END OF FILE --

