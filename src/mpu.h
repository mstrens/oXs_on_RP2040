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
    void begin();
    bool getAccZWorld();
    void calibrationExecute();
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


// -- END OF FILE --

