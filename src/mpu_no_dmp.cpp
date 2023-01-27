#ifdef TEMP
/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "math.h"
#include "helper_3dmath.h"
#include "KalmanFilter.h"
#include "MS5611.h"
#include "tools.h"
#include "vario.h"

#define PI 3.14
/* Example code to talk to a MPU6050 MEMS accelerometer and gyroscope
   This is taking to simple approach of simply reading registers. It's perfectly
   possible to link up an interrupt line and set things up to read from the
   inbuilt FIFO to make it more useful.
   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefor I2C) cannot be used at 5v.
   You will need to use a level shifter on the I2C lines if you want to run the
   board at 5v.
   Connections on Raspberry Pi Pico board, other boards may vary.
   GPIO PICO_DEFAULT_I2C_SDA_PIN (On Pico this is GP4 (pin 6)) -> SDA on MPU6050 board
   GPIO PICO_DEFAULT_I2C_SCL_PIN (On Pico this is GP5 (pin 7)) -> SCL on MPU6050 board
   3.3v (pin 36) -> VCC on MPU6050 board
   GND (pin 38)  -> GND on MPU6050 board
*/

float aRes, gRes; // scale resolutions per LSB for the sensors
// Pin definitions
//int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
//#define blinkPin 13  // Blink LED on Teensy or Pro Mini when updating
//boolean blinkOn = false;
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
float ax, ay, az;       // Stores the real accel value in g's

//int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
//float gyrox, gyroy, gyroz;       // Stores the real gyro value in degrees per seconds

//float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer

//float SelfTest[6];
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion
uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0;  // used to control display output rate

float pitch, yaw, roll;
// parameters for 6 DoF sensor fusion calculations
float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float deltat = 0.0f;                              // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
uint32_t Now = 0;                                 // used to calculate integration interval

Quaternion qq;                // quaternion
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorFloat gravity;          // added by mstrens to calculate Z world acc
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements

//Kalman
KalmanFilter kalman1 ;
KalmanFilter kalman2 ;
  float zTrack1 ;
  float vTrack1 ;
  float zTrack2 ;
  float vTrack2 ;
  
  //struct ONE_MEASUREMENT vSpeedImu ;
  //bool vTrackAvailable ;
  //bool switchVTrackAvailable ;
  //extern float linear_acceleration_x ;
  //extern float linear_acceleration_y ;
  //extern float linear_acceleration_z ;
  //extern float world_linear_acceleration_z ;
  //extern bool newImuAvailable;
  //float altitudeToKalman ;
  //int countAltitudeToKalman = 100 ;
  //int32_t altitudeOffsetToKalman ;
  //extern volatile uint32_t lastImuInterruptMillis ;

MS5611 baro1( (uint8_t) 0x77  );    // class to handle MS5611; adress = 0x77 or 0x76
VARIO vario1;

// By default these devices  are on bus address 0x68
static int addr = 0x68;
#define i2c_default i2c1
#define PICO_DEFAULT_I2C_SDA_PIN 2
#define PICO_DEFAULT_I2C_SCL_PIN 3 

void GetGravity(VectorFloat *v, Quaternion *q);

void GetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) ;

void GetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) ;


static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
}


static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[14];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr, buffer, 14, false);

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
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
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
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;
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



int main() {
    stdio_init_all();
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
    #warning i2c/mpu6050_i2c example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else
    printf("Hello, MPU6050! Reading raw data from registers...\n");

    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    
    gpio_init(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_init(PICO_DEFAULT_I2C_SCL_PIN);
    gpio_set_dir(PICO_DEFAULT_I2C_SDA_PIN, GPIO_IN);
    gpio_set_dir(PICO_DEFAULT_I2C_SCL_PIN, GPIO_OUT);

    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    for (uint8_t i=0; i<20; i++){
        gpio_put(PICO_DEFAULT_I2C_SCL_PIN, 0);
        sleep_us(5);
        gpio_put(PICO_DEFAULT_I2C_SCL_PIN, 1);
        sleep_us(5);
    }
    gpio_set_dir(PICO_DEFAULT_I2C_SCL_PIN, GPIO_IN);
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    mpu6050_reset();

          baro1.begin();  // check MS5611; when ok, baro1.baroInstalled  = true


    int16_t acceleration[3], gyro[3], temp;

    while (1) {
        if ( baro1.getAltitude() == 0) { //  an altitude is calculated
        vario1.calculateAltVspeed(baro1.altitude , baro1.altIntervalMicros ); // Then calculate Vspeed ...
        //printf("baro %10.0f  %10.0f\n", (float) baro1.altitude , (float(baro1.altIntervalMicros )));
        
    #define USE_IMU
    #ifdef USE_IMU
        mpu6050_read_raw(acceleration, gyro);

        // These are the raw numbers from the chip, so will need tweaking to be really useful.
        // See the datasheet for more information
        //printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
        //printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        
  Now = micros();
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
    
    qq.w = q[0];
    qq.x = q[1];
    qq.y = q[2];
    qq.z = q[3];
    GetGravity(&gravity, &qq);
    aa.x = acceleration[0];
    aa.y = acceleration[1];
    aa.z = acceleration[2];
    GetLinearAccel(&aaReal, &aa, &gravity);
    //printf("aaReal: %d,\t %d,\t %d\n", aaReal.x, aaReal.y, aaReal.z);
    GetLinearAccelInWorld(&aaWorld, &aaReal, &qq);
    //printf("aworld: %d,\t %d,\t %d\n", aaWorld.x, aaWorld.y, aaWorld.z);

// Kalman
            //if ( countAltitudeToKalman != 0) {                                   // this calculate the initial altitude
            //    if( oXs_MS5611.varioData.rawAltitude != 0) {
            //      countAltitudeToKalman-- ;
            //      altitudeOffsetToKalman = oXs_MS5611.varioData.rawAltitude ;
            //    }        
            //}
            //altitudeToKalman = (oXs_MS5611.varioData.rawAltitude - altitudeOffsetToKalman ) / 100 ; // convert from * 100cm to cm
            //kalman.Update((float) baro1.altitude/100  , ((float) (aaWorld.z - 1000)) /16384.0 * 981.0 ,  &zTrack, &vTrack);  // Altitude and acceleration are in cm

            kalman1.Update((float) baro1.altitude/100  , 0 ,  &zTrack1, &vTrack1);  // Altitude and acceleration are in cm
            kalman2.Update((float) baro1.altitude/100  , ((float) (aaWorld.z - 1000)) /16384.0 * 981.0 ,  &zTrack2, &vTrack2);  // Altitude and acceleration are in cm

    #endif
  // Serial print and/or display at 0.5 s rate independent of data rates
  delt_t = millis() - count;
  if (delt_t > 200) { // update LCD once per half-second independent of read rate
    //printf("aworld: %d,\t %d,\t %d, \t %d  \t %f  \t %f\n", aaWorld.x, aaWorld.y, aaWorld.z, -(acceleration[2]+16384), zTrack, vTrack);
    //printf("aworldz : %d\n", aaWorld.z);
    printf(" %d,\t %2.0f,\t %2.0f,\t %2.0f, \t %5.2f  \n", aaWorld.z-1000, vTrack1 , vTrack2 , vario1.climbRateFloat  ,
         (float) baro1.altitude* 0.0001 );
    //
    /*
        Serial.print("ax = "); Serial.print((int)1000*ax);
        Serial.print(" ay = "); Serial.print((int)1000*ay);
        Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
        Serial.print("gyrox = "); Serial.print( gyrox, 1);
        Serial.print(" gyroy = "); Serial.print( gyroy, 1);
        Serial.print(" gyroz = "); Serial.print( gyroz, 1); Serial.println(" deg/s");
        Serial.print("q0 = "); Serial.print(q[0]);
        Serial.print(" qx = "); Serial.print(q[1]);
        Serial.print(" qy = "); Serial.print(q[2]);
        Serial.print(" qz = "); Serial.println(q[3]);
    */
    // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    // In this coordinate system, the positive z-axis is down toward Earth.
    // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    // applied in the correct order which for this configuration is yaw, pitch, and then roll.
    // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI;
    roll  *= 180.0f / PI;

//    printf("Yaw, Pitch, Roll: %f %f %f\n", yaw, pitch, roll);
    
    //    Serial.print("average rate = "); Serial.print(1.0f/deltat, 2); Serial.println(" Hz");
/*
    Serial.println(" x\t  y\t  z  ");

    Serial.print((int)(1000 * ax)); Serial.print('\t');
    Serial.print((int)(1000 * ay)); Serial.print('\t');
    Serial.print((int)(1000 * az));
    Serial.println(" mg");

    Serial.print((int)(gyrox)); Serial.print('\t');
    Serial.print((int)(gyroy)); Serial.print('\t');
    Serial.print((int)(gyroz));
    Serial.println(" o/s");

    Serial.print((int)(yaw)); Serial.print('\t');
    Serial.print((int)(pitch)); Serial.print('\t');
    Serial.print((int)(roll));
    Serial.println(" ypr");

    Serial.print("rt: "); Serial.print(1.0f / deltat, 2); Serial.println(" Hz");
*/
    count = millis();
  }

        //sleep_ms(1);
        }
        }

#endif
    return 0;
}
#endif