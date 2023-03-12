
#ifndef KALMAN_FILTER4_H_
#define KALMAN_FILTER4_H_

void kalmanFilter4d_configure(float aVariance, float kAdapt, float zInitial, float vInitial, float aInitial);
void kalmanFilter4d_predict(float dt);
void kalmanFilter4d_update(float zm, float am, float* pz, float* pv);

// here we put extracts from 2 other files 
// *************   extract from config.h in original github source **************************************


// Kalman filter configuration
// actual variance value used is accel_variance*1000
#define KF_ACCEL_VARIANCE_DEFAULT            100
#define KF_ACCEL_VARIANCE_MIN                50
#define KF_ACCEL_VARIANCE_MAX                150


// If you find that gyro calibration fails when you leave
// the unit undisturbed, possibly your unit has an MPU9250 device
// with a high gyro bias on one or more axes. Try increasing this limit  
// until you find the calibration works consistently.

#define GYRO_OFFSET_LIMIT_1000DPS_DEFAULT  	150
#define GYRO_OFFSET_LIMIT_1000DPS_MIN     	25
#define GYRO_OFFSET_LIMIT_1000DPS_MAX		200



// altitude noise variance can be measured offline by calculating the 
// statistical variance in cm^2 of altitude samples from 
// the baro sensor at rest
#define KF_Z_MEAS_VARIANCE            200  //   ms : it was 200 in the version on github

// KF4 Acceleration Measurement Noise variance
#define KF_A_MEAS_VARIANCE   		50.0f

// This is set low as the residual acceleration bias after calibration
// is expected to have little variation/drift
#define KF_ACCELBIAS_VARIANCE   0.005f

// injects additional uncertainty depending on magnitude of acceleration
// helps respond quickly to large accelerations while heavily filtering
// in low acceleration situations.  Range : 0.5 - 1.5
#define KF_ADAPT			0.5f  // original value is 1.0f


// *********************  extract from common.h in original github source ************************************
//#define MIN(x,y)                 ((x) < (y) ? (x) : (y))
//#define MAX(x,y)                 ((x) > (y) ? (x) : (y))
#define ABS(x)                   ((x) < 0 ? -(x) : (x))
#define CLAMP(x,mn,mx)           {if (x <= (mn)) x = (mn); else if (x >= (mx)) x = (mx);}
#define CORE(x,t)                {if (ABS(x) <= (t)) x = 0;}
#define MCORE(x,t)               {if (x > (t)) x -= (t); else if (x < -(t)) x += (t); else x = 0;}
#define CORRECT(x,mx,mn)  		   (((float)((x)-(mn))/(float)((mx)-(mn))) - 0.5f)
#define INTEGER_ROUNDUP(val)     ((val) >= 0.0f ? (int32_t)((val)+0.5f) : (int32_t)((val)-0.5f))

#define RAD2DEG(r)   ((r)*57.29577951f)
#define DEG2RAD(d)   ((d)*0.017453292f)

#define _180_DIV_PI         57.2957795f
#define PI_DIV_180          0.017453292f
#define _2_PI              6.2831853f


#endif
