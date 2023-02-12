#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

//#include "oXs_config_basic.h"
//#include "oXs_config_advanced.h"
//#include "oXs_config_macros.h"


class KalmanFilter {

public :

KalmanFilter() ;
void Update(float z, float a,  float* pZ, float* pV);
private :

// State being tracked
	float z_;  // position
	float v_;  // velocity
	float aBias_;  // acceleration

// 3x3 State Covariance matrix
	float Pzz_ ;
	float Pzv_;
	float Pza_;
	float Pvz_;
	float Pvv_ ;
	float Pva_;
	float Paz_;
	float Pav_;
 	float Paa_ ;

// those float have been replaced by #define to save flash memory
//  float zAccelBiasVariance_; // assumed fixed.
//	float zAccelVariance_;  // dynamic acceleration variance
//	float zVariance_; //  z measurement noise variance fixed

};

#endif
