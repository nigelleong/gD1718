#ifndef KalmanFus_h
#define KalmanFus_h

// Fusion of odometry and IMU data

#include "MatrixMath.h"
#include "Mecanum.h"
#include "Pose.h"

class KalmanFus
{
private:
	const float pi = 3.14159265359;
	float Eye[3][3];
	float Jacobi_F[3][3];
	float Jacobi_F_trans[3][3];
	float Jacobi_W[3][3];
	float Jacobi_W_trans[3][3];
	float H_IMU[3]; // Sensor model for IMU
	float P[3][3]; //Error Model
	float P_pred[3][3]; //predicted error model
	float Kalman_Gain[3];
	float Pose_pred[3];
	float Q[3][3]; //Cov_Action
	float z_pred;
	float z_diff;
	float Cov_Sensor;
	
	void calcJacobiTransition(float* , Mecanum, Pose, float);
	void calcJacobiAction(Pose);
	void predictNewState(float*, Mecanum, Pose, float);
	void predictErrorModel();
	void calcKalmanGain();
	void predictSensor(float);
	void updateErrorModel();
public:	
	KalmanFus();
	float new_State[3];
	void calcNewState(float* w_is, Mecanum, Pose, float);
};

#endif
