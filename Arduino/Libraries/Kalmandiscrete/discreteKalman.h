#ifndef discreteKalman_h
#define discreteKalmanh

// Fusion of odometry and IMU data

#include "MatrixMath.h"
#include "Mecanum.h"
#include "Pose.h"

class discreteKalman
{
private:
	const float pi = 3.14159265359;
	float yaw_est;
	float tag_prev[2];
	float tag_det[2]; // corresponds to z in KalmanFus.h
	float tag_pred[2]; // corresponds to z_pred in KalmanFus.h
	float pose_tag_prev[2];
	bool checkRobotinZone(Pose);
	float centre_dist[2]; //distance between current pose and centre of detected tag
	float Radius; //radius of circle around center of tag in which the tag is detected
	float R[2][2]; //To considere uncertainty due to tag misplacement and communication delay
	float P[3][3]; //Error Model
	float H[2][3]; // Sensor model 
	float H_trans[3][2]; // Sensor model transpose
	float Kalman_Gain[3][2];
	float Kalman_Gain_trans[2][3];
	float Eye[3][3];
	float Q[2][2]; //Cov_quantized measurement
	void calcKalmanGain();
	void updateErrorModel();
	void orientationRobot(Pose, float*, float*);
	
public:	
	discreteKalman();
	float new_State[3];
	void newTagdetected(float, float);
	void calcNewState(Pose);
};

#endif