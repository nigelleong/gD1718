
#ifndef Pose_h
#define Pose_h

#include "MatrixMath.h"
#include "Mecanum.h"



class Pose
{
private:
	const float pi = 3.14159265359;
	
	void calcTrafo(float);
	float toGo_global[3];
	
	
public:	
	Pose();
	Pose(float, float, float);
	float Trafo[3][3];
	float Trafo_inv[3][3];
	float calc_v_to_Pose_should(float*, float, float);	//float* v, float max speed, float max rotationsl speed
	double globalPose_should[3];
	double globalPose[3];
	float odometryPose[3];
	float IMU_angle;
	void setglobalPose(float, float, float);
	void setglobalPose_should(float, float, float);
	void newPoseOdometry(float* , Mecanum, float );
	void newAngleIMU(float, float, float);
	void calctoGo_local();
	void speed_to_local_v(float*, float, float, float);
	double toGo_local[3];
	double heading_angle;
};

#endif
