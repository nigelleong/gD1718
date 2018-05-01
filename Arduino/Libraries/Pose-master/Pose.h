
#ifndef Pose_h
#define Pose_h

#include "MatrixMath.h"
#include "Mecanum.h"



class Pose
{
private:
	const float pi = 3.14159265359;
	float Trafo[3][3];
	float Trafo_inv[3][3];
	void calcTrafo(float);
public:	
	float globalPose_should[3];
	float globalPose[3];
	float odometryPose[3];
	float IMU_angle;
	Pose();
	Pose(float, float, float);
	void setglobalPose(float, float, float);
	void setglobalPose_should(float, float, float);
	void newPoseOdometry(float* , Mecanum, float );
	void newAngleIMU(float, float);
};

#endif
