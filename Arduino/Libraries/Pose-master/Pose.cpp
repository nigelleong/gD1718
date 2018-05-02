
#include <Pose.h>

Pose::Pose() {
	setglobalPose(0,0,0);
}


Pose::Pose(float x, float y, float delta) {
	setglobalPose(x,y,delta);
}


// Set global Pose and calculate transformation matrix
void Pose::setglobalPose(float x, float y, float delta) {
	globalPose[0] = x;
	globalPose[1] = y;
	globalPose[2] = delta;
	calcTrafo(delta);
}

// Set desired global Pose
void Pose::setglobalPose_should(float x, float y, float delta) {
	globalPose_should[0] = x;
	globalPose_should[1] = y;
	globalPose_should[2] = delta;
}

// Calculate current transformation matrix (from local system to global system)
void Pose::calcTrafo(float delta) {
	Trafo[0][0] = cos(delta);
	Trafo[0][1] = -sin(delta);
	Trafo[0][2] = 0;
	Trafo[1][0] = sin(delta);
	Trafo[1][1] = cos(delta);
	Trafo[1][2] = 0;
	Trafo[2][0] = 0;
	Trafo[2][1] = 0;
	Trafo[2][2] = 1;
	
	Trafo_inv[0][0] = cos(delta);
	Trafo_inv[0][1] = sin(delta);
	Trafo_inv[0][2] = 0;
	Trafo_inv[1][0] = -sin(delta);
	Trafo_inv[1][1] = cos(delta);
	Trafo_inv[1][2] = 0;
	Trafo_inv[2][0] = 0;
	Trafo_inv[2][1] = 0;
	Trafo_inv[2][2] = 1;
}

// Calculate new pose from odometry data only
// w_is includes the direction of ration --> with sign
void Pose::newPoseOdometry(float * w_is, Mecanum SRT_Mecanum, float delta_time) {
	// delta_time is in milliseconds!
	odometryPose[0] = globalPose[0] + delta_time/1000*SRT_Mecanum.R/4*sqrt(2)*(sin(globalPose[2]+pi/4)*w_is[0]+cos(globalPose[2]+pi/4)*w_is[1]+sin(globalPose[2]+pi/4)*w_is[2]+cos(globalPose[2]+pi/4)*w_is[3]);
	odometryPose[1] = globalPose[1] + delta_time/1000*SRT_Mecanum.R/4*sqrt(2)*(-cos(globalPose[2]+pi/4)*w_is[0]+sin(globalPose[2]+pi/4)*w_is[1]-cos(globalPose[2]+pi/4)*w_is[2]+sin(globalPose[2]+pi/4)*w_is[3]);
	odometryPose[2] = globalPose[2] + delta_time/1000*SRT_Mecanum.R/(4*(SRT_Mecanum.l_x+SRT_Mecanum.l_y))*(w_is[0]-w_is[1]-w_is[2]+w_is[3]);	
}

// Calculate new pose from NFC tag


// Calculate new orientation from IMU data only
// !!!! Requires delta angle from last time step !!!!!!!!!!!
void Pose::newAngleIMU(float yaw_prev, float yaw_is) {
	float delta_angle;
	delta_angle = yaw_is - yaw_prev;
	IMU_angle = globalPose[2] + delta_angle;
}
