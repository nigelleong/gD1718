
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
// w_is_sign includes the direction of ration --> with sign
void Pose::newPoseOdometry(float * w_is_sign, Mecanum SRT_Mecanum, float delta_time) {
	// delta_time is in milliseconds!
	odometryPose[0] = globalPose[0] + delta_time/1000*SRT_Mecanum.R/4*sqrt(2)*(cos(globalPose[2]+pi/4)*w_is_sign[0]+sin(globalPose[2]+pi/4)*w_is_sign[1]+cos(globalPose[2]+pi/4)*w_is_sign[2]+sin(globalPose[2]+pi/4)*w_is_sign[3]);
	odometryPose[1] = globalPose[1] + delta_time/1000*SRT_Mecanum.R/4*sqrt(2)*(sin(globalPose[2]+pi/4)*w_is_sign[0]-cos(globalPose[2]+pi/4)*w_is_sign[1]+sin(globalPose[2]+pi/4)*w_is_sign[2]-cos(globalPose[2]+pi/4)*w_is_sign[3]);
	odometryPose[2] = globalPose[2] + delta_time/1000*SRT_Mecanum.R/(4*(SRT_Mecanum.l_x+SRT_Mecanum.l_y))*(w_is_sign[0]-w_is_sign[1]-w_is_sign[2]+w_is_sign[3]);	
	if(odometryPose[2]>pi){
		odometryPose[2] = - 2*pi + odometryPose[2];
	}
	else if(odometryPose[2]<-pi){
		odometryPose[2] = 2*pi + odometryPose[2];
	}
}



// Calculate new orientation from IMU data only
// !!!! Requires delta angle from last time step !!!!!!!!!!!
void Pose::newAngleIMU(float yaw_prev_robot, float yaw_prev_IMU, float yaw_is_IMU) {
	float delta_angle;
	delta_angle = yaw_is_IMU - yaw_prev_IMU;
	IMU_angle = yaw_prev_robot + delta_angle;
	if(IMU_angle>pi){
		IMU_angle = -2*pi +IMU_angle;
	}
	else if(IMU_angle<-pi){
		IMU_angle = 2*pi + IMU_angle;
	}
}

void Pose::calctoGo_local(){
	toGo_global[0] = globalPose_should[0] - globalPose[0];
	toGo_global[1] = globalPose_should[1] - globalPose[1];
	toGo_global[2] = globalPose_should[2] - globalPose[2];
	Matrix.Multiply((float*) Trafo_inv, (float*) toGo_global, 3, 3, 1, (float*) toGo_local );
	heading_angle = atan2(toGo_global[1],toGo_global[0]);
	if(fabs(heading_angle-globalPose[2])>pi/2){
		heading_angle += pi;
		if(heading_angle>pi){
			heading_angle = -2*pi + heading_angle;
		}
		else if(heading_angle<-pi){
			heading_angle = 2*pi + heading_angle;
		}
	}
}

void Pose::speed_to_local_v(float * v, float x_global, float y_global, float w_global){
	float v_global[3] = {x_global, y_global, w_global};
	Matrix.Multiply((float*)Trafo_inv, (float*)v_global, 3, 3, 1, (float*)v);
}
