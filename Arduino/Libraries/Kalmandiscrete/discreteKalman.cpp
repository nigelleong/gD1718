
#include <discreteKalman.h>

discreteKalman::discreteKalman() {
	NFC_angle = true;
	// Radius of detection area around tags
	Radius = 33.081192756874980;
	
	// COvariance for area around tags
	Q[0][0] = sq(Radius)/4;
	Q[0][1] = 0;
	
	Q[1][0] = 0;
	Q[1][1] = sq(Radius)/4;
	
	// Initialize Error Model
	P[0][0] = 272.5913085937;
	P[0][1] = 0;
	P[0][2] = 0;
	
	P[1][0] = 0;
	P[1][1] = 272.5913085937;
	P[1][2] = 0;
	
	P[2][0] = 0;
	P[2][1] = 0;
	P[2][2] = 1;
	
	// Covariance tag misplacement: Assuming a misplacement
	R[0][0] = 1;
	R[0][1] = 0;
	
	R[1][0] = 0;
	R[1][1] = 1;
	
	// Sensor Model
	H[0][0] = 1;
	H[0][1] = 0;
	H[0][2] = 0;
	
	H[1][0] = 0;
	H[1][1] = 1;
	H[1][2] = 0;
	
	// Sensor Model transpose
	H_trans[0][0] = 1;
	H_trans[0][1] = 0;
	
	H_trans[1][0] = 0;
	H_trans[1][1] = 1;
	
	H_trans[2][0] = 0;
	H_trans[2][1] = 0;
		
	// Identity matrix
	Eye[0][0] = 1;
	Eye[0][1] = 0;
	Eye[0][2] = 0;
	
	Eye[1][0] = 0;
	Eye[1][1] = 1;
	Eye[1][2] = 0;
	
	Eye[2][0] = 0;
	Eye[2][1] = 0;
	Eye[2][2] = 1;
	
}

void discreteKalman::orientationRobot(Pose SRT_Pose){
	double tag_dis[2]; //destances between tags (x and y)
	double pose_dis[2]; // distances between current estimated pose and previous tag
	// center_dists between tags and poses at tags
	for (int i=0;i<2;i++){
		tag_dis[i] = tag_det[i] - tag_prev[i];
		pose_dis[i] = SRT_Pose.globalPose[i] - tag_prev[i];
	}
	
	// heading can be estimated after detecting two RFID tags
	yaw_est = SRT_Pose.globalPose[2] + atan2(tag_dis[1], tag_dis[0]) - atan2(pose_dis[1], pose_dis[0]);
}

void discreteKalman::newTagdetected(float x, float y){
	// Previous tag is updated
	for (int i=0;i<2;i++){
		tag_prev[i] = tag_det[i];
	}
	// New tag is copied
	tag_det[0] = x;
	tag_det[1] = y;
}

void discreteKalman::calcNewState(Pose SRT_Pose){
	// If the robot is estimated to be inside the tag zone, 
	// update is not neccessary
	if(checkRobotinZone(SRT_Pose)){
		new_State[0] = SRT_Pose.globalPose[0];
		new_State[1] = SRT_Pose.globalPose[1];
		new_State[2] = SRT_Pose.globalPose[2];
	}
	// if the robot is outside the tag zone--> update
	else{
				
		calcKalmanGain();
		// Sensor prediction is skipped --> global pose is the predicted sensor values
		Matrix.Multiply((float*) Kalman_Gain, (float*) center_dist, 3, 2, 1, (float*) correction);
		new_State[0] = SRT_Pose.globalPose[0] + correction[0];
		new_State[1] = SRT_Pose.globalPose[1] + correction[1];
		if(NFC_angle){
			new_State[2] = yaw_est;
		}
		else{
			new_State[2] = SRT_Pose.globalPose[2];
		}
		updateErrorModel();
	}
}
	
bool discreteKalman::checkRobotinZone(Pose SRT_Pose){
	for (int i=0;i<2;i++){
		center_dist[i] = tag_det[i] - SRT_Pose.globalPose[i];
	}
	if(sqrt(sq(center_dist[0])+sq(center_dist[1])<Radius)){
		return true;
	}
	else return false;
}

void discreteKalman::calcKalmanGain() {
	float Product[2][2];
	float Error_Sensor_Product[3][2];
	float Sum[2][2]; // This is also the inverted Sum matrix!
	Matrix.Multiply((float*) P, (float*) H_trans, 3, 3, 2, (float*) Error_Sensor_Product);
	Matrix.Multiply((float*) H, (float*) Error_Sensor_Product, 2, 3, 2,  (float*) Product);
	Matrix.Add((float*) Product, (float*) R, 2, 2, (float*) Sum );
	Matrix.Invert((float*) Sum, 2);
	Matrix.Multiply((float*) Error_Sensor_Product, (float*) Sum, 3, 2, 2, (float*) Kalman_Gain);
	// Transpose Galman Gain matrix
	Matrix.Transpose((float*) Kalman_Gain, 3, 2, (float*) Kalman_Gain_trans);
}

void discreteKalman::updateErrorModel(){
	float KQ_Product[3][2];
	float KQK_Product[3][3];
	float KH_Product[3][3];
	float I_KH_Diff[3][3];
	float Diff_P_Product[3][3];
	float P_new[3][3];
	Matrix.Multiply((float*) Kalman_Gain, (float*) Q, 3, 2, 2, (float*) KQ_Product);
	Matrix.Multiply((float*) KQ_Product, (float*) Kalman_Gain_trans, 3, 2, 3, (float*) KQK_Product);
	Matrix.Multiply((float*) Kalman_Gain, (float*) H, 3, 2, 3, (float*) KH_Product);
	Matrix.Subtract((float*) Eye, (float*) KH_Product, 3, 3, (float*) I_KH_Diff);
	Matrix.Multiply((float*) I_KH_Diff, (float*) P, 3, 3, 3, (float*) Diff_P_Product);
	Matrix.Add((float*) Diff_P_Product, (float*) KQK_Product, 3, 3, (float*)P_new );
	Matrix.Copy((float*) P_new, 3,  3, (float*) P);
}
void discreteKalman::offset_Reader(Pose SRT_Pose){
	float tag_det_offset[3];
	Matrix.Multiply((float*) SRT_Pose.Trafo, (float*) Reader_Offset, 3, 3, 1, (float*) tag_det_offset);
	tag_det[0] -= tag_det_offset[0];
	tag_det[1] -= tag_det_offset[1];

}