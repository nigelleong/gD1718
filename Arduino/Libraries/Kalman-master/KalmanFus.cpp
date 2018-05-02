
#include <KalmanFus.h>

KalmanFus::KalmanFus() {
	// Initialize Error Model
	P[0][0] = 1;
	P[0][1] = 0;
	P[0][2] = 0;
	
	P[1][0] = 0;
	P[1][1] = 1;
	P[1][2] = 0;
	
	P[2][0] = 0;
	P[2][1] = 0;
	P[2][2] = 1;
	
	// Initialize covariance action matrix
	Q[0][0] = 5;
	Q[0][1] = 0;
	Q[0][2] = 0;
	
	Q[1][0] = 0;
	Q[1][1] = 5;
	Q[1][2] = 0;
	
	Q[2][0] = 0;
	Q[2][1] = 0;
	Q[2][2] = 0.1;
	
	// Sensor uncertainty (IMU)
	Cov_Sensor = 0.1;
	
	// Sensor Model IMU
	H_IMU[0] = 0;	
	H_IMU[1] = 0;	
	H_IMU[2] = 1;	
	
	
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

// Calculate Jacobian of transition model --> Extended Kalman Filter
void KalmanFus::calcJacobiTransition(float * w_is, Mecanum SRT_Mecanum, Pose SRT_Pose, float delta_time) {
	Jacobi_F[0][0] = 1;
	Jacobi_F[0][1] = 0;
	Jacobi_F[0][2] = delta_time*SRT_Mecanum.R/4*sqrt(2)*(cos(SRT_Pose.globalPose[2]+pi/4)*w_is[0]-sin(SRT_Pose.globalPose[2]+pi/4)*w_is[1]+cos(SRT_Pose.globalPose[2]+pi/4)*w_is[2]-sin(SRT_Pose.globalPose[2]+pi/4)*w_is[3]);
	
	Jacobi_F[1][0] = 0;
	Jacobi_F[1][1] = 1;
	Jacobi_F[1][2] = delta_time*SRT_Mecanum.R/4*sqrt(2)*(sin(SRT_Pose.globalPose[2]+pi/4)*w_is[0]+cos(SRT_Pose.globalPose[2]+pi/4)*w_is[1]+sin(SRT_Pose.globalPose[2]+pi/4)*w_is[2]+cos(SRT_Pose.globalPose[2]+pi/4)*w_is[3]);
	
	Jacobi_F[2][0] = 0;
	Jacobi_F[2][1] = 0;
	Jacobi_F[2][2] = 1;	
	
	Matrix.Transpose((float*) Jacobi_F, 3, 3, (float*) Jacobi_F_trans);
}

// Calculate Jacobian df/dw (w is vector with uncertainties) 
// This is equavilatn to df/du (u is local velocity of robot)
void KalmanFus::calcJacobiAction(Pose SRT_Pose){
	Jacobi_W[0][0] = cos(SRT_Pose.globalPose[2]);
	Jacobi_W[0][1] = -sin(SRT_Pose.globalPose[2]);
	Jacobi_W[0][2] = 0;
	
	Jacobi_W[1][0] = sin(SRT_Pose.globalPose[2]);
	Jacobi_W[1][1] = cos(SRT_Pose.globalPose[2]);
	Jacobi_W[1][2] = 0;
	
	Jacobi_W[2][0] = 0;
	Jacobi_W[2][1] = 0;
	Jacobi_W[2][2] = 1;	
	
	Matrix.Transpose((float*) Jacobi_W, 3, 3, (float*) Jacobi_W_trans);
}

// x = f(x,u)
// Predict new state: w needed to correct sign of w_is (w_is has no sign)
void KalmanFus::predictNewState(float * w_is, Mecanum SRT_Mecanum, Pose SRT_Pose, float delta_time) {
	SRT_Pose.newPoseOdometry(w_is, SRT_Mecanum, delta_time );
	Pose_pred[0] = SRT_Pose.odometryPose[0];
	Pose_pred[1] = SRT_Pose.odometryPose[1];
	Pose_pred[2] = SRT_Pose.odometryPose[2];
}

// Calculate predicted P: P_pred
void KalmanFus::predictErrorModel() {
	float Jacobi_Error_Product[3][3];
	float Product_Error[3][3];
	float Product_Action[3][3];
	// First term (F*P*F^T) -> Product_Error
	Matrix.Multiply((float*) Jacobi_F, (float*) P, 3, 3, 3, (float*) Jacobi_Error_Product );
	Matrix.Multiply((float*) Jacobi_Error_Product, (float*) Jacobi_F_trans, 3, 3, 3, (float*) Product_Error);
	// Second term (W*Q*W^T) -> Product_Action
	Matrix.Multiply((float*) Jacobi_W, (float*) Q, 3, 3, 3, (float*) Jacobi_Error_Product );
	Matrix.Multiply((float*) Jacobi_Error_Product, (float*) Jacobi_W_trans, 3, 3, 3, (float*) Product_Action);
	// Add both terms to obtain predicted Error model
	Matrix.Add((float*) Product_Error, (float*) Product_Action, 3, 3, (float*) P_pred);	
}

void KalmanFus::calcKalmanGain() {
	float Sensor_Error_Product[3];
	float Product_1D;
	float Sum;
	Matrix.Multiply((float*) H_IMU, (float*) P_pred, 1, 3, 3, (float*) Sensor_Error_Product );
	Matrix.Multiply((float*) Sensor_Error_Product, (float*) H_IMU, 1, 3, 1,  &Product_1D);
	Sum = Product_1D + Cov_Sensor;
	Matrix.Multiply((float*) P_pred, (float*) H_IMU, 3, 3, 1, (float*) Kalman_Gain );
	Kalman_Gain[0] *= 1/Sum;
	Kalman_Gain[1] *= 1/Sum;
	Kalman_Gain[2] *= 1/Sum;
}
// Predict sensor value based on predicted pose
// Calculate difference between actual sensor value and predicted sensor value
void KalmanFus::predictSensor(float z) {
	Matrix.Multiply((float*) H_IMU, (float*) Pose_pred, 1, 3, 1, &z_pred );
	z_diff = z - z_pred;
}

// Calculate new estimated pose (fused odometry and IMU data)
void KalmanFus::calcNewState(float * w_is, Mecanum SRT_Mecanum, Pose SRT_Pose, float delta_time) {
	float Gain_Difference_Product[3];
	calcJacobiTransition(w_is , SRT_Mecanum, SRT_Pose, delta_time);
	calcJacobiAction(SRT_Pose);
	predictNewState(w_is, SRT_Mecanum, SRT_Pose, delta_time);
	predictErrorModel();
	calcKalmanGain();
	updateErrorModel();
	predictSensor(SRT_Pose.IMU_angle);
	Matrix.Multiply((float*) Kalman_Gain, &z_diff, 3, 1, 1, (float*) Gain_Difference_Product);
	new_State[0] = Pose_pred[0] + Gain_Difference_Product[0];
	new_State[1] = Pose_pred[1] + Gain_Difference_Product[1];
	new_State[2] = Pose_pred[2] + Gain_Difference_Product[2];
	updateErrorModel();	
}

void KalmanFus::updateErrorModel() {
	float Gain_Sensor_Product[3][3];
	float Difference[3][3];
	Matrix.Multiply((float*) Kalman_Gain, (float*) H_IMU, 3, 1, 3, (float*) Gain_Sensor_Product );
	Matrix.Subtract((float*) Eye, (float*) Gain_Sensor_Product, 3, 3, (float*) Difference);
	Matrix.Multiply((float*) Difference, (float*) P_pred, 3, 3, 3, (float*) P );
}
















