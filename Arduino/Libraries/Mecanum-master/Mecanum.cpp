
#include "Mecanum.h"

Mecanum::Mecanum(float lx, float ly, float r) {
	R = r;
	l_x = lx;
	l_y = ly;	
	
	T_inv[0][0] = 1/R;
	T_inv[0][1] = -1/R;
	T_inv[0][2] = 1/R*(l_x+l_y);
	T_inv[1][0] = 1/R;
	T_inv[1][1] = 1/R;
	T_inv[1][2] = 1/R*(-l_x-l_y);
	T_inv[2][0] = 1/R;
	T_inv[2][1] = -1/R;
	T_inv[2][2] = 1/R*(-l_x-l_y);
	T_inv[3][0] = 1/R;
	T_inv[3][1] = 1/R;
	T_inv[3][2] = 1/R*(l_x+l_y);
	
	T[0][0] = R/4;
	T[0][1] = R/4;
	T[0][2] = R/4;
	T[0][3] = R/4;
	T[1][0] = -R/4;
	T[1][1] = R/4;
	T[1][2] = -R/4;
	T[1][3] = R/4;
	T[2][0] = R/(4*(l_x+l_y));
	T[2][1] = -R/(4*(l_x+l_y));
	T[2][2] = -R/(4*(l_x+l_y));
	T[2][3] = R/(4*(l_x+l_y));
	

};

//Calculate wheel's speeds from given velocity and rotation
void Mecanum::CalcWheelSpeeds(float * v, float * w)
{
	// v = input vector with speeds in x and y direction and rotation around z
	// w = output vector containing rotaional speed of wheels
	Matrix.Multiply((float*)T_inv, (float*)v, 4, 3, 1, (float*)w);
}

// Calculate velocity of the robot (robot's coordinate system)
void Mecanum::CalcVelocity(float * w, float * v)
{
	// v = output vector with speeds in x and y direction and rotation around z
	// w = input vector containing rotaional speed of wheels
	Matrix.Multiply((float*)T, (float*)w, 3, 4, 1, (float*)v);
}

void Mecanum::WheelSpeeds_NoSign(float * w, float * w_should){
	for(int i=0;i<4;i++)
	{
		if(w[i]<0) w_should[i] = -w[i];
		else w_should[i] = w[i];
	}
}
