

#include "MatrixMath.h"


#ifndef Mecanum_h
#define Mecanum_h

class Mecanum
{
private: 
	// Matrices for Mecanum kinematics and inverse kinematics
	float T_inv[4][3];
	float T[3][4];
public:
	Mecanum(float, float, float);
	float R;
	float l_x;
	float l_y;
	void CalcWheelSpeeds(float* , float* );
	void CalcVelocity(float*, float*);
	void WheelSpeeds_NoSign(float* , float* );

};

#endif
