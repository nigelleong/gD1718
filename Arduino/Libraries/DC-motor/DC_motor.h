

#ifndef DC_motor_h
#define DC_motor_h


class DC_motor
{


public:
	DC_motor(int, int, float);
	int pin_dir;
	int pin_speed;
	float translation;
	void map_wheelspeed(float, double);
	void change_translation(float);
	int dir;
	float mapped_speed;
	void run(float);	

};

#endif