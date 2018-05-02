
#include "DC_motor.h"

// Define pins
DC_motor::DC_motor(int pin_d, int pin_sp, float trans) {
	pin_dir = pin_d;
	pin_speed = pin_sp;	
	translation = trans;
}

void DC_motor::map_wheelspeed(float speed, double PID)
{
	if(speed>=0){
		dir = 1;
		mapped_speed = 255-PID;
	}		
	else {
		dir = 0;
		mapped_speed = PID; 
	}
}

void DC_motor::change_translation(float trans)
{
	translation = trans;
}

