
#ifndef __ENCODER_H__
#define __ENCODER_H__

//#include "WProgram.h"

class Encoder {

	private:
		float counts_per_round;
		const float pi = 3.14159265359;
	public:
		Encoder(float);
		void calcSpeed(int); //Calculates rotational speed in rad/sec!
		float speed;
		long int count = 0;
		void update();


};

#endif // __ENCODER_H__
