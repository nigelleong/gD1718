#include "Encoder.h"


Encoder::Encoder(float cpr){
	counts_per_round = cpr;
}

void Encoder::update(){
	count++;
}

float Encoder::calcSpeed(int time_diff){
	speed = (float)count/time_diff/counts_per_round*2*pi*1000;  //rad/sec!!!
	return speed;
}
