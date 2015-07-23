#include "mini_md.hpp"
#include "util.h"
MiniMD::MiniMD(Digital &cw,Digital &ccw,Pwm &pwm){
	this->cw=&cw;
	this->ccw=&ccw;
	this->pwm=&pwm;
}

int MiniMD::setup(float frequency){
	int i=0;
	if(cw->setupDigitalOut()) i++;
	if(ccw->setupDigitalOut()) i++;
	if(pwm->setupPwmOut(frequency,1.0)) i++;
	return i;
}

void MiniMD::duty(float value){
	request=constrain(value,-1.0,1.0);
}

void MiniMD::cycle(){
	if(request>0.0){
		cw->digitalHigh();
		ccw->digitalLow();
		pwm->pwmWrite(1.0-request);
	}else if(request<0.0){
		cw->digitalLow();
		ccw->digitalHigh();
		pwm->pwmWrite(1.0+request);//1.0-(-request)
	}else{
		cw->digitalHigh();
		ccw->digitalHigh();
		pwm->pwmWrite(1.0);
	}
}
