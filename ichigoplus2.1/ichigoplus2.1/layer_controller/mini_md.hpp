/*******************************************
 * mini_md ver1.0 2015/7/3
 * mini md circuit driver.
 *
 * [Dependency]
 * digital
 * pwm
 * util
 *
 * [Note]
 *
 * [Change history]
 * ver1.0 2015/ 7/ 3 [Nagatani]The first version.
 ******************************************/

#ifndef MINI_MD_HPP
#define MINI_MD_HPP

#include "digital.hpp"
#include "pwm.hpp"

class MiniMD{
private:
	Digital *cw;
	Digital *ccw;
	Pwm *pwm;
	float request;
public:
	MiniMD(Digital &cwPin,Digital &ccwPin,Pwm &pwmPin);
	int setup(float frequency=10000);
	void cycle();
	void duty(float value);
};

#endif//MINI_MD_HPP
