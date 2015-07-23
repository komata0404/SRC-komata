//libraries
#include "math.h"

//application

//controller
#include "layer_controller/mini_md.hpp"
//base
#include "system.h"
#include "mcutime.h"

//board
#include "pin.hpp"

//circuit
#define pi 3.14159265358979


class JKIT{
public:
		float L0=0;
		float L1=0;
		float L2=0;
		float L=115.0;
		float x1=0;
		float y1=0;
		float x2=0;
		float y2=0;
		float x3=0;
		float y3=0;
		float lx=0;
		float ly=0;
		float r1=0;
		float r2=0;
		float r3=0;
		float lr;
		float R=0;
		float lR=0;
		float X=0;
		float tx=0;
		float ty=0;
		float dis;
		float ang;
		float o1;
		float o2;
		float o3;
		float sdf;
		float x;
		float y;
		float w1=0;
		float w2=0;
		float w3=0;
		float xp=0;
		float yp=0;
		float jp=0;
		float fp=0;
		float xy=0;
		float d=0;
		float dr=0;
		float wr=0;
		float po=0;
		float wo1=0;
		float wo2=0;
		float wo3=0;
		float k1=0;
		float k2=0;
		float k3=0;
		CW0 cw0;
		CCW0 ccw0;
		Pwm0 pwm0;
		CW1 cw1;
		CCW1 ccw1;
		Pwm1 pwm1;
		CW2 cw2;
		CCW2 ccw2;
		Pwm2 pwm2;
		Enc0 enc0;
		Enc1 enc1;
		Enc2 enc2;
		Serial0 siri;
		JKIT();
		void MC();
		void EN();
};
void JKIT::MC(){
	while(1){
		k1=M_PI/2-po;
		k2=-M_PI/6-po;
		k3=-2*M_PI/3-po;
		wo1=(yp*cos(k1))+((jp-fp)*100);
		wo2=((cos(k2)*yp+cos(k2)*xp)*10)+((jp-fp)*100);
		wo3=((cos(k3)*yp+(cos(k3)*xp))*10)+((jp-fp)*100);
		//o1=(sin(d))+(sin(d-wr));
		sdf=fabs(wo1);
		if(sdf<fabs(wo2)){
			sdf=fabs(wo2);
		}
		if(sdf<fabs(wo3)){
			sdf=fabs(wo3);
		}
		if(sdf!=0){
			w1=(wo1)/sdf;
			w3=(wo2)/sdf;
			w2=(wo3)/sdf;
		}

		else {
			w1=0;
			w2=0;
			w3=0;
		}
		//siri.printf("w1 %f w2 %f w3 %f\n\r",w1,w2,w3);
		/*w1=w1+o1;
		w2=w2+o1;
		w3=w3+o1;
		sdf=fabs(wo1);
		if(sdf<fabs(wo2)){
			sdf=fabs(wo2);
		}
		if(sdf<fabs(wo3)){
			sdf=fabs(wo3);
		}
		if(sdf!=0){
			w1=(wo1)/sdf;
			w3=(wo2)/sdf;
			w2=(wo3)/sdf;
		}

		else {
			w1=0;
			w2=0;
			w3=0;
		}*/
		/*w1=0;
		w2=-1;
		w3=1;*/
		//siri.printf("rt%f,%f,%f\n\r",w1,w2,w3);
		MiniMD motor0(cw0,ccw0,pwm0);
		MiniMD motor1(cw1,ccw1,pwm1);
		MiniMD motor2(cw2,ccw2,pwm2);
		motor0.setup();
		motor1.setup();
		motor2.setup();
		motor0.duty(w1);
		motor0.cycle();
		motor1.duty(w2);
		motor1.cycle();
		motor2.duty(w3);
		motor2.cycle();
		EN();
}}
void JKIT::EN(){
		L0=(float)enc0.count()/200*40*M_PI;
		L1=(float)enc1.count()/200*40*M_PI;
		L2=(float)enc2.count()/200*40*M_PI;
		 //siri.printf("L0 %2f L1 %2f L2 %2f\n\r",L0,L1,L2);
		lr=(L0+L1+L2)/3/115;
		yp=y-ty;
		xp=x-tx;
		wr=d;
		jp=xy-X;//Œ»Ý‚Ì‹——£‚Ì•Î·
		d=0-lr;
		x1=-tan(M_PI/6)*(L0+L2/sin(M_PI/6));
		y1=L0;
		 //siri.printf("x1 %f y1 %f\n\r",x1,y1);
		x2=tan(M_PI/6)*(L0+L1/sin(M_PI/6));
		y2=L0;
		 //siri.printf("x2 %f y2 %f\n\r",x2,y2);
		x3=tan(M_PI/6)*(L1-L2)/(2*sin(M_PI/6));
		y3=(-L1-L2)/(2*sin(M_PI/6));
		 //siri.printf("x3 %f y3 %f\n\r",x3,y3);
		lx=(x1+x2+x3)/3;
		ly=(y1+y2+y3)/3;
		X=hypot(lx,ly);
		//siri.printf("JKIT dis %f\n\r",X);
		//siri.printf("x%f,y%f\n\r",lx,ly);
		tx=lx*cos(lr)-ly*sin(lr);
		ty=ly*cos(lr)+lx*sin(lr);
		siri.printf("x%f,y%f \n\r",tx,ty);
		fp=jp;//‰ß‹Ž‚Ì•Î·
		//siri.printf("%d,%d,%d\n",enc1.count(),enc2.count(),enc3.count());
}
JKIT::JKIT(){
		enc1.setup();
		enc1.count();
		enc2.setup();
		enc2.count();
		enc0.setup();
		enc0.count();
		siri.setup(115200);
		x=500;
		y=500;
		/*po=atanf(y/x);
		L=90;
		if(x==0){
			po=M_PI/2;
		}
		if(x<0){
			po=po+M_PI;
		}*/
		xy=hypotf(x,y);
}
int main(void)
{
	JKIT J;
	J.MC();
	return 0;
}
