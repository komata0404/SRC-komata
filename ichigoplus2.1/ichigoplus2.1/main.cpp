//libraries
#include "math.h"
//#include "time.h"
//application

//controller
#include "layer_controller/mini_md.hpp"
#include "layer_controller/selfLocation.hpp"
//base
#include "system.h"
#include "mcutime.h"
//#include "can.h"

//board
#include "pin.hpp"

//circuit
#include "circuit/can_encoder.hpp"

class ROBOT{
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
		float x_Coordinate=0;
		float y_Coordinate=0;
		float bodyAngle=0;
		float correct_X_Coordinate=0;
		float correct_Y_Coordinate=0;
		float currentDeviation=0;
		float x=0;
		float y=0;
		int i;
		float target_x_Coordinate=0;
		float target_y_Coordinate=0;
		float targetDistance=0;
		float max=0;
		float xy=0;
		float time=0;
		float x_Deviation=0;
		float y_Deviation=0;
		float distance=0;
		float beforeDeviation=0;
		float output[2]={0};
		float angleOutput=0;
		float deviation=0;
		float pastDeviation=0;
		Buzzer buz;
		Can0 can;
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
		Serial0 serial;
		ROBOT();
		void targetOutput();
		void motorCycle();
		void selfLocation();
		void angleLock();
		void total();
		void test();
};
void ROBOT::targetOutput(){
		output[0]=(y_Deviation*sin(-M_PI/6)+x_Deviation*cos(-M_PI/6))+((currentDeviation-pastDeviation)*100);
		output[1]=(y_Deviation*sin(M_PI/2)+x_Deviation*cos(M_PI/2))+((currentDeviation-pastDeviation)*100);
		output[2]=(y_Deviation*sin(-5*M_PI/6)+x_Deviation*cos(-5*M_PI/6))+((currentDeviation-pastDeviation)*100);
		max=fabs(output[0]);
		for(i=1;i<=2;i++){
			if(max<fabs(output[i])){
				max=fabs(output[i]);
			}
		}
		if(max!=0){
		for(i=0;i<=2;i++){
			output[i]=output[i]/max;
		}
		}
		//serial.printf("%f,%f,%f\n\r",output[0],output[1],output[2]);
}
void ROBOT::angleLock(){
		deviation=0-bodyAngle*(180/M_PI);
		angleOutput=(0.0022*deviation)+(0.33*(deviation-beforeDeviation));
		beforeDeviation=deviation;
}
void ROBOT::total(){
		MiniMD motor0(cw0,ccw0,pwm0);
		MiniMD motor1(cw1,ccw1,pwm1);
		MiniMD motor2(cw2,ccw2,pwm2);
		for(i=0;i<=2;i++){
			output[i]+=angleOutput;
		}
		max=fabs(output[0]);
		for(i=1;i<=2;i++){
			if(max<fabs(output[i])){
				max=fabs(output[i]);
			}
		}
		if(max!=0){
		for(i=0;i<=2;i++){
			output[i]=output[i]/max;
		}
		//serial.printf("%f,%f,%f\n\r",output[0],output[1],output[2]);
		motor0.duty(output[0]);
		motor0.cycle();
		motor1.duty(output[1]);
		motor1.cycle();
		motor2.duty(output[2]);
		motor2.cycle();
		}
}
void ROBOT::selfLocation(){
		CanEncoder canEnc0(can,0,0x500);
		CanEncoder canEnc1(can,1,0x500);
		CanEncoder canEnc2(can,2,0x500);
		canEnc0.setup();
		canEnc1.setup();
		canEnc2.setup();
		L0=(float)canEnc0.count()/400.0*30.0*M_PI;
		L1=(float)canEnc1.count()/1000.0*30.0*M_PI;
		L2=(float)canEnc2.count()/200.0*30.0*M_PI;
		//serial.printf("%.2f %.2f %.2f\n\r",L0,L1,L2);
/****************************ここから角度***************************************/
		bodyAngle=(L0+L1-L2)/3.0/115.0;
		i=(bodyAngle/M_PI);//180～-180までに収める
		if(i>2){
			i=ceil(bodyAngle/M_PI);
		}
		if(i>=1&&i<2){
			bodyAngle=bodyAngle-i*2*M_PI;
		}
		else{
			bodyAngle=bodyAngle-i*M_PI;
		}
/***************************ここから自己位置************************************/
		x1=-tan(M_PI/6)*(L0-L2/sin(M_PI/6));
		y1=L0;
		//serial.printf("x1 %f y1 %f\n\r",x1,y1);
		x2=tan(M_PI/6)*(L0+L1/sin(M_PI/6));
		y2=L0;
		//serial.printf("x2 %f y2 %f\n\r",x2,y2);
		x3=tan(M_PI/6)*(L1+L2)/(2*sin(M_PI/6));
		y3=(L1-L2)/(2*sin(M_PI/6));
		//serial.printf("x3 %f y3 %f\n\r",x3,y3);
		x_Coordinate=(x1+x2+x3)/3;
		y_Coordinate=(y1+y2+y3)/3;
		distance=hypot(x_Coordinate,y_Coordinate);
		serial.printf("%f,%f\n\r",x_Coordinate,y_Coordinate);
		correct_X_Coordinate=x_Coordinate*cos(bodyAngle)-y_Coordinate*sin(bodyAngle);
		correct_Y_Coordinate=x_Coordinate*sin(bodyAngle)+y_Coordinate*cos(bodyAngle);
		//serial.printf("%f,%f \n\r",correct_X_Coordinate,correct_Y_Coordinate);
		y_Deviation=y-correct_Y_Coordinate;//x軸の偏差
		x_Deviation=x-correct_X_Coordinate;//ｙ軸の偏差
		currentDeviation=targetDistance-distance;//現在の距離と目標距離の偏差
		pastDeviation=currentDeviation;//過去の距離と目標距離の偏差
		//serial.printf("%d,%d,%d\n",enc1.count(),enc2.count(),enc3.count());
}
void ROBOT::test(){
		while(1){
		MiniMD motor0(cw0,ccw0,pwm0);
		MiniMD motor1(cw1,ccw1,pwm1);
		MiniMD motor2(cw2,ccw2,pwm2);
		motor0.duty(0);
		motor0.cycle();
		motor1.duty(1);
		motor1.cycle();
		motor2.duty(1);
		motor2.cycle();
}
}
ROBOT::ROBOT(){
		MiniMD motor0(cw0,ccw0,pwm0);
		MiniMD motor1(cw1,ccw1,pwm1);
		MiniMD motor2(cw2,ccw2,pwm2);
		can.setup();
		buz.setupDigitalOut();
		motor0.setup();
		motor1.setup();
		motor2.setup();
		enc1.setup();
		enc2.setup();
		enc0.setup();
		serial.setup(115200);
		target_x_Coordinate=500.0;//目標座標
		target_y_Coordinate=0;
		targetDistance=hypotf(x,y);//目標距離
	}
int main(void){
		float time=0;
		ROBOT J;
		while(1){
			if(millis()-time>5){
				time=millis();
				J.selfLocation();
				J.targetOutput();
				J.angleLock();
				J.total();
				//J.test();
			}
		}
	return 0;
	}
