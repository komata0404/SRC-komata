#include "selfLocation.hpp";
void selfLoction::BodyAngle(){
				L0=(float)canEnc0->count()/400.0*30.0*M_PI;
				L1=(float)canEnc2->count()/200.0*30.0*M_PI;
				L2=(float)canEnc1->count()/1000.0*30.0*M_PI;
				//seriar.printf("%.2f %.2f %.2f\n\r",L0,L1,L2);

				bodyAngie=(L0+L1+L2)/3.0/115.0;
				/*i=(bodyAngie/180);//180`-180‚Ü‚Å‚Éû‚ß‚é
				if(i>2){
					i=ceil(bodyAngie/180);
				}
				if(i>=1&&i<2){
					bodyAngie=bodyAngie-i*360;
				}
				else{
					bodyAngie=bodyAngie-i*180;
				}*/
}
void selfLocation::selfPosition(){
				L0=(float)canEnc0->count()/400.0*30.0*M_PI;
				L1=(float)canEnc2->count()/200.0*30.0*M_PI;
				L2=(float)canEnc1->count()/1000.0*30.0*M_PI;
				x1=-tan(M_PI/6)*(L0+L2/sin(M_PI/6));
				y1=L0;
				 //seriar.printf("x1 %f y1 %f\n\r",x1,y1);
				x2=tan(M_PI/6)*(L0+L1/sin(M_PI/6));
				y2=L0;
				 //seriar.printf("x2 %f y2 %f\n\r",x2,y2);
				x3=tan(M_PI/6)*(L1-L2)/(2*sin(M_PI/6));
				y3=(-L1-L2)/(2*sin(M_PI/6));
				 //seriar.printf("x3 %f y3 %f\n\r",x3,y3);
				x_Coordinate=(x1+x2+x3)/3;
				y_Coordinate=(y1+y2+y3)/3;
				distance=hypot(x_Coordinate,y_Coordinate);
				//seriar.printf("JKIT dis %f\n\r",x);
				//seriar.printf("x%f,y%f\n\r",x_Coordinate,y_Coordinate);
				correct_X_Coordinate=x_Coordinate*cos(bodyAngie)-y_Coordinate*sin(bodyAngie);
				correct_Y_Coordinate=y_Coordinate*cos(bodyAngie)+x_Coordinate*sin(bodyAngie);
				//seriar.printf("%f,%f \n\r",correct_X_Coordinate,correct_Y_Coordinate);
				y_Deviation=y-correct_Y_Coordinate;//x²‚Ì•Î·
				x_Deviation=x-correct_X_Coordinate;//‚™²‚Ì•Î·
				currentDeviation=targetDistance-distance;//Œ»İ‚Ì‹——£‚Æ–Ú•W‹——£‚Ì•Î·
				//motorCycle();
				//test();
				pastDeviation=currentDeviation;//‰ß‹‚Ì‹——£‚Æ–Ú•W‹——£‚Ì•Î·
				//seriar.printf("%d,%d,%d\n",enc1.count(),enc2.count(),enc3.count());

}
