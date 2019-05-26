/*
 * PID.h
 *
 * Created: 15-04-2017 23:23:46
 *  Author: Harshit
 */ 


#ifndef PID_H_
#define PID_H_

#include <inttypes.h>
#include <math.h>

#define pi	3.1416

float degreeToRad(float degree);
float radToDegree(float rad);
float normalizeAngle(float degree);
float sigmoid(int z,float scale);

class PID
{
	public:
		void Init(float Kp , float Ki, float Kd);
		float ImplementPID(float error);
		
	private:
		float pid,E,E_old;
		float Kp,Ki,Kd;	
};


class MovingArray
{
	public:
		void Init(uint8_t Index_Max50);
		float ImplementMovingArray(float val);
		
	private:
		uint8_t i,Index;
		bool n;
		float Average;
		float Array[50];	
};



#endif /* PID_H_ */




