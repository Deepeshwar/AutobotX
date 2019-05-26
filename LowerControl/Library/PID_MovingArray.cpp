
#include "PID_MovingArray.h"


float degreeToRad(float degree) 
{
	return degree * pi / 180.0;
}

float radToDegree(float rad) 
{
	return rad * 180.0 / pi;
}

float normalizeAngle(float angle) 
{
	int temp;
	
	if(angle<0)
	{
		angle = angle - ((int)angle/360 - 1)*360;
	}
	
	temp = (int)angle/180;
	
	return ((temp%2)?(angle - temp*180.0 - 180):(angle - temp*180.0));
	
	//return radToDegree(atan2(sin(degreeToRad(angle)), cos(degreeToRad(angle))));
}

float sigmoid(int z,float scale)
{
	return tanh(z/scale);
}

void PID::Init(float p , float i, float d)
{
	Kp = p;
	Ki = i;
	Kd = d;
};

float PID::ImplementPID(float error)
{
	
	PID::E += error;
	
	
	PID::pid = (PID::Kp*error) + (PID::Ki*PID::E) + (PID::Kd*(error - PID::E_old));
	
	PID::E_old = error;
	return PID::pid;
};

void MovingArray::Init(uint8_t Ind)
{
	Index = Ind;
	n = 1;
	i = 255;
	Average = 0;
};

float MovingArray::ImplementMovingArray(float val)
{
	i++;
	
	if(n)				//Only execute at starting
	{
		Array[i] = val;
		Average += val;
		
		if (i == (Index-1))
		{
			n = 0;
			Average = Average/Index;
		}
		 
		return val;
	}
	else
	{
		if (i == Index)
		{
			i = 0;
		}
		
		Average = (Average*Index - Array[i] + val)/Index;
		Array[i] = val;
		
		
		return Average;			
	}
	
};
