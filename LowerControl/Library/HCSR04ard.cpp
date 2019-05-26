#include "HCSR04ard.h"
#include <arduino.h>

void Ultrasonic_init(void)
{
	pinMode(HCSR_01_ECHO, INPUT);
    pinMode(HCSR_01_TRIG, OUTPUT);
	
	pinMode(HCSR_02_ECHO, INPUT);
    pinMode(HCSR_02_TRIG, OUTPUT);
	
	pinMode(HCSR_03_ECHO, INPUT);
    pinMode(HCSR_03_TRIG, OUTPUT);
	
	pinMode(HCSR_04_ECHO, INPUT);
    pinMode(HCSR_04_TRIG, OUTPUT);
	
	pinMode(HCSR_05_ECHO, INPUT);
    pinMode(HCSR_05_TRIG, OUTPUT);
}
	
void Ultrasonic_get_distance(float* distance)
{
	digitalWrite(HCSR_01_TRIG, LOW);
	delayMicroseconds(5);
	digitalWrite(HCSR_01_TRIG, HIGH);
	delayMicroseconds(10);
	digitalWrite(HCSR_01_TRIG, LOW);

	float dt = pulseIn(HCSR_01_ECHO, HIGH, TimeOut);
	distance[0] = dt*0.017;
	
	digitalWrite(HCSR_02_TRIG, LOW);
	delayMicroseconds(5);
	digitalWrite(HCSR_02_TRIG, HIGH);
	delayMicroseconds(10);
	digitalWrite(HCSR_02_TRIG, LOW);
	
	dt = pulseIn(HCSR_02_ECHO, HIGH, TimeOut);
	distance[1] = dt*0.017;
	
	digitalWrite(HCSR_03_TRIG, LOW);
	delayMicroseconds(5);
	digitalWrite(HCSR_03_TRIG, HIGH);
	delayMicroseconds(10);
	digitalWrite(HCSR_03_TRIG, LOW);
	
	dt = pulseIn(HCSR_03_ECHO, HIGH, TimeOut);
	distance[2] = dt*0.017;
	
	digitalWrite(HCSR_04_TRIG, LOW);
	delayMicroseconds(5);
	digitalWrite(HCSR_04_TRIG, HIGH);
	delayMicroseconds(10);
	digitalWrite(HCSR_04_TRIG, LOW);
	
	dt = pulseIn(HCSR_04_ECHO, HIGH, TimeOut);
	distance[3] = dt*0.017;
	
	digitalWrite(HCSR_05_TRIG, LOW);
	delayMicroseconds(5);
	digitalWrite(HCSR_05_TRIG, HIGH);
	delayMicroseconds(10);
	digitalWrite(HCSR_05_TRIG, LOW);
	
	dt = pulseIn(HCSR_05_ECHO, HIGH, TimeOut);
	distance[4] = dt*0.017;
	
	//distance[4] = 10;
}