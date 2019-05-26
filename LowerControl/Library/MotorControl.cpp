#include "MotorControl.h"
#include <arduino.h>

DuePWM pwm(PWM_FREQ, PWM_FREQ);
MotorControl motorcontrol;

void MotorControl::cal_rpm(void)
{
  MotorControl::leftmotor.currentrpm = MotorControl::array_leftmotor.ImplementMovingArray((((float)MotorControl::leftmotor.ticks*(float)PID_LOOP_FREQ*60.0)/(float)MotorControl::leftmotor.ppr));
  MotorControl::leftmotor.ticks = 0;
  
  MotorControl::rightmotor.currentrpm = MotorControl::array_rightmotor.ImplementMovingArray((((float)MotorControl::rightmotor.ticks*(float)PID_LOOP_FREQ*60.0)/(float)MotorControl::rightmotor.ppr));
  MotorControl::rightmotor.ticks = 0;
}

void MotorControl::PERIPHERAL_init(void)
{
  pinMode(MotorControl::leftmotor.dirpin, OUTPUT);
  pinMode(MotorControl::leftmotor.pwmpin, OUTPUT);
  pinMode(MotorControl::leftmotor.encoderApin, INPUT);
  pinMode(MotorControl::leftmotor.encoderBpin, INPUT);

  pinMode(MotorControl::rightmotor.dirpin, OUTPUT);
  pinMode(MotorControl::rightmotor.pwmpin, OUTPUT);
  pinMode(MotorControl::rightmotor.encoderApin, INPUT);
  pinMode(MotorControl::rightmotor.encoderBpin, INPUT);

  pwm.setFreq1(PWM_FREQ);
  pwm.pinFreq1(MotorControl::leftmotor.pwmpin);
  pwm.pinFreq1(MotorControl::rightmotor.pwmpin);
  
}

void leftmotor_interrupt_Handler(void)
{
  if (digitalRead(motorcontrol.leftmotor.encoderBpin))
    motorcontrol.leftmotor.ticks += 1;
  else
    motorcontrol.leftmotor.ticks -= 1;
}

void rightmotor_interrupt_Handler(void)
{
  if (digitalRead(motorcontrol.rightmotor.encoderBpin))
    motorcontrol.rightmotor.ticks += 1;
  else
    motorcontrol.rightmotor.ticks -= 1;
}

void PWM_update_dutycycle(float duty_leftmotor, float duty_rightmotor)
{
  pwm.pinDuty(motorcontrol.leftmotor.pwmpin, (duty_leftmotor/100.0)*PWM_MAX_DUTYCYCLE);
  pwm.pinDuty(motorcontrol.rightmotor.pwmpin, (duty_rightmotor/100.0)*PWM_MAX_DUTYCYCLE);
}

void actuate_wheels(float duty_leftmotor, float duty_rightmotor)
{
	if(duty_leftmotor >= 0)
	{
		digitalWrite(motorcontrol.leftmotor.dirpin, LOW);
		if(duty_leftmotor > 100)
		{
			duty_leftmotor = 100;
		}
	}
	else
	{
		digitalWrite(motorcontrol.leftmotor.dirpin, HIGH);
		if(duty_leftmotor < -100)
		{
			duty_leftmotor = -100;
		}
		duty_leftmotor *= -1;
	}

	if(duty_rightmotor >= 0)
	{
		digitalWrite(motorcontrol.rightmotor.dirpin, LOW);
		if(duty_rightmotor > 100)
		{
			duty_rightmotor = 100;
		}
	}
	else
	{
		digitalWrite(motorcontrol.rightmotor.dirpin, HIGH);
		if(duty_rightmotor < -100)
		{
			duty_rightmotor = -100;
		}
		duty_rightmotor *= -1;
	}
	
	PWM_update_dutycycle(duty_leftmotor, duty_rightmotor);
}

