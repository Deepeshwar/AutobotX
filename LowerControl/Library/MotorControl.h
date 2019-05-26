#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_
#include "PID_MovingArray.h"
#include <DuePWM.h>

#define LEFT_MOTOR_PWM_PIN    6   //MOTOR1 = LEFT MOTOR
#define RIGHT_MOTOR_PWM_PIN   7
#define LEFT_MOTOR_DIR_PIN    2
#define RIGHT_MOTOR_DIR_PIN   3

#define LEFT_MOTOR_ENCA_PIN   8		//PC22
#define LEFT_MOTOR_ENCB_PIN   9
#define RIGHT_MOTOR_ENCA_PIN  12	//PD8
#define RIGHT_MOTOR_ENCB_PIN  13

#define PID_LOOP_FREQ         100    //in Hz
#define PWM_FREQ              3000      //in Hz
#define PWM_MAX_DUTYCYCLE     255.0

class MotorControl
{
  public:
    struct motor_t {
      uint32_t dirpin; 
      uint32_t pwmpin;
      uint32_t encoderApin;
      uint32_t encoderBpin;
      volatile int32_t ticks;
      uint32_t ppr;
      float desirerpm;
      float currentrpm;
      float pwm;    //0-100
     };
	 
	float robot_wheel_radius, robot_distance_bw_wheels;
	float cmd_val_v, cmd_val_w;

    struct motor_t leftmotor, rightmotor;

    PID pid_leftmotor, pid_rightmotor;
    MovingArray array_leftmotor, array_rightmotor;

    void cal_rpm();
    void PERIPHERAL_init(void);
};

void PWM_update_dutycycle(float duty_leftmotor, float duty_rightmotor);

extern MotorControl motorcontrol;
extern DuePWM pwm;

void leftmotor_interrupt_Handler(void);
void rightmotor_interrupt_Handler(void);
void actuate_wheels(float duty_leftmotor, float duty_rightmotor);
#endif
