#include "MotorControl.h"
#include "mavlink_communication.h"
#include <DueTimer.h>
#include <Wire.h>
#include <hmc5883l.h>
#include "HCSR04ard.h"

/**
 * define value as per required characteristic (i.e. data to send to ROS using MAVLink)
 */
#define   SEND_ODOMETRY
//#define   SEND_RPM
//#define   SEND_ULTRASONIC_READING

//#define   PRINT_ODOMETRY
//#define   PRINT_RPM
//#define   PRINT_ULTRASONIC_READING

/**
 * other important parameters
 */
#define   MOV_ARRAY_SIZE  5
#define   ROBOT_DIM_L   31.8  //in cm
#define   ROBOT_DIM_R   3.7   //in cm
#define   ROBOT_DIM_PERIPHERY   23.2478

float heading, initial_heading;
hmc5883l compass;
float ultra_reading[5] = {0, 0, 0, 0, 0};
uint16_t counter = 0;
float delta_x = 0;
float delta_y = 0;
float delta_heading = 0;
float last_heading = 0;
float delta_theta = 0;
float robot_wheel_peri = 0;
float x = 0, y =0;
float leftDist, rightDist, dist;
uint32_t mil = 0, last_mil = 0; 
uint8_t rec;

void setup() {
  motorcontrol.leftmotor = {
    LEFT_MOTOR_DIR_PIN,
    LEFT_MOTOR_PWM_PIN,
    LEFT_MOTOR_ENCA_PIN,
    LEFT_MOTOR_ENCB_PIN,
    0,
    7380,
    0.0,
    0.0,
    0.0
  };

  motorcontrol.rightmotor = {
    RIGHT_MOTOR_DIR_PIN,
    RIGHT_MOTOR_PWM_PIN,
    RIGHT_MOTOR_ENCA_PIN,
    RIGHT_MOTOR_ENCB_PIN,
    0,
    7380,
    0.0,
    0.0,
    0.0
  };

  motorcontrol.PERIPHERAL_init();
  motorcontrol.pid_leftmotor.Init(1.5, 0.1, 0.2);
  motorcontrol.pid_rightmotor.Init(1.5, 0.1, 0.2);

  motorcontrol.array_leftmotor.Init(MOV_ARRAY_SIZE);
  motorcontrol.array_rightmotor.Init(MOV_ARRAY_SIZE);

  motorcontrol.robot_distance_bw_wheels = ROBOT_DIM_L;   //in cm
  motorcontrol.robot_wheel_radius = ROBOT_DIM_R;
  robot_wheel_peri = ROBOT_DIM_PERIPHERY; //2*pi*motorcontrol.robot_wheel_radius;
  
  Serial.begin(115200);

//  Wire.begin();
//  compass = hmc5883l();
//  compass.SetScale(1.3); 
//  compass.SetMeasurementMode(Measurement_Continuous);
//  delay(100);
//  initial_heading = 0;  //getHeading(compass);

  Ultrasonic_init();
  
  attachInterrupt(digitalPinToInterrupt(motorcontrol.leftmotor.encoderApin), leftmotor_interrupt_Handler, FALLING);
  attachInterrupt(digitalPinToInterrupt(motorcontrol.rightmotor.encoderApin), rightmotor_interrupt_Handler, FALLING);

  Timer2.attachInterrupt(pidloop_interrupt_Handler);
  Timer2.setFrequency(PID_LOOP_FREQ);

  //pwm.pinDuty(LEFT_MOTOR_PWM_PIN, 0.99*255);
  //pwm.pinDuty(RIGHT_MOTOR_PWM_PIN, 0.99*255);
  motorcontrol.leftmotor.desirerpm = 0;
  motorcontrol.rightmotor.desirerpm = 0;
  //motorcontrol.leftmotor.pwm = 60;
  //motorcontrol.rightmotor.pwm = 60;
  actuate_wheels(motorcontrol.leftmotor.pwm, motorcontrol.rightmotor.pwm);

  NVIC_SetPriority(PIOC_IRQn, 0);   //LEFT_MOTOR_ENCA_PIN
  NVIC_SetPriority(PIOD_IRQn, 1);   //RIGHT_MOTOR_ENCA_PIN
  NVIC_SetPriority(UART_IRQn, 2);
  NVIC_SetPriority(TC2_IRQn, 15);
  //NVIC_SetPriority(TC0_IRQn, 15);
  //NVIC_SetPriority(TC1_IRQn, 15);
  
  Timer2.start();
  //millis();
  //micros();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void pidloop_interrupt_Handler_test3(void)
{
    mav_send_robot_sensor_readings(100, 100, 10.0);
}

void pidloop_interrupt_Handler_test2(void)
{
  motorcontrol.cal_rpm();
  motorcontrol.leftmotor.pwm = motorcontrol.pid_leftmotor.ImplementPID(motorcontrol.leftmotor.desirerpm - motorcontrol.leftmotor.currentrpm);
  motorcontrol.rightmotor.pwm = motorcontrol.pid_rightmotor.ImplementPID(motorcontrol.rightmotor.desirerpm - motorcontrol.rightmotor.currentrpm);
  
  actuate_wheels(motorcontrol.leftmotor.pwm, motorcontrol.rightmotor.pwm);  //motorcontrol.rightmotor.pwm);
  Serial.print(motorcontrol.leftmotor.currentrpm);
  Serial.write('\t');
  Serial.println(motorcontrol.rightmotor.currentrpm);
}

void pidloop_interrupt_Handler_test(void)
{
//  float k1 = motorcontrol.rightmotor.ticks;
//  float k2 = motorcontrol.leftmotor.ticks;

  Serial.print(motorcontrol.leftmotor.ticks);
  Serial.print(",");
  Serial.print(motorcontrol.rightmotor.ticks);
  Serial.write('\t');

  motorcontrol.cal_rpm();

  #ifdef PRINT_RPM
    Serial.print(motorcontrol.leftmotor.currentrpm);
    Serial.print(",");
    Serial.println(motorcontrol.rightmotor.currentrpm);
//    Serial.print(",");
//    Serial.print(initial_heading);
//    Serial.print(",");
//    Serial.println(heading);
  #endif
}
void pidloop_interrupt_Handler(void)
{
  motorcontrol.cal_rpm();

  leftDist = ((motorcontrol.leftmotor.currentrpm/60.0) * robot_wheel_peri) / PID_LOOP_FREQ;
  rightDist = ((motorcontrol.rightmotor.currentrpm/60.0) * robot_wheel_peri) / PID_LOOP_FREQ;
  dist = (leftDist + rightDist) / 2;
  heading += radToDegree((rightDist - leftDist))/motorcontrol.robot_distance_bw_wheels;
  //heading = getHeading(compass);    //it causes missing of encoder ticks;
  heading = normalizeAngle(heading);// - initial_heading);
  delta_x += dist * cos(degreeToRad(heading));
  delta_y += dist * sin(degreeToRad(heading));

  if((motorcontrol.leftmotor.desirerpm == 0) && (motorcontrol.rightmotor.desirerpm == 0))
  {
    motorcontrol.leftmotor.pwm = 0;
    motorcontrol.rightmotor.pwm = 0;
  }
  else
  {
    motorcontrol.leftmotor.pwm = motorcontrol.pid_leftmotor.ImplementPID(motorcontrol.leftmotor.desirerpm - motorcontrol.leftmotor.currentrpm);
    motorcontrol.rightmotor.pwm = motorcontrol.pid_rightmotor.ImplementPID(motorcontrol.rightmotor.desirerpm - motorcontrol.rightmotor.currentrpm);
  
  }
  
  
  actuate_wheels(motorcontrol.leftmotor.pwm, motorcontrol.rightmotor.pwm);
    
  counter++;
  if((counter%10) == 0)
  {
    odometry_interrupt_Handler();
    if(counter == 90)
    {
      //ultrasonic_interrupt_Handler();
    }
    if(counter == 100)
    {
      //ultrasonic_interrupt_Handler();
      
      counter = 0;
    }
  }

  #ifdef PRINT_RPM
    Serial.print(motorcontrol.leftmotor.currentrpm);
    Serial.print(",");
    Serial.println(motorcontrol.rightmotor.currentrpm);
//    Serial.print(",");
//    Serial.print(initial_heading);
//    Serial.print(",");
//    Serial.println(heading);
   #endif
  
  #ifdef SEND_RPM
    mav_send_robot_sensor_readings(motorcontrol.leftmotor.currentrpm, motorcontrol.rightmotor.currentrpm, heading);
  #endif

  //mil = micros() - mil;
  //Serial.println(mil);
}

void odometry_interrupt_Handler(void)
{
  //transmit delta_x, delta_y, delta_theta
  //delta_heading = normalizeAngle(heading - last_heading);

  #ifdef PRINT_ODOMETRY
    x += delta_x;
    y += delta_y;
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(",");
    Serial.println(heading);
  #endif
  
  #ifdef SEND_ODOMETRY
    mav_send_robot_position_change(delta_x, delta_y, heading);
  #endif

  //last_heading = heading;
  delta_x = 0;
  delta_y = 0;
}

void ultrasonic_interrupt_Handler(void)
{
  //read ultrasonic and transmit the data
  Ultrasonic_get_distance(ultra_reading);

  #ifdef PRINT_ULTRASONIC_READING
    Serial.print(ultra_reading[0]);
    Serial.print(",");
    Serial.print(ultra_reading[1]);
    Serial.print(",");
    Serial.print(ultra_reading[2]);
    Serial.print(",");
    Serial.print(ultra_reading[3]);
    Serial.print(",");
    Serial.println(ultra_reading[4]);
  #endif

  #ifdef SEND_ULTRASONIC_READING
    mav_send_robot_distance_sensor_readings((const uint16_t*)ultra_reading);
  #endif
}

void serialEvent() 
{
  if(Serial.available()) 
  {
      rec = Serial.read();
      //Serial.write(rec);
      mav_decode_rec_byte(rec);
  }
}
