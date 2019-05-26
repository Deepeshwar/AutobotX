#include <stdio.h>
#include <boost/thread.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "autobotx/Unicycle.h"
#include "serial_port.h"
#include "mavlink_communication.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

std::string uart_name = "/dev/ttyACM0";
int baudrate = 115200;
Serial_Port *serial_port;

#define unit_corr   100.0

float v = 0;
float w = 0;

uint16_t x_mav = 0;



mavlink_message_t message;
mavlink_robot_sensor_readings_t mav_robot_sensor_read;
mavlink_robot_position_change_t mav_robot_position_change;


void mav_decode();


void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  v = msg->linear.x*100;
  w = msg->angular.z;
  mavlink_message_t message_send;
  printf("velocity: [%f] \t w: [%f]\n", v, w);
  x_mav = mavlink_msg_desire_cmd_val_pack(SYSTEM_ID, COMPONENT_ID, &message_send, v, w);
  serial_port->write_message(message_send, x_mav);
}

double x = 0.0;
double y = 0.0;
double th = 0.0;
double prev_th = 0.0;
double d_th = 0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

struct pose{
  float x;
  float y;
  float theta;
} robot_pose;

ros::Time current_time, last_time;
nav_msgs::Odometry odom;
tf::TransformBroadcaster *odom_broadcaster;

ros::Publisher *pub_odom;
inline void publish_odom(void)
{

  current_time = ros::Time::now();



  robot_pose.x /= unit_corr;
  robot_pose.y /= unit_corr;

  x += robot_pose.x;
  y += robot_pose.y;
  th = robot_pose.theta;
  //th = atan2(sin(th), cos(th));

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  geometry_msgs::TransformStamped odom_trans;
  current_time = ros::Time::now();
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;





  vx = sqrt(robot_pose.x*robot_pose.x + robot_pose.y*robot_pose.y) * 10;

  d_th = th - prev_th;
  vth = atan2(sin(d_th), cos(d_th)) * 10;

  current_time = ros::Time::now();
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;

  prev_th = th;
  //publish the message
  odom_broadcaster->sendTransform(odom_trans);
  pub_odom->publish(odom);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "link");

  ros::NodeHandle n;
  ros::NodeHandle nh_private_("~");
  ros::Subscriber sub = n.subscribe("cmd_vel", 100, chatterCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  pub_odom = &odom_pub;
  tf::TransformBroadcaster odom_broadcaster2;
  odom_broadcaster = &odom_broadcaster2;

  nh_private_.getParam("controller_usb", uart_name);

  Serial_Port SerialPort((const char *)uart_name.c_str(), baudrate);
  SerialPort.start();

  serial_port = &SerialPort;
  ros::Rate r(1000);

  while (ros::ok())
  {
    mav_decode();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}


void mav_decode(void)
{

  bool success = serial_port->read_message(message);

  if(success){
    switch(message.msgid){
      case MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS :
        mavlink_msg_robot_sensor_readings_decode(&message, &mav_robot_sensor_read);
        printf("left wheel ticks:[%u] \t right wheel ticks:[%u] \t curr heading:[%f]\n", mav_robot_sensor_read.left_wheel_ticks,mav_robot_sensor_read.right_wheel_ticks,mav_robot_sensor_read.curr_heading);
        break;

      case MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE :

        mavlink_msg_robot_position_change_decode(&message, &mav_robot_position_change);
        robot_pose.x = mav_robot_position_change.delta_x;
        robot_pose.y = mav_robot_position_change.delta_y;
        robot_pose.theta = mav_robot_position_change.delta_theta*3.1416/180.0;
        //printf("delta x:[%f] \t delta y:[%f] \t delta theta:[%f]\n", mav_robot_position_change.delta_x,mav_robot_position_change.delta_y,mav_robot_position_change.delta_theta);
        publish_odom();
        break;

      default:
        break;
    }
  }
}
