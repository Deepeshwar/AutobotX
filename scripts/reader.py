#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from autobotx.msg import Unicycle
def to_speed(data):
     print data


    # twist = Twist()
    # twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
    # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
    # print("currently:\tspeed %s\tturn %s " % (data,data))
    #rospy.loginfo( 'linera %f angular %f',data.linear,data.angular)
    #return "currently:\fspeed %s\tturn %s " % (twist.linear,twist.angular)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('cmd_vel',Unicycle,to_speed)

    rospy.spin()





if __name__ == '__main__':
    listener()
