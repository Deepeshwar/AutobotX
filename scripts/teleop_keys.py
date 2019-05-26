#!/usr/bin/env python

from __future__ import print_function

import roslib
import rospy
from autobotx.msg import Unicycle
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Unicycle v and w!
---------------------------
key     v       w
---------------------------
w       +       0
a       0       -
s       0       +
d       0       +
q       -       -
e       +       +

' '     0       0

CTRL-C to quit

"""
'''
moveBindings = {
        'w':(1,0),
        'a':(1,1),
        'd':(1,-1),
        's':(-1,0),
        'q':(0,1),
        'e':(0,-1),
        ' ':(0,0),

    }

speedBindings={
        '8':(0.1,0),
        '2':(-0.1,0),
        '4':(0,-0.1),
        '6':(0,0.1),
    }
'''
moveBindings = {
        ' ':(0,0),

    }

speedBindings={
        'w':(0.1,0),
        's':(-0.1,0),
        'd':(0,-0.1),
        'a':(0,0.1),

        'e':(0.1,-0.1),
        'q':(-0.1,0.1),

    }
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    #pub = rospy.Publisher('cmd_vel',Unicycle, queue_size = 1)
    pub = rospy.Publisher('cmd_vel',Twist, queue_size = 1)
    rospy.init_node('teleop_twist_keyboard')
    #speed = rospy.get_param("~speed", 0.5)
    #turn = rospy.get_param("~turn", 1.0)
    speed=0
    turn=0

    print(msg)
    print(vels(speed,turn))
    while(1):
        key = getKey()
        if key in moveBindings.keys():
            speed = moveBindings[key][0]
            turn = moveBindings[key][1]

        if key in speedBindings.keys():

            speed = speed + 34 * speedBindings[key][0]
            turn = turn   +1.5 * speedBindings[key][1] #2.31503 * speedBindings[key][1]
            if speed >= 34 :
                speed = 34
            elif speed <=-34 :
                speed =-34
            if turn >= 2.31503 :
                turn =2.31503
            elif turn <= -2.31503:
                turn =-2.31503



            #print(vels(speed,turn))

        else:
            x = 0
            y = 0
            z = 0
            th = 0
            if (key == '\x03'):
                break

        #pub.publish(speed,turn)

        unicycle = Unicycle()
        twist=Twist()
        #twist.linear.x = speed; twist.linear.y = 0; twist.linear.z =0;void chatterCallback(const autobotx::Unicycle::ConstPtr& msg)

        #twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn
        #pub.publish(twist.linear,twist.angular)

        twist.linear.x=float(speed/100)
        twist.angular.z = float(turn)
        pub.publish(twist)

        print(twist)
'''
finally:
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
'''
