#!/usr/bin/env python

from __future__ import print_function

import roslib 
# roslib.load_manifest('teleop')
import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

def getKey():
    """
    Function to get keyboard input

    :return: key pressed
    :rtype: char
    """
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    """
    Function to get current velocity (speed and heading direction)
    :param speed: linear velocity (m/sec)
    :param turn: heading direction (radians) 
    
    :return: typeset string useful for displaying current velocity
    :rtype: string
    """
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('teleop_twist_keyboard')

    # Publisher for velocity command fed to Whiskeye
    pub = rospy.Publisher('/whiskeye/body/cmd_vel', Twist, queue_size = 1)
    # Publisher for odometry fed to vitaslam
    pub_odom = rospy.Publisher('/whiskeye/odom', Odometry, queue_size = 1)

    speed = rospy.get_param("~speed", 0.5)  # target linear velocity
    turn = rospy.get_param("~turn", 1.0)  # angle for turning

    # Position variables
    x = 0
    y = 0
    z = 0
    
    # orientation variables
    th = 0
    
    try:
        print(msg)  # print usage instructions
        print(vels(speed,turn)) # print robot velocity information

        while(1):
            key = getKey()  # get the key pressed

            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]

            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))


            else:
                # Reset parameters if arbitrary key is pressed
                x = 0
                y = 0
                z = 0
                th = 0
                print(msg)  # Show the usage instructions again

                if (key == '\x03'):  # CTRL-C pressed
                    break

            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn

            # Publish commands to the robot
            pub.publish(twist)
            # Publish the odometry to vitaslam
            o = Odometry()
            o.header.stamp = rospy.Time.now()
            o.twist.twist.linear.x = twist.linear.x
            o.twist.twist.linear.y = twist.linear.y
            o.twist.twist.angular.z = twist.angular.z
            pub_odom.publish(o)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
