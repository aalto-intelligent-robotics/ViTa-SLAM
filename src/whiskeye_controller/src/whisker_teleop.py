#!/usr/bin/env python
from __future__ import print_function
import roslib 
import rospy
from std_msgs.msg import Float32MultiArray
import sys, select, termios, tty
import numpy as np

whisker_number = 24
msg = """
___________________________________________________________________#
WhiskEye Whisker Teleop
Controls all 24 whiskers at once
Reading from the keyboard and Publishing to /whiskeye/head/whisker_cmd
____________________________________________________________________
Decrease theta: d
Increase theta: f
Increase whisker speed: a
Decrease whisker speed: s
"""

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
    rospy.init_node('teleop_whiskeye_whisker')

    # Publisher for velocity command fed to Whiskeye
    pub = rospy.Publisher('/whiskeye/head/theta_cmd', Float32MultiArray, queue_size = 10)

    speed = rospy.get_param("~speed", 0.05)  # target linear velocity

    theta = 0
    
    
    try:
        print(msg)  # print usage instructions

        while(1):
            key = getKey()  # get the key pressed

            if (key == '\x03'):  # CTRL-C pressed
                break

            print("key: {}".format(key))
            if key == 'f' and theta < 0.95:
                theta += speed
            elif key == 'd' and theta > -0.95:
                theta -= speed
            elif key == 'a' and speed + 0.05 < 0.5:
                speed += 0.05
                rospy.loginfo("whisker speed increased to {}".format(speed))
            elif key == 's' and speed - 0.05 > 0:
                speed -= 0.05
                rospy.loginfo("whisker speed decreased to {}".format(speed))
            elif key == 'r':
                theta = 0


            out = Float32MultiArray()
            out.data = [theta]*whisker_number

            # Publish commands to the robot
            pub.publish(out)

    except Exception as e:
        print(e)

    finally:
        out = Float32MultiArray()
        out.data = [theta]
        pub.publish(out)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
