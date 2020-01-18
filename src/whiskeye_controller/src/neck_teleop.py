#!/usr/bin/env python
from __future__ import print_function
import roslib 
import rospy
from std_msgs.msg import Float32MultiArray
import sys, select, termios, tty
import numpy as np

msg = """
___________________________________________________________________#
WhiskEye Neck Teleop
Reading from the keyboard and Publishing to /whiskeye/head/neck_cmd
____________________________________________________________________
Decrease neck elevation: j
Increase neck eleveation: k
Decrese neck pitch: u
Increase neck pitch: i
Decrease neck yaw: h
Increase neck yaw: l
Reset to original: r
Increase speed: +
Decrease speed: -
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
    rospy.init_node('teleop_whiskeye_neck')

    # Publisher for velocity command fed to Whiskeye
    pub = rospy.Publisher('/whiskeye/head/neck_cmd', Float32MultiArray, queue_size = 10)

    speed = rospy.get_param("~speed", 0.05)  # target linear velocity

    neck_elevation = -np.pi/4
    neck_pitch = np.pi/4 
    neck_yaw = 0
    
    
    try:
        print(msg)  # print usage instructions

        while(1):
            key = getKey()  # get the key pressed

            if (key == '\x03'):  # CTRL-C pressed
                break

            print("key: {}".format(key))
            if key == 'j' and neck_elevation <= -0.65:
                neck_elevation += speed
            elif key == 'k' and neck_elevation >= -1.4:
                neck_elevation -= speed
            elif key == 'u' and neck_pitch <= 2.9:
                neck_pitch += speed
            elif key == 'i' and neck_pitch >= -1:
                neck_pitch -= speed
            elif key == 'h' and neck_yaw <= 1.5:
                neck_yaw += speed
            elif key == 'l' and neck_yaw >= -1.5:
                neck_yaw -= speed
            elif key == '+' and speed-0.05 <= 0.4:
                speed += 0.05
                rospy.loginfo("neck speed increased to {}".format(speed))
            elif key == '-' and speed-0.05 > 0:
                speed -= 0.05
                rospy.loginfo("neck speed decreased to {}".format(speed))
            elif key == 'r':
                neck_elevation = -np.pi/4
                neck_pitch = np.pi/4
                neck_yaw = 0

            print(neck_pitch)
            out = Float32MultiArray()
            out.data = [neck_elevation, neck_pitch, neck_yaw]

            # Publish commands to the robot
            pub.publish(out)

    except Exception as e:
        print(e)

    finally:
        out = Float32MultiArray()
        out.data = [neck_elevation, neck_pitch, neck_yaw]
        pub.publish(out)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
