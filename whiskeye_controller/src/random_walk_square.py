#!/usr/bin/env python
from __future__ import print_function
import roslib
# roslib.load_manifest('teleop_twist_keyboard')
import rospy
import random
import numpy as np
import math
import tf
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import String

import os
ros_version_name = os.environ['ROS_DISTRO']

class RandomWalker():
    """  
    Random Walker
    A simple controller that makes a robot move in a random direction starting from the center. In the square world scenario the robot bounces off the walls according to its incidence angle
    """
    def __init__(self):
        # Initializing node, cmd_vel publisher and pose subscriber
        rospy.init_node('whiskeye_controller')

        rospy.Subscriber("/whiskeye/body/pose", Pose2D, self.pose_callback)
        self.pub = rospy.Publisher("/whiskeye/body/cmd_vel", Twist, queue_size=1)

        ## Run at 100 Hz
        self.rate = rospy.Rate( rospy.get_param("~pub_rate", 100) )

        ## Radius of the robot (estimated)
        self.robot_radius = rospy.get_param("~robot_radius", 0.35)

        ## Current position of the robot
        self.position = None

        ## Current Angle of the robot
        self.currAngle = None

        ## Variable to set the robot into rotating or translating mode
        self.rotating = True

        ## The direction in which the robot rotates when in rotating mode (0 is initial value, -1: clockwise, 1 counterclockwise)
        self.rotation_direction = 0

        ## The angle tolerance at which the robot considers an angle reached
        self.angle_tol = 10*(math.pi/180)

        ## Length of one edge of the squared arena
        self.arena_length = rospy.get_param("~arena_len", 10.0)

        ## Define the rectangle that is the arena
        self.arena = (-self.arena_length/2,-self.arena_length/2,\
                    self.arena_length/2,self.arena_length/2) # Arena rectangle (x_min, y_min, x_max, y_max)

        ## Compute the safety distance the robot keeps from the wall
        self.safety_margin = 0.5 + self.robot_radius

        ## Define the rectangle that is the safety area based on the arena size
        self.safety_area = (self.arena[0]+self.safety_margin, self.arena[1]+self.safety_margin, self.arena[2]-self.safety_margin, self.arena[3]-self.safety_margin)

        ## Center point of the arena in 3D space
        self.arenaCenter = (0,0,0)

        ## Robot translation speed
        self.translate_speed = rospy.get_param("~linear_speed", 2.25)

        ## Robot translation speed
        self.rotate_speed = rospy.get_param("~rot_speed", 6.0)

        ## Variable to store where the robot bounced off the wall last time
        self.last_bounce_pos = (10,10)

        ## Initialize with a random target angle
        self.target_angle = random.random() * 360 * (math.pi/180)# Start in a random direction

        ## Add a random angle multiplier when calculating the new target angle after a bounce
        self.turnaround_angle = 80*(math.pi/180)

        ## Execute the main routine
        self.main_routine()

    def pose_callback(self, data):
        """
        Fuction to retrieve the position and angle data from the pose subscriber
        """
        p = data
        self.position = (p.x, p.y)
        self.currAngle = self.clip_rad_360(p.theta) # In radians

    def check_robot_inside_safety_margin(self):
        """
        Function to check if the robots location is inside the safety margin
        :return: which wall we collided with
         """
        out = 0
        if self.position[0] < self.safety_area[0]:
            out = 1 # left wall
        elif self.position[1] < self.safety_area[1]:
            out = 2 # bottom wall
        elif self.position[0] > self.safety_area[2]: 
            out = 3 # right wall
        elif self.position[1] > self.safety_area[3]:
            out = 4 # top wall
        return out 

    def clip_rad_360(self,angle):
        """
        Function to clip the angle within 0,2pi
        :param angle: angle to be clipped
        :return: Clipped angle in radians, number of rotations
        """
        # Account for the number of rotations
        rots = 1+int(abs(angle) / (2.0 * np.pi))
        if angle < 0:
            angle += (2.0 * np.pi)*rots
         
        elif angle >= 2.0 * np.pi:
            angle -= (2.0 * np.pi)*rots
        return angle

    def get_rotation_direction(self, currAngle, target_angle):
        """
        This function gets two angles and returns the direction one has to turn to reach
        the target_angle form the currAngle in the fastest possible way
        :param curAngle: current angle of the robot
        :param target_angle: target angle the robot has to turn to
        :return: direction in which the robot has to turn (-1: clockwise, +1 counterclockwise)
        """
        if self.currAngle < self.target_angle:
            if abs(self.currAngle - self.target_angle) < math.pi:
                rotation_direction =  1
            else:
                rotation_direction = -1
        else:
            if abs(self.currAngle - self.target_angle) < math.pi:
                rotation_direction = -1
            else:
                rotation_direction = 1
        return rotation_direction


    def main_routine(self):
        """
        Main routine of the node. Contains the while loop that runs the controller logic. Initialized in the __init__ function
        """
        while not rospy.is_shutdown():
            twist = Twist()
            # Turn to the target angle if we are not heading there
            # Only move once we are facing in the direction of the target angle
            if self.currAngle == None or self.position == None:
                if (ros_version_name is "melodic"):
                    rospy.loginfo_once("No pose being published yet")  # for ROS Melodic
                else:
                    rospy.loginfo_once("No pose being published yet")  # for ROS Kinetic

            if self.currAngle is not None and self.position is not None:
                # Make sure the angle is between 0 and 2pi
                self.currAngle = self.clip_rad_360(self.currAngle)
                # Check if the robot is safe or if not which wall he collided with
                safe = self.check_robot_inside_safety_margin()
                # Calculate the distance to the last bounce, we have to make sure that we are at least a certain distance away form the last bounce otherwise the robot can get stuck in the safety margin and spin 24/7
                distance_to_last_bounce = math.sqrt((self.position[0]-self.last_bounce_pos[0])**2+(self.position[1]-self.last_bounce_pos[1])**2)
                # If the robot hits a wall and are at least a certain distance from the last bounce point
                if safe is not 0 and distance_to_last_bounce > self.safety_margin:
                    # Alternative bounce mechanic (turn around 180 degrees +- some random deviation
                    #self.target_angle = self.clip_rad_360(self.currAngle - np.pi + (random.random()-0.5)*self.turnaround_angle)

                    # Set the new target angle
                    if safe == 1 or safe == 3: # left, right
                        self.target_angle = self.clip_rad_360((np.pi-self.currAngle))
                    if safe == 2 or safe == 4: # bottom, top
                        self.target_angle = self.clip_rad_360((2*np.pi-self.currAngle))
                    # Based on the new target angle compute the shortes rotation direction and update the last bounce position
                    self.rotation_direction = self.get_rotation_direction(self.currAngle, self.target_angle)
                    self.last_bounce_pos = (self.position[0], self.position[1])
                    # Yes we want to rotate
                    self.rotating = True
                else:
                    #If the robot has not reached the target angle while rotating we continue rotating
                    if abs(self.currAngle - self.target_angle) > self.angle_tol:
                        self.rotating = True
                        # If we are in the initial position no rotation direction is set
                        if self.rotation_direction == 0:
                            self.rotation_direction = self.get_rotation_direction(self.currAngle, self.target_angle)
                    else:
                        # We want to translate
                        self.rotating = False

                # rotate or translate the robot by setting the twist
                if self.rotating == True: 
                    twist.linear.x = twist.linear.y = twist.linear.z = twist.angular.x = twist.angular.y = 0
                    twist.angular.z = self.rotation_direction * self.rotate_speed
                if self.rotating == False:
                    twist.angular.x = twist.angular.y = twist.angular.z = twist.linear.y = twist.linear.z = 0
                    twist.linear.x = self.translate_speed
                # Publish result and wait
                self.pub.publish(twist)
            self.rate.sleep()

if __name__ == '__main__':
    rw = RandomWalker()
