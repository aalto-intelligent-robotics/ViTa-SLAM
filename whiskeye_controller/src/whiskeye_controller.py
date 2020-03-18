#!/usr/bin/env python
import rospy
import time
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry

X = 0; Y = 1; THETA = 2

WHISKING_RANGE = 0.66

class Control:
    def __init__(self):
        rospy.init_node("whiskeye_control")
        rospy.Subscriber("/whiskeye/body/pose", Pose2D, self.pose_callback)
        self.pub_vel = rospy.Publisher(
                "/whiskeye/body/cmd_vel", Twist, queue_size = 1)
        self.pub_odom = rospy.Publisher(
                "/whiskeye/odom", Odometry, queue_size = 1)
        self.rate = rospy.Rate(rospy.get_param("~pub_rate"))
        self.pose = Pose2D()
        self.twist = Twist()
        self.odom = Odometry()

        # Testing new orbit_() method
        self.contact = False
        rospy.Subscriber("/whiskeye/head/contact_world", 
                Float32MultiArray, self.contact_callback)

    def main(self):
        # Cylinder
        self.move_to((1.7 - WHISKING_RANGE, 0))
        self.orbit((2.2, 0), radians = 4 * np.pi)
        # Origin
        self.move_to((0, 0))

        # Rock
        self.move_to((0, 1))
        self.orbit_((0, 2.5), radians = 4 * np.pi, speed = 1.5)
        # Origin
        self.move_to((0, 0))

        print("DONE")

        rospy.spin()

    def update(self):
        info = "\033[1;36;40mTwist: x: %8.3f y: %8.3f theta: %8.3f\n" % (
                self.twist.linear.x, self.twist.linear.y, self.twist.angular.z)
        info += "\033[1;37;40mPose:  x: %8.3f y: %8.3f theta: %8.3f" % (
                self.pose.x, self.pose.y, self.pose.theta)
        #print(info)

        self.odom.twist.twist = self.twist
        self.odom.header.stamp = rospy.Time.now()
        self.pub_odom.publish(self.odom)
        self.pub_vel.publish(self.twist)
        self.twist = Twist()
        self.rate.sleep()

    def contact_callback(self, contact):
        self.contact = np.any(contact.data) 

    def pose_callback(self, pose):
        self.pose = pose

        if self.pose.theta < 0:
            self.pose.theta = 2 * np.pi - np.abs(self.pose.theta) % (2 * np.pi)
        else:
            self.pose.theta = self.pose.theta % (2 * np.pi)
    
    def move_to(self, position, angle = 0, speed = 1, kp = 20):
        control = np.zeros(3)

        # Initial rotation
        self.rotate(position, angle, speed)

        # Control loop
        while True:
            diff = self.get_diff(position)
            diff_angle = self.get_diff(position, angle)
            dist = np.sqrt(np.power(diff[X], 2) + np.power(diff[Y], 2))

            if np.isclose(dist, 0, atol = 0.01):
                break
            
            control[THETA] = kp * diff_angle[THETA]
            control[X] = kp * np.cos(diff[THETA]) * dist
            control[Y] = kp * np.sin(diff[THETA]) * dist

            if np.abs(control[THETA]) > speed:
                self.twist.angular.z = np.sign(diff_angle[THETA]) * speed
            else:
                self.twist.angular.z = control[THETA]
                
            if np.sqrt(
                    np.power(control[X], 2) + np.power(control[Y], 2)) > speed:
                self.twist.linear.x = np.cos(diff[THETA]) * speed
                self.twist.linear.y = np.sin(diff[THETA]) * speed
            else:
                self.twist.linear.x = control[X]
                self.twist.linear.y = control[Y]

            self.update()

    def orbit(self, center, radians = 2 * np.pi, speed = 1, kp = 20):
        diff = self.get_diff(center)
        radius = np.sqrt(np.power(diff[X], 2) + np.power(diff[Y], 2))

        # Initial rotation
        self.rotate(center, speed = speed)

        theta_sum = 0
        previous_theta = self.pose.theta

        # Control loop
        while True:
            if np.isclose(theta_sum, radians, atol = 0.1):
                break

            diff = self.get_diff(center)
            dist = np.sqrt(np.power(diff[X], 2) + np.power(diff[Y], 2))
            self.twist.angular.z = kp * diff[THETA]
            self.twist.linear.x = kp * (dist - radius)
            self.twist.linear.y = speed
            
            theta_sum += min(
                np.abs(self.pose.theta - previous_theta), 
                np.abs(self.pose.theta - previous_theta - 2 * np.pi))
            previous_theta = self.pose.theta

            self.update()

    def orbit_(self, center, radians = 2 * np.pi, speed = 0.5, kp = 20):
        diff = self.get_diff(center)

        # Initial rotation
        self.rotate(center, speed = speed)

        since_contact = 0
        theta_sum = 0
        previous_theta = self.pose.theta

        # Control loop
        while True:
            if np.isclose(theta_sum, radians, atol = 0.1):
                break

            diff = self.get_diff(center)
            self.twist.angular.z = kp * diff[THETA]

            if self.contact:
                self.twist.linear.x = -0.1
                since_contact = 0
            elif since_contact > 20:
               self.twist.linear.x = 0.1
            
            since_contact += 1
            #print("Time since contact: %d" % since_contact)
            self.twist.linear.y = speed
            
            theta_sum += min(
                np.abs(self.pose.theta - previous_theta), 
                np.abs(self.pose.theta - previous_theta - 2 * np.pi))
            previous_theta = self.pose.theta

            self.update()

    def rotate(self, position, angle = 0, speed = 0):
        while True:
            diff_angle = self.get_diff(position, angle)

            if np.abs(diff_angle[THETA]) > 0.1:
                self.twist.angular.z = np.sign(diff_angle[THETA]) * speed
            else:
                break

            self.update()

    def get_diff(self, position, angle = 0):
        diff_x = position[X] - self.pose.x
        diff_y = position[Y] - self.pose.y
        theta = np.arctan2(diff_y, diff_x)

        if theta < 0:
            theta += 2 * np.pi

        diff_theta = theta - self.pose.theta - angle

        if np.abs(diff_theta) > np.pi:
            diff_theta -= np.sign(diff_theta) * 2 * np.pi

        return (diff_x, diff_y, diff_theta)

if __name__ == "__main__":
    # Waiting for gazebo to start before moving
    start_time = rospy.Time.from_sec(time.time())

    while start_time + rospy.Duration.from_sec(8) > \
            rospy.Time.from_sec(time.time()):
        rospy.loginfo_once("waiting 8 seconds for gazebo to open GUI")

    control = Control()
    try:
        control.main()
    except rospy.ROSInterruptException:
        pass
