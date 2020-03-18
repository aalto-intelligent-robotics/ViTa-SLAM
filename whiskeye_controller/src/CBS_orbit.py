#!/usr/bin/env python
import rospy
import math
import time
import numpy as np
from geometry_msgs.msg import Twist, Pose2D
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry

class State:
    def __init__(self):
        self.cur_state = 'turn'
class Obstacle:
    def __init__(self, name, x , y, radius):
        self.name = name
        self.x = x
        self.y = y
        self.position = (self.x, self.y)
        self.radius = radius

class Orbit:

    def __init__(self):
        rospy.init_node('orbit_trajectory')
        # Create subscriber for the robot pose
        rospy.Subscriber("/whiskeye/body/pose", Pose2D, self.pose_callback)
        # Create a publisher which can "talk" to Turtlesim and tell it to move
        self.pub = rospy.Publisher('/whiskeye/body/cmd_vel', Twist, queue_size=1)
        self.pub_odom = rospy.Publisher('/whiskeye/odom', Odometry, queue_size=1)
        ## Run at 100 Hz
        self.rate = rospy.Rate(rospy.get_param("~pub_rate", 250) )

        # Set goal to rotate around
        self.goal_position = (0, 0)

    def clip_rad_360(self, angle):
        """
        Function to clip the angle within 0,2pi
        :param angle: angle to be clipped
        :return: Clipped angle in radians, number of rotations
        """
        # Account for the number of rotations
        rots = int(abs(angle) / (2.0 * np.pi))
        if angle < 0:
            angle += (2.0 * np.pi)*rots
         
        elif angle >= 2.0 * np.pi:
            angle -= (2.0 * np.pi)*rots
        return angle
    def clip_rad_180(self, angle):
        """
        Function to clip the angle within 0,pi
        :param angle: angle to be clipped
        :return: Clipped angle in radians, number of rotations
        """
        # Account for the number of rotations
        rots = int(abs(angle) / (np.pi))
        if angle < 0:
            angle += (np.pi)*rots
         
        elif angle >= np.pi:
            angle -= (np.pi)*rots
        return angle, (rots-1)

    def pose_callback(self,data):
        """
        Fuction to retrieve the position and angle data from the pose subscriber
        """
        self.p = data
        self.position = (self.p.x, self.p.y)
        self.currAngle = self.clip_rad_360(self.p.theta+2*math.pi) # In radians
    def main(self):
        # Save current time and set publish rate at 10 Hz
        now = rospy.Time.now()
        cur_obstacle = 0
        duration = rospy.Duration.from_sec(2000)
        start_time = rospy.Time.from_sec(time.time())
        while start_time + rospy.Duration.from_sec(8) > rospy.Time.from_sec(time.time()):
            rospy.loginfo_once("waiting 8 seconds for gazebo to open gui")

        while not rospy.is_shutdown():
            if start_time + duration > rospy.Time.from_sec(time.time()):
                self.orbit_around(self.goal_position)
            else:
                print("done")
                break
            self.rate.sleep()

    def orbit_around(self, goal_position):
        """
        diff_x = goal_position[0] - self.position[0]
        diff_y = goal_position[1] - self.position[1]
        distance = math.sqrt(math.pow(diff_x,2)+math.pow(diff_y,2))
        theta_diff = math.atan2(diff_y, diff_x) + math.pi
        target_angle = (theta_diff+math.pi)%(2*math.pi)
        self.currAngle = self.currAngle%(2*math.pi)
        
        difference = (target_angle - self.currAngle)%(2*math.pi)
        if difference > 4:
            difference = 0
        else:
            difference = (target_angle - self.currAngle)%(2*math.pi)
        """

        o = Odometry()
        o.header.stamp = rospy.Time.now()
        o.twist.twist.angular.z = -0.3
        o.twist.twist.linear.x = 0#-1*(self.whisking_range + self.obstacle.radius - distance)*6
        o.twist.twist.linear.y = -0.3
        self.pub.publish(o.twist.twist)
        self.pub_odom.publish(o)
        
        #print("goal_pos: {}, self_pos: {}, self.angle: {}".format(goal_position, self.position, self.currAngle))
        #print("theta_diff: {}, target_angle: {} ,distance: {}, x_vel: {}".format(theta_diff, target_angle, distance, t.linear.x))
        #print("difference: {}, distance: {}".format(difference,distance))


if __name__ == '__main__':
    orbit = Orbit()
    print("XXXXXXXX waiting 10 sceonds for bagfile")
    rate = rospy.Rate(1)
    rate.sleep()
    print("GOOOOOOOOOOOOOOOOO")
    try:
        orbit.main()
    except rospy.ROSInterruptException:
        pass
