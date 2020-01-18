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

        self.max_speed = 1
        self.position = (0,0)
        self.currAngle= 0
        self.whisking_range = 0.12

        # Set up the state machine
        self.state = State()

        # Create list of obstacles in the order in which they will be used
        self.obstacles = [Obstacle("cylinder", 2.2, 0.0, 1.05), Obstacle("Cube", 0.0, 2.5, 1.35)]

        # Set the initial goal position
        self.obstacle = self.obstacles[0]
        self.goal_position = self.obstacle.position

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
        self.obstacle = self.obstacles[cur_obstacle]
        time_set = False
        duration = rospy.Duration.from_sec(54.0*2)
        start_time = rospy.Time.from_sec(time.time())
        while start_time + rospy.Duration.from_sec(8) > rospy.Time.from_sec(time.time()):
            rospy.loginfo_once("waiting 8 seconds for gazebo to open gui")
        while not rospy.is_shutdown():
            #print("time:set: {}, start_time: {}, duration: {}, lenObst: {}, cur_obst: {}".format(time_set, start_time, duration,len(self.obstacles),cur_obstacle))
            if self.state.cur_state == 'orbiting':
                if time_set == False:
                    start_time = rospy.Time.from_sec(time.time())
                    time_set = True
                if time_set and start_time + duration > rospy.Time.from_sec(time.time()):
                    self.orbit_around(self.goal_position)
                else:
                    cur_obstacle +=1
                    if cur_obstacle >= len(self.obstacles):
                        print("done")
                    else:
                        time_set = False
                        self.obstacle = self.obstacles[cur_obstacle]
                        self.goal_position = self.obstacle.position
                        self.state.cur_state = 'turn'
            elif self.state.cur_state == 'moveto' or self.state.cur_state == 'turn':
                self.moveto(self.goal_position)
            self.rate.sleep()
    def orbit_around(self, goal_position):
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
        #t = Twist()
        o = Odometry()
        o.header.stamp = rospy.Time.now()
        o.twist.twist.angular.z = -2 + difference * 4 
        #t.angular.z = -2 + difference*4
        x = -1*(self.whisking_range + self.obstacle.radius - distance)*6

        #t.linear.x = x
        #t.linear.y = 2
        o.twist.twist.linear.x = x
        o.twist.twist.linear.y = 2
        self.pub.publish(o.twist.twist)
        self.pub_odom.publish(o)
        
        #print("goal_pos: {}, self_pos: {}, self.angle: {}".format(goal_position, self.position, self.currAngle))
        #print("theta_diff: {}, target_angle: {} ,distance: {}, x_vel: {}".format(theta_diff, target_angle, distance, t.linear.x))
        #print("difference: {}, distance: {}".format(difference,distance))

    def moveto(self, goal_position):
        diff_x = goal_position[0] - self.position[0]
        diff_y = goal_position[1] - self.position[1]
        distance = math.sqrt(math.pow(diff_x,2)+math.pow(diff_y,2))
        if diff_x != 0: 
            diff_theta = math.atan2(diff_y,diff_x) 
        else:
            diff_theta = math.atan2(diff_y,0.00001)
        #print("distance: {}".format(distance))
        #print("state: {}".format(self.state.cur_state))
        #print("")
        #move_cmd = Twist()
        o = Odometry()
        o.header.stamp = rospy.Time.now()
        if self.currAngle != 0 and self.position[0] != 0:
            if abs(diff_theta - self.currAngle) <= 0.15:
                self.state.cur_state = 'moveto'
            if self.state.cur_state == 'turn':
                #move_cmd.angular.z = 4
                o.twist.twist.angular.z = 4
            elif self.state.cur_state == 'moveto':
                # Create a Twist message and add linear x and angular z values
                #move_cmd.angular.z = 0
                o.twist.twist.angular.z = 0
                #move_cmd.linear.x = 2
                o.twist.twist.linear.x = 2

        # Check if the robot is in whisking range
        if distance <= self.obstacle.radius + self.whisking_range:
            self.state.cur_state = 'orbiting'
        else:
            self.pub.publish(o.twist.twist)
            self.pub_odom.publish(o)



if __name__ == '__main__':
    orbit = Orbit()
    try:
        orbit.main()
    except rospy.ROSInterruptException:
        pass
