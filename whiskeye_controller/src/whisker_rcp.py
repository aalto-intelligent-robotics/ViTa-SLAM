#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

WHISK_NUM = 24
# for recording
# WHISK_RATE = 0.15
# DAMP_FACTOR = 0.000000005 * WHISK_RATE
WHISK_RATE = 0.5
DAMP_FACTOR = 0.5 * WHISK_RATE

class RCP():
    def __init__(self):
        rospy.init_node('rcp_whiskeye_whisker', anonymous=True)
        rospy.Subscriber('/whiskeye/head/contact_world', 
                Float32MultiArray, self.contact_callback)
        self.pub_theta_cmd = rospy.Publisher(
                '/whiskeye/head/theta_cmd', Float32MultiArray, queue_size = 10)
        self.pub_rcp_state = rospy.Publisher(
                '/whiskeye/head/rcp_state', Bool, queue_size=10)
        self.rate = rospy.Rate(rospy.get_param("~rate"))

        self.contact = False

    def main(self):
        theta_cmds = Float32MultiArray()
        rcp_states = Float32MultiArray()

        abs_min_theta = min_theta = -2.0
        abs_max_theta = max_theta =  2.0
        previous_theta = None
        t = 0; eps = 0.1
       
        while True:
            A = (max_theta - min_theta) / 2
            bias = (max_theta + min_theta) / 2
            theta = A * np.cos(t) + bias
            t += WHISK_RATE

            if theta > previous_theta and previous_theta != None:
                rcp_states.data = [True] * WHISK_NUM
            elif previous_theta != None:
                rcp_states.data = [False] * WHISK_NUM
       
            """
            if self.contact:
                max_theta = theta - eps
                t = 0
            """

            if max_theta < abs_max_theta:
                max_theta += (abs_max_theta - max_theta) * DAMP_FACTOR
             
            #print("TIME: %f" % t)
            #print("MAX_THETA: %f" % max_theta)
            if theta > 0:
                self.pub_rcp_state.publish(True)
            else:
                self.pub_rcp_state.publish(False)
            previous_theta = theta
            theta_cmds.data = [theta] * WHISK_NUM
            self.pub_theta_cmd.publish(theta_cmds)
            self.rate.sleep()

    def contact_callback(self, contact):
        self.contact = np.any(contact.data)

if __name__=="__main__":
    rcp = RCP()
    try:
        rcp.main()
    except rospy.ROSInterruptException:
        pass
