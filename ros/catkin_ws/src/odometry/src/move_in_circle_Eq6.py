#!/usr/bin/env python
""" Implements a node that moves the Puzzlebot in a circle of given radius, at given linear velocity.
"""

import sys
import numpy as np
import rospy
from geometry_msgs.msg import Twist

class MoveInCircleCommander():
    def __init__(self):

        rospy.init_node('MoveInCircle')

        # Publish to the /cmd_vel topic
        # self.pub = rospy.Publisher("/vel_cmd", Twist, queue_size=1) # Original was wrong
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(20)
        rospy.on_shutdown(self.end_callback) # If node dies, stop robot
        

    def main(self, radie=1.2, linvel=0.1 ):

        while not rospy.is_shutdown():
            self.rate.sleep()

            self._move_in_circle(radie, linvel)
                    
                    
    
    def _move_in_circle(self, radie, linvel):
		"""
		Moves Puzzlebot in a circular trajectory of radius = radie
		and at linear velocity = linvel
		"""

        tw = Twist()
        
        #----------------------------------------------------------------------------
        # Your code here
        tw.linear.x = linvel
        tw.angular.z = -linvel / radie # w = v / r, negative so it turns relatively around center

        #----------------------------------------------------------------------------

        self.pub.publish(tw)

    def end_callback(self):
		"""
		If node dies, for instance, by keyboard interrupt, we stop
		the robot
		"""
		tw = Twist()
		tw.linear.x = 0
		tw.angular.z = 0
		self.pub.publish(tw)
        

        
if __name__ == '__main__':

    try:
        node = MoveInCircleCommander()
        if len(sys.argv) >= 3:
            node.main(radie=sys.argv[1], linvel=sys.argv[2])
        else:
            node.main()
    except (rospy.ROSInterruptException, rospy.ROSException("topic was closed during publish()")):
        pass
