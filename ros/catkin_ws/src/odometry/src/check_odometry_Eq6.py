#!/usr/bin/env python
""" Implement a node that checks the odometry calculated with the true odometry
"""

import numpy as np
import rospy
import tf2_ros
from tf import transformations as trf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, TransformStamped, PoseWithCovariance,TwistWithCovariance
from std_msgs.msg import Float64


class OdometryComparator():
    def __init__(self):

        rospy.init_node('OdometryComparator')

        # Subscribe to the odometry topics
        rospy.Subscriber("/odometry", Odometry, self._true_odom_callback)
        rospy.Subscriber("/true_odometry", Odometry, self._odom_callback)
        
        # Publish the errors
        self.dist_pub = rospy.Publisher("/odometry_error/distance", Float64, queue_size=1)
        self.ang_pub = rospy.Publisher("/odometry_error/angle", Float64, queue_size=1)

        self.rate = rospy.Rate(20)

        self.true_odom = None
        self.odom = None
        
    def _true_odom_callback(self, data):
        self.true_odom = data.data
    def _odom_callback(self, data):
        self.odom = data.data
        
        
    def pose2np(self, p):
		'''
		Returns rigid transformation matrix from pose
		'''
		a = np.eye(4)
		q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
		R = trf.quaternion_matrix(q)
		a[:3, :3] = R[:3, :3]
		a[:3, 3] = [p.position.x, p.position.y, p.position.z]
		
		return a
    
    def main(self):


        while not rospy.is_shutdown():
            self.rate.sleep()

            if self.odom is not None and self.true_odom is not None:
                
                #----------------------------------------------------------------------------
                # Your code here
                #
                # Compute the transformation between the two poses contained
                # in odom and true_odom
                true_pose = self.true_odom.pose.pose
                est_pose = self.odom.pose.pose
                
                gsa = self.pose2np(true_pose)
                gsb = self.pose2np(est_pose)
                
                gab = np.dot(np.linalg.inv(gsa), gsb)
                
                # Compute the error in absolute distance, and the error in
                # absolute angle 
                err_q = trf.quaternion_from_matrix(gab)
                err_th = 2 * np.arccos(err_q[0])
                d = gab[:3, 3]
                err_d = np.linalg.norm(d)
                
                self.dist_pub.publish(err_d)
                self.ang_pub.publish(err_th)
                #
                #----------------------------------------------------------------------------
                    
if __name__ == '__main__':

    try:
        aux = OdometryComparator()
        aux.main()

    except (rospy.ROSInterruptException, rospy.ROSException("topic was closed during publish()")):
        pass
