#!/usr/bin/env python
""" Implements an estimated odometry using the wheel velocities.
"""

import numpy as np
import rospy
import tf2_ros
from tf import transformations as trf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, TransformStamped, PoseWithCovariance,TwistWithCovariance, Pose
from std_msgs.msg import Float32, Float64


class MyOdometryPublisher():
    def __init__(self):

        rospy.init_node('OdometryPublisher')

        # Subscribe to the wheel velocity topics
        rospy.Subscriber("/wl", Float32, self._wl_callback)
        rospy.Subscriber("/wr", Float32, self._wr_callback)
        # Initialize velocities as 0
        self.wl = 0
        self.wr = 0
        self.r = 0.05 # Radius of wheels
        self.d = 0.18 # Distance between wheels
        
        # Publish to the odometry topic
        self.odom_pub = rospy.Publisher("/odometry", Odometry, queue_size=1)

        # Publish the simpler (estimated) state to separate topics
        self.x_pub = rospy.Publisher("/est_state/x", Float64, queue_size=1)
        self.y_pub = rospy.Publisher("/est_state/y", Float64, queue_size=1)
        self.th_pub = rospy.Publisher("/est_state/theta", Float64, queue_size=1)

        self.initial_state = np.array([0.0, 0.0, 0.0]) # (theta, x, y)
        self.model_state = None
        self.model_twist = None

        # Keep track of time between updates to the state of the robot
        self.current_time = rospy.get_time()
        self.last_time = self.current_time
        
        # For broadcasting transform from base_link to odom 
        self.br = tf2_ros.TransformBroadcaster() 

        self.rate = rospy.Rate(20)
        
    def _wl_callback(self, data):
        self.wl = data.data
    def _wr_callback(self, data):
        self.wr = data.data
        
        
    def main(self):
        is_set = False
        while not rospy.is_shutdown():
            self.rate.sleep()
            # A lot of time may have passed from construction to main(), so
            # we "restart the timer"
            if not is_set:
				self.current_time = rospy.get_time()
				self.last_time = self.current_time
				dt = 0
				is_set = True
            # If not, then we calculate dt based on the previous update
            else:
				self.current_time = rospy.get_time()
				dt  = self.current_time - self.last_time
				self.last_time = self.current_time

            #----------------------------------------------------------------------------
            # Your code here
            #
            # Update the model state
            vr = self.wr * self.r
            vl = self.wl * self.r
            v = (vr + vl) / 2
            w = (vr - vl) / self.d
            # Euler method
            new_state = self.initial_state + dt * np.array([w, v * np.cos(self.initial_state[0]), v * np.sin(self.initial_state[0])])
            
            # Calculate the pose
            th_est = self.initial_state[0] # We publish the actual state, not the next estimate
            
            # Limit theta from -pi to pi
            if new_state[0] < -np.pi:
				new_state[0] += 2 * np.pi
			
            if new_state[0] >= np.pi:
				new_state[0] -= 2 * np.pi
			
            x_est = self.initial_state[1] # We publish the actual state, not the next estimate
            y_est = self.initial_state[2] # We publish the actual state, not the next estimate
            
            p = Pose()
            p.position.x = x_est
            p.position.y = y_est
            p.position.z = 0.0
            
            q = trf.quaternion_from_euler(0, 0, th_est)
            
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]
            
            self.model_state = p # Update model_state
            
            self.initial_state = new_state # Update initial state for next iteration

            #
            #----------------------------------------------------------------------------

            # Publish the transform between the odometry frame (fixed) and the base_link frame

            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation = self.model_state.position
            t.transform.rotation = self.model_state.orientation
            self.br.sendTransform(t)

            
            # Publish the odometry message
            odom = Odometry()
            # Fill the message with your data
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "odom"
            # Set the position
            odom.pose.pose = Pose(t.transform.translation, t.transform.rotation)
            # Set the velocity
            odom.twist.twist.linear.x = v
            odom.twist.twist.angular.z = w

            # publish the message
            self.odom_pub.publish(odom)

            # Publish the state
            self.x_pub.publish(x_est)
            self.y_pub.publish(y_est)
            self.th_pub.publish(th_est)
                    
if __name__ == '__main__':

    try:
        aux = MyOdometryPublisher()
        aux.main()

    except (rospy.ROSInterruptException, rospy.ROSException("topic was closed during publish()")):
        pass
