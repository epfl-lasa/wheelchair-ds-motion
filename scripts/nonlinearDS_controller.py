#!/usr/bin/env python

#'''
#velocity publisher class
#
#@author nbfigueroa
#@date 2018-10-20
#
#'''


# General MATH / Matrix calculation
import numpy as np
import numpy.linalg as LA
import numpy.polynomial.polynomial as poly
from math import pi
import random

random.seed(0) # Specific seed, change with system time

# Copy tool
import copy

# ROS tools    
import rospy
from geometry_msgs.msg import Twist, Pose2D, PointStamped

# import copy # copying of lists 
import sys # To pass argument to node

class VelocityController():
    def __init__(self,dsController=False, freq=100, n_intSteps=20, dt_simu=0.1):
        rospy.init_node('VelocityController' , anonymous=True)
        rospy.on_shutdown(self.call_shutdown) 
        rate = rospy.Rate(freq) 

        # Class variablesd
        self.dt = 1./freq
        self.dim = 2
        self.awaitingPos = True
        self.robo_pos = 0
        self.ds_vel   = 0
        
        # Create position subscriber
        self.sub_pos    = rospy.Subscriber("/Read_joint_state/quickie_state", Pose2D, self.callback_pos)
        self.sub_ds_vel = rospy.Subscriber("/ds_cmd_vel", Twist, self.callback_ds)
        self.sub_ds_att = rospy.Subscriber("/ds1/DS/target", PointStamped, self.callback_attr)

        # Create velocity publisher
        self.pub_vel  = rospy.Publisher('cmd_vel',   Twist, queue_size=10)


        # Robot and environment gemoetry
        self.wheelRad = 0.155  # [m]

        #################### MAIN CONTROL LOOP #################### 
        print('Starting loop.') 
        # Prepare variables before loop
        n_traj = 2
        ds  = np.zeros((self.dim, n_traj))
        while not rospy.is_shutdown(): 
            # Initial velocity
            for ii in range(n_traj):
                # Velocity from non-linear DS
                ds_mod = np.array([self.ds_vel.linear.x, self.ds_vel.linear.y])
                velFactor = 1.0
                ds_mod = ds_mod /np.linalg.norm(ds_mod)*velFactor
                print('ds', ds_mod)
                ds[:,ii] = ds_mod
                
            # Publish velocity
            vel = Twist()
            vel.linear.x = LA.norm(ds[:,0])/self.wheelRad            
            phi0 = self.robo_pos.theta
            phi1 = np.arctan2(ds[1,1], ds[0,1])
            dPhi = phi1-phi0

            # correct for discontinuity at -pi/pi
            while dPhi > pi: 
                dPhi = pi-2*pi
            while dPhi < -pi:
                dPhi = pi+2*pi
            dPhi = dPhi/self.dt
            dPhi_max = 180./180*pi
            if np.abs(dPhi) > dPhi_max:
                dPhi = np.copysign(dPhi_max, dPhi)
                vel.linear.x = vel.linear.x/5
                print('WAIT -- repositioning')        
            vel.angular.z = dPhi*0.8

            self.pub_vel.publish(vel)
            rate.sleep()

        print('finished')
        self.call_shutdown()


    def callback_pos(self, msg):
        self.robo_pos = msg
        self.awaitingPos = False
        
    def callback_ds(self, msg): 
        self.ds_vel = msg

    def call_shutdown(self):
        vel = Twist()
        vel.linear.x = 0
        vel.angular.z = 0
        self.pub_vel.publish(vel)
        rospy.loginfo('Zero velocity command after shutdown.')

    def callback_attr(self, msg):
        self.pos_attr = np.array([msg.point.x, msg.point.y])
        

if __name__ == '__main__':
        VelocityController()


