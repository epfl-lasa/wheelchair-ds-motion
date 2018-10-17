#!/usr/bin/env python

#'''
#obstacle publisher class
#
#@author lukashuber
#@date 2018-06-08
#
#'''

# Custom libraries
import sys 
lib_string = "/home/nbfigueroa/catkin_ws/src/wheelchair-ds-motion/scripts/lib/"
if not any (lib_string in s for s in sys.path):
    sys.path.append(lib_string)

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

# Obstacle Avdoidance - Custom libraries
from class_obstacle import *
from lib_modulation import *
from dynamicalSystem_lib import constantVel_positionBased
from dynamicalSystem_lib import linearAttractor_const

class VelocityController():
    def __init__(self, n_obs = 0, obs_pos=[[9,0]],dsController=False, freq=100, n_intSteps=20, dt_simu=0.1):
        rospy.init_node('VelocityController' , anonymous=True)
        rospy.on_shutdown(self.call_shutdown) # shutdown command
        rate = rospy.Rate(freq) # Frequency

        # Class variables
        self.dt = 1./freq
        self.dim = 2 # Dimensionaliyty of obstacle avoidance problem
        self.n_obs = int(n_obs) # number of obstacles, at the moment manually do automatic
        self.awaitingPos = True
        self.robo_pos = 0
        self.ds_vel   = 0
        
        # Create position subscriber
        self.sub_pos    = rospy.Subscriber("/Read_joint_state/quickie_state", Pose2D, self.callback_pos)
        self.sub_ds_vel = rospy.Subscriber("/ds_cmd_vel", Twist, self.callback_ds)
        self.sub_ds_att = rospy.Subscriber("/ds1/DS/target", PointStamped, self.callback_attr)

        # Create velocity publisher
        self.pub_vel  = rospy.Publisher('cmd_vel',   Twist, queue_size=10)

        # Human motion limits
        self.velHuman_max =3.0/3.6
        self.velHuman_lim = 5.0/3.6 # [m/s]
        self.distLim_origin = 6 # [m]

        # Robot and environment gemoetry
        self.wheelRad = 0.155  # [m]
        self.robo_dim = np.array([0.66, 0.78])  # [m]


        #  Initialize obstacle stuff
        if self.n_obs > 0:
            self.awaitingObs = [True]*self.n_obs
            self.obs_pos = [0]*self.n_obs
            self.pub_obs = [0]*self.n_obs           
            for oo in range(self.n_obs):
                self.pub_obs[oo] = rospy.Publisher("/Read_obstacle_Position/Desired_Pose_sphere"+str(oo+1), Pose2D, queue_size=10)            

            self.obs_read = 0.5 # [m]
            self.obs_dim = [0.5,0.5] # [m]            
 
            # Initialize obstacles
            if self.n_obs > 1: # Circular initial set up
                R_obsPos = 7
                for oo in range(self.n_obs):
                    self.obs_pos[oo] = Pose2D()
                    self.obs_pos[oo].x = R_obsPos*np.cos(2*pi/self.n_obs*oo)
                    self.obs_pos[oo].y = R_obsPos*np.sin(2*pi/self.n_obs*oo)
                    self.obs_pos[oo].theta = 0
                    self.pub_obs[oo].publish(self.obs_pos[oo])
            if self.n_obs == 1:
                for oo in range(self.n_obs):
                    self.obs_pos[oo] = Pose2D()
                    self.obs_pos[oo].x = obs_pos[oo][0]
                    self.obs_pos[oo].y = obs_pos[oo][1]            
                    self.obs_pos[oo].theta = 0
                    self.pub_obs[oo].publish(self.obs_pos[oo])                    
            
        while(self.awaitingPos):
            print('WAITING - missing robot position')
            rospy.sleep(0.1)
        
        if self.n_obs > 0:
            # Create obstacles
            sf_a = LA.norm(self.robo_dim/2)*1.05
            self.obs = []
            for oo in range(self.n_obs):
                self.obs.append(
                    Obstacle(
                        a=[0.5+sf_a, 0.5+sf_a], 
                        # a=[1.48,1.48],
                        # TODO : more exact safety margin
                        #sf = LA.norm(self.robo_dim)/np.min(self.obs_dim)+1,
                        sf = 1,
                        th_r= self.obs_pos[oo].theta,  # zero for the moment
                        p=1, # Circular obstacles
                        x0= [self.obs_pos[oo].x, self.obs_pos[oo].y], 
                        xd = [0,0]
                    )
                )
                print('got obstacle {}'.format(oo))
            # only for cirucla obstacles
            self.dist_min = self.obs[0].a[0]*2+0.01

        #################### MAIN CONTROL LOOP #################### 
        print('Starting loop.')
 
        # Prepare variables before loop
        n_traj = 2
        pos = np.zeros((self.dim, n_traj+1))
        ds  = np.zeros((self.dim, n_traj))
        while not rospy.is_shutdown(): 
                        
            if self.n_obs > 0:
                # Update position and velocity
                self.update_obsPosition()
                 # Prediction of obstacle
                obs_pred = copy.deepcopy(self.obs)


            pos[:,0] = np.array([self.robo_pos.x, self.robo_pos.y])            

            # Initial velocity
            for ii in range(n_traj):
                # Velocity from non-linear DS
                ds_init = np.array([self.ds_vel.linear.x, self.ds_vel.linear.y])
                velFactor = 0.8
                ds_init = ds_init /np.linalg.norm(ds_init)*velFactor
                
                if self.n_obs > 0:
                    ds_mod = obs_avoidance_interpolation(pos[:,ii], ds_init, self.obs, 
                                                     vel_lim=0.8, attractor=self.pos_attr)
                    print('ds mod', ds_mod)
                    print('mag ds mod', LA.norm(ds_mod) )
                else:
                    ds_mod = ds_init
                    print('ds init', ds_mod)

                ds[:,ii] = ds_mod
                pos[:,ii+1] = self.dt*ds_mod + pos[:, ii]

                for oo in range(self.n_obs): 
                    obs_pred[oo].x0 = obs_pred[oo].x0 + obs_pred[oo].xd*self.dt
                
            # Publish velocity
            vel = Twist()
            vel.linear.x = LA.norm(ds[:,0])/self.wheelRad            
            phi0 = self.robo_pos.theta
            phi1 = np.arctan2(ds[1,1], ds[0,1])
            dPhi = phi1-phi0

            # correct for disoncitnuity at -pi/pi
            while dPhi > pi: 
                dPhi = pi-2*pi
            while dPhi < -pi:
                dPhi = pi+2*pi
            dPhi = dPhi/self.dt
            dPhi_max = 40./180*pi
            if np.abs(dPhi) > dPhi_max:
                dPhi = np.copysign(dPhi_max, dPhi)
                vel.linear.x = 0
                print('WAIT -- repositioning')
            
            vel.angular.z = dPhi*0.8
            if False:
                vel.angular.z = 0
                vel.linear.x = 0

            self.pub_vel.publish(vel)
            rate.sleep()

        print('finished')
        self.call_shutdown()

    def update_obsPosition(self):
        for oo in range(self.n_obs):
        #     # To imrove: dominance of first robot
        #     w_rand = 0.004
        #     obs_dVel = np.array([random.uniform(-w_rand,w_rand), random.uniform(-w_rand,w_rand)])
        #     dist_origin = LA.norm(np.array(self.obs[oo].x0))        
        #     if dist_origin > self.distLim_origin: # obstacle far away
        #         k_orig, pow_orig = 1, 1
        #         obs_dVel = obs_dVel - np.array(self.obs[oo].x0)*(k_orig* np.abs(dist_origin-self.distLim_origin) )**(pow_orig) # exponentail border to stay inside
        #     self.obs[oo].xd = self.obs[oo].xd + obs_dVel

        #     norm_vel = LA.norm(self.obs[oo].xd)
        #     if norm_vel > self.velHuman_max: # maybe .. use soft limit?! 
        #         self.obs[oo].xd = self.velHuman_max / norm_vel*self.obs[oo].xd
        #     new_x0 = self.obs[oo].x0 + self.obs[oo].xd*self.dt

        #     noCollision = True
        #     for pp in range(self.n_obs):
        #         if not(pp==oo) and self.dist_min > LA.norm(new_x0 - np.array(self.obs[pp].x0) ):
        #             noCollision = False
        #             self.obs[oo].xd = np.array([0,0])
        #             return
        #     if noCollision: # Check distance to robot, don't bump into it
        #         if self.dist_min*0.5 > LA.norm(np.array([self.robo_pos.x, self.robo_pos.y]) 
        #                                    - np.array(self.obs[oo].x0) ):
        #             self.obs[oo].xd = np.array([0,0])
        #         else:
        #             self.obs[oo].x0 = new_x0
            self.obs[oo].xd = np.array([0,0])
            pos = Pose2D()
            pos.x = self.obs[oo].x0[0]
            pos.y = self.obs[oo].x0[1]
            pos.theta = 0
            self.pub_obs[oo].publish(pos)


    def callback_pos(self, msg): # Callback to get robots posisiton
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

    def check_collision(self, oo, pp='robo'):
        if type(pp)==str and pp=='robo':
            dist = LA.norm(np.array([self.robo_pos.x, self.robo_pos.y]) - np.array(self.obs[oo].x0 ) )
        else:
            dist = LA.norm(np.array(self.obs[oo].x0) - np.array(self.obs[pp].x0))
        return dist_min > dist

    def callback_attr(self, msg):
        self.pos_attr = np.array([msg.point.x, msg.point.y])
        

if __name__ == '__main__':
    if len(sys.argv) < 2:        
        VelocityController(0)
    elif len(sys.argv) < 3:
        VelocityController(sys.argv[1])


