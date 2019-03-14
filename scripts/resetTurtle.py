#!/usr/bin/env python

import rospy
import numpy as np
import tf
from nav_msgs.msg import OccupancyGrid, MapMetaData
from grids import StochOccupancyGrid2D
from geometry_msgs.msg import Pose2D

class freeTheTurtle:

    def __init__(self):
        rospy.init_node('Frontier_Explorer', anonymous=True)
        self.trans_listener = tf.TransformListener()
        # map parameters
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0,0]
        self.map_probs = []
        self.occupancy = None
        self.run_reset = False
        self.prev_mode = None

        self.x_reset = 0
        self.y_reset = 0
        self.theta_reset = 0
        self.buffer_size = 5
        rospy.Subscriber('/map_metadata', MapMetaData, self.map_md_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/state', String, self.state_callback)

    def state_callback(self, msg):
    	if msg.data == "RESET" and (self.prev_mode != "RESET"):
    		self.prev_mode = "RESET"
    		self.run_reset = True
    	else:
    		self.prev_mode = msg.data
    		self.run_reset = False

    def map_md_callback(self, msg):
        self.map_width = msg.width
        self.map_height = msg.height
        self.map_resolution = msg.resolution
        self.map_origin = (msg.origin.position.x,msg.origin.position.y)

    def map_callback(self,msg):
            self.map_probs = msg.data
            if self.map_width>0 and self.map_height>0 and len(self.map_probs)>0:
                self.occupancy = StochOccupancyGrid2D(self.map_resolution, self.map_width, self.map_height,
                                                      self.map_origin[0], self.map_origin[1], 8, self.map_probs)

    def ind2pos(self,ind):
        x = ind[0]*self.occupancy.resolution + self.map_origin[0]
        y = ind[1]*self.occupancy.resolution + self.map_origin[1]
        return (x,y)

    def pos2ind(self,pos):
        # input is tuple of x,y positions 
        p = self.occupancy.snap_to_grid(pos)
        x_ind = (p[0] - self.map_origin[0])/self.occupancy.resolution
        y_ind = (p[1] - self.map_origin[1])/self.occupancy.resolution
        return (x_ind,y_ind)

    def buildGrid(self):
        grid = np.zeros((self.map_width, self.map_height))
        for i in range(len(self.occupancy.probs)):
            gy = int(i/self.map_width)
            gx = i % self.map_width
            grid[gx,gy] = self.occupancy.probs[i]
        return grid

    def get_reset_pose(self):
    	if not self.run_reset:
    		return None

    	




    def get_unstuck(self):
        """ 
        moves the robot to a nearby state in an attempt to get the robot unstuck from its current 
        position where it either cannot compute a path to the goal, or it is stuck against a wall.
        
        INPUTS: none
        OUTPUTS: none

        """
        nMax = 4        # maximum number of states to try
        if self.stuck_iter not in range(nMax):
            self.stuck_iter = 0
            self.nav_stuck_pub.publish(Bool(True))
            self.current_plan = []
            rospy.logwarn("Navigator: Totally stuck, send help!")            
            return
      
        dx = np.random.choice(np.array([-1, 1])) * (3*self.map_resolution)
        dy = np.random.choice(np.array([-1, 1])) * (3*self.map_resolution)

        # send perturbed pose to pose controller
        pose_msg = Pose2D()
        pose_msg.x = self.x  # - dx
        pose_msg.y = self.y   # - dy
        pose_msg.theta = self.theta + 90*(np.pi/180)
        self.nav_pose_pub.publish(pose_msg)
        self.stuck_iter += 1
        rospy.sleep(1)      # wait for 1 sec after command
        return

    def kick(self):
        """
        Perturbes the current state in an effort to handle x_init glitches in the A* algorithm
        """
        # dx = np.random.choice(np.array([-1, 1])) * (3*self.map_resolution)
        # dy = np.random.choice(np.array([-1, 1])) * (3*self.map_resolution)

        # # send perturbed pose to pose controller
        # pose_msg = Pose2D()
        # pose_msg.x = self.x # - dx
        # pose_msg.y = self.y # - dy
        # pose_msg.theta = self.theta + 90*(np.pi/180)
        # self.nav_pose_pub.publish(pose_msg)

        cmd = Twist()
        cmd.linear.x = -0.08
        cmd.angular.z = 0.15
        self.nav_vel_pub.publish(cmd)   # backup and spin
        rospy.sleep(1.2)      # wait for 1 sec after command
        return
        
    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            reset_pose = self.get_reset_pose()
            if reset_pose is not None:
            	self.pose_pub.publish(reset_pose)
            rate.sleep()

if __name__ == '__main__':\
    resetPose = freeTheTurtle()
    resetPose.run()