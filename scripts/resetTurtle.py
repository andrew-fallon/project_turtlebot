#!/usr/bin/env python

import rospy
import numpy as np
import tf
from nav_msgs.msg import OccupancyGrid, MapMetaData
from grids import StochOccupancyGrid2D
from geometry_msgs.msg import Pose2D

MIN_DIST = 4    # move 4 cells minimum (~0.2 m)
MAX_DIST = 8    # move 8 cells max (~0.4 m)


class freeTheTurtle:

    def __init__(self):
        rospy.init_node('reset', anonymous=True)
        self.x = 0
        self.y = 0
        self.theta = 0
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
        self.buffer_size = 3    # 0.05 m for every grid cell
        rospy.Subscriber('/map_metadata', MapMetaData, self.map_md_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/state', String, self.state_callback)

        self.pose_pub = rospy.Publisher('/reset_pose', Pose2D, queue_size=10)

        # if using gazebo, then subscribe to model states
        if use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
 
        self.trans_listener = tf.TransformListener()


    def gazebo_callback(self, data):
        if "turtlebot3_burger" in data.name:
            pose = data.pose[data.name.index("turtlebot3_burger")]
            twist = data.twist[data.name.index("turtlebot3_burger")]
            self.x = pose.position.x
            self.y = pose.position.y
            quaternion = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.theta = euler[2]

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
        else:
            if not use_gazebo:
                try:
                    origin_frame = "/map" if mapping else "/odom"
                    (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                    self.x = translation[0]
                    self.y = translation[1]
                    euler = tf.transformations.euler_from_quaternion(rotation)
                    self.theta = euler[2]
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    pass


            # build possible locations for reset
            grid = self.buildGrid()
            curr_grid_state = pos2ind((self.x, self.y))

            x_ind, y_ind = np.where(grid==0)
            possibleLocations = set()
            n = self.buffer_size
            for i in range(0,len(x_ind)):
                neighbor_grid = np.ones((2*n-1, 2*n-1))
                norm = ((curr_grid_state[0] - x_ind[i])**2 + (curr_grid_state[1] - y_ind[i])**2)**0.5
                for j in range(-n,n):
                    for k in range(-n,n):
                        neighbor_grid[j,k] = grid[x_ind[i]+j,y_ind[i]+k]
                if np.all(neighbor_grid == 0) and (norm < MAX_DIST and norm > MIN_DIST):
                    possibleLocations.add((x_ind[i],y_ind[i]))

            # randomly chose a new location
            newGridState = possibleLocations[np.random.randint(0, len(possibleLocations))]
            self.x_reset, self.y_reset = ind2pos(newGridState)
            reset_pose = Pose2D()
            reset_pose.x = self.x_reset
            reset_pose.y = self.y_reset
            reset_pose.theta = self.theta_reset
            return reset_pose

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            reset_pose = self.get_reset_pose()
            if reset_pose is not None:
                self.pose_pub.publish(reset_pose)
            rate.sleep()

if __name__ == '__main__':
    resetPose = freeTheTurtle()
    resetPose.run()