#!/usr/bin/env python

import rospy
import numpy as np
import tf
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Bool
from grids import StochOccupancyGrid2D
from geometry_msgs.msg import Pose2D, Point

class Explorer:

    def __init__(self):
        rospy.init_node('Frontier_Explorer', anonymous=True)
        self.trans_listener = tf.TransformListener()
        
        # map parameters
        self.map_width = 0.0
        self.map_height = 0.0
        self.map_resolution = 0.0
        self.map_origin = [0.0,0.0]
        self.map_probs = []
        self.occupancy = None
        self.occupancy_updated = False
        self.explored = set()
        self.exploring = False

        # Frontier parameters
        self.frontier = None
        self.goal = None
        self.buffer_size = 5
        self.HOME_THRESHOLD = 0.1
        self.THRESHOLD = 0.25
        self.home = None
        self.zone_radius = 0.5

        # Flags
        self.home_flag = False

        # Publishers and subscribers
        self.frontier_pub = rospy.Publisher('/expl_pose', Pose2D, queue_size=10)
        self.end_exploration_pub = rospy.Publisher('/done_exploring', Bool, queue_size=10)
        rospy.Subscriber('/map_metadata', MapMetaData, self.map_md_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/is_stuck', Bool, self.bad_goal_callback)
        rospy.Subscriber('/breadcrumb_marker', Marker, self.trail_callback)
        rospy.sleep(8)
        self.RosRate = 1

    def trail_callback(self, msg):
        if not self.occupancy:
            return
        for i in range(0,len(msg.points)):
            if msg.points[i] is not None:
                p = (msg.points[i].x, msg.points[i].y)
                self.removeRegionFromSearch(p)

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
                self.occupancy_updated = True

    def bad_goal_callback(self,msg):
        if msg.data:
            rospy.loginfo("Explorer: Adding bad goals to explored list")
            self.explored.add(self.goal)

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

    def findFrontier(self,grid,ind):
        x_ind, y_ind = np.where(grid==0)
        possibleLocations = set()
        n = self.buffer_size
        for i in range(0,len(x_ind)):
            neighbor_grid = np.ones((2*n-1, 2*n-1))
            for j in range(-n,n):
                for k in range(-n,n):
                    neighbor_grid[j,k] = grid[x_ind[i]+j,y_ind[i]+k]
            if np.all(neighbor_grid == 0): # BAD GOALS LIST
                possibleLocations.add((x_ind[i],y_ind[i]))

        unexploredGrid = possibleLocations.difference(self.explored)
        # if not self.Exploring:
        self.explored = self.explored.union(unexploredGrid)

        distances = []
        for possible_ind in unexploredGrid:
            x_bot_ind,y_bot_ind = ind
            x_front_ind,y_front_ind = possible_ind
            dist = ((x_bot_ind-x_front_ind)**2+(y_bot_ind-y_front_ind)**2)**0.5
            distances.append(dist)

        if not distances:
            return False
        else:
            frontier_ind = list(unexploredGrid)[np.argmax(distances)]
            self.frontier = self.ind2pos(frontier_ind)
            return True

    def PUBLISH(self):
        self.frontier_pub.publish(Pose2D(self.goal[0],self.goal[1],0.0))

    def getCurrentLocation(self):
        (translation,rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        x = translation[0]
        y = translation[1]
        return x,y

    def distanceToGoal(self):
        x,y = self.getCurrentLocation()
        return ((x-self.goal[0])**2+(y-self.goal[1])**2)**0.5

    def removeRegionFromSearch(self,p):
            origin_ind_x,origin_ind_y = self.pos2ind(p)
            zone_pm = int(self.zone_radius*self.occupancy.resolution)
            for i in range(-zone_pm,zone_pm+1):
                for j in range(-zone_pm,zone_pm+1):
                    origin_zone = (origin_ind_x+i,origin_ind_y+j)
                    self.explored.add(origin_zone)

    def searchForFrontier(self):
        x,y = self.getCurrentLocation()
        # Get robot index in grid frame
        robot_ind = self.pos2ind((x,y))
        grid = self.buildGrid()
        # Frontier exists, publish new goal and start exploring
        if self.findFrontier(grid,robot_ind):
            self.exploring = True
            self.goal = self.frontier
            rospy.loginfo("Explorer: New goal defined")
            self.PUBLISH()
            self.removeRegionFromSearch(self.goal)
        # NO frontier, go back to home
        else:
            rospy.loginfo("Explorer: Sending turtle home")
            self.goal = self.home
            self.exploring = False
            self.PUBLISH()
            if self.distanceToGoal() < self.HOME_THRESHOLD:
                self.end_exploration_pub.publish(Bool(True))

    def loop(self):
        # If map does not exist, break
        if not self.occupancy:
            return
        # Account for blind spot
        if not self.home_flag:
            x,y = self.getCurrentLocation()
            self.home_flag = True
            self.home = (x,y)
            self.removeRegionFromSearch(self.home)
        # Navigating to frontier - DO NOT calculate a new frontier
        if self.exploring:
            # Check if close enough to the goal
            if self.distanceToGoal() < self.THRESHOLD:
                self.exploring = False
        # Done exploring i.e. close enough to goal pose - calculate a new frontier
        else:
            # Find a new frontier if it exists
            rospy.loginfo("Explorer: Searching for new frontier")
            self.searchForFrontier()

    def run(self):
        rate = rospy.Rate(self.RosRate) # Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == "__main__":
    Explorer().run()