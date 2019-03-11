#!/usr/bin/env python

import rospy
import numpy as np
import tf
from nav_msgs.msg import OccupancyGrid, MapMetaData
from grids import StochOccupancyGrid2D
from geometry_msgs.msg import Pose2D

class Explorer:

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
        self.occupancy_updated = False
        self.explored = set()
        self.frontier = None
        self.frontier_pub = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        rospy.Subscriber('/map_metadata', MapMetaData, self.map_md_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

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

    def ind2pos(self,ind):
        x = (ind[0]-self.map_width/2.0)*self.occupancy.resolution
        y = (ind[1]-self.map_height/2.0)*self.occupancy.resolution
        return (x,y)

    def pos2ind(self,pos):
        # input is tuple of x,y positions 
        p = self.occupancy.snap_to_grid(pos)
        x_ind = int(p[0]/self.occupancy.resolution+self.map_width/2.0)
        y_ind = int(p[1]/self.occupancy.resolution+self.map_height/2.0)
        return (x_ind,y_ind)

    def buildGrid(self):
        grid = np.zeros((self.map_width, self.map_height))
        for i in range(len(self.occupancy.probs)):
            # convert i to (x,y)
            gy = int(i/self.map_width)
            gx = i % self.map_width
            grid[gx,gy] = self.occupancy.probs[i]
        return grid

    def findFrontier(self,grid,ind):
        x_ind, y_ind = np.where(grid==0)
        possibleLocations = set()
        for i in range(0,len(x_ind)):
            possibleLocations.add((x_ind[i],y_ind[i]))

        unexploredGrid = possibleLocations.difference(self.explored)
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

    def loop(self):
        if self.occupancy:
            # print type(self.occupancy.probs)
            (translation,rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            x = translation[0]
            y = translation[1]
            robot_ind = self.pos2ind((x,y))
            robot_pos = self.ind2pos(robot_ind)
            print 'real: ',x,y,' guess ',robot_pos[0],robot_pos[1]
            grid = self.buildGrid()
            if self.findFrontier(grid,robot_ind):
                self.frontier_pub.publish(Pose2D(self.frontier[0],self.frontier[1],0.0))
                print self.frontier
            else:
                print "No frontier found"

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == "__main__":
    Explorer().run()