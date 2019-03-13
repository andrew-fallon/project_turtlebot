#!/usr/bin/env python

import rospy
import numpy as np
import tf
from nav_msgs.msg import OccupancyGrid, MapMetaData
from grids import StochOccupancyGrid2D
from geometry_msgs.msg import Pose2D
from getFrontier import getfrontier

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
        self.MapData = None
        self.front_msg = Pose2D()
        self.goal_pose = None

        self.frontier_pub = rospy.Publisher('/nav_pose', Pose2D, queue_size=10)
        rospy.Subscriber('/map_metadata', MapMetaData, self.map_md_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

    def map_md_callback(self, msg):
        self.map_width = msg.width
        self.map_height = msg.height
        self.map_resolution = msg.resolution
        self.map_origin = (msg.origin.position.x,msg.origin.position.y)

    def map_callback(self,msg):
        self.MapData =msg
        self.map_probs = msg.data
        if self.map_width>0 and self.map_height>0 and len(self.map_probs)>0:
            self.occupancy = StochOccupancyGrid2D(self.map_resolution, self.map_width, self.map_height,
                                                  self.map_origin[0], self.map_origin[1], 8, self.map_probs)
            self.occupancy_updated = True

    def ind2pos(self,ind):
        # x = (ind[0]-self.map_width/2.0)*self.occupancy.resolution
        # y = (ind[1]-self.map_height/2.0)*self.occupancy.resolution

        x = ind[0]*self.occupancy.resolution + self.map_origin[0]
        y = ind[1]*self.occupancy.resolution + self.map_origin[1]
        return (x,y)

    def pos2ind(self,pos):
        # input is tuple of x,y positions 
        p = self.occupancy.snap_to_grid(pos)
        # x_ind = int(p[0]/self.occupancy.resolution+self.map_width/2.0)
        # y_ind = int(p[1]/self.occupancy.resolution+self.map_height/2.0)

        x_ind = (p[0]-self.map_origin[0])/self.occupancy.resolution
        y_ind = (p[1]-self.map_origin[1])/self.occupancy.resolution
        return (x_ind,y_ind)

    def buildGrid(self):
        grid = np.zeros((self.map_width, self.map_height))
        for i in range(len(self.occupancy.probs)):
            # convert i to (x,y)
            gy = int(i/self.map_width)
            gx = i % self.map_width
            grid[gx,gy] = self.occupancy.probs[i]
        # np.savetxt('test.txt',grid)
        # print "saved graph"
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
            print frontier_ind
            return True

    def loop(self):
        if self.occupancy:
            (translation,rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            x = translation[0]
            y = translation[1]

            if not self.occupancy.is_free((x,y)):
                rospy.logwarn('Explorer: X init not free, should kick the turtle')
                return

            # print type(self.occupancy.probs)
            PossibleLocations = getfrontier(self.MapData)
            FreeLocations = []
            distances = []
            Centroid = None
            for location in PossibleLocations:
                if self.occupancy.is_free(location):
                    # print "Location is free: ",location
                    FreeLocations.append(location)
                    dist = ((location[0]-x)**2+(location[1]-y)**2)**0.5
                    distances.append(dist)
            if not distances:
                rospy.loginfo('Explorer: No frontiers at this location')
            else:
                Centroid = FreeLocations[np.argmin(distances)]

            if self.goal_pose is None and Centroid is not None:
                print "Setting new goal!",Centroid
                print "Goal free? ",self.occupancy.is_free(Centroid)
                self.goal_pose = Centroid
                self.front_msg = Pose2D(Centroid[0],Centroid[1],0.0)
            elif self.goal_pose is None and Centroid is None:
                rospy.loginfo('Explorer: Map fully explored!')
                return
            elif not self.occupancy.is_free(self.goal_pose):
                rospy.logwarn("Explorer: Goal is occupied, pick a new frontier")
                self.goal_pose = None
                return
            else:
                print"No new goal, currently going to: ",self.goal_pose

            # robot_ind = self.pos2ind((x,y))
            # # print 'robot ind ',robot_ind
            # robot_pos = self.ind2pos(robot_ind)
            # # print 'real: ',x,y,' guess ',robot_pos[0],robot_pos[1]
            # grid = self.buildGrid()
            # if self.findFrontier(grid,robot_ind) and self.goal_pose is None:
            #     self.front_msg = Pose2D(self.frontier[0],self.frontier[1],0.0)
            #     self.goal_pose = (self.frontier[0],self.frontier[1])
            #     print 'New goal: ',self.goal_pose
            # elif self.findFrontier(grid,robot_ind) and self.goal_pose is not None:
            #     print "Frontier found, continuing to original goal"
            # else:
            #     print "No new frontier found"

            if ((x-self.goal_pose[0])**2+(y-self.goal_pose[1])**2)**0.5 < .25:
                print "At frontier, resetting goal pose to none"
                self.goal_pose = None

            self.frontier_pub.publish(self.front_msg)

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == "__main__":
    Explorer().run()