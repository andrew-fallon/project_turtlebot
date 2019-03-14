#!/usr/bin/env python

import rospy
import numpy as np
import tf
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import String, Bool
from grids import StochOccupancyGrid2D
from geometry_msgs.msg import Pose2D
from getFrontier import getfrontier
from visualization_msgs.msg import Marker, MarkerArray

class Delivery:

	def __init__(self):
		rospy.init_node('Delivery_Mode', anonymous=True)
		self.trans_listener = tf.TransformListener()
		self.x = 1
		self.y = 1
		self.theta = 1
		self.x_g = 0
		self.y_g = 0
		self.theta_g = 0
		self.deliv_queue = []
		self.dq_empty = True
		self.delivery_pub = rospy.Publisher('/deli_pose', Pose2D, queue_size=10)
		self.delivery_dq_empty_pub = rospy.Publisher('/dq_empty', Bool, queue_size=10)
		rospy.Subscriber('/delivery_request', String, self.delivery_request_callback)
		rospy.Subscriber('/curr_pose', Pose2D, self.curr_pose_callback)

	def delivery_request_callback(self, msg):
		self.deliv_queue.extend(msg.data.split(","))
		print(self.deliv_queue)

	def curr_pose_callback(self, msg):
		self.x = msg.x
		self.y = msg.y
		self.theta = msg.theta

	def loop(self):
		
		if len(self.deliv_queue) == 0:
			self.dq_empty = True
			self.x_g = 0
			self.y_g = 0
			self.theta_g = 0
			rospy.loginfo('Que is empty, goal is home')
		else:
			# set current goal location as the first in delivery request queue
			self.dq_empty = False
			if self.deliv_queue[0] == 'bottle':
				m_arrays = rospy.wait_for_message("bottle_markers", MarkerArray)
				rospy.loginfo('Current goal is bottle location')
			elif self.deliv_queue[0] == 'stop_sign': 
				m_arrays = rospy.wait_for_message("stop_sign_markers", MarkerArray)
				rospy.loginfo('Current goal is stop sign location')
			else:
				rospy.logwarn("The delivery request is unknown, going home")
				self.x_g = 0
				self.y_g = 0
				self.theta_g = 0
				m_arrays = MarkerArray()

			if len(m_arrays.markers) != 0:
				# print(m_arrays)
				# print(m_arrays.markers)
				m_array = m_arrays.markers[0]
				# print(m_array)
				self.x_g = m_array.pose.position[0]
				self.y_g = m_array.pose.position[1]
				euler = tf.transformation.euler_from_quaternion(m_array.pose.orientation)
				self.theta_g = euler[2]

			# check if x y coord is close enough, if so then update queue and update goal location
			if ((self.x-self.x_g)**2+(self.y-self.y_g)**2)**0.5 < .25:
				self.deliv_queue.pop(0)
				rospy.loginfo('Was sufficiently close to current goal, updated queue and going to next delivery request')
				
				if len(self.deliv_queue) == 0:
					self.dq_empty = True
					rospy.loginfo('Updated Queue is empty going home')
					self.x_g = 0
					self.y_g = 0
					self.theta_g = 0
				else:
					self.dq_empty =False
					if self.deliv_queue[0] == 'bottle':
						m_arrays = rospy.wait_for_message("bottle_markers", MarkerArray)
						rospy.loginfo('Updated goal is bottle location')
					elif self.deliv_queue[0] == 'stop_sign': 
						m_arrays = rospy.wait_for_message("stop_sign_markers", MarkerArray)
						rospy.loginfo('Updated goal is stop sign location')
					else:
						rospy.logwarn("The updated delivery request is unknown, going home")
						self.x_g = 0
						self.y_g = 0
						self.theta_g = 0
						m_arrays = MarkerArray()
					if len(m_arrays.markers) != 0:
						m_array = m_arrays.markers[0]
						self.x_g = m_array.pose.position[0]
						self.y_g = m_array.pose.position[1]
						euler = tf.transformation.euler_from_quaternion(m_array.pose.orientation)
						self.theta_g = euler[2]
		self.delivery_pub.publish(Pose2D(self.x_g,self.y_g,self.theta_g))
		self.delivery_dq_empty_pub.publish(Bool(self.dq_empty))

	def run(self):
		rate = rospy.Rate(10) # 10 Hz
		while not rospy.is_shutdown():
			self.loop()
			rate.sleep()

if __name__ == "__main__":
	Delivery().run()