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

# threshold at which we consider the robot at a location
POS_EPS = 0.1    # meters
THETA_EPS = 0.15  # radians

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
		self.dq_empty = False
		# self.bottle_markers = MarkerArray()
		self.delivery_pub = rospy.Publisher('/deli_pose', Pose2D, queue_size=10)
		self.delivery_dq_empty_pub = rospy.Publisher('/dq_empty', Bool, queue_size=10)
		rospy.Subscriber('/delivery_request', String, self.delivery_request_callback)
		rospy.Subscriber('/curr_pose', Pose2D, self.curr_pose_callback)
		rospy.Subscriber('bottle_markers', MarkerArray, self.bottle_markers_callback)
		rospy.Subscriber('apple_markers', MarkerArray, self.apple_markers_callback)
		rospy.Subscriber('banana_markers', MarkerArray, self.banana_markers_callback)
		rospy.Subscriber('broccoli_markers', MarkerArray, self.broccoli_markers_callback)


	def delivery_request_callback(self, msg):
		self.deliv_queue.extend(msg.data.split(","))
		print(self.deliv_queue)

	def bottle_markers_callback(self, msg):
		self.bottle_markers = msg

	def apple_markers_callback(self, msg):
		self.apple_markers = msg

	def banana_markers_callback(self, msg):
		self.banana_markers = msg

	def broccoli_markers_callback(self, msg):
		self.broccoli_markers = msg

	def curr_pose_callback(self, msg):
		self.x = msg.x
		self.y = msg.y
		self.theta = msg.theta

	def loop(self):

		while not self.dq_empty:
			request = self.deliv_queue[0].split(" ")
			item = request[0]
			number = int(request[1])

			if item == 'bottle':
				m_arrays = self.bottle_markers
				rospy.loginfo('Current goal is bottle location')
			elif item == 'apple': 
				m_arrays = self.apple_markers
				rospy.loginfo('Current goal is apple location')
			elif item == 'banana': 
				m_arrays = self.banana_markers
				rospy.loginfo('Current goal is banana location')
			elif item == 'broccoli': 
				m_arrays = self.broccoli_markers
				rospy.loginfo('Current goal is broccoli location')
			else:
				rospy.logwarn("The delivery request is unknown, going home")
				self.x_g = 0
				self.y_g = 0
				self.theta_g = 0
				rospy.loginfo('Queue is empty, goal is home')

			if m_arrays not None and len(m_arrays.markers) != 0:
				marker = m_arrays.markers[item]
				self.x_g = marker.pose.position.x
				self.y_g = marker.pose.position.y
				# euler = tf.transformations.euler_from_quaternion(m_array.pose.orientation)
				self.theta_g = 0#euler[2]

			# check if x y coord is close enough, if so then update queue and update goal location
			if ((self.x_g-self.x)**2+(self.x_g-self.x)**2)**0.5 < POS_EPS and abs(self.theta_g-self.theta)<THETA_EPS:
				self.deliv_queue.pop(0)
				rospy.loginfo('Was sufficiently close to current goal, updated queue and going to next delivery request')

			if len(self.deliv_queue) == 0:
				self.dq_empty = True

			self.delivery_pub.publish(Pose2D(self.x_g,self.y_g,self.theta_g))
			self.delivery_dq_empty_pub.publish(Bool(self.dq_empty))

		# if len(self.deliv_queue) == 0:
		# 	self.dq_empty = True
		# 	self.x_g = 0
		# 	self.y_g = 0
		# 	self.theta_g = 0
		# 	rospy.loginfo('Queue is empty, goal is home')
		# else:
		# 	# set current goal location as the first in delivery request queue
		# 	self.dq_empty = False

		# 	request = self.deliv_queue[0].split(" ")
		# 	item = request[0]
		# 	number = int(request[1])

		# 	if item == 'bottle':
		# 		m_arrays = self.bottle_markers
		# 		rospy.loginfo('Current goal is bottle location')
		# 	elif item == 'apple': 
		# 		m_arrays = self.apple_markers
		# 		rospy.loginfo('Current goal is apple location')
		# 	elif item == 'banana': 
		# 		m_arrays = self.banana_markers
		# 		rospy.loginfo('Current goal is banana location')
		# 	elif item == 'broccoli': 
		# 		m_arrays = self.broccoli_markers
		# 		rospy.loginfo('Current goal is broccoli location')
		# 	else:
		# 		rospy.logwarn("The delivery request is unknown, going home")
		# 		self.x_g = 0
		# 		self.y_g = 0
		# 		self.theta_g = 0
		# 		m_arrays = MarkerArray()

		# 	if len(m_arrays.markers) != 0:
		# 		# print(m_arrays)
		# 		# print(m_arrays.markers)
		# 		m_array = m_arrays.markers[0]
		# 		# print(m_array)
		# 		self.x_g = m_array.pose.position.x
		# 		self.y_g = m_array.pose.position.y
		# 		# euler = tf.transformations.euler_from_quaternion(m_array.pose.orientation)
		# 		self.theta_g = 0#euler[2]

			# # check if x y coord is close enough, if so then update queue and update goal location
			# if abs(self.x_g-self.x)<POS_EPS and abs(self.y_g-self.y)<POS_EPS and abs(self.theta_g-self.theta)<THETA_EPS < .25:
			# 	self.deliv_queue.pop(0)
			# 	rospy.loginfo('Was sufficiently close to current goal, updated queue and going to next delivery request')
				
			# 	if len(self.deliv_queue) == 0:
			# 		self.dq_empty = True
			# 		rospy.loginfo('Updated Queue is empty going home')
			# 		self.x_g = 0
			# 		self.y_g = 0
			# 		self.theta_g = 0
			# 	else:
			# 		self.dq_empty = False

			# 		request = self.deliv_queue[0].split(" ")
			# 		item = request[0]
			# 		number = int(request[1])
 
			# 		if self.deliv_queue[0][:-2] == 'bottle':
			# 			m_arrays = self.bottle_markers
			# 			rospy.loginfo('Updated goal is bottle location')
			# 		elif self.deliv_queue[0][:-2] == 'apple': 
			# 			m_arrays = self.apple_markers
			# 			rospy.loginfo('Current goal is apple location')
			# 		elif self.deliv_queue[0][:-2] == 'banana': 
			# 			m_arrays = self.banana_markers
			# 			rospy.loginfo('Current goal is banana location')
			# 		elif self.deliv_queue[0][:-2] == 'broccoli'
			# 			m_arrays = self.broccoli_markers
			# 			rospy.loginfo('Current goal is broccoli location')
			# 		else:
			# 			rospy.logwarn("The updated delivery request is unknown, going home")
			# 			self.x_g = 0
			# 			self.y_g = 0
			# 			self.theta_g = 0
			# 			m_arrays = MarkerArray()
			# 		if len(m_arrays.markers) != 0:
			# 			m_array = m_arrays.markers[0]
			# 			self.x_g = m_array.pose.position.x
			# 			self.y_g = m_array.pose.position.y
			# 			# euler = tf.transformations.euler_from_quaternion(m_array.pose.orientation)
			# 			self.theta_g = 0 #euler[2]

	def run(self):
		rate = rospy.Rate(5) # 10 Hz
		while not rospy.is_shutdown():
			self.loop()
			rate.sleep()

if __name__ == "__main__":
	Delivery().run()