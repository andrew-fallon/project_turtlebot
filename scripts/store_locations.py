#!/usr/bin/env python

import rospy
import numpy as np
import tf
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Vector3, Point, Quaternion
from std_msgs.msg import Header, ColorRGBA
from asl_turtlebot.msg import DetectedObject, DetectedObjectList

class Locations:

	def __init__(self):
		rospy.init_node('detected_locations', anonymous=True)
		self.tf_listener = tf.TransformListener()
		self.stop_pub = rospy.Publisher('stop_sign_markers', MarkerArray, queue_size=10)
		self.bottle_pub = rospy.Publisher('bottle_markers', MarkerArray, queue_size=10)
		self.apple_pub = rospy.Publisher('apple_markers', MarkerArray, queue_size=10)
		self.banana_pub = rospy.Publisher('banana_markers', MarkerArray, queue_size=10)
		self.brocoli_pub = rospy.Publisher('brocoli_markers', MarkerArray, queue_size=10)

		self.detection_thresh = 0.5
		self.known_names = ["stop_sign", "bottle", "apple", "banana", "brocoli"]

		self.stop_array = MarkerArray()
		self.bottle_array = MarkerArray()
		self.apple_array = MarkerArray()
		self.banana_array = MarkerArray()
		self.brocoli_array = MarkerArray()

		self.stop_id = 0
		self.bottle_id = 0
		self.apple_id = 0
		self.banana_id = 0
		self.brocoli_id = 0
		self.stop_color = ColorRGBA(1.0,0.0,0.0,1.0)
		self.bottle_color = ColorRGBA(0.0,0.0,1.0,1.0)
		self.apple_color = ColorRGBA(0.5,0.0,0.0,1.0)
		self.banana_color = ColorRGBA(1.0,1.0,1.0,1.0)
		self.brocoli_color = ColorRGBA(0.5,0.6,1.0,1.0)
		rospy.Subscriber('/detector/objects', DetectedObjectList, self.detected_callback)

	def buildMarkers(self, array, id_count, color):
		(translation, rotation) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
		marker = Marker()
		marker.header.frame_id = '/map'
		marker.header.stamp = rospy.Time(0)
		marker.id = id_count
		marker.type = Marker().CYLINDER
		marker.action = Marker().ADD
		marker.color = color
		marker.scale = Vector3(0.05, 0.05, 0.50)
		marker.pose.position = Point(translation[0], translation[1], translation[2])
		marker.pose.orientation = Quaternion(rotation[0], rotation[1], rotation[2], rotation[3])

		distances = []
		for m in array.markers:
			prev_pose = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
			curr_pose = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
			dist = np.linalg.norm(prev_pose - curr_pose)
			distances.append(dist)

		if not distances or min(distances) > self.detection_thresh:
			id_count += 1
			array.markers.append(marker)

		return array, id_count

	def detected_callback(self, detected_objects):
		if detected_objects is not None:
			for ob_msg in detected_objects.ob_msgs:
				object_name = ob_msg.name
				if object_name == self.known_names[0]:
					self.stop_array, self.stop_id = self.buildMarkers(self.stop_array, self.stop_id, self.stop_color)
				elif object_name == self.known_names[1]:
					self.bottle_array, self.bottle_id = self.buildMarkers(self.bottle_array, self.bottle_id, self.bottle_color)
				elif object_name == self.known_names[2]:
					self.apple_array, self.apple_id = self.buildMarkers(self.apple_array, self.apple_id, self.apple_color)
				elif object_name == self.known_names[3]:
					self.banana_array, self.banana_id = self.buildMarkers(self.banana_array, self.banana_id, self.banana_color)
				elif object_name == self.known_names[3]:
					self.brocoli_array, self.banana_id = self.buildMarkers(self.brocoli_array, self.brocoli_id, self.brocoli_color)



	def loop(self):
		self.stop_pub.publish(self.stop_array)
		self.bottle_pub.publish(self.bottle_array)
		self.apple_pub.publish(self.apple_array)
		self.banana_pub.publish(self.banana_array)
		self.brocoli_pub.publish(self.brocoli_array)


	def run(self):
		rate = rospy.Rate(10) # 10 Hz
		while not rospy.is_shutdown():
			self.loop()
			rate.sleep()

if __name__ == "__main__":
	loc = Locations()
	loc.run()