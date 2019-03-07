#!/usr/bin/env python

import rospy
import numpy as np
import tf
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Header, ColorRGBA
from asl_turtlebot.msg import DetectedObject, DetectedObjectList

class Locations:

	def __init__(self):
		rospy.init_node('detected_locations', anonymous=True)
		self.stop_pub = rospy.Publisher('stop_sign_markers', MarkerArray, queue_size=10)
		self.bottle_pub = rospy.Publisher('bottle_markers', MarkerArray, queue_size=10)

		self.tf_listener = tf.TransformListener()
		self.detection_thresh = 0.5

		self.stop_array = MarkerArray()
		self.bottle_array = MarkerArray()
		self.known_names = ["stop_sign", "bottle"]
		self.stop_id = 0
		self.bottle_id = 0
		self.stop_color = ColorRGBA(1.0,0.0,0.0,1.0)
		self.bottle_color = ColorRGBA(0.0,0.0,1.0,1.0)
		self.stop_flag = False
		self.bottle_flag = False
		rospy.Subscriber('/detector/objects', DetectedObjectList, self.detected_callback)

	def buildMarkers(self, known_name, object_name, array, id_count, update_flag, color):
		if object_name == known_name:
			update_flag = True
			(translation, rotation) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
			marker = Marker()
			marker.header.frame_id = '/map'
			marker.header.stamp = rospy.Time(0)
			marker.id = id_count
			marker.type = Marker().CYLINDER
			marker.action = Marker().ADD
			marker.color = color
			marker.scale = Vector3(0.05, 0.05, 0.50)
			marker.pose.position.x = translation[0]
			marker.pose.position.y = translation[1]
			marker.pose.position.z = translation[2]
			marker.pose.orientation.x = rotation[0]
			marker.pose.orientation.y = rotation[1]
			marker.pose.orientation.z = rotation[2]
			marker.pose.orientation.w = rotation[3]

			if len(array.markers) < 1:
				array.markers.append(marker)
				id_count += 1

			distances = []
			for m in array.markers:
				prev_pose = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
				curr_pose = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
				dist = np.linalg.norm(prev_pose - curr_pose)
				distances.append(dist)

			if min(distances) > self.detection_thresh:
				id_count += 1
				array.markers.append(marker)

		return array, update_flag, id_count


	def detected_callback(self, detected_objects):
		if detected_objects is not None:
			for ob_msg in detected_objects.ob_msgs:
				object_name = ob_msg.name
				self.Update_Locations = True

				self.stop_array, self.stop_flag, self.stop_id = self.buildMarkers(
												  self.known_names[0], object_name, self.stop_array, self.stop_id, self.stop_flag, self.stop_color)

				self.bottle_array, self.bottle_flag, self.bottle_id = self.buildMarkers(
												  self.known_names[1], object_name, self.bottle_array, self.bottle_id, self.bottle_flag, self.bottle_color)

	def loop(self):
		if self.stop_flag:
			print 'Publishing'
			print len(self.stop_array.markers)
			self.stop_pub.publish(self.stop_array)
			print len(self.stop_array.markers)
			self.stop_flag = False
		if self.bottle_flag:
			# print 'Publishing'
			# print len(self.stop_array.markers)
			self.bottle_pub.publish(self.bottle_array)
			# print len(self.stop_array.markers)
			self.bottle_flag = False


	def run(self):
		rate = rospy.Rate(10) # 10 Hz
		while not rospy.is_shutdown():
			self.loop()
			rate.sleep()

if __name__ == "__main__":
	loc = Locations()
	loc.run()