#!/usr/bin/env python

import rospy
import tf
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Header, ColorRGBA
from asl_turtlebot.msg import DetectedObject, DetectedObjectList


class Locations:

	def __init__(self):
		rospy.init_node('detected_locations', anonymous=True)
		self.stop_pub = rospy.Publisher('stop_sign_markers', Marker, queue_size=10)

		self.tf_listener = tf.TransformListener()

		self.known_names = "stop_sign"
		self.stop_marker = Marker()
		self.stop_marker.header.frame_id = '/map'
		self.stop_marker.header.stamp = rospy.Time(0)
		self.stop_marker.id = 0
		self.stop_marker.type = Marker().CYLINDER
		self.stop_marker.action = Marker().ADD
		self.stop_marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
		self.stop_marker.scale = Vector3(0.15, 0.15, 0.30)
		self.Update_Locations = False
		rospy.Subscriber('/detector/objects', DetectedObjectList, self.detected_callback)


	def detected_callback(self, detected_objects):
		print "callback"
		if detected_objects is not None:
			print "detected object"
			for ob_msg in detected_objects.ob_msgs:
				object_name = ob_msg.name
				if object_name == self.known_names:
					(translation, rotation) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
					self.stop_marker.pose.position.x = translation[0]
					self.stop_marker.pose.position.y = translation[1]
					self.stop_marker.pose.position.z = translation[2]
					self.stop_marker.pose.orientation.x = rotation[0]
					self.stop_marker.pose.orientation.y = rotation[1]
					self.stop_marker.pose.orientation.z = rotation[2]
					self.stop_marker.pose.orientation.w = rotation[3]
					self.Update_Locations = True
					print "updated location"


	def loop(self):
		if self.Update_Locations:
			self.stop_pub.publish(self.stop_marker)
			self.Update_Locations = False


	def run(self):
		rate = rospy.Rate(10) # 10 Hz
		while not rospy.is_shutdown():
			self.loop()
			rate.sleep()

if __name__ == "__main__":
	loc = Locations()
	loc.run()