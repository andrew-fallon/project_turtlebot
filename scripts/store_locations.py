#!/usr/bin/env python

import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Header, ColorRGBA
from asl_turtlebot.msg import DetectedObject, DetectedObjectList

rospy.init_node("detected_locations", anonymous=True)
stop_pub = rospy.Publisher("stop_sign_marker", Marker, queue_size=10)

known_names = 'stop_sign'

tf_listener = tf.TransformListener()


def detected_callback(msg):
	detected_objects = msg
	if detected_objects is not None:
		for ob_msg in detected_objects.ob_msgs:
			object_name = ob_msg.name
			if object_name == known_names:
				(translation, rotation) = tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))

				stop_marker = Marker()
				stop_marker.header.frame_id = '/map'
				stop_marker.header.stamp = rospy.Time(0)
				stop_marker.id = 123456
				stop_marker.type = Marker().CYLINDER
				stop_marker.action = Marker().ADD
				stop_marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
				stop_marker.scale = Vector3(0.15, 0.15, 0.30)
				stop_marker.pose.position = translation
				stop_marker.pose.orientation = rotation
				print stop_marker.pose
				stop_pub.publish(stop_marker)

rate = rospy.Rate(10) # 10 Hz
while not rospy.is_shutdown():
	rospy.Subscriber('/detector/objects', DetectedObjectList, detected_callback)
	rate.sleep()