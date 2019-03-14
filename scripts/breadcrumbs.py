#!/usr/bin/env python

import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

class Breadcrumbs:
	def __init__(self):
		rospy.init_node('breadcrumbs', anonymous=True)

		#Breadcrumb variables
		self.rate = 10
		self.dist_thr = 0.25

		self.pub = rospy.Publisher('breadcrumb_marker', Marker, queue_size=10)
		self.trans_listener = tf.TransformListener()

		self.points = []
		self.x = None
		self.y = None
		self.last_x = None
		self.last_y = None

	def loop(self):

		# Get current location
		try:
			origin_frame = "/map"
			(translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
			self.x = translation[0]
			self.y = translation[1]
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass

		if self.last_x:
			dist = ((self.x-self.last_x)**2.0 + (self.y-self.last_y)**2.0)**0.5
			if dist > self.dist_thr:
				self.points.append(Point(self.x, self.y, 0.1))
				self.last_x = self.x
				self.last_y = self.y
		else:
			print 'ping', self.last_x
			self.points.append(Point(self.x, self.y, 0.1))
			self.last_x = self.x
			self.last_y = self.y

		markers = Marker(
			type=Marker.POINTS,
			id=0,
			lifetime=rospy.Duration(0),
			pose=Pose(Point(0.0, 0.0, 0.0), Quaternion(0, 0, 0, 1)),
			scale=Vector3(0.02, 0.02, 0.02),
			header=Header(frame_id='/map'),
			color=ColorRGBA(1.0, 1.0, 0.1, 1.0),
			points=self.points)
		self.pub.publish(markers)

	def run(self):
		rate = rospy.Rate(self.rate) # 10 Hz
		while not rospy.is_shutdown():
			self.loop()
			rate.sleep()

if __name__ == '__main__':
	breadcrumb_marker = Breadcrumbs()
	breadcrumb_marker.run()