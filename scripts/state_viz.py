#!/usr/bin/env python

import rospy
import tf
from visualization_msgs.msg import Marker
<<<<<<< HEAD
from geometry_msgs.msg import Pose2D, Vector3
=======
from geometry_msgs.msg import PoseStamped, Vector3
>>>>>>> 6cd2d5e28dc763c0f680e08dc35fbfb1cc2a2c60
from std_msgs.msg import Header, ColorRGBA
from std_msgs.msg import String


<<<<<<< HEAD
=======

>>>>>>> 6cd2d5e28dc763c0f680e08dc35fbfb1cc2a2c60
class State_Viz:

    def __init__(self):
	rospy.init_node("RViz_State", anonymous=True)
	self.state_publisher = rospy.Publisher("turtlebot_marker", Marker, queue_size=10)
	rospy.Subscriber('/state', String, self.rviz_callback)
<<<<<<< HEAD
	rospy.Subscriber('/curr_pose', Pose2D, self.pose_callback)
	self.state = Marker()
	self.x = 0
	self.y = 0

    def pose_callback(self, msg):
	self.x = msg.x
	self.y = msg.y
=======
	self.state = Marker()
>>>>>>> 6cd2d5e28dc763c0f680e08dc35fbfb1cc2a2c60

    def rviz_callback(self, msg):
    
	self.state.header.frame_id = '/map'
	self.state.header.stamp = rospy.Time(0)
	self.state.id = 1
	self.state.type = Marker().TEXT_VIEW_FACING
	self.state.action = Marker().ADD
	self.state.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)
<<<<<<< HEAD
	self.state.scale = Vector3(1.0, 1.0, 0.3)
	self.state.pose.position.x = self.x
	self.state.pose.position.y = self.y
=======
	self.state.scale = Vector3(0.3, 0.3, 0.1)
	self.state.pose.position.x = 3.0
	self.state.pose.position.y = 3.0
>>>>>>> 6cd2d5e28dc763c0f680e08dc35fbfb1cc2a2c60
	self.state.pose.orientation.x = 0.0
	self.state.pose.orientation.y = 0.0
	self.state.pose.orientation.z = 0.0
	self.state.text = msg.data
<<<<<<< HEAD
	self.lifetime = rospy.Duration(1)
=======
	self.lifetime = rospy.Duration(20)
>>>>>>> 6cd2d5e28dc763c0f680e08dc35fbfb1cc2a2c60
	self.state_publisher.publish(self.state)

    def run(self):
	rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            
            rate.sleep()

st = State_Viz()
st.run()  
