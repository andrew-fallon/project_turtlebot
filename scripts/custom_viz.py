#!/usr/bin/env python

import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Header, ColorRGBA

rospy.init_node("RViz_Custom_Markers", anonymous=True)

bot_topic = "turtlebot_marker"
goal_topic = 'goal_marker'
bot_publisher = rospy.Publisher(bot_topic, Marker, queue_size=10)
goal_publisher = rospy.Publisher(goal_topic, Marker, queue_size=10)

def rviz_goal_callback(msg):
    tf_listener = tf.TransformListener()
    nav_pose_origin = tf_listener.transformPose('/map', msg)
    goal = Marker()
    goal.header.frame_id = '/map'
    goal.header.stamp = rospy.Time(0)
    goal.id = 1
    goal.type = Marker().ARROW
    goal.action = Marker().ADD
    goal.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)
    goal.scale = Vector3(0.3, 0.03, 0.01)
    goal.pose.position = nav_pose_origin.pose.position
    goal.pose.orientation = nav_pose_origin.pose.orientation
    goal_publisher.publish(goal)

turtlebot = Marker()
turtlebot.header.frame_id = '/base_footprint'
turtlebot.header.stamp = rospy.Time(0)
turtlebot.id = 0
turtlebot.type = Marker().CYLINDER
turtlebot.action = Marker().ADD
turtlebot.color = ColorRGBA(1.0, 1.0, 0.0, 1.0)
turtlebot.scale = Vector3(0.15, 0.15, 0.30)
turtlebot.pose.position.x = 0.0
turtlebot.pose.position.y = 0.0

rate = rospy.Rate(10) # 10 Hz
while not rospy.is_shutdown():
    bot_publisher.publish(turtlebot)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, rviz_goal_callback)
    rate.sleep()     