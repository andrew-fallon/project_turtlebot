#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String, Int16, Bool
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
import tf
import math
from enum import Enum
import numpy as np

# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup (hw2+)
use_gazebo = rospy.get_param("sim")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")


# threshold at which we consider the robot at a location
POS_EPS = .1    # meters
THETA_EPS = .3  # radians

# time to stop at a stop sign
STOP_TIME = 4

# minimum distance from a stop sign to obey it
STOP_MIN_DIST = 1.5

# time taken to cross an intersection
CROSSING_TIME = 10

# state machine modes, not all implemented
class Mode(Enum):
    IDLE = 1
    POSE = 2
    STOP = 3
    CROSS = 4
    NAV = 5
    MANUAL = 6
    INITIAL = 7


print "supervisor settings:\n"
print "use_gazebo = %s\n" % use_gazebo
print "mapping = %s\n" % mapping

class Supervisor:

    def __init__(self):
        rospy.init_node('turtlebot_supervisor_nav', anonymous=True)
        # initialize variables
        self.x = 0
        self.y = 0
        self.theta = 0
        self.mode = Mode.INITIAL
        self.last_mode_printed = None
        # initialize delivery flag to false to start in explore mode:
        self.deliv_flag = False
        self.trans_listener = tf.TransformListener()
        # command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        # nav pose for controller
        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        # command vel (used for idling)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.waiting = 1 	# Used to keep track of waiting (0=disarmed, 1=armed, 2=waiting)
        self.init_state = 0	# Keeps track of initialization state
        self.state_publisher = rospy.Publisher('/state', String, queue_size=10)

        # subscribers
        # stop sign detector
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)
        # high-level navigation pose
        rospy.Subscriber('/nav_pose', Pose2D, self.nav_pose_callback)
        # if using gazebo, we have access to perfect state
        if use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        # we can subscribe to nav goal click
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        rospy.Subscriber('/is_stuck', Bool, self.is_stuck_callback)
        # subscribe to 
        rospy.Subscriber('/delivery_request', String, self.delivery_request_callback)
        
    def is_stuck_callback(self, msg):
        print(msg)
        if msg.data:
            self.mode = Mode.IDLE

    def delivery_request_callback(self, msg):
        print(msg)
        print(msg.data)
        if msg.data == "" or msg.data == None:
            self.deliv_flag = False
            # TO DO:loop through and associate delivery requests with marker locations 
        else:
            self.deliv_flag = True
        print(self.deliv_flag)

    def gazebo_callback(self, msg):
        pose = msg.pose[msg.name.index("turtlebot3_burger")]
        twist = msg.twist[msg.name.index("turtlebot3_burger")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]

    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """
        origin_frame = "/map" if mapping else "/odom"
        print("rviz command received!")
        try:
            
            nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
            self.x_g = nav_pose_origin.pose.position.x
            self.y_g = nav_pose_origin.pose.position.y
            quaternion = (
                    nav_pose_origin.pose.orientation.x,
                    nav_pose_origin.pose.orientation.y,
                    nav_pose_origin.pose.orientation.z,
                    nav_pose_origin.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.theta_g = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        self.mode = Mode.NAV

    def nav_pose_callback(self, msg):
        self.x_g = msg.x
        self.y_g = msg.y
        self.theta_g = msg.theta
        self.mode = Mode.NAV

    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign
        dist = msg.distance
        # if close enough and in nav mode, stop
        if dist > 0 and dist < STOP_MIN_DIST and self.mode == Mode.NAV:
            self.init_stop_sign()

    def go_to_pose(self):
        """ sends the current desired pose to the pose controller """

        pose_g_msg = Pose2D()
        pose_g_msg.x = self.x_g
        pose_g_msg.y = self.y_g
        pose_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(pose_g_msg)

    def nav_to_pose(self):
        """ sends the current desired pose to the navigator """

        nav_g_msg = Pose2D()
        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.theta_g

        self.nav_goal_publisher.publish(nav_g_msg)

    def stay_idle(self):
        """ sends zero velocity to stay put """
        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self,x,y,theta):
        """ checks if the robot is at a pose within some threshold """

        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS and abs(theta-self.theta)<THETA_EPS)

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """

        self.stop_sign_start = rospy.get_rostime()
        self.mode = Mode.STOP

    def has_stopped(self):
        """ checks if stop sign maneuver is over """

        return (self.mode == Mode.STOP and (rospy.get_rostime()-self.stop_sign_start)>rospy.Duration.from_sec(STOP_TIME))
        
    def init_crossing(self):
        """ initiates an intersection crossing maneuver """

        self.cross_start = rospy.get_rostime()
        self.mode = Mode.CROSS

    def has_crossed(self):
        """ checks if crossing maneuver is over """

        return (self.mode == Mode.CROSS and (rospy.get_rostime()-self.cross_start)>rospy.Duration.from_sec(CROSSING_TIME))

    def delay(self, time):
        """ delays for specified number of seconds (time must be integer) """
        rate = rospy.Rate(1) # 1 Hz
        for i in range(time):
            rate.sleep() # Sleeps for 1/rate sec

    def get_time(self):
    	""" Gets current ROS time """
    	self.time = rospy.get_rostime()

    def wait(self, wait_time):
    	if self.waiting == 1:	# If waiting armed
    		self.waiting = 2	# Set wating mode
    		self.start_time = rospy.get_rostime()
    	if rospy.get_rostime() - self.start_time > rospy.Duration.from_sec(wait_time):
    		self.waiting = 0	# Disarm waiting


    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

    
        if not use_gazebo:
            try:
                origin_frame = "/map" if mapping else "/odom"
                (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        # logs the current mode
        if not(self.last_mode_printed == self.mode):
            rospy.loginfo("Current Mode: %s", self.mode)
            self.last_mode_printed = self.mode

        # checks wich mode it is in and acts accordingly
        if self.mode == Mode.IDLE:
            # send zero velocity
            self.stay_idle()
            self.state_publisher.publish("IDLE")

        elif self.mode == Mode.POSE:
            # moving towards a desired pose
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.go_to_pose()
            self.state_publisher.publish("POSE")

        elif self.mode == Mode.STOP:
            # at a stop sign
            if self.has_stopped():
                self.init_crossing()
            else:
                self.stay_idle()
            self.state_publisher.publish("STOP")

        elif self.mode == Mode.CROSS:
            # crossing an intersection
            if self.has_crossed():
                self.mode = Mode.NAV
            else:
                self.nav_to_pose()
            self.state_publisher.publish("CROSS")

        elif self.mode == Mode.NAV:
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.nav_to_pose()
            self.state_publisher.publish("NAV")

        elif self.mode == Mode.INITIAL:
        	# Wait for startup
			if self.init_state == 0:
				self.wait(2)
				if self.waiting == 0:
					self.waiting = 1 		# Re-arm waiting
					self.init_state = 1		# Switch to drive forward state

			# Drive straight forward		
			elif self.init_state == 1:
				vel = Twist()
				vel.linear.x = 0.15			# ** SPEED TO DRIVE FORWARD **
				self.cmd_vel_publisher.publish(vel) # Send drive command
				self.wait(2.0)				# ** TIME TO DRIVE FORWARD **
				if self.waiting == 0:		# If waiting disarmed
					self.stay_idle()		# Stop Moving
					self.waiting = 1 		# Re-arm waiting
					self.x_g = self.x
					self.y_g = self.y
					self.theta_g = self.theta + np.pi 	#Wrap to pi
					if self.theta_g > np.pi:
						self.theta_g = self.theta_g - 2*np.pi
					self.init_state = 2		# Switch to turn state

			# # Turn 180 degrees
			# elif self.init_state == 2:
			# 	vel = Twist()
			# 	vel.angular.z = 1.0					# Turning speed
			# 	self.cmd_vel_publisher.publish(vel)	# Start turning
			# 	if self.close_to(self.x_g,self.y_g,self.theta_g):
			# 		self.stay_idle			# Stop
			# 		self.init_state = 3
			# 		# Next pose
			# 		dist = 0.25			# Distance to drive forward
			# 		self.x_g = self.x + dist*math.cos(self.theta)
			# 		self.y_g = self.y + dist*math.sin(self.theta)

			# # Drive forward 1.2m
			# elif self.init_state == 3:
			# 	self.go_to_pose()
			# 	if self.close_to(self.x_g,self.y_g,self.theta_g):
			# 		self.stay_idle			# Stop
			# 		self.init_state = 4

			else:
				self.mode = Mode.IDLE   # Switch to idle


        else:
            raise Exception('This mode is not supported: %s'
                % str(self.mode))

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    # jasdfi
    sup = Supervisor()
    sup.run()
