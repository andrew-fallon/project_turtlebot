#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String, Int16, Bool
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from asl_turtlebot.msg import DetectedObject
import tf
import math
from enum import Enum
import numpy as np

# check later if explore script is updated
from explore import Explorer

# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup (hw2+)
use_gazebo = rospy.get_param("sim")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")


# threshold at which we consider the robot at a location
POS_EPS = .1    # meters
THETA_EPS = .15  # radians

# time to stop at a stop sign
STOP_TIME = 4.

# minimum distance from a stop sign to obey it
STOP_MIN_DIST = 1.0

# time taken to cross an intersection
CROSSING_TIME = 10.

# initial delay timing
INIT_DELAY = 5.0
DRIVE_FWD = 2.0

# state machine modes
class Mode(Enum):
    INIT = 0
    IDLE = 1
    EXPL = 2
    DELI = 3
    STOP = 4
    CROSS = 5
    RESET = 6
    MANUAL = 7

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
        self.mode = Mode.INIT
        self.last_mode_printed = None
        self.waiting = 1    # Used to keep track of waiting (0=disarmed, 1=armed, 2=waiting)
        self.init_state = 0 # Keeps track of initialization state
        self.b4stop = None   # used in CROSS mode logic
        self.dq_empty = True

        # home state
        self.x_home = 0
        self.y_home = 0
        self.theta_home = 0
        # current delivery goal
        self.x_deli = None
        self.y_deli = None
        self.theta_deli = None
        # current exploration goal
        self.x_expl = None
        self.y_expl = None
        self.theta_expl = None
        # reset goal
        self.x_reset = None
        self.y_reset = None
        self.theta_reset = None

        self.trans_listener = tf.TransformListener()
        # command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        # nav pose for controller
        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        # command vel (used for idling)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.state_publisher = rospy.Publisher('/state', String, queue_size=10)
        self.pose_publisher = rospy.Publisher('/curr_pose', Pose2D, queue_size=10)
        self.start_nav = rospy.Publisher('/start_exploration', Bool, queue_size=10)

        # subscribers
        # stop sign detector
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)
        # high-level navigation pose
        rospy.Subscriber('/nav_pose', Pose2D, self.nav_pose_callback)
        # if using gazebo, we have access to perfect state
        if use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        # subscribe to nav goal click
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)

        # custom subscribers
        rospy.Subscriber('/is_stuck', Bool, self.is_stuck_callback)
        rospy.Subscriber('/delivery_request', String, self.delivery_request_callback)
        rospy.Subscriber('/done_exploring', Bool, self.done_exploring_callback)
        rospy.Subscriber('/deli_pose', Pose2D, self.deli_goal_callback)
        rospy.Subscriber('/expl_pose', Pose2D, self.expl_goal_callback)
        rospy.Subscriber('/reset_pose', Pose2D, self.reset_goal_callback)
        rospy.Subscriber('/dq_empty', Bool, self.dq_empty_callback)
        rospy.Subscriber('/start_exploration', Bool, self.start_exp_callback)

    def dq_empty_callback(self, msg):
        self.dq_empty = msg.data

    def done_exploring_callback(self, msg):
        if msg.data:
            self.mode = Mode.IDLE

    def reset_goal_callback(self, msg):
        self.x_reset = msg.x
        self.y_reset = msg.y
        self.theta_reset = msg.theta

    def deli_goal_callback(self, msg):
        self.x_deli = msg.x
        self.y_deli = msg.y
        self.theta_deli = msg.theta

    def expl_goal_callback(self, msg):
        self.x_expl = msg.x
        self.y_expl = msg.y
        self.theta_expl = msg.theta

    def start_exp_callback(self, msg):
        if msg.data:
            self.mode = Mode.EXPL
    # send to RESET mode if robot cannot overcome NAV errors        
    def is_stuck_callback(self, msg):
        if msg.data:
            rospy.logwarn("Cogswell got stuck, resetting...")
            self.b4reset = self.mode
            self.mode = Mode.RESET

    def delivery_request_callback(self, msg):
        self.mode = Mode.DELI

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
        self.mode = Mode.MANUAL

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
        if dist > 0 and dist < STOP_MIN_DIST and (self.mode == Mode.EXPL or self.mode == Mode.DELI or self.mode == Mode.MANUAL):
            self.init_stop_sign()

    def nav_to_turtle_goal(self, x, y, theta):
        """
        """
        if x is not None:
            nav_g_msg = Pose2D()
            nav_g_msg.x = x
            nav_g_msg.y = y
            nav_g_msg.theta = theta
            self.nav_goal_publisher.publish(nav_g_msg)


    def go_to_pose(self, x, y, theta):
        """ sends the current desired pose to the pose controller """

        pose_g_msg = Pose2D()
        pose_g_msg.x = x
        pose_g_msg.y = y
        pose_g_msg.theta = theta

        self.pose_goal_publisher.publish(pose_g_msg)

    def nav_to_pose(self):
        """ sends the current desired pose to the navigator """

        nav_g_msg = Pose2D()
        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.theta_g

        self.nav_goal_publisher.publish(nav_g_msg)

    def nav_to_home(self):
        if self.close_to(self.x_home,self.y_home,self.theta_home):
            self.mode = Mode.IDLE
        else:
            # go home
            self.nav_to_turtle_goal(self.x_home, self.y_home, self.theta_home)


    def stay_idle(self):
        """ sends zero velocity to stay put """
        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self,x,y,theta):
        """ checks if the robot is at a pose within some threshold """

        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS and abs(theta-self.theta)<THETA_EPS)

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """
        self.b4stop = self.mode
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
        if self.waiting == 1:   # If waiting armed
            self.waiting = 2    # Set wating mode
            self.start_time = rospy.get_rostime()
        if rospy.get_rostime() - self.start_time > rospy.Duration.from_sec(wait_time):
            self.waiting = 0    # Disarm waiting

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

        ################################################
        # SIR FRANCIS BACON STATE MACHINE
        ################################################

        # INIT only runs when the robot is booted up
        if self.mode == Mode.INIT:
            self.state_publisher.publish(String("INIT"))
            # Wait for startup
            if self.init_state == 0:
                self.wait(INIT_DELAY)
                if self.waiting == 0:
                    self.waiting = 1        # Re-arm waiting
                    self.init_state = 1     # Switch to drive forward state

            # Drive straight forward        
            elif self.init_state == 1:
                vel = Twist()
                vel.linear.x = 0.15         # ** SPEED TO DRIVE FORWARD **
                self.cmd_vel_publisher.publish(vel) # Send drive command
                self.wait(DRIVE_FWD)              # ** TIME TO DRIVE FORWARD **
                if self.waiting == 0:       # If waiting disarmed
                    self.stay_idle()        # Stop Moving
                    self.waiting = 1        # Re-arm waiting
                    self.x_g = self.x
                    self.y_g = self.y
                    self.theta_g = self.theta + np.pi   #Wrap to pi
                    if self.theta_g > np.pi:
                        self.theta_g = self.theta_g - 2*np.pi
                    self.init_state = 2     # Switch to turn state

            else:
                self.start_nav.publish(Bool(True))
                self.mode = Mode.EXPL   # Switch to explore! yay!

        # IDLE runs during the following conditions:
        #       - stopped at stop sign
        #       - reached rviz goal state
        #       - done exploring and waiting for commands
        #       - reached home after running delivery mode
        elif self.mode == Mode.IDLE:
            self.state_publisher.publish(String("IDLE"))
            # send zero velocity
            self.stay_idle()

        # EXPL runs only once, directly after the init mode
        elif self.mode == Mode.EXPL:
            self.state_publisher.publish(String("EXPL"))
            self.nav_to_turtle_goal(self.x_expl, self.y_expl, self.theta_expl)

        # DELI mode runs every time a new delivery input is received and terminates after reaching home
        elif self.mode == Mode.DELI:
            self.state_publisher.publish(String("DELI"))
            if not self.dq_empty:
                self.nav_to_turtle_goal(self.x_deli, self.y_deli, self.theta_deli)
            else:
                self.nav_to_home()

        # STOP mode runs every time a stop sign is detected within a specified range
        elif self.mode == Mode.STOP:
            self.state_publisher.publish(String("STOP"))
            # at a stop sign
            if self.has_stopped():
                self.init_crossing()
            else:
                self.stay_idle()

        # CROSS mode runs directly after STOP and resumes the state that was running before STOP
        elif self.mode == Mode.CROSS:
            self.state_publisher.publish(String("CROSS"))
            # crossing an intersection
            if self.has_crossed():
                self.mode = self.b4stop
            else:
                if self.b4stop == Mode.DELI:
                    if not self.dq_empty:
                        self.nav_to_turtle_goal(self.x_deli, self.y_deli, self.theta_deli)
                    else:
                        self.nav_to_home()
                elif self.b4stop == Mode.EXPL:
                    self.nav_to_turtle_goal(self.x_expl, self.y_expl, self.theta_expl)
                elif self.b4stop == Mode.MANUAL:
                    self.nav_to_turtle_goal(self.x_g, self.y_g, self.theta_g)
                elif self.b4stop == Mode.RESET:
                    self.nav_to_turtle_goal(self.x_reset, self.y_reset, self.theta_reset)
                else:
                    rospy.logwarn('WHAT IS HAPPENING!')

        # MANUAL mode runs every time a 2D nav goal is received from rviz
        elif self.mode == Mode.MANUAL:
            self.state_publisher.publish(String("MANUAL"))
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.nav_to_turtle_goal(self.x_g, self.y_g, self.theta_g)

        # RESET mode runs every time there is an error within navigator.py, it commands and new, nearby 
        #   nav goal and then resumes the previous mode it was in
        elif self.mode == Mode.RESET:
            self.state_publisher.publish(String("RESET"))
            rospy.sleep(1.5)
            norm = ( (self.x - self.x_reset)**2 + (self.y - self.y_reset)**2 )**0.5
            if norm < POS_EPS:
                rospy.loginfo("RESET STATE REACHED")
                self.stay_idle()    # command zero velocity before switching
                self.mode = self.b4reset
            else:
                self.go_to_pose(self.x_reset, self.y_reset, self.theta_reset)

        else:
            raise Exception('This mode is not supported: %s' % str(self.mode))

        curr_position = Pose2D()
        curr_position.x = self.x
        curr_position.y = self.y
        curr_position.theta = self.theta
        self.pose_publisher.publish(curr_position)

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
