#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from tf.transformations import euler_from_quaternion
import math

from tracking_turtlebot import clamp, makeSimpleProfile, PID

# Minimums from Burger/Waffle
MAX_LIN_VEL = 0.22
MAX_ANG_VEL = 1.82

# max vel changes
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1


class TurtleBot:
	
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous = True)
        
        # Create a publisher for movement
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # Create an odometry subscriber for pose
        # Should use a magnetometer instead
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # Create a subscriber for polaris heading
        self.heading_sub = rospy.Subscriber('/heading_polaris', Float64,
                                            self.heading_callback)
        # Create a velocity subscriber
        self.vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.vel_callback)
        
        # Initialize Variables
        self.target_heading = 0.0
        self.control_angular_vel = 0.0

        self.rate = rospy.Rate(5) # 5hz 
        #(a higher rate would be good but my VM struggles
        
        self.twist = Twist()  # initialize with zeros
        self.odom_callback(Odometry()) # initialize with zeros
        self.heading_polaris = self.rpy['yaw'] # initialize to current yaw
        
        # PID might be overkill. Poorly tuned PID, but it's fine
        self.pid = PID(P=0.1, I=0.0, D=0)
        
        # Run loop
        self.face_polaris()
        
    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        
        # for conveinnece convert to rpy
        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.rpy = {'roll':roll*360/math.pi,
                    'pitch':pitch*360/math.pi,
                    'yaw':yaw*360/math.pi}
        
    def heading_callback(self, heading):
        self.heading_polaris = heading.data
            
    def vel_callback(self, twist):
        self.twist = twist
        
    def face_polaris(self):
        while not rospy.is_shutdown():
            self.rotate()
            
    def rotate(self):
        # TODO: implement logic to take the shortest path to orientation
        
        target = self.heading_polaris  # For debugging, should subscribe to heading
        state = self.rpy['yaw']
        
        self.target_angular_vel = self.pid.calc(target, state)
        
        # print("state: ", round(state), ';',
        #       'error: ', round(state-target), ';')
        
        twist = Twist()
        # check the robot is stopped before reorienting
        if all([self.twist.linear.x == 0,
                self.twist.linear.y == 0,
                self.twist.linear.z == 0]):
            twist.linear = self.twist.linear
            
            # This might causes issues with slop in the PID controller
            control_angular_vel = makeSimpleProfile(self.control_angular_vel,
                                                    self.target_angular_vel,
                                                    (ANG_VEL_STEP_SIZE/2.0))
            control_angular_vel = clamp(control_angular_vel,
                                        -MAX_ANG_VEL, MAX_ANG_VEL)
            self.control_angular_vel = control_angular_vel
            
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_vel
            
            self.pub.publish(twist)
            
        else: # something else is controlling the robot, wait for it to stop
            
            rospy.loginfo('Something else is trying to move the robot')
            self.pid.empty_states()  # Reset the PID

if __name__ == '__main__':
	try:
	    turtle = TurtleBot()
	except rospy.ROSInterruptException:
		pass