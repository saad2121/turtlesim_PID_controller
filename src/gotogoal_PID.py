#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
from math import pow,atan2,sqrt
PI = 3.1415926535897

# PID parameters for rotation
p_r = 3
i_r = 0.00001
d_r = 1

# PID parameters for translation
p_t = 1.3
i_t = 0.0001
d_t = 1
class turtlebot():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        self.pose = Pose()
        self.rate = rospy.Rate(10)

    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
       

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance

    def move2goal(self):
        goal_pose = Pose()
        goal_pose.x = input("Set your x goal:")
        goal_pose.y = input("Set your y goal:")
	goal_theta = input("Set your theta goal:")
        distance_tolerance = input("Set your distance tolerance:")
        theta_tol = input("Set your theta tolerance:")

	while goal_theta > 360:
		goal_theta = goal_theta - 360
	
	# conerting angles from degrees to radians
	goal_pose.theta = goal_theta * PI / 180
	theta_tolerance = theta_tol * PI / 180
	 
        vel_msg = Twist()

	#PID Controller

	#rotate delta_1
	e_theta1_old = 0
	ei_theta1 = 0
	while abs(atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta) >= theta_tolerance:
	    en_theta1 = atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta
	    ed_theta1 = en_theta1 - e_theta1_old
	    ei_theta1 = ei_theta1 + en_theta1 
	    #linear velocity 
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = p_r * en_theta1 + i_r * ei_theta1 + d_r * ed_theta1

            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
	    
	    e_theta1_old = en_theta1
        #Stopping our robot after the movement is over
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)


	#delta translate
	e_translatn_old = 0
	ei_translatn = 0
        while sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2)) >= distance_tolerance:
	    en_translatn = sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
	    ed_translatn = en_translatn - e_translatn_old
	    ei_translatn = ei_translatn + en_translatn

            #linear velocity in the x-axis:
            vel_msg.linear.x = p_t * en_translatn + i_t * ei_translatn + d_t * ed_translatn
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
	    e_translatn_old = en_translatn	    

        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)

	#delta rotation 2
	e_theta2_old = 0
	ei_theta2 = 0
	while abs(goal_pose.theta - self.pose.theta) >= theta_tolerance:
	    en_theta2 = goal_pose.theta - self.pose.theta
	    ed_theta2 = en_theta2 - e_theta2_old
	    ei_theta2 = ei_theta2 + en_theta2
	    
	    #linear velocity 
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = p_r * en_theta2 + i_r * ei_theta2 + d_r * ed_theta2

            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
	    e_theta2_old = en_theta2
        #Stopping our robot after the movement is over
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        rospy.spin()

if __name__ == '__main__':
    try:
        #Testing our function
        x = turtlebot()
        x.move2goal()

    except rospy.ROSInterruptException: pass
