import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

global movementPredThres = 50


getPredVal():




velocity_control(predictionValue):
	pub = rospy.Publisher("/jackal_velocity_control/cmd_vel", Twist, queue_size = 1)
	speed = Twist()
	if (predictionValue > movementPredThres):
		speed.linear.x = 4
		speed.angular.z = 0 
	else:
		speed.linear.x = 0
		speed.angular.z = 4
	pub.publish(speed)

		
