import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

global movementPredThres = 50
global speedPub = rospy.Publisher("/jackal_velocity_control/cmd_vel", Twist, queue_size = 1)
global speed = Twist()


def getPredVal():



def adjust_movement(x, y) :
	speed.linear.x = x
	speed.angular.z = y
	speedPub.publish(speed)

def velocity_control(predictionValue):
	if (predictionValue > movementPredThres):
		adjustMovement(4, 0)
	else:
		adjustMovement(0, 4)

def main():
	# may have to replace while(true) with some rate limit
	moveForwardOld = getPredVal() > movementPresThres
	while(True):
		predVal = getPredVal()
		moveForwardNew = predVal > movementPresThres
		if (moveForwardOld != moveForwardNew):
			velocity_control(predVal)
		moveForwardOld = moveForwardNew

if __name__ == "__main__":
    main()
