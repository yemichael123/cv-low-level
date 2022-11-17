import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

movementPredThres = 50
speedPub = rospy.Publisher("/jackal_velocity_control/cmd_vel", Twist, queue_size = 1)
speed = Twist()

roadPercentage = 0

def getPredVal(msg):
    global roadPercentage
    roadPercentage = msg.data


def adjust_movement(x, y) :
    speed.linear.x = x
    speed.angular.z = y
    speedPub.publish(speed)

def velocity_control(moveForward):
    # if (predictionValue > movementPredThres):
    #   adjustMovement(4, 0)
    # else:
    #   adjustMovement(0, 4)
    adjustMovement(4 if moveForward else 0, 0 if moveForward else 4)

def main():
    # may have to replace while(true) with some rate limit
    global roadPercentage
    global movementPredThres
    sub = rospy.Subscriber("/cv_nav/road_percentage", Float64, getPredVal)
    moveForwardOld = roadPercentage > movementPredThres
    while(True):
        moveForwardNew = roadPercentage > movementPredThres
        if (moveForwardOld != moveForwardNew):
            velocity_control(moveForwardNew)
        moveForwardOld = moveForwardNew

if __name__ == "__main__":
    main()
