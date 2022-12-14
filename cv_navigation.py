import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import math
import numpy as np
import time

movementPredThres = 75
speedPub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size = 1)
speed = Twist()

forwardRoadPercentage = 0
rotateLeftRoadPercentage = 0
rotateRightRoadPercentage = 0

x = 0.0
y = 0.0
theta = 0.0

def getForwardPercent(msg):
    global forwardRoadPercentage
    forwardRoadPercentage = msg.data

def getRotateLeftPercent(msg):
    global rotateLeftRoadPercentage
    rotateLeftRoadPercentage = msg.data

def getRotateRightPercent(msg):
    global rotateRightRoadPercentage
    rotateRightRoadPercentage = msg.data

def adjust_movement(x, y):
    global speed
    global speedPub
    speed.linear.x = x
    speed.angular.z = y
    print("velocity set to: " + str(x) + ", " + str(y))
    speedPub.publish(speed)

def velocity_control(moveForward):
    if (moveForward):
        adjust_movement(.2, 0)
    else:
        rotateDirection = -1 if rotateRightRoadPercentage > rotateLeftRoadPercentage else 1 # CW if right side has more road, else CCW
        t_end = time.time() + 2
        while time.time() < t_end:
            adjust_movement(0, rotateDirection * .2)


def newOdom(msg):
    """
    Updates x y coordinates based on odometry data, and calculates theta from quaternion
    """
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def rotate_within_bounds(lowerBound, upperBound, rotateDirection, initialSess):
    while not rospy.is_shutdown():
        adjust_movement(0, rotateDirection * .2)
        if (initialSess and (theta > upperBound or theta < lowerBound)):
            break
        if (rotateDirection == -1 and theta > upperBound): # if moving clockwise, it can pass the lowerbound but not upper bound
            break
        if (rotateDirection == 1 and theta < lowerBound): # if moving counter clockwise, it can pass the upperbound but not lower bound
            break



def main():
    rospy.init_node("cv_low_level")
    # may have to replace while(true) with some rate limit
    global forwardRoadPercentage
    global movementPredThres
    global theta

    boundRotation = False

    forwardSub = rospy.Subscriber("/cv_nav/road_percentage/forward", Float64, getForwardPercent)
    rotateRightSub = rospy.Subscriber("/cv_nav/road_percentage/rotateRight", Float64, getRotateRightPercent)
    rotateLeftSub = rospy.Subscriber("/cv_nav/road_percentage/rotateLeft", Float64, getRotateLeftPercent)

    odomSub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
    r = rospy.Rate(40)

    while not rospy.is_shutdown():
        print("current pred val " + str(forwardRoadPercentage))
        moveForward = forwardRoadPercentage > movementPredThres
        if (boundRotation and not moveForward):
            lowerBound = np.arctan2(np.sin(theta - math.pi / 2), np.cos(theta - math.pi / 2))
            upperBound = np.arctan2(np.sin(theta + math.pi / 2), np.cos(theta + math.pi / 2))
            initialTheta = theta
            rotateDirection = -1 if rotateRightRoadPercentage > rotateLeftRoadPercentage else 1 # CW if right side has more road, else CCW
            initialSess = True
            while not rospy.is_shutdown():
                print("lower bound: " + str(lowerBound))
                print("upper bound: " + str(upperBound))
                print("curr theta: " + str(theta))
                print("initial theta: " + str(initialTheta))
                rotate_within_bounds(lowerBound, upperBound, rotateDirection, initialSess)
                if (theta < lowerBound or theta > upperBound):
                    rotateDirection *= -1
                    initialSess = False
                if (rotateLeftRoadPercentage + rotateRightRoadPercentage) / 2 > movementPredThres:
                    break
        else:
            velocity_control(moveForward)
        r.sleep()

if __name__ == "__main__":
    main()
