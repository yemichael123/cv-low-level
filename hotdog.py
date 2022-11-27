import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import math
import numpy as np

movementPredThres = 75
speedPub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size = 10)
speed = Twist()

forwardRoadPercentage = 0
rotateRoadPercentage = 0
leftRoadPercentage = 0
rightRoadPercentage = 0

x = 0.0
y = 0.0
theta = 0.0

def getForwardPercent(msg):
    global forwardRoadPercentage
    forwardRoadPercentage = msg.data

def getRotatePercent(msg):
    global rotateRoadPercentage
    rotateRoadPercentage = msg.data

def getRightPercent(msg):
    global rightRoadPercentage
    rightRoadPercentage = msg.data

def getLeftPercent(msg):
    global leftRoadPercentage
    leftRoadPercentage = msg.data

def adjust_movement(x, y):
    global speed
    global speedPub
    speed.linear.x = x
    speed.angular.z = y
    print("velocity set to: " + str(x) + ", " + str(y))
    speedPub.publish(speed)

def velocity_control(moveForward):
    adjust_movement(.2 if moveForward else 0, 0 if moveForward else .2)

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
        if (rotateDirection == 1 and theta > upperBound): # if moving clockwise, it can pass the lowerbound but not upper bound
            break
        if (rotateDirection == -1 and theta < lowerBound): # if moving counter clockwise, it can pass the upperbound but not lower bound
            break



def main():
    rospy.init_node("cv_low_level")
    # may have to replace while(true) with some rate limit
    global forwardRoadPercentage
    global movementPredThres
    global theta

    boundRotation = True

    forwardSub = rospy.Subscriber("/cv_nav/road_percentage/forward", Float64, getForwardPercent)
    rotateSub = rospy.Subscriber("/cv_nav/road_percentage/rotate", Float64, getRotatePercent)
    rightSub = rospy.Subscriber("/cv_nav/road_percentage/right", Float64, getRightPercent)
    leftSub = rospy.Subscriber("/cv_nav/road_percentage/left", Float64, getLeftPercent)

    odomSub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
    r = rospy.Rate(40)

    while not rospy.is_shutdown():
        print("current pred val " + str(forwardRoadPercentage))
        moveForward = forwardRoadPercentage > movementPredThres
        if (boundRotation and not moveForward):
            lowerBound = np.arctan2(np.sin(theta - math.pi / 2), np.cos(theta - math.pi / 2))
            upperBound = np.arctan2(np.sin(theta + math.pi / 2), np.cos(theta + math.pi / 2))
            initialTheta = theta
            rotateDirection = -1 if rightRoadPercentage > leftRoadPercentage else 1 # CW if right side has more road, else CCW
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
                if rotateRoadPercentage > movementPredThres:
                    break
        else:
            velocity_control(moveForward)
        r.sleep()

if __name__ == "__main__":
    main()
