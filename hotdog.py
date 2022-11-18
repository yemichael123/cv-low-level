import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

movementPredThres = 50
speedPub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size = 10)
speed = Twist()

roadPercentage = 0

def getPredVal(msg):
    global roadPercentage
    print(f"pred value: {msg.data}")
    roadPercentage = msg.data

def adjust_movement(x, y):
    global speed
    global speedPub
    speed.linear.x = x
    speed.angular.z = y
    print(f"velocity set to: {x}, {y}")
    speedPub.publish(speed)

def velocity_control(moveForward):
    # if (predictionValue > movementPredThres):
    #   adjustMovement(4, 0)
    # else:
    #   adjustMovement(0, 4)
    #print("setting movement...")
    adjust_movement(.2 if moveForward else 0, 0 if moveForward else .2)

def main():
    rospy.init_node("cv_low_level")
    # may have to replace while(true) with some rate limit
    global roadPercentage
    global movementPredThres
    sub = rospy.Subscriber("/cv_nav/road_percentage", Float64, getPredVal)
    r = rospy.Rate(40)
    #moveForwardOld = roadPercentage > movementPredThres
    #velocity_control(True)
    while(True):
        # print(f"in while pred value: {roadPercentage}")
        #moveForwardNew = roadPercentage > movementPredThres
        #if (moveForwardOld != moveForwardNew):
        moveForward = roadPercentage > movementPredThres
        velocity_control(moveForward)
        #moveForwardOld = moveForwardNew

if __name__ == "__main__":
    main()
