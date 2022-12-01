import rospy
import numpy as np
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, PoseStamped
from math import atan2
from std_msgs.msg import Empty
from time import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal

import actionlib

x = 0.0
y = 0.0
theta = 0.0


resetSub = None

path = Path()

# Set to true if running coordinates from Astar
runningAStar = False

coordinates = [[0,3], [3, 3], [3,0], [0, 0]]

if runningAStar:
    # open file containing coordinates
    f = open('coordinates.txt', 'r')
    textStr = f.read()
    coordinates = []
    for coordStr in textStr.splitlines():
        x, y = coordStr.split()
        x, y = (x - 30) / 10, (y - 60) / 10
        coordinates.append([y, x])



def generateGraphCoordinates(graph):
    """
    Generates graph coordinates for testing
    """
    global coordinates
    coordinates = []

    if (graph == "sine"):
        for i in range(30):
            coordinates.append([i, np.sin(i)])
    elif (graph == "tan"):
        for i in range(60):
            coordinates.append([i / 2, np.tan(i)])
    elif (graph == "sine incr"):
        for i in range(200):
            coordinates.append([i / 4, np.sin(i) * i / 2])


def odom_cb(data):
    """
    Writes odometry position data to /path topic for rviz visualization
    """
    global path
    path_pub = rospy.Publisher('/path', Path, queue_size=10)
    path.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    path.poses.append(pose)
    path_pub.publish(path)

def newCoordinate():
    """
    Returns a set of new waypoint coordinates
    """
    global coordinates
    global x
    global y
    goalX, goalY = coordinates[0][0], coordinates[0][1]
    if (len(coordinates) != 1):
        coordinates = coordinates[1:]
    if runningAStar:
        else if (((x - goalX)**2 + (y - goalY)**2)**(1/2)) < 1:
            return newCoordinate()
    return goalX, goalY

def resetOdom(msg):
    """
    Resets odometry position to [0, 0 ,0] and orientation to [0, 0, 0, 1]

    Currently not working at intended because odometry data gets overwritten after publishing
    """
    # global newSub
    reset_odom = rospy.Publisher('/jackal_velocity_controller/odom', Odometry, queue_size=10)
    # reset_odom = rospy.Publisher('/jackal_velocity_controller/odom', Empty, queue_size=10)

    # msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = 0, 0, 0
    # msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = 0, 0, 0, 0
    
    odom = Odometry()
    odom.pose.pose.position.x = 0
    odom.pose.pose.position.y = 0
    odom.pose.pose.position.z = 0
    odom.pose.pose.orientation.x = 0
    odom.pose.pose.orientation.y = 0
    odom.pose.pose.orientation.z = 0
    odom.pose.pose.orientation.w = 0

    timer = time()
    while time() - timer < 0.25:
        print(str(timer) + " " + str(time())) 
        print("odometry/filtered as empty...")
        reset_odom.publish(odom)
    print("done")
    # newSub.unregister()
    
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


def drive():
    """
    Personal waypoint algorithm. This function is doo doo, do not use
    """
    
    rospy.init_node("speed_controller")

    # global resetSub
    # resetSub = rospy.Subscriber("/odometry/filtered", Odometry, resetOdom) 
    
    sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
    pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size = 1)
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odom_cb)

    speed = Twist()

    r = rospy.Rate(40)

    goal = Point()
    goal.x, goal.y = newCoordinate()

    

    while not rospy.is_shutdown():
        inc_x = goal.x - x
        inc_y = goal.y - y
        
        print("goal x: " + str(goal.x))
        print("goal y: " + str(goal.y))
        print("x: " + str(x))
        print("y: " + str(y)) 

        angle_to_goal = atan2(inc_y, inc_x)
        angle_to_goal = min(angle_to_goal, 3.14 - angle_to_goal)
        
        print("angle_to_goal: " + str(angle_to_goal))
        print("theta: " + str(theta))

        if angle_to_goal - theta > 0.1:
            print("turning CW")
            speed.linear.x = 0.0
            speed.angular.z = 0.2
        elif angle_to_goal - theta < -0.1:
            print("turning CCW")
            speed.linear.x = 0.0
            speed.angular.z = -0.2
        else:
            print("moving forward")
            speed.linear.x = 9
            speed.angular.z = 0.0

        if (abs(inc_x) < .1) and (abs(inc_y) < .1):
            print("stopping...")
            speed.linear.x = 0
            speed.angular.z = 0
            goal.x, goal.y = newCoordinate()
        
        
        pub.publish(speed)
        r.sleep()


def move_base_drive():
    """
    Waypoint navigation with move_base
    """

    rospy.init_node("speed_controller")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    print("connecting to server...")
    client.wait_for_server()
    print("server connected")
    
    sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odom_cb)

    while (True):
        goalx, goaly = newCoordinate()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goalx
        goal.target_pose.pose.position.y = goaly
        goal.target_pose.pose.orientation.w = 1.0


        # poseSt = PoseStamped()
        # poseSt.header.stamp = rospy.Time.now()
        # poseSt.header.frame_id = "odom"
        # poseSt.pose.position.x = 5
        # poseSt.pose.position.y = 3
        # poseSt.pose.orientation.w = 1
        # pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 10)
        # pub.publish(poseSt)

        print("setting new waypoint to " + "(" + str(goalx) + ", " + str(goaly) + ")")
        client.send_goal(goal)

        while (abs(goalx - x) > .3 or abs(goaly - y) > .3):
            print("goal x: " + str(goalx))
            print("goal y: " + str(goaly))
            print("x: " + str(x))
            print("y: " + str(y))
            continue

    '''
    wait = client.wait_for_result()
    print("wait done")
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result() 
    '''


def main():
    # generateGraphCoordinates("sine incr")
    # drive()
    move_base_drive()

if __name__ == "__main__":
    main()
