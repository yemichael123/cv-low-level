import rospy
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, PoseStamped
from math import atan2
from std_msgs.msg import Empty
from time import time

x = 0.0
y = 0.0
theta = 0.0

coordinates = [[0,3], [1, 2], [0,0]]

resetSub = None


path = Path()

def odom_cb(data):
    global path
    path_pub = rospy.Publisher('/path', Path, queue_size=10)
    path.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    path.poses.append(pose)
    path_pub.publish(path)

def newCoordinate():
    global coordinates
    x, y = coordinates[0][0], coordinates[0][1]
    if (len(coordinates) != 1):
        coordinates = coordinates[1:]
    return x, y

def resetOdom(msg):
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
    # print("in odom")
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def drive():
    # rospy.init_node("speed_controller")

    rospy.init_node("speed_controller")

    # resetOdom("df")
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
        
        print("x: " + str(goal.x))
        print("y: " + str(goal.y))
        angle_to_goal = atan2(inc_y, inc_x)

        if angle_to_goal - theta > 0.01:
            print("turning CW")
            speed.linear.x = 0.0
            speed.angular.z = 0.3
        elif angle_to_goal - theta < -0.01:
            print("turning CCW")
            speed.linear.x = 0.0
            speed.angular.z = -0.3
        else:
            print("moving forward")
            speed.linear.x = 0.5
            speed.angular.z = 0.0

        if (abs(inc_x) < .1) and (abs(inc_y) < .1):
            print("stopping...")
            speed.linear.x = 0
            speed.angular.z = 0
            goal.x, goal.y = newCoordinate()

        pub.publish(speed)


def main():
    drive()

if __name__ == "__main__":
    main()
