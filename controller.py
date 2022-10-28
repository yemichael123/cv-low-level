import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from std_msgs.msg import Empty
from time import time

x = 0.0
y = 0.0
theta = 0.0

def resetOdom():
    rospy.init_node('reset_odom')

    reset_odom = rospy.Publisher('/odometry/filtered', Empty, queue_size=10)

    timer = time()
    while time() - timer < 0.25:
        reset_odom.publish(Empty())

    
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
    rospy.init_node("speed_controller")

    sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
    pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size = 1)

    speed = Twist()

    r = rospy.Rate(4)

    goal = Point()
    goal.x = 0
    goal.y = 5

    while not rospy.is_shutdown():
        inc_x = goal.x - x
        inc_y = goal.y - y

        angle_to_goal = atan2(inc_y, inc_x)

        if angle_to_goal - theta > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
        elif angle_to_goal - theta < -0.1:
            speed.linear.x = 0.0
            speed.angular.z = -0.3
        else:
            speed.linear.x = 0.5
            speed.angular.z = 0.0

        if (abs(inc_x - x) < .5) and (abs(inc_y - y) < .5):
            speed.linear.x = 0
            speed.angular.z = 0

        pub.publish(speed)
        r.sleep()

def main():
    reset_odom()
    drive()

if __name__ == "__main__":
    main()