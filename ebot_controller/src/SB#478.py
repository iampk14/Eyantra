#!/usr/bin/env python

"""This code will move the ebot on the given path, then avoid the
obstacle and goto the final position"""

import math
import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

PI = 3.1415926535897932
x = 0.0
y = 0.0
theta = 0.0
speed = Twist()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
global rate
regions = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left':   0,
    }


def go_to_angle(user_theta):
    """This will make the bot face the desired angle"""
    global rate
    theta_new = user_theta - theta
    if theta_new > 0:
        # Left
        while abs(user_theta - theta) > 0.05:
            speed.linear.x = 0
            speed.angular.z = 0.4
            pub.publish(speed)
            rate.sleep()
    else:
        # Take a Right
        while abs(user_theta - theta) > 0.05:
            speed.linear.x = 0
            speed.angular.z = - 0.4
            pub.publish(speed)
            rate.sleep()
    speed.linear.x = 0
    speed.angular.z = 0
    pub.publish(speed)


def obs_avoid(z):
    """This is used for obstacle avoidance"""
    while regions['front'] < 8:
        speed.linear.x = 0
        speed.angular.z = z
        pub.publish(speed)
        rate.sleep()
    speed.linear.x = 1.3
    speed.angular.z = 0
    pub.publish(speed)
    rate.sleep()
    rospy.sleep(2)


def go_to_x_y(user_x, user_y, deltaball=1.0):
    """This will move the bot to the desired location"""
    global regions
    goal = Point()
    goal.x = user_x
    goal.y = user_y
    inc_x = user_x - x
    inc_y = user_y - y
    th = math.atan2(inc_y, inc_x)
    if th > 3.14:
        th = -(6.28 - th)
    elif th < -3.14:
        th = 6.28 + th
    go_to_angle(th)
    while math.sqrt(((user_x - x) * (user_x - x)) + ((user_y - y) * (user_y - y))) > deltaball:
        if regions['front'] > 2:
            if math.sqrt(((goal.x - x) * (goal.x - x)) + ((goal.y - y) * (goal.y - y))) > deltaball:
                speed.linear.x = 0.8
                speed.angular.z = 0
                pub.publish(speed)
                rate.sleep()
        else:
            speed.linear.x = 0
            speed.angular.z = 0
            pub.publish(speed)
            rate.sleep()
            # Let's find possible path
            if regions['right'] > 3:
                # Right path available
                obs_avoid(-0.5)
                inc_x = goal.x - x
                inc_y = goal.y - y
                th = math.atan2(inc_y, inc_x)
                go_to_angle(th)
            elif regions['left'] > 3:
                # front left path available
                obs_avoid(0.5)
                inc_x = goal.x - x
                inc_y = goal.y - y
                th = math.atan2(inc_y, inc_x)
                go_to_angle(th)
            elif regions['fright'] > 3:
                # Right path available
                obs_avoid(-0.5)
                inc_x = goal.x - x
                inc_y = goal.y - y
                th = math.atan2(inc_y, inc_x)
                go_to_angle(th)
            elif regions['fleft'] > 3:
                # front left path available
                obs_avoid(0.5)
                inc_x = goal.x - x
                inc_y = goal.y - y
                th = math.atan2(inc_y, inc_x)
                go_to_angle(th)
            else:
                obs_avoid(-0.5)
                inc_x = goal.x - x
                inc_y = goal.y - y
                th = math.atan2(inc_y, inc_x)
                go_to_angle(th)
        rate.sleep()
    speed.linear.x = 0
    speed.angular.z = 0
    pub.publish(speed)


def waypoints(t):
    """This will provide the waypoints for the bot to track the path"""
    global x
    xx = x + ((2 * PI)/t)
    yy = 2*(math.sin(xx))*(math.sin(xx/2))
    return [xx, yy]


def laser_callback(msg):
    """This will set the range for the obstacle avoidance"""
    global regions
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }


def control_loop():
    """This is the function where all the subscribers and publishers are assigned
    and this is where the bot moves to the final goal"""
    global rate, regions, speed
    rospy.init_node('ebot_controller')
    rate = rospy.Rate(10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    speed = Twist()
    speed.linear.x = 0
    speed.angular.z = 0
    pub.publish(speed)
    while not rospy.is_shutdown():
        if x <= 6.28:
            user_x, user_y = waypoints(15)
            go_to_x_y(user_x, user_y, 1)
            go_to_x_y(user_x, user_y, 0.25)
        else:
            go_to_x_y(12.5, 0, 3)
            go_to_x_y(12.5, 0, 1)
            go_to_x_y(12.5, 0, 0.2)
            exit()
        speed.linear.x = 0
        speed.angular.z = 0
        pub.publish(speed)
        rate.sleep()


def odom_callback(data):
    """This will provide the location of the bot"""
    global x
    global y
    global theta
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    rot_q = data.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
