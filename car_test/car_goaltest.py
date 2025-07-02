#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import math
import tf

# 目标点坐标
goal_x = 1.8
goal_y = 1.8
tolerance_pos = 0.05  # 5cm容忍范围

# 当前状态变量
current_x = 0.1
current_y = 0.1
current_theta = 0.0  # 弧度

def odom_callback(msg):
    global current_x, current_y, current_theta

    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y

    orientation = msg.pose.pose.orientation
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    current_theta = euler[2]  # yaw角，弧度

def move_to_goal():
    global current_x, current_y, current_theta

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        dx = goal_x - current_x
        dy = goal_y - current_y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance <= tolerance_pos:
            rospy.loginfo("Goal reached!")
            stop_cmd = Twist()
            pub.publish(stop_cmd)
            break

        goal_angle = math.atan2(dy, dx)
        angle_diff = goal_angle - current_theta

        # 将角度归一化到 [-pi, pi]
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        cmd = Twist()

        if abs(angle_diff) > 0.1:
            cmd.angular.z = 0.5 if angle_diff > 0 else -0.5
            cmd.linear.x = 0.0
        else:
            cmd.angular.z = 0.0
            cmd.linear.x = min(0.3, distance)

        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('move_to_goal_node')
    rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, odom_callback)
    rospy.sleep(1)  # 等待接收初始位姿

    try:
        move_to_goal()
    except rospy.ROSInterruptException:
        pass