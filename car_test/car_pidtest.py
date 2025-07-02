#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
"""
增强版区域导航控制器（坐标系转换修复版）
功能：
1. 正确的世界坐标系到本体系转换
2. 支持全局角度和相对角度转换
3. 动态避障集成
4. 状态机安全控制
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

class EnhancedNavigation:
    def __init__(self):
        rospy.init_node('enhanced_navigation')

        # ========== 地图参数 ==========
        self.world_size = 3.6    # 地图尺寸
        self.zone_size = 1.8     # 子区域尺寸
        self.center_point = Point(1.8, -1.8, 0)  # 地图中心

        # ========== 导航点序列 ==========
        self.waypoints = [
            {'pos': Point(1.0, -1.0, 0), 'yaw': math.radians(0)},
            {'pos': Point(0.57, -1.0, 0), 'yaw': math.radians(90)},
            {'pos': Point(2.67, -1.0, 0), 'yaw': math.radians(-90)},
            {'pos': Point(3.2, -1.0, 0), 'yaw': math.radians(0)},
            {'pos': Point(2.67, -2.6, 0), 'yaw': math.radians(0)},
            {'pos': Point(2.32, -2.6, 0), 'yaw': math.radians(90)},
            {'pos': Point(1.50, -2.30, 0), 'yaw': math.radians(135)},
            {'pos': Point(1.60, -3.40, 0), 'yaw': math.radians(90)}
        ]
        self.current_waypoint_index = 0

        # ========== 控制参数 ==========
        self.kp_linear = 0.6      # 线速度比例系数
        self.kp_angular = 1.2     # 角速度比例系数
        self.max_linear = 0.5     # 最大线速度(m/s)
        self.max_angular = 1.0    # 最大角速度(rad/s)
        self.pos_tolerance = 0.1  # 位置容差(m)
        self.yaw_tolerance = math.radians(2)  # 角度容差(rad)
        self.safe_distance = 0.5  # 安全避障距离(m)

        # ========== 系统状态 ==========
        self.current_pose = Point()
        self.current_yaw = 0.0
        self.scan_data = None
        self.state = "PLANNING"   # 状态机状态

        # ========== ROS通信 ==========
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def odom_cb(self, msg):
        """处理里程计数据"""
        self.current_pose = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([
            orientation.x, orientation.y,
            orientation.z, orientation.w
        ])

    def scan_cb(self, msg):
        """处理激光雷达数据"""
        self.scan_data = msg

    def get_current_zone(self, point):
        """计算坐标所属区域索引"""
        x = point.x
        y = point.y
        if x < self.zone_size:
            return 0 if y < self.zone_size else 3
        else:
            return 1 if y < self.zone_size else 2

    def calculate_avoidance(self):
        """避障控制量计算"""
        if not self.scan_data:
            return 0, 0
        
        ranges = np.array(self.scan_data.ranges)
        ranges[np.isinf(ranges)] = 10  # 处理无效值
        
        # 分析前方180度区域
        front_ranges = np.concatenate((ranges[-30:], ranges[:30]))
        min_front = np.min(front_ranges)
        
        avoid_x = 0.0
        avoid_y = 0.0
        
        if min_front < self.safe_distance:
            # 获取左右侧最小距离
            left_min = np.min(ranges[60:120])
            right_min = np.min(ranges[240:300])
            
            # 选择障碍物较少的一侧
            if left_min > right_min:
                avoid_y = -0.3  # 向右横向移动
            else:
                avoid_y = 0.3   # 向左横向移动
            avoid_x = -0.2      # 减速
        
        return avoid_x, avoid_y

    def world_to_body_frame(self, target_point):
        """世界坐标系到本体系转换"""
        dx = target_point.x - self.current_pose.x
        dy = target_point.y - self.current_pose.y
        
        # 旋转矩阵计算
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        
        # 坐标变换
        body_x = dx * cos_yaw + dy * sin_yaw
        body_y = -dx * sin_yaw + dy * cos_yaw
        
        return body_x, body_y

    def move_to_target(self, target_pos):
        """移动到目标位置（基于本体系）"""
        twist = Twist()
        
        # 坐标系转换
        body_x, body_y = self.world_to_body_frame(target_pos)
        distance = math.hypot(body_x, body_y)

        rospy.loginfo_throttle(1,
            "\n[位置控制]\n当前坐标: (%.2f, %.2f)\n目标坐标: (%.2f, %.2f)\n直线误差: %.2fm",
            self.current_pose.x, self.current_pose.y,
            target_pos.x, target_pos.y,
            distance
        )
        
        if distance < self.pos_tolerance:
            return None  # 到达目标位置
        
        # 本体系下的速度计算
        linear_x = self.kp_linear * body_x
        linear_y = self.kp_linear * body_y
        
        # 计算目标全局角度误差
        global_target_angle = math.atan2(
            target_pos.y - self.current_pose.y,
            target_pos.x - self.current_pose.x
        )
        angle_error = global_target_angle - self.current_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        # 角速度控制
        angular_z = self.kp_angular * angle_error
        
        # 避障调整
        avoid_x, avoid_y = self.calculate_avoidance()
        linear_x += avoid_x
        linear_y += avoid_y
        
        # 速度限幅
        twist.linear.x = np.clip(linear_x, -self.max_linear, self.max_linear)
        twist.linear.y = np.clip(linear_y, -self.max_linear, self.max_linear)
        twist.angular.z = np.clip(angular_z, -self.max_angular, self.max_angular)
        
        return twist

    def adjust_heading(self, target_yaw):
        """调整到目标全局朝向"""
        twist = Twist()
        error = target_yaw - self.current_yaw
        error = math.atan2(math.sin(error), math.cos(error))

        # 打印角度信息（每秒2次）
        rospy.loginfo_throttle(0.5,
            "\n[角度调整]\n当前朝向: %.1f°\n目标朝向: %.1f°\n角度误差: %.1f°",
            math.degrees(self.current_yaw),
            math.degrees(target_yaw),
            math.degrees(error)
        )
        
        if abs(error) < self.yaw_tolerance:
            return None  # 角度调整完成
        
        angular_z = self.kp_angular * error
        twist.angular.z = np.clip(angular_z, -self.max_angular, self.max_angular)
        return twist

    def run(self):
        """主控制循环"""
        while not rospy.is_shutdown():
            cmd = None  # 显式初始化cmd变量
            if self.current_waypoint_index >= len(self.waypoints):
                rospy.loginfo("所有导航点已完成!")
                self.cmd_pub.publish(Twist())
                self.rate.sleep()
                continue

            current_waypoint = self.waypoints[self.current_waypoint_index]
            target_pos = current_waypoint['pos']
            target_yaw = current_waypoint['yaw']
            
            # 获取区域信息
            current_zone = self.get_current_zone(self.current_pose)
            target_zone = self.get_current_zone(target_pos)

            rospy.loginfo_throttle(1,
                "\n[全局状态]\n世界坐标: (%.2f, %.2f)\n当前朝向: %.1f°\n当前状态: %s",
                self.current_pose.x,
                self.current_pose.y,
                math.degrees(self.current_yaw),
                self.state
            )
            
            # 状态机逻辑
            if self.state == "PLANNING":
                if current_zone == target_zone:
                    self.state = "MOVE_DIRECT"
                    rospy.loginfo("同区域，直行至目标")
                    cmd = Twist()
                else:
                    self.state = "MOVE_TO_CENTER"
                    rospy.loginfo("跨区域，先前往中心点")
                    cmd = Twist()

            elif self.state == "MOVE_TO_CENTER":
                cmd = self.move_to_target(self.center_point)
                if not cmd:  # 到达中心点
                    self.state = "MOVE_TO_TARGET"
                    rospy.loginfo("已到达中心点，前往目标")

            elif self.state == "MOVE_DIRECT":
                cmd = self.move_to_target(target_pos)
                if not cmd:  # 到达目标位置
                    self.state = "ADJUST_HEADING"
                    rospy.loginfo("位置到达，开始调整朝向")

            elif self.state == "MOVE_TO_TARGET":
                cmd = self.move_to_target(target_pos)
                if not cmd:  # 到达目标位置
                    self.state = "ADJUST_HEADING"
                    rospy.loginfo("位置到达，开始调整朝向")

            elif self.state == "ADJUST_HEADING":
                cmd = self.adjust_heading(target_yaw)
                if not cmd:  # 完成角度调整
                    self.current_waypoint_index += 1
                    self.state = "PLANNING"
                    rospy.loginfo("朝向调整完成，前往下个航点")

            else:  # 异常状态处理
                rospy.logwarn("未知状态：%s，重置为PLANNING", self.state)
                self.state = "PLANNING"
                cmd = Twist()

            # 发布控制指令
            if cmd is not None:
                self.cmd_pub.publish(cmd)
            else:
                self.cmd_pub.publish(Twist())
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = EnhancedNavigation()
        controller.run()
    except rospy.ROSInterruptException:
        pass