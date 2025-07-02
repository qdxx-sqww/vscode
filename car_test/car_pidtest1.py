#!/usr/bin/env python
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf.transformations import euler_from_quaternion

# 显式继承 object 启用新式类（Python 2 推荐）
class NavigationControl(object):
    def __init__(self):
        rospy.init_node('navigation_control')
        
        # PID参数
        self.kp_linear = 0.5
        self.ki_linear = 0.0
        self.kd_linear = 0.1
        self.kp_angular = 1.0
        self.ki_angular = 0.0
        self.kd_angular = 0.2
        
        # 初始化PID控制器（前、后、左、右、角度）
        self.pid_front = PID(self.kp_linear, self.ki_linear, self.kd_linear)
        self.pid_back = PID(self.kp_linear, self.ki_linear, self.kd_linear)
        self.pid_left = PID(self.kp_linear, self.ki_linear, self.kd_linear)
        self.pid_right = PID(self.kp_linear, self.ki_linear, self.kd_linear)
        self.pid_angle = PID(self.kp_angular, self.ki_angular, self.kd_angular)
        
        # 目标参数
        self.target_position = Point(0.8, 0.8, 0.0)
        self.target_yaw = math.radians(180)  # 初始目标方向180度
        self.final_yaw = math.radians(90)    # 第二阶段目标方向90度
        self.state = "MOVE_TO_POSITION"
        self.state_timer = None
        
        # 当前状态
        self.current_pose = Point()
        self.current_yaw = 0.0
        self.scan_distances = {"front": 0, "back": 0, "left": 0, "right": 0}
        
        # 订阅者
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_cb)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        
        # 发布者
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.rate = rospy.Rate(10)  # 10Hz

    def odom_cb(self, msg):
        # 获取当前位姿
        self.current_pose = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_yaw = yaw

    def imu_cb(self, msg):
        # 使用IMU数据校正方向
        orientation = msg.orientation
        _, _, yaw_imu = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_yaw = yaw_imu  # 优先使用IMU的yaw

    def scan_cb(self, msg):
        # 处理雷达数据获取四向距离
        num_readings = len(msg.ranges)
        self.scan_distances = {
            "front": min(msg.ranges[num_readings//4 : 3*num_readings//4]),
            "back": min(msg.ranges[:num_readings//4] + msg.ranges[3*num_readings//4:]),
            "left": min(msg.ranges[num_readings//2 : 3*num_readings//4]),
            "right": min(msg.ranges[num_readings//4 : num_readings//2])
        }
        # 新增：打印前后左右距离（保留2位小数）
        rospy.loginfo(
            "前后左右距离: 前=%.2f, 后=%.2f, 左=%.2f, 右=%.2f",
            self.scan_distances["front"],
            self.scan_distances["back"],
            self.scan_distances["left"],
            self.scan_distances["right"]
        )

    def calculate_errors(self):
        # 计算世界坐标系下的位置误差
        dx = self.target_position.x - self.current_pose.x
        dy = self.target_position.y - self.current_pose.y
        
        # 转换到本体系
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        error_front = dx * cos_yaw + dy * sin_yaw
        error_left = -dx * sin_yaw + dy * cos_yaw
        
        # 方向误差
        error_yaw = self.target_yaw - self.current_yaw
        error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))  # 归一化到[-π, π]
        
        return error_front, error_left, error_yaw

    def control_loop(self):
        while not rospy.is_shutdown():
            twist = Twist()
            
            if self.state == "MOVE_TO_POSITION":
                ef, el, ey = self.calculate_errors()
                
                # 位置控制（原有逻辑）
                if ef > 0:
                    vx = self.pid_front.update(ef)
                else:
                    vx = -self.pid_back.update(-ef)
                
                if el > 0:
                    vy = self.pid_left.update(el)
                else:
                    vy = -self.pid_right.update(-el)
                
                # 新增：速度饱和限制（关键修改）
                vx = max(min(vx, self.max_linear_speed), -self.max_linear_speed)
                vy = max(min(vy, self.max_linear_speed), -self.max_linear_speed)
                
                wz = self.pid_angle.update(ey)
                twist.linear.x = vx
                twist.linear.y = vy
                twist.angular.z = wz
                
                # 调整：增大误差阈值（可选，原0.05→0.08）
                if abs(ef) < 0.08 and abs(el) < 0.08 and abs(ey) < 0.05:
                    self.state = "WAIT"
                    self.state_timer = rospy.Time.now() + rospy.Duration(2)
                    self.pid_front.reset()
                    self.pid_back.reset()
                    self.pid_left.reset()
                    self.pid_right.reset()
                    rospy.loginfo("Position reached. Waiting 2 seconds...")
            
            elif self.state == "WAIT":
                if rospy.Time.now() > self.state_timer:
                    self.state = "ADJUST_HEADING"
                    self.target_yaw = self.final_yaw
                    rospy.loginfo("Adjusting heading to 90 degrees...")
            
            elif self.state == "ADJUST_HEADING":
                ey = self.final_yaw - self.current_yaw
                ey = math.atan2(math.sin(ey), math.cos(ey))
                wz = self.pid_angle.update(ey)
                twist.angular.z = wz
                
                if abs(ey) < 0.02:
                    self.state = "DONE"
                    rospy.loginfo("Heading adjusted. Mission complete.")
            
            elif self.state == "DONE":
                twist = Twist()  # 停止
            
            self.cmd_pub.publish(twist)
            self.rate.sleep()

# 显式继承 object 启用新式类
class PID(object):
    def __init__(self, kp, ki, kd, integral_limit=0.5):
        # 建议：降低线性运动的微分项（原kd_linear=0.1→0.05）
        # 示例：若初始化时传入的kd_linear是0.1，可改为0.05（需在NavigationControl的__init__中调整）
        self.kp = kp
        self.ki = ki
        self.kd = kd  # 实际值由NavigationControl初始化时传入，需同步调整
        self.integral = 0
        self.prev_error = 0
        self.integral_limit = integral_limit

    def update(self, error, dt=0.1):
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

    def reset(self):
        self.integral = 0
        self.prev_error = 0

if __name__ == '__main__':
    try:
        controller = NavigationControl()
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass