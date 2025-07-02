#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
from __future__ import print_function  # 放在文件最顶部（紧跟在 #!/usr/bin/env python2.7 后面）

import rosp   y
import math
import os
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from playsound import playsound
import sys
from std_msgs.msg import Int32, String  # 新增：导入String类型
import subprocess
from rosgraph_msgs.msg import Log  # 输出日志
import logging

# 让 rospy 日志输出到标准输出（兼容 Python2）
logging.basicConfig(level=logging.INFO)
rospy.loginfo = lambda msg, *args: print("[INFO]", msg % args if args else msg)
rospy.logwarn = lambda msg, *args: print("[WARN]", msg % args if args else msg)
rospy.logerr = lambda msg, *args: print("[ERROR]", msg % args if args else msg)
# 你可以根据需要添加 logdebug 等

def normalize_angle(angle):
    """将任意角度归一化到[-π, π]范围内"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def convert_chinese_time(time_str):
    """将中文时间点转换为阿拉伯数字"""
    conversion_map = {
        "十一点": 11,  # 直接返回整数
        "十二点": 12,
        "十三点": 13
    }
    return conversion_map.get(time_str, 11)  # 默认返回11

class NavigationController(object):
    def __init__(self, end_key=11):  # 新增end_key参数
        self.end_key = end_key  # 存储最终导航点键
        self.target_key = None
        self.re_key = None
        self.audio_lock = False
        self.audio_thread = None
        self.last_detection = None
        self.detect_proxy = None  # 新增服务代理
        self.service_timeout = 30  # 服务超时时间
        self.param_update_time = rospy.Time.now()
        # 新增：存储上一次有效雷达值的变量
        self.last_lidar_distances = {
            'front': None,
            'left': None,
            'rear': None,
            'right': None
        }
        
        # 新增：存储当前速度的变量
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        
        # 传感器数据存储
        self.lidar_distances = {'front': 0.0, 'left': 0.0, 'rear': 0.0, 'right': 0.0}
        self.current_yaw = normalize_angle(-math.pi)  # 初始朝向南方（归一化）
        self.raw_yaw = 0.0
        
        # 导航状态参数
        self.current_task_stage = 0
        self.target_lidar = {'front': 0.87, 'left': 0.82}  # 初始目标
        self.target_yaw = normalize_angle(-math.pi)  # 初始保持南方朝向
        self.waiting = False
        
        # 雷达角度定义
        self.lidar_angles = {
            'front': math.radians(0),
            'left': math.radians(90),
            'rear': math.radians(180),
            'right': math.radians(-90)
        }
        
        # 完整点位定义
        self.point_targets = {
            1: {'front': 1.0, 'left': 0.53},#1西
            2: {'front': 1.0, 'left': 0.88},#2西
            3: {'front': 0.98, 'left': 1.23},#3西
            4: {'front': 0.98, 'right': 1.27},#4西
            5: {'front': 0.98, 'right': 0.92},#5西
            6: {'front': 0.98, 'right': 0.56},#6西
            7: {'front': 1.0, 'left': 0.51},#7东
            8: {'front': 0.97, 'left': 0.86},#8东
            9: {'front': 1.0, 'left': 1.21},#9东
            11: {'rear': 0.26, 'left': 1.63},#西
            12: {'rear': 1.56, 'left': 0.22},#西
            13: {'rear': 0.26, 'left': 0.23},#西
            14: {'front': 1.53, 'left': 1.52},#123转折点西
            15: {'rear': 1.53, 'left': 1.53},  # 456转折点东
            16: {'rear': 1.53, 'right': 1.53},  # 789转折点西
            17: {'front': 1.53, 'left': 1.52},  # 789转折点东
            18: {'rear': 1.0, 'left': 0.90},  # 456拍照点东
            19: {'rear': 1.0, 'right': 0.86},  # 789拍照点西
            20: {'front': 1.53, 'right': 1.54},  # 456转折点西
            21: {'rear': 1.53, 'left': 1.53},  # end转折点西
        }
        
        # 订阅与发布初始化
        rospy.Subscriber('/scan_filtered', LaserScan, self.lidar_callback)
        rospy.Subscriber('/result', Int32, self.result_callback)
        rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 新增：订阅/photo_recognition话题
        self.photo_key = None
        self.photo_key_seq = 0
        self.photo_key_seq_used = 0
        self.waiting_for_photo = False
        rospy.Subscriber('/photo_recognition', String, self.photo_recognition_callback)

        rospy.on_shutdown(self.shutdown_handler)
        self.main_loop()

    def chinese_top_callback(self, msg):
        """处理/chinese_top话题消息"""
        self.chinese_top_value = msg.data
        rospy.loginfo("接收到/chinese_top话题数据：%s", self.chinese_top_value)

    def lidar_callback(self, msg):
        """处理雷达数据"""
        angle_inc = msg.angle_increment
        skip_position = not self.target_lidar
        current_used_dirs = list(self.target_lidar.keys()) if not skip_position else []

        for dir, angle in self.lidar_angles.items():
            idx = int((angle - msg.angle_min) / angle_inc)
            raw_distance = msg.ranges[min(max(idx, 0), len(msg.ranges)-1)]
            
            if skip_position:
                filtered_distance = raw_distance
                self.last_lidar_distances[dir] = filtered_distance
            else:
                if dir in current_used_dirs:
                    if self.last_lidar_distances[dir] is None:
                        filtered_distance = raw_distance
                        self.last_lidar_distances[dir] = filtered_distance
                    else:
                        distance_diff = abs(raw_distance - self.last_lidar_distances[dir])
                        if distance_diff > 1.0:
                            filtered_distance = self.last_lidar_distances[dir]
                        else:
                            filtered_distance = raw_distance
                            self.last_lidar_distances[dir] = filtered_distance
                else:
                    filtered_distance = raw_distance
                    self.last_lidar_distances[dir] = filtered_distance
            
            self.lidar_distances[dir] = filtered_distance

    def odom_callback(self, msg):
        """处理融合定位数据"""
        quat = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w], axes='sxyz')
        
        self.raw_yaw = yaw
        self.current_yaw = normalize_angle(yaw + math.pi)

    def pid_control(self):
        """麦克纳姆轮全向PID控制"""
        skip_position = not self.target_lidar
        pid_params = {
            'linear_x': {'Kp': 1.5, 'Ki': 0.02, 'Kd': 0.15, 'imax': 0.3},
            'linear_y': {'Kp': 1.5, 'Ki': 0.02, 'Kd': 0.15, 'imax': 0.3},
            'angular': {'Kp': 4.0, 'Ki': 0.01, 'Kd': 0.3, 'imax': 0.5}
        }
        
        if not hasattr(self, 'e_front_prev'):
            self.e_front_prev = 0.0
            self.e_left_prev = 0.0
            self.e_angle_prev = 0.0
            self.i_front = 0.0
            self.i_left = 0.0
            self.i_angle = 0.0
        
        angle_err = normalize_angle(self.target_yaw - self.current_yaw)
        dt = 0.1

        linear_x = 0.0
        if not skip_position:
            e_front = self.lidar_distances['front'] - self.target_lidar.get('front', 0)
            e_rear  = self.target_lidar.get('rear', 0) - self.lidar_distances['rear']
            e_total = e_front if 'front' in self.target_lidar else e_rear
            
            self.i_front += e_total * dt
            self.i_front = max(-pid_params['linear_x']['imax'], min(self.i_front, pid_params['linear_x']['imax']))
            d_front = (e_total - self.e_front_prev) / dt
            linear_x = pid_params['linear_x']['Kp']*e_total + pid_params['linear_x']['Ki']*self.i_front + pid_params['linear_x']['Kd']*d_front
            self.e_front_prev = e_total
        else:
            linear_x = 0.0
            self.i_front = 0.0

        linear_y = 0.0
        if not skip_position:
            e_left = self.lidar_distances['left'] - self.target_lidar.get('left', 0)
            e_right = self.target_lidar.get('right', 0) - self.lidar_distances['right']
            e_total = e_left if 'left' in self.target_lidar else e_right
            
            self.i_left += e_total * dt
            self.i_left = max(-pid_params['linear_y']['imax'], min(self.i_left, pid_params['linear_y']['imax']))
            d_left = (e_total - self.e_left_prev) / dt
            linear_y = pid_params['linear_y']['Kp']*e_total + pid_params['linear_y']['Ki']*self.i_left + pid_params['linear_y']['Kd']*d_left
            self.e_left_prev = e_total
        else:
            linear_y = 0.0
            self.i_left = 0.0

        self.i_angle += angle_err * dt
        self.i_angle = max(-pid_params['angular']['imax'], min(self.i_angle, pid_params['angular']['imax']))
        d_angle = (angle_err - self.e_angle_prev) / dt
        angular_z = pid_params['angular']['Kp']*angle_err + pid_params['angular']['Ki']*self.i_angle + pid_params['angular']['Kd']*d_angle
        self.e_angle_prev = angle_err

        linear_x = max(-5.0, min(linear_x, 5.0))
        linear_y = max(-5.0, min(linear_y, 5.0))
        angular_z = max(-5.0, min(angular_z, 5.0))

        self.current_linear_x = linear_x
        self.current_linear_y = linear_y
        self.current_angular_z = angular_z

        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

   
    def is_arrived(self):
        """判断是否到达目标（位置+角度）"""
        dist_ok = True
        if self.target_lidar:
            dist_ok = all(
                abs(target - self.lidar_distances.get(dir, 0)) < 0.03
                for dir, target in self.target_lidar.items()
            )
        angle_ok = abs(normalize_angle(self.current_yaw - self.target_yaw)) < 0.174
        return dist_ok and angle_ok

    def is_position_arrived(self):
        """仅判断位置是否到达目标（忽略角度）"""
        if not self.target_lidar:
            return True
        return all(
            abs(target - self.lidar_distances.get(dir, 0)) < 0.03
            for dir, target in self.target_lidar.items()
        )

    def play_audio(self, target_key):
        """同步阻塞式音频播放"""
        if self.audio_lock:
            rospy.logwarn("音频播放冲突，当前已有音频在播放")
            return
            
        self.audio_lock = True
        try:
            audio_path = "/home/abot/cryd_wss/src/robot_slam/music/{0}.mp3".format(target_key)
            if not os.path.exists(audio_path):
                rospy.logerr("音频文件不存在：%s", audio_path)
                return
                
            rospy.loginfo("开始播放目标点%d音频：%s", target_key, audio_path)
            playsound(audio_path)  # 同步阻塞调用
            rospy.loginfo("目标点%d音频播放完成", target_key)
            
        except Exception as e:
            rospy.logerr("播放音频时出错：%s", str(e))
        finally:
            self.audio_lock = False


    def result_callback(self, msg):
        """处理/result话题消息"""
        self.target_key = msg.data
        rospy.loginfo("接收到/result话题数据，目标点键更新为：%d", self.target_key)


    # 在文件顶部添加
    
    
    # 在delayed_param_cb中重置检测状态
    def delayed_param_cb(self, event):
        """改进的延迟参数设置"""
        rospy.set_param("/top_view_shot_node/im_flag", 1)
        rospy.loginfo("已触发拍照，等待新检测值...")  
    def handle_stage(self):
        if self.waiting or self.audio_lock:
            return

        # 阶段0-12保持不变...
                # 阶段0（无音频）
        if self.current_task_stage == 0 and self.is_position_arrived():
            rospy.loginfo("到达123照相点")
            # 新增等待参数更新逻辑
            self.set_photo_flag()
            self.current_task_stage = 100
            return
        elif self.current_task_stage == 100:
            if self.wait_photo_result():
                return
            rospy.loginfo("123拍照识别1")
            self.photo_key = None
            self.current_task_stage = 1
            self.target_yaw = normalize_angle(math.pi/2)  # 转向西方（归一化）
            self.target_lidar = {}  # 清空位置目标（仅角度）
        
        elif self.current_task_stage == 1 and self.is_arrived():
            rospy.sleep(0.5)
           # rospy.Timer(rospy.Duration(2), self.delayed_param_cb, oneshot=True)
            rospy.loginfo("已转向西方")
            # 新增等待参数更新逻辑
            self.set_photo_flag()
            self.current_task_stage = 101
            return
        elif self.current_task_stage == 101:
            if self.wait_photo_result():
                return
            rospy.loginfo("123拍照识别2")
            self.photo_key = None
            rospy.sleep(3)
            self.current_task_stage = 2
            
            # 新增等待/result话题值的逻辑
            while self.target_key is None and not rospy.is_shutdown():
                rospy.loginfo("等待/result话题更新...")
                rospy.sleep(0.1)
            
            self.target_lidar = self.point_targets[self.target_key]
            self.target_yaw = normalize_angle(math.pi/2)
            

        elif self.current_task_stage == 2 and self.is_arrived():
            rospy.loginfo("已到达123导航点")
            self.play_audio(self.target_key)  # 播放目标点1的音频（对应point_targets[1]）
            self.target_key = None  # 新增：使用后立即重置
            self.current_task_stage = 3
            self.target_lidar = self.point_targets[14]  # 前往123转折点
            self.target_yaw = normalize_angle(math.pi/2)

        elif self.current_task_stage == 3 and self.is_position_arrived():
            
            self.current_task_stage = 4
            self.target_lidar = self.point_targets[16] # 前往789转折点西
            self.target_yaw = normalize_angle(math.pi/2)
        
        elif self.current_task_stage == 4 and self.is_position_arrived():
            rospy.loginfo("到达789转折点")
            self.current_task_stage = 5
            self.target_lidar = self.point_targets[19] # 前往789拍照点西
            self.target_yaw = normalize_angle(math.pi/2)

        elif self.current_task_stage == 5 and self.is_position_arrived():
            self.current_task_stage = 6
            self.target_yaw = normalize_angle(0)#转向北
            self.target_lidar = {}
    
        elif self.current_task_stage == 6 and self.is_arrived():
            rospy.loginfo("到达789拍照点")
            rospy.sleep(0.5)
            # 新增等待参数更新逻辑
            self.set_photo_flag()
            self.current_task_stage = 104
            return
        elif self.current_task_stage == 104:
            if self.wait_photo_result():
                return
            rospy.loginfo("789拍照识别1")
            self.photo_key = None
            self.current_task_stage = 7
            self.target_yaw = normalize_angle(-math.pi/2)  # 转向东方（归一化）
            self.target_lidar = {}  # 清空位置目标（仅角度）
        
        elif self.current_task_stage == 7 and self.is_arrived():
            rospy.sleep(0.5)
            self.set_photo_flag()
            self.current_task_stage = 105
            return
        elif self.current_task_stage == 105:
            if self.wait_photo_result():
                return
            rospy.loginfo("789拍照识别2")
            self.photo_key = None
            # 新增等待/result话题值的逻辑
            while self.target_key is None and not rospy.is_shutdown():
                rospy.loginfo("等待/result话题更新...")
                rospy.sleep(0.1)
            self.current_task_stage = 8           
            self.target_lidar = self.point_targets[self.target_key]
            self.target_yaw = normalize_angle(-math.pi/2)
        elif self.current_task_stage == 8 and self.is_arrived():
            rospy.loginfo("已到达789导航点")
            self.play_audio(self.target_key)
            rospy.sleep(0.1)
            self.target_key = None  # 新增：使用后立即重置
            self.current_task_stage = 9
            self.target_lidar = self.point_targets[17] # 前往789转折点东
            self.target_yaw = normalize_angle(-math.pi/2)

        elif self.current_task_stage == 9 and self.is_arrived():
            self.current_task_stage = 10
            self.target_lidar = self.point_targets[15] # 前往456转折点东
            self.target_yaw = normalize_angle(-math.pi/2)

        elif self.current_task_stage == 10 and self.is_arrived():
            self.current_task_stage = 11
            self.target_lidar = self.point_targets[18] # 前往456拍照点东
            self.target_yaw = normalize_angle(-math.pi/2)
        elif self.current_task_stage == 11 and self.is_arrived():
            self.current_task_stage = 12
            self.target_yaw = normalize_angle(0)#转向北
            self.target_lidar = {}
        elif self.current_task_stage == 12 and self.is_arrived():
            rospy.sleep(0.5)
            # 新增等待参数更新逻辑
            self.set_photo_flag()
            self.current_task_stage = 111
            return
        elif self.current_task_stage == 111:
            if self.wait_photo_result():
                return
            rospy.loginfo("456拍照识别1")
            self.photo_key = None
            self.current_task_stage = 13
            self.target_yaw = normalize_angle(math.pi/2)
            self.target_lidar = {}
        elif self.current_task_stage == 13 and self.is_arrived():
            rospy.sleep(0.5)
            rospy.loginfo("已转向西方")
            # 新增等待参数更新逻辑
            self.set_photo_flag()
            self.current_task_stage = 112
            return
        elif self.current_task_stage == 112:
            if self.wait_photo_result():
                return
            rospy.loginfo("456拍照识别2")
            self.photo_key = None
            self.current_task_stage = 14
            
            # 新增等待/result话题值的逻辑
            while self.target_key is None and not rospy.is_shutdown():
                rospy.loginfo("等待/result话题更新...")
                rospy.sleep(0.1)
            self.target_lidar = self.point_targets[self.target_key]
            self.target_yaw = normalize_angle(math.pi/2)
        elif self.current_task_stage == 14 and self.is_position_arrived():
           # rospy.loginfo("到达前往第456个区域的转折点，阶段3完成")
            rospy.loginfo("已到达456导航点")
            self.play_audio(self.target_key)
            rospy.sleep(0.1)
            self.current_task_stage = 15
            self.target_lidar = self.point_targets[20]  # 前往456转折点西

        elif self.current_task_stage == 15 and self.is_arrived():
            self.current_task_stage = 16
            self.target_lidar = self.point_targets[21]  # 前往end转折点西
            self.target_yaw = normalize_angle(math.pi/2)
        # 阶段13：使用/chinese_top话题的值作为导航点
        elif self.current_task_stage == 16 and self.is_arrived():
            rospy.loginfo("阶段12完成，进入最终导航阶段")
            self.current_task_stage = 17
            # 直接使用预存的end_key
            if self.end_key in self.point_targets:
                self.target_lidar = self.point_targets[self.end_key]
                self.target_yaw = normalize_angle(math.pi/2)
            else:
                rospy.logwarn("终点键%d不存在，使用默认11", self.end_key)
                self.target_lidar = self.point_targets[11]

        elif self.current_task_stage == 17 and self.is_arrived():
            rospy.loginfo("已到达终点%d，任务完成！", self.end_key)
            self.play_audio(self.end_key)  # 使用end_key播放音频
            self.cmd_vel_pub.publish(Twist())
            rospy.signal_shutdown("任务完成")

    def main_loop(self):
        """主控制循环"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pid_control()
            self.handle_stage()
            rate.sleep()

    def shutdown_handler(self):
        """关闭时停止小车"""
        self.cmd_vel_pub.publish(Twist())
        rospy.loginfo("程序已退出")

    def set_photo_flag(self):
        rospy.set_param('/top_view_shot_node/im_flag', 1)
        self.waiting_for_photo = True

    def wait_photo_result(self):
        if not self.waiting_for_photo:
            return False
        if self.photo_key_seq == self.photo_key_seq_used:
            return True  # 还没收到新消息，继续等待
        self.waiting_for_photo = False
        self.photo_key_seq_used = self.photo_key_seq
        rospy.loginfo("拍照识别完成，识别值: %s", str(self.photo_key))
        return False

    def photo_recognition_callback(self, msg):
        self.photo_key = msg.data
        self.photo_key_seq += 1
        rospy.loginfo("收到/photo_recognition识别结果: %s, seq: %d", str(self.photo_key), self.photo_key_seq)

if __name__ == '__main__':
    try:
        rospy.init_node('main_controller', anonymous=True)
        
        user_input = raw_input("请输入1启动导航程序：")
        if user_input.strip() != '1':
            rospy.loginfo("输入错误，程序退出")
            sys.exit(0)
        
        # 启动fa.py后台进程
        fa_script = "/home/abot/cryd_wss/src/robot_slam/scripts/fa.py"
        subprocess.Popen(["python2.7", fa_script])
        rospy.loginfo("基础服务已启动")

        # 新增：提前获取并转换中文值
        rospy.loginfo("等待/chinese_topic话题数据...")
        chinese_msg = rospy.wait_for_message('/chinese_topic', String)
        end_key = convert_chinese_time(chinese_msg.data)
        
        # 初始化控制器并注入转换值
        # 修改主程序调用方式
        # 初始化控制器时直接传递值
    # 初始化时注入end_key
        controller = NavigationController(end_key=end_key)
        rospy.loginfo("启动导航系统，最终目标点：%d", end_key)
        controller.main_loop()

    except rospy.ROSInterruptException:
        rospy.loginfo("程序退出")