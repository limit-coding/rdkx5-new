import rclpy
from rclpy.node import Node
from interfaces.msg import Position
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import numpy as np
import math
import time
from enum import Enum
class dynamics:
    def __init__(self):
        
        self.current_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw':0.0}
        self.target_pose = {'x': 0, 'y': 0, 'z': 0, 'yaw':0}
        self.initial_pose = {'x':0.0, 'y':0.0, 'z': 0.0, 'yaw':0.0}
        self.position_error = [0,0,0,0]
        self.target_speed = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.quat_x=0
        self.quat_y=0
        self.quat_z=0
        self.quat_w=0

class state(Enum):
    Takeoff = 0
    Stop = 1
    Cruise = 2
    MoveTo = 3
    Landing = 4
    Finish = 5
    
    

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.pos_pub = self.create_publisher(Position, '/pos_pub', 10)
        self.speed_pub = self.create_publisher(Position, '/speed_pub', 10)
        
        self.height_sub = self.create_subscription(
            Int32, '/height', self.height_callback, 10)
        self.slam_pose_sub = self.create_subscription(
            Odometry, '/Odometry', self.slam_pose_callback, 10)
        self.unlock_sub = self.create_subscription(
            String, '/unlock', self.unlock_callback, 10)
        
        self.pos_timer = self.create_timer(0.1,lambda: self.publish_position_error(self.dynamics.position_error))
        
        self.speed_timer = self.create_timer(0.1,lambda: self.publish_speed(self.dynamics.target_speed))
        self.speed_timer.cancel()
        
        self.dynamics = dynamics()
        self.takeoff_height= 1.2
        self.position_tolerance_xy = 0.1
        self.position_tolerance_z = 0.07
        self.movepoint = 0
        self.count = 0
        
        self.state = state.MoveTo
        
    def goal_callback(self,msg):
        self.dynamics.target_pose['x'] = msg.x + self.dynamics.current_pose['x']
        self.dynamics.target_pose['y'] = msg.y + self.dynamics.current_pose['y']
        self.dynamics.target_pose['z'] = msg.z + self.dynamics.current_pose['z']
        yaw = msg.yaw + self.dynamics.current_pose['yaw']
        yaw = (yaw + 180) % 360 - 180
        self.target_pose['yaw'] = yaw
        
    def height_callback(self, msg):
        """更新高度信息"""
        # self.get_logger().info(f'成功接收高度！{msg.data}')
        self.dynamics.current_pose['z'] = msg.data / 100.0
        
    def unlock_callback(self, msg):
        self.get_logger().info(f'成功接收指令！')
        if msg.data == "unlock":
            self.initial_pose['x'] = self.dynamics.current_pose['x']
            self.initial_pose['y'] = self.dynamics.current_pose['y']
            self.initial_pose['z'] = self.dynamics.current_pose['z']
            self.initial_pose['yaw'] = self.dynamics.current_pose['yaw']
        else:
            self.get_logger().error('收到无效指令')

    def slam_pose_callback(self, msg):
        """更新SLAM定位信息"""
        self.dynamics.current_pose['x'] = msg.pose.pose.position.x - self.dynamics.initial_pose['x']
        self.dynamics.current_pose['y'] = msg.pose.pose.position.y - self.dynamics.initial_pose['y']
        self.x = msg.pose.pose.orientation.x
        self.y = msg.pose.pose.orientation.y
        self.z = msg.pose.pose.orientation.z
        self.w = msg.pose.pose.orientation.w
        self.dynamics.current_pose['yaw'] = self.quaternion_to_euler() - self.dynamics.initial_pose['yaw']
    def calculate_position_error(self):
        error_odom = [self.dynamics.target_pose['x'] - self.dynamics.current_pose['x'] ,
                      self.dynamics.target_pose['y'] - self.dynamics.current_pose['y'] ,
                      self.dynamics.target_pose['z'] - self.dynamics.current_pose['z'] ,
                      self.dynamics.target_pose['yaw'] - self.dynamics.current_pose['yaw'] ]
        yaw = np.radians(self.dynamics.current_pose['yaw'])    # 偏航角

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        error_drone = [float(error_odom[0] * cos_yaw + error_odom[1] * sin_yaw),
                      float(-error_odom[0] * sin_yaw + error_odom[1] * cos_yaw),
                       float(error_odom[2]),
                       float(error_odom[3])]
        return error_drone
        
    def publish_speed(self,speed):
        msg = Position()
        msg.x = float(speed['x'])
        msg.y = float(speed['y'])
        msg.z = float(speed['z'])
        msg.yaw = 0.0
        self.pos_pub.publish(msg)

    def publish_position_error(self,error_drone):
        msg = Position()
        msg.x = float(error_drone[0])
        msg.y = float(error_drone[1])
        msg.z = float(error_drone[2])
        msg.yaw = float(error_drone[3])
        self.pos_pub.publish(msg)

    def to_point_error(self,error_drone):
        return math.sqrt(error_drone[0]**2 + error_drone[1]**2)
        
    def quaternion_to_euler(self,degrees=True):
        # 计算偏航角 (Z轴)
        siny_cosp = 2.0 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        if degrees:
            yaw = math.degrees(yaw)
                
        return yaw