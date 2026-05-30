import rclpy
import rclpy
from rclpy.node import Node
import math
import numpy as np
from enum import Enum
import time
from nav_msgs.msg import Odometry
from interfaces.msg import Position
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3

class pid_test(Node):
    def __init__(self):
        super().__init__('pid_test')
        self.goal_sub = self.create_subscription(Position,'/goal',self.goal_callback,10)
        
        self.pos_pub = self.create_publisher(Position, '/pos_pub', 10)
        self.speed_pub = self.create_publisher(Position, '/speed_pub', 10)
        self.height_sub = self.create_subscription(
            Int32, '/height', self.height_callback, 10)
        self.slam_pose_sub = self.create_subscription(
            Odometry, '/Odometry', self.slam_pose_callback, 10)
        self.control_timer = self.create_timer(0.1,self.publish_position_error)
        self.current_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw':0.0}
        self.current_speed = {'x': 0.0, 'y': 0.0}
        self.target_pose = {'x': 0, 'y': 0, 'z': 0, 'yaw':0}
        self.initial_pose = {'x':0.0, 'y':0.0, 'z': 0.0, 'yaw':0.0}
        self.real_pose = {'x':0.0, 'y':0.0, 'z': 0.0, 'yaw':0.0}

        self.x=0
        self.y=0
        self.z=0
        self.w=0

        self.last_time=0
        
    def goal_callback(self,msg):
        self.target_pose['x'] = msg.x + self.current_pose['x']
        self.target_pose['y'] = msg.y + self.current_pose['y']
        self.target_pose['z'] = msg.z + self.current_pose['z']
        yaw = msg.yaw + self.current_pose['yaw']
        yaw = (yaw + 180) % 360 - 180
        self.target_pose['yaw'] = yaw
        self.get_logger().info(f"{self.target_pose['x'],self.target_pose['y'],self.target_pose['z'],self.target_pose['yaw']}")
        
    def height_callback(self, msg):
        """更新高度信息"""
        # self.get_logger().info(f'成功接收高度！{msg.data}')
        self.current_pose['z'] = msg.data / 100.0
        
    def slam_pose_callback(self, msg):
        """更新SLAM定位信息"""
        # dt=time.time()-self.last_time
        # self.last_time=time.time()
        # self.current_speed['x'] =( msg.pose.pose.position.x - self.current_pose['x'] ) /dt * 100
        # self.current_speed['y'] =( msg.pose.pose.position.y - self.current_pose['y'] ) /dt * 100
        # speed_msg = Position()
        # speed_msg.x=self.current_speed['x']
        # speed_msg.y=self.current_speed['y']
        # self.speed_pub.publish(speed_msg)

        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        self.x = msg.pose.pose.orientation.x
        self.y = msg.pose.pose.orientation.y
        self.z = msg.pose.pose.orientation.z
        self.w = msg.pose.pose.orientation.w
        self.current_pose['yaw'] = self.quaternion_to_euler()
        
    def publish_position_error(self):
        """计算位置差值并发布"""
        error_msg = Position()
        error_msg.x = self.target_pose['x'] - self.current_pose['x'] + self.initial_pose['x']
        error_msg.y = self.target_pose['y'] - self.current_pose['y'] + self.initial_pose['y']
        error_msg.z = self.target_pose['z'] - self.current_pose['z'] + self.initial_pose['z']
        error_msg.yaw = self.target_pose['yaw'] - self.current_pose['yaw'] + self.initial_pose['yaw']
        
        self.current_pose['yaw'] = self.quaternion_to_euler()
        yaw = np.radians(self.current_pose['yaw'])    # 偏航角

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        error_msg_trans = Position()
        error_msg_trans.x = error_msg.x * cos_yaw + error_msg.y * sin_yaw
        error_msg_trans.y = -error_msg.x * sin_yaw + error_msg.y * cos_yaw
        error_msg_trans.z = error_msg.z
        error_msg_trans.yaw = error_msg.yaw
        
        self.pos_pub.publish(error_msg_trans)
        self.get_logger().info(f"发布坐标转换后位置差值：{error_msg_trans.x,error_msg_trans.y,error_msg_trans.z}")
    def quaternion_to_euler(self,degrees=True):
        # 计算偏航角 (Z轴)
        siny_cosp = 2.0 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        if degrees:
            yaw = math.degrees(yaw)
                
        return yaw
    
def main(args=None):
    rclpy.init(args=args)
    planner = pid_test()
    planner.get_logger().info("节点已开启")
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info("节点已安全关闭")
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()