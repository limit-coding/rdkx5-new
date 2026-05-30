import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from interfaces.msg import Position
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
import matplotlib.pyplot as plt
from rcl_interfaces.msg import (
    Parameter,            # 参数对象
    ParameterType,        # 参数类型枚举
    ParameterValue,       # 参数值
    SetParametersResult,  # 设置参数的结果
)
# 在类初始化中添加队列（用于主线程与回调线程通信）
import queue


class OdomToTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_to_tf_broadcaster')
        
        self.declare_parameter("record_x",False)
        self.declare_parameter("record_y",False)
        self.declare_parameter("record_z",False)
        self.declare_parameter("record_yaw",False)
        self.declare_parameter("show_graph",False)
        self.if_record_x = self.get_parameter('record_x').value
        self.if_record_y = self.get_parameter('record_y').value
        self.if_record_z = self.get_parameter('record_z').value
        self.if_record_yaw = self.get_parameter('record_yaw').value
        self.if_show_graph = self.get_parameter('show_graph').value
        self.record_x = []
        self.record_y = []
        self.record_z = []
        self.record_yaw = []
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.plot_queue = queue.Queue()  # 新增：绘图请求队列
        # 创建TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 初始化目标TF消息，设置固定的frame_id和child_frame_id
        self.t_target = TransformStamped()
        self.t_target.header.frame_id = 'map'
        self.t_target.child_frame_id = 'target_point'
        
        self.current_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw':0.0}
        self.x = 0
        self.y = 0
        self.z = 0
        self.w = 0
        
        self.fig = None
        self.axes = None

        # 订阅里程计消息
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/Odometry',  # 订阅的里程计话题名称
            self.odom_callback,
            10)
        
        # 订阅目标点消息
        self.target_subscription = self.create_subscription(
            Position,
            '/goal',  # 订阅的目标点话题名称
            self.target_callback,
            10)

        self.height_subscription = self.create_subscription(
            Int32,
            '/height',  # 订阅的目标点话题名称
            self.height_callback,
            10)
        
        # 创建高频定时器用于发布目标TF（10Hz）
        self.tf_target_publish_timer = self.create_timer(0.1, self.publish_target_tf)
        
        # 标记是否接收到目标点消息
        self.target_received = False
    def parameter_callback(self,params):
        for param in params:
            if param.name == 'record_x' and param.value != self.if_record_x:
                self.if_record_x = param.value
            if param.name == 'record_y' and param.value != self.if_record_y:
                self.if_record_y = param.value
            if param.name == 'record_z' and param.value != self.if_record_z:
                self.if_record_z = param.value
            if param.name == 'record_yaw' and param.value != self.if_record_yaw:
                self.if_record_yaw = param.value
            if param.name == 'show_graph' and param.value != self.if_show_graph:
                self.if_show_graph = param.value
                if(self.if_show_graph):
                    self.plot_queue.put(True)
                else:
                    plt.close()
        self.get_logger().info("参数已更改")
        return SetParametersResult(successful=True)
    
    def show_graph(self):
        record = []
        graph_name = []
        if(self.record_x):
            record_x = np.array(self.record_x)
            record.append(record_x)
            graph_name.append("x")
        if(self.record_y):
            record_y = np.array(self.record_y)
            record.append(record_y)
            graph_name.append("y")
            
        if(self.record_z):
            record_z = np.array(self.record_z)
            record.append(record_z)
            graph_name.append("z")
            
        if(self.record_yaw):
            record_yaw = np.array(self.record_yaw)
            record.append(record_yaw)
            graph_name.append("yaw")
            
        if not record:
            self.get_logger().info("无记录数据，不绘制图表")
            return  # 关键：无数据时直接退出函数
    
        if self.fig is None:
            # 创建新图表
            self.fig, self.axes = plt.subplots(1,len(record),figsize=(20, 8))
            self.axes = np.array(self.axes).flatten() 
            for i in range(len(record)):
                
                # 在子图上绘制曲线
                self.axes[i].plot(record[i][:,0], record[i][:,1], 'b-' ,label=graph_name[i])

                # 设置标题和标签
                self.axes[i].set_title(graph_name[i], fontsize=14)
                self.axes[i].set_xlabel('t', fontsize=12)
                self.axes[i].set_ylabel(graph_name[i], fontsize=12)

                # 添加网格和图例
                self.axes[i].grid(True)
                self.axes[i].legend()
        else:
            # 清除旧数据，复用已有图表
            for ax in self.axes:
                ax.clear()
        
        # 显示图形
        
        plt.tight_layout()  # 自动调整布局
        plt.show(block=False)
        plt.draw()
        plt.pause(0.1)
        self.record_x,self.record_y,self.record_z,self.record_yaw = [],[],[],[]
        self.if_record_x,self.if_record_y,self.if_record_z,self.if_record_yaw = False, False, False, False
        self.get_logger().info("参数、数据 已归零")        
    def publish_target_tf(self):
        # 只在接收到目标点消息后才发布TF
        if self.target_received:
            # 更新时间戳
            self.t_target.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.t_target)

    def target_callback(self, msg):
        # 更新目标点位置和朝向
        self.t_target.transform.translation.x = msg.x + self.current_pose['x']
        self.t_target.transform.translation.y = msg.y + self.current_pose['y']
        self.t_target.transform.translation.z = msg.z + self.current_pose['z']
        
        # 设置目标点的朝向
        yaw = msg.yaw + self.current_pose['yaw']
        yaw = (yaw + 180) % 360 - 180
        self.t_target.transform.rotation = self.yaw_to_quaternion(yaw)
        
        # 标记已接收到目标点消息
        self.target_received = True

    def odom_callback(self, msg):
        # 创建里程计到基坐标系的TF变换
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        self.x = msg.pose.pose.orientation.x
        self.y = msg.pose.pose.orientation.y
        self.z = msg.pose.pose.orientation.z
        self.w = msg.pose.pose.orientation.w
        self.current_pose['yaw'] = self.quaternion_to_euler()
        # 设置平移和旋转
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = self.current_pose['z']
        t.transform.rotation = msg.pose.pose.orientation
        if(self.if_record_x):
            self.record_x.append([self.get_clock().now().nanoseconds * 1e-9,self.current_pose['x']])
        if(self.if_record_y):
            self.record_y.append([self.get_clock().now().nanoseconds * 1e-9,self.current_pose['y']])
        if(self.if_record_yaw):
            self.record_yaw.append([self.get_clock().now().nanoseconds * 1e-9,self.current_pose['yaw']])
        # 广播TF变换
        self.tf_broadcaster.sendTransform(t)

    def height_callback(self, msg):
        """更新高度信息"""
        # self.get_logger().info(f'成功接收高度！{msg.data}')
        self.current_pose['z'] = msg.data / 100.0
        if(self.if_record_z):
            self.record_z.append([self.get_clock().now().nanoseconds * 1e-9,self.current_pose['z']])

    def yaw_to_quaternion(self, yaw_degrees):
        """将偏航角(yaw，单位：度)转换为四元数表示"""
        yaw_radians = math.radians(yaw_degrees)
        q = TransformStamped().transform.rotation
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw_radians / 2)
        q.w = math.cos(yaw_radians / 2)
        return q

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
    node = OdomToTfBroadcaster()
    
    # 主线程循环：处理ROS消息 + 绘图请求
    while rclpy.ok():
        # 处理ROS回调（非阻塞）
        rclpy.spin_once(node, timeout_sec=0.1)
        # 检查是否有绘图请求
        try:
            # 从队列获取请求（非阻塞）
            if node.plot_queue.get_nowait():
                node.show_graph()  # 在主线程调用绘图
        except queue.Empty:
            pass  # 无请求则继续循环
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()