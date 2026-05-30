import rclpy
from rclpy.node import Node
import math
import numpy as np

from interfaces.msg import Animalcnt
from collections import Counter  # 需要导入Counter用于统计
from std_msgs.msg import Int32MultiArray, MultiArrayDimension
import time

from interfaces.msg import Position
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


from geometry_msgs.msg import Vector3
from interfaces.srv import Pathplan
from interfaces.srv import PositionNum
from interfaces.msg import Pair
from map.map import Map
from controller.controller import Controller
from planner.planner import Pathplanner
from controller.controller import state
import threading

class main_ctrl(Node):
    def __init__(self):
        super().__init__('main_ctrl')
        self.pathplanner = Pathplanner()
        self.controller = Controller()
        self.goal_sub = self.create_subscription(Position,'goal',self.controller.goal_callback,10)
        self.planner_server = self.create_service(Pathplan,'/pathplan_srv',self.pathplan_callback)
        
        self.picture_sub = self.create_subscription(Int32MultiArray, '/pic_cnt', self.pic_callback, 10)
        self.animal_pub=self.create_publisher(Animalcnt,'/animals',10)
        
        self.initialized = False
        self.initialization_event - threading.Event()
        self.wait_thread = threading.Thread(target=self.wait_for_initialization)
        #self.get_logger().info(f"{self.target_pose['x'],self.target_pose['y'],self.target_pose['z'],self.target_pose['yaw']}")
        
        self.wait_timer = self.create_timer(0.5,self.decide_state)
        self.wait_timer.cancel()
        
        self.path_msg = Path()
        self.path_pub = self.create_publisher(Path,'/path',10)
        self.path_pub_timer = self.create_timer(0.1,self.path_pub_callback)
        self.wait_timer = self.create_timer(0.5,self.wait_timeout)
        self.wait_timer.cancel()

        self.planpath_event = threading.Event()
        
        self.animals=[[],[],[],[],[]]
        self.observed = False
        self.moved = False
        self.pic_enable = True
        
    
    def pathplan_callback(self,request,response):
        self.pathplanner.grid_map.map[request.blocks[0].first][request.blocks[0].second] = 1
        self.pathplanner.grid_map.map[request.blocks[1].first][request.blocks[1].second] = 1
        self.pathplanner.grid_map.map[request.blocks[2].first][request.blocks[2].second] = 1

        self.pathplanner.grid_map.blocks.append((request.blocks[0].first,request.blocks[0].second))#blocks = [(2,3),...]
        self.pathplanner.grid_map.blocks.append((request.blocks[1].first,request.blocks[1].second))
        self.pathplanner.grid_map.blocks.append((request.blocks[2].first,request.blocks[2].second))
        if(self.pathplanner.plan()):
            response.if_succeed = True
            for pair in self.pathplanner.path:
                pair_ = Pair()
                pair_.first = pair[0]
                pair_.second = pair[1]
                response.path.append(pair_)
            self.get_logger().info(f"路径规划成功，路径长度: {len(response.path)}")
            
            
            self.planpath_event.set()
        else:
            response.if_succeed = False
            self.get_logger().warn(f"路径规划失败")
        return response
    def pic_callback(self,msg):
        if self.observed == False and self.pic_enable == True:
            self.observed = True
            if not self.controller.state == state.MoveTo:
                self.controller.state = state.MoveTo
            return
        elif self.observed == True and self.moved == False:
            return
        elif self.observed ==  True and self.moved == True:
            self.animals[0].append(msg.data[0])
            self.animals[1].append(msg.data[1])
            self.animals[2].append(msg.data[2])
            self.animals[3].append(msg.data[3])
            self.animals[4].append(msg.data[4])
            return
        else:
            return
            
            
            
        
    def ProcessPath(self):
        
        for point in self.pathplanner.path:
            self.pathplanner.way_point.append((point[1]*0.5,(8-point[0])*0.5))
        
        self.path_msg.header.frame_id = "odom" 
        self.path_msg.header.stamp = self.get_clock().now().to_msg()  # 添加时间戳
        for point in self.pathplanner.way_point:
            point_ = PoseStamped()
            point_.header.frame_id = 'odom'
            point_.pose.position.x = point[0]
            point_.pose.position.y = point[1]
            point_.pose.position.z = self.controller.takeoff_height
            self.path_msg.poses.append(point_)
                
        for i in range(len(self.pathplanner.path)):
            if i==0 or i==len(self.pathplanner.path)-1:
                # self.pathplanner.processed_path.append(self.pathplanner.path[i])
                self.pathplanner.corner_index.append(i)
                continue
            
            if (self.pathplanner.path[i][0] - self.pathplanner.path[i-1][0] 
             != self.pathplanner.path[i+1][0] - self.pathplanner.path[i][0] 
             or self.pathplanner.path[i][1] - self.pathplanner.path[i-1][1] 
             != self.pathplanner.path[i+1][1] - self.pathplanner.path[i][1]):
                # self.pathplanner.sparse_path.append(self.pathplanner.path[i])
                self.pathplanner.corner_index.append(i)
        self.pathplanner.corner_index = list(reversed(self.pathplanner.corner_index))
        self.get_logger().info(f"路径处理成功")

    def calculate_reliable_counts(self):
        """计算并输出5种动物的最可信数量"""
        
        
        reliable_counts = []
        for i in range(5):  # 遍历5种动物
            counts = self.animals[i]
            if not counts:  # 若列表为空，默认数量为0
                reliable_counts.append(0)
                continue
            
            # 取众数（出现次数最多的数量）
            count_freq = Counter(counts)
            most_common = count_freq.most_common(1)[0][0]  # 得到出现次数最多的值
            reliable_counts.append(most_common)
        
        msg=Animalcnt()
        msg.animals.layout.dim=[MultiArrayDimension(label='animals', size=5, stride=5)]
        msg.animals.data=reliable_counts
        msg.pos.first=self.pathplanner.path[self.controller.movepoint][0]
        msg.pos.second=self.pathplanner.path[self.controller.movepoint][1]
        self.animal_pub.publish(msg)

        self.get_logger().info(
            f"最可信数量：孔雀={reliable_counts[0]}, 狼={reliable_counts[1]}, "
            f"猴子={reliable_counts[2]}, 大象={reliable_counts[3]}, 老虎={reliable_counts[4]}"
        )
    def path_pub_callback(self):
        self.path_pub.publish(self.path_msg)
    def run(self):
        self.blocks_event.wait(timeout= None)
        self.ProcessPath()#path corner_index waypoint
        timer_ = self.create_timer(0.05,self.mode_fsm)
        
    def decide_state(self):
        if not self.wait_timer.is_canceled():
            self.wait_timer.cancel()
        if self.observed ==  True and self.moved == True:
            self.observed = False
            
            self.pic_enable = False
            self.calculate_reliable_counts()
        self.controller.movepoint += 1
        self.path_msg.poses.pop(0)
        self.moved = False
        #状态变化
        if self.controller.movepoint == self.pathplanner.corner_index[-1]:
            self.pathplanner.corner_index.pop()
            self.get_logger().info("状态切换至：MoveTo")
            self.controller.state = state.MoveTo
            return
        elif self.controller.movepoint == len(self.pathplanner.path)-1:
            self.get_logger().info("状态切换至：Landing")
            self.controller.state = state.Landing
            return
        else:
            self.get_logger().info("状态切换至：Cruise")
            self.controller.state = state.Cruise
            return
        
    def mode_fsm(self):
       
        if self.controller.state == state.Stop:
            self.controller.dynamics.target_pose['x'] = self.pathplanner.way_point[self.controller.movepoint][0]
            self.controller.dynamics.target_pose['y'] = self.pathplanner.way_point[self.controller.movepoint][1]
            self.controller.dynamics.target_pose['z'] = self.controller.takeoff_height
            self.controller.dynamics.target_pose['yaw'] = 0
            
            self.controller.dynamics.position_error = self.controller.calculate_position_error()
            
            
                
        if self.controller.state == state.Cruise:
            
            if(self.controller.speed_timer.is_canceled()):
                self.controller.speed_timer.reset()
            
            self.controller.dynamics.target_pose['x'] = self.pathplanner.way_point[self.controller.movepoint][0]
            self.controller.dynamics.target_pose['y'] = self.pathplanner.way_point[self.controller.movepoint][1]
            self.controller.dynamics.target_pose['z'] = self.controller.takeoff_height
            self.controller.dynamics.target_pose['yaw'] = 0
            
            temp = self.controller.calculate_position_error()
            if self.controller.dynamics.target_speed['y'] == 0:
                temp[0] = 0
            else :
                temp[1] = 0
            self.controller.dynamics.position_error = temp
            if self.controller.to_point_error(self.controller.dynamics.position_error) < self.controller.position_tolerance_xy:
                if self.pathplanner.path[self.controller.movepoint][2] == 0:
                    self.pic_enable = True
                self.wait_timer.cancel()
                time.sleep(0.12)               #参数
                self.wait_timer.reset()
                if self.observed :
                    return
                self.decide_state()
                
            return
        
        if self.controller.state == state.MoveTo:
            if(not self.controller.speed_timer.is_canceled):
                self.controller.speed_timer.cancel()
            self.controller.dynamics.target_pose['x'] = self.pathplanner.way_point[self.controller.movepoint][0]
            self.controller.dynamics.target_pose['y'] = self.pathplanner.way_point[self.controller.movepoint][1]
            self.controller.dynamics.target_pose['z'] = self.controller.takeoff_height
            self.controller.dynamics.target_pose['yaw'] = 0
            if(not self.controller.movepoint == len(self.pathplanner.path)-1):
                self.controller.dynamics.target_speed['x'] = self.pathplanner.path[self.controller.movepoint+1][1]-self.pathplanner.path[self.controller.movepoint][1]
                self.controller.dynamics.target_speed['y'] = self.pathplanner.path[self.controller.movepoint][0]-self.pathplanner.path[self.controller.movepoint+1][0]
                self.controller.dynamics.target_speed['z'] = 0 
            self.controller.dynamics.position_error = self.controller.calculate_position_error
            if self.controller.to_point_error(self.controller.dynamics.position_error) < self.controller.position_tolerance_xy:
                if self.pathplanner.path[self.controller.movepoint][2] == 0:
                    self.pic_enable = True
                
                self.moved = True
                
                self.wait_timer.reset()
                self.controller.state = state.Stop
                
                
            return
        if self.controller.state == state.Landing:
            pass
            
    
    
def main(args=None):
    #开启激光笔
    output_pin = 37
    GPIO.setmode(GPIO.BOARD)
    # 设置为输出模式
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
    rclpy.init(args=args)
    main_ctrl = main_ctrl()
    # 创建多线程执行器，同时管理main_node和其内部的controller
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(main)
    executor.add_node(main.controller)  # 将Controller节点加入执行器

    main_ctrl.get_logger().info("节点已开启")
    try:
        executor.spin()
    except KeyboardInterrupt:
        main_ctrl.get_logger().info("节点已安全关闭")
    finally:
        executor.shutdown()
        main.destroy_node()
        main.controller.destroy_node()  # 销毁Controller节点
        rclpy.shutdown()
        GPIO.output(output_pin, GPIO.LOW)
        GPIO.cleanup()

if __name__ == '__main__':
    main()