import rclpy
from rclpy.node import Node
import json
import math
import numpy as np

import time
import threading
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from interfaces.msg import Position
from interfaces.msg import Animalcnt
from geometry_msgs.msg import Vector3
from interfaces.srv import Pathplan
from interfaces.msg import Pair
from std_msgs.msg import Int32MultiArray,Int32,String, MultiArrayDimension

from map.map import Map
from controller.controller import Controller
from planner.planner import Pathplanner
from controller.controller import state

from collections import Counter  # 需要导入Counter用于统计
import Hobot.GPIO as GPIO

class main_ctrl(Node):
    def __init__(self):
        super().__init__('main_ctrl')
        self.pathplanner = Pathplanner()
        self.controller = Controller()
        self.controller.state = state.Takeoff
        self.controller.pos_timer.cancel()
        

        self.picenable_pub = self.create_publisher(Int32,'/pic_enable',10)
        self.qr_enable_pub = self.create_publisher(Int32, '/qr_enable', 10)
        self.animal_pub=self.create_publisher(Animalcnt,'/animals',10)
        self.lock_pub = self.create_publisher(String,'/lock',10)
        self.start_pub = self.create_publisher(String,'/start',10)
        self.task_status_pub = self.create_publisher(Int32MultiArray, '/task_status', 10)

        self.picture_sub = self.create_subscription(Int32MultiArray, '/pic_cnt', self.pic_callback, 10)
        self.qr_text_sub = self.create_subscription(String, '/qr_code/text', self.qr_text_callback, 10)
        self.qr_result_sub = self.create_subscription(String, '/qr_code/result', self.qr_result_callback, 10)
        self.qr_offset_sub = self.create_subscription(Int32MultiArray, '/qr_code/offset', self.qr_offset_callback, 10)
        self.vision_detections_sub = self.create_subscription(String, '/vision/detections', self.vision_detections_callback, 10)
        self.goal_sub = self.create_subscription(Position,'goal',self.controller.goal_callback,10)
        
        self.planner_server = self.create_service(Pathplan,'/pathplan_srv',self.pathplan_callback)

        self.path_msg = Path()
        self.path_pub = self.create_publisher(Path,'/path',10)
        self.path_pub_timer = self.create_timer(0.1,self.path_pub_callback)
        self.task_status_timer = self.create_timer(0.1, self.publish_task_status)
        self.wait_timer = self.create_timer(0.5,self.wait_timeout)
        self.wait_timer.cancel()

        self.blocks_event = threading.Event()

        self.animals=[[],[],[],[],[]]
        self.task_state = 0x01
        self.landing_state = 0x01
        self.target_class_1 = ""
        self.target_class_2 = ""
        self.target_classes = set()
        self.landing_side = ""
        self.qr_task_ready = False
        self.qr_offset = [0, 0, 0, 0, 0, 0]
        self.qr_task_confirm_frames = 3
        self._qr_candidate_text = ""
        self._qr_candidate_count = 0
        self.latest_vision_detections = []
        self.latest_targets = {
            'picture_target': [],
            'special_target': [],
            'ring': [],
            'landing_h': [],
            'red_light': [],
            'blue_light': [],
        }
        self.best_picture_target = None
        self.best_landing_h = None
        self.target_confirm_frames = 3
        self.target_disappear_frames = 5
        self.target_max_count = 4
        self.target_candidate_label = ""
        self.target_candidate_count = 0
        self.locked_target_label = ""
        self.locked_target_missing_count = 0
        self.recognized_target_count = 0

        # 45度降落参数
        self.safety_threshold = 0.1  # 接近地面的阈值高度(m)
        self.constant_total_speed = 30  # 恒定总速度大小(m/s)
        # 计算45度角的速度分量 (sin(45°) = cos(45°) = √2/2 ≈ 0.7071)
        self.speed_component = self.constant_total_speed * math.sin(math.pi / 4)

        #self.get_logger().info(f"{self.target_pose['x'],self.target_pose['y'],self.target_pose['z'],self.target_pose['yaw']}")

    def publish_task_status(self):
        msg = Int32MultiArray()
        msg.data = [int(self.task_state), int(self.landing_state)]
        self.task_status_pub.publish(msg)

    def set_task_status(self, task_state=None, landing_state=None):
        changed = False
        if task_state is not None and self.task_state != task_state:
            self.task_state = int(task_state)
            changed = True
        if landing_state is not None and self.landing_state != landing_state:
            self.landing_state = int(landing_state)
            changed = True
        if changed:
            self.publish_task_status()
            self.get_logger().info(
                f"task status updated: task_state=0x{self.task_state:02X}, "
                f"landing_state=0x{self.landing_state:02X}"
            )

    def parse_qr_task_text(self, text):
        value = text.strip()
        if not value:
            return None

        replacements = {
            '，': ',',
            '、': ',',
            '；': ';',
            '：': ':',
            ';': ',',
            '|': ',',
            '/': ',',
            '\\': ',',
            '[': ' ',
            ']': ' ',
            '{': ' ',
            '}': ' ',
            '(': ' ',
            ')': ' ',
            '"': ' ',
            "'": ' ',
            '=': ' ',
            ':': ' ',
        }
        for old, new in replacements.items():
            value = value.replace(old, new)

        ignored = {
            'class', 'class1', 'class2', 'target', 'target1', 'target2',
            'side', 'landing', 'landing_side', 'land', 'qr', 'task',
        }
        tokens = []
        for raw in value.replace(',', ' ').split():
            token = raw.strip().lower()
            if token and token not in ignored:
                tokens.append(token)

        landing_side = ""
        classes = []
        for token in tokens:
            if token in ('left', 'right'):
                landing_side = token
            else:
                classes.append(token)

        if len(classes) < 2 or not landing_side:
            return None
        return classes[0], classes[1], landing_side

    def update_qr_task_candidate(self, text):
        task = self.parse_qr_task_text(text)
        if task is None:
            self.get_logger().warn(f"Invalid QR task text: {text}")
            return

        normalized = f"{task[0]},{task[1]},{task[2]}"
        if normalized == self._qr_candidate_text:
            self._qr_candidate_count += 1
        else:
            self._qr_candidate_text = normalized
            self._qr_candidate_count = 1

        if self._qr_candidate_count < self.qr_task_confirm_frames:
            return

        if (
            self.qr_task_ready
            and self.target_class_1 == task[0]
            and self.target_class_2 == task[1]
            and self.landing_side == task[2]
        ):
            return

        self.target_class_1 = task[0]
        self.target_class_2 = task[1]
        self.target_classes = {self.target_class_1, self.target_class_2}
        self.landing_side = task[2]
        self.qr_task_ready = True
        landing_state = 0x02 if self.landing_side == 'left' else 0x03
        self.set_task_status(task_state=0x02, landing_state=landing_state)
        msg = Int32()
        msg.data = 1
        self.picenable_pub.publish(msg)
        qr_enable_msg = Int32()
        qr_enable_msg.data = 0
        self.qr_enable_pub.publish(qr_enable_msg)
        self.get_logger().info(
            f"QR task confirmed: target_class_1={self.target_class_1}, "
            f"target_class_2={self.target_class_2}, landing_side={self.landing_side}"
        )

    def qr_text_callback(self, msg):
        self.update_qr_task_candidate(msg.data)

    def qr_result_callback(self, msg):
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid QR result JSON: {msg.data}")
            return

        if result.get('found') and result.get('text'):
            self.update_qr_task_candidate(str(result['text']))

    def qr_offset_callback(self, msg):
        self.qr_offset = list(msg.data)

    def vision_detections_callback(self, msg):
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid vision detections JSON: {msg.data}")
            return

        detections = result.get('detections', [])
        if not isinstance(detections, list):
            self.get_logger().warn(f"Invalid vision detections payload: {msg.data}")
            return

        self.latest_vision_detections = detections
        for key in self.latest_targets:
            self.latest_targets[key] = []

        for det in detections:
            if not isinstance(det, dict):
                continue
            class_name = str(det.get('class_name', ''))
            if class_name in self.latest_targets:
                self.latest_targets[class_name].append(det)

        self.best_picture_target = self.pick_best_detection(self.latest_targets['picture_target'])
        self.best_landing_h = self.pick_best_detection(self.latest_targets['landing_h'])

        summary = []
        for key, values in self.latest_targets.items():
            if values:
                summary.append(f"{key}={len(values)}")
        if summary:
            self.get_logger().info(
                f"vision targets: {', '.join(summary)}",
                throttle_duration_sec=1,
            )

        if self.qr_task_ready:
            self.update_yolo_task_state(detections)

    def pick_best_detection(self, detections):
        if not detections:
            return None
        return max(
            detections,
            key=lambda det: (
                float(det.get('score', 0.0)),
                int(det.get('area', 0)),
            ),
        )

    def pick_best_task_detection(self, detections):
        ignored_classes = {'ring', 'landing_h', 'red_light', 'blue_light', 'obstacle'}
        candidates = []
        for det in detections:
            if not isinstance(det, dict):
                continue
            class_name = str(det.get('class_name', '')).strip().lower()
            if not class_name or class_name in ignored_classes:
                continue
            candidates.append(det)
        return self.pick_best_detection(candidates)

    def update_yolo_task_state(self, detections):
        best = self.pick_best_task_detection(detections)
        if best is None:
            self.handle_no_task_target()
            return

        label = str(best.get('class_name', '')).strip().lower()
        if not label:
            self.handle_no_task_target()
            return

        if self.locked_target_label:
            if label == self.locked_target_label:
                self.locked_target_missing_count = 0
                return
            self.locked_target_missing_count += 1
            if self.locked_target_missing_count < self.target_disappear_frames:
                return
            self.clear_target_lock()

        if label == self.target_candidate_label:
            self.target_candidate_count += 1
        else:
            self.target_candidate_label = label
            self.target_candidate_count = 1

        if self.target_candidate_count < self.target_confirm_frames:
            return

        self.confirm_new_task_target(label)

    def handle_no_task_target(self):
        self.target_candidate_label = ""
        self.target_candidate_count = 0
        if not self.locked_target_label:
            return
        self.locked_target_missing_count += 1
        if self.locked_target_missing_count >= self.target_disappear_frames:
            self.clear_target_lock()

    def clear_target_lock(self):
        self.locked_target_label = ""
        self.locked_target_missing_count = 0
        self.target_candidate_label = ""
        self.target_candidate_count = 0

    def confirm_new_task_target(self, label):
        if self.recognized_target_count >= self.target_max_count:
            return

        self.recognized_target_count += 1
        is_target = label in self.target_classes
        task_state = 0x03 + (self.recognized_target_count - 1) * 2
        if not is_target:
            task_state += 1

        self.set_task_status(task_state=task_state)
        self.locked_target_label = label
        self.locked_target_missing_count = 0
        self.target_candidate_label = ""
        self.target_candidate_count = 0
        self.get_logger().info(
            f"YOLO target confirmed: index={self.recognized_target_count}, "
            f"label={label}, qr_target={is_target}, task_state=0x{task_state:02X}"
        )
    
    def path_pub_callback(self):
        self.path_pub.publish(self.path_msg)
    def pathplan_callback(self,request,response):
        self.pathplanner.grid_map.map[request.blocks[0].first][request.blocks[0].second] = 1
        self.pathplanner.grid_map.map[request.blocks[1].first][request.blocks[1].second] = 1
        self.pathplanner.grid_map.map[request.blocks[2].first][request.blocks[2].second] = 1

        self.pathplanner.grid_map.blocks.append((request.blocks[0].first,request.blocks[0].second))#blocks = [(2,3),...]
        self.pathplanner.grid_map.blocks.append((request.blocks[1].first,request.blocks[1].second))
        self.pathplanner.grid_map.blocks.append((request.blocks[2].first,request.blocks[2].second))

        if(self.pathplanner.plan_stop()):
            self.blocks_event.set() 
            self.ProcessPath()
            
            self.path_msg.header.frame_id = "odom" 
            self.path_msg.header.stamp = self.get_clock().now().to_msg()  # 添加时间戳
            for point in self.pathplanner.way_point:
                point_ = PoseStamped()
                point_.header.frame_id = 'odom'
                point_.pose.position.x = point[0]
                point_.pose.position.y = point[1]
                point_.pose.position.z = self.controller.takeoff_height
                self.path_msg.poses.append(point_)
                
            response.if_succeed = True
            for i in reversed(self.pathplanner.corner_index):
                pair_ = Pair()
                pair_.first = self.pathplanner.path[i][0]
                pair_.second = self.pathplanner.path[i][1]
                response.path.append(pair_)
        else:
            response.if_succeed = False
        return response

    def ProcessPath(self):
        for point in self.pathplanner.path[:-1]:
            self.pathplanner.way_point.append((point[1]*0.5,(8-point[0])*0.5))
        if (self.pathplanner.path[-2][0]==8):
            self.pathplanner.way_point.append((1.2,0,0))
        else:
            self.pathplanner.way_point.append((0,1.2,0))
        
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
            
    def pic_callback(self,msg):
        self.animals[0].append(msg.data[0])
        self.animals[1].append(msg.data[1])
        self.animals[2].append(msg.data[2])
        self.animals[3].append(msg.data[3])
        self.animals[4].append(msg.data[4])

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

    def wait_timeout(self):
        if not self.wait_timer.is_canceled():
            self.wait_timer.cancel()

        msg=Int32()
        msg.data=0
        self.picenable_pub.publish(msg)

        self.calculate_reliable_counts()
        self.path_msg.poses.pop(0)
        self.controller.movepoint+=1
        self.controller.state=state.MoveTo

    def run(self):
        msg=String()
        msg.data="start"
        self.start_pub.publish(msg)

        self.blocks_event.wait(timeout= None)
        self.control_timer = self.create_timer(0.1, self.main_fsm)

    def main_fsm(self):
        self.get_logger().info(f"进入状态: {self.controller.state.name}，当前位置：{self.controller.dynamics.current_pose['x']},{self.controller.dynamics.current_pose['y']},{self.controller.dynamics.current_pose['z']},{self.controller.dynamics.current_pose['yaw']}",throttle_duration_sec=1)
        if self.controller.state == state.Takeoff:
            self.controller.dynamics.target_pose['x'] = 0
            self.controller.dynamics.target_pose['y'] = 0
            self.controller.dynamics.target_pose['z'] = self.controller.takeoff_height
            self.controller.position_error = self.controller.calculate_position_error()
            self.controller.publish_position_error(self.controller.position_error)
            if abs(self.controller.dynamics.target_pose['z']-self.controller.dynamics.current_pose['z'])<self.controller.position_tolerance_z:
                self.controller.state = state.MoveTo
                self.get_logger().info("起飞完成")

        elif self.controller.state == state.MoveTo:
            self.controller.dynamics.target_pose['x'] = self.pathplanner.way_point[self.controller.movepoint][0]
            self.controller.dynamics.target_pose['y'] = self.pathplanner.way_point[self.controller.movepoint][1] 
            self.controller.dynamics.target_pose['z'] = self.controller.takeoff_height
            self.controller.position_error = self.controller.calculate_position_error()
            self.controller.publish_position_error(self.controller.position_error)
            if self.controller.to_point_error(self.controller.position_error) < self.controller.position_tolerance_xy:
                if self.pathplanner.path[self.controller.movepoint][2] == 1:
                    self.controller.state = state.Stop
                    self.get_logger().info(f"到达A{self.pathplanner.path[self.controller.movepoint][0]+1},B{self.pathplanner.path[self.controller.movepoint][1]+1}")
                    if(self.wait_timer.is_canceled()):
                        self.wait_timer.reset()

                    msg=Int32()
                    msg.data=1
                    self.picenable_pub.publish(msg)
                    self.animals=[[],[],[],[],[]]
                else:
                    self.controller.movepoint+=1
                    if self.controller.movepoint >= len(self.pathplanner.way_point):
                        self.controller.state=state.Landing

            
        elif self.controller.state == state.Stop:
            self.controller.dynamics.target_pose['x'] = self.pathplanner.way_point[self.controller.movepoint][0]
            self.controller.dynamics.target_pose['y'] = self.pathplanner.way_point[self.controller.movepoint][1] 
            self.controller.dynamics.target_pose['z'] = self.controller.takeoff_height
            error = self.controller.calculate_position_error()
            self.controller.publish_position_error(error)

        elif self.controller.state == state.Landing:
            self.controller.dynamics.target_pose['x'] = 0
            self.controller.dynamics.target_pose['y'] = 0
            self.controller.dynamics.target_pose['z'] = 0
            self.controller.position_error = self.controller.calculate_position_error()
            self.controller.publish_position_error(self.controller.position_error)

            if abs(self.pathplanner.way_point[-1][0] - 1.2) < 1e-6:
                # 计算当前x和z方向的位移比例
                x_error = self.controller.position_error[0]
                z_error = self.controller.position_error[2]
                
                # 确保不会除以零
                if z_error < 0.01:
                    x_speed = 0.0
                    z_speed = 0.0
                else:
                    # 计算速度比例因子，确保x和z速度分量保持45度比例
                    ratio = x_error / z_error
                    
                    # 接近地面时减速
                    if z_error < self.safety_threshold:
                        speed_scale = z_error / self.safety_threshold  # 0~1之间的缩放因子
                    else:
                        speed_scale = 1.0  # 正常速度
                    
                    # 根据比例和总速度计算各轴速度分量
                    # 确保x和z速度分量的比值与位移比值一致，保持45度轨迹
                    if abs(ratio) > 1.0:
                        x_speed = self.speed_component * speed_scale * (x_error / abs(x_error))
                        z_speed = self.speed_component * speed_scale * (x_error / abs(x_error)) / ratio
                    else:
                        x_speed = self.speed_component * speed_scale * (z_error / abs(z_error)) * ratio
                        z_speed = self.speed_component * speed_scale * (z_error / abs(z_error))
                
                # 设置目标速度
                self.controller.dynamics.target_speed['x'] = x_speed  # 使用x轴进行水平移动
                self.controller.dynamics.target_speed['y'] = 0.0
                self.controller.dynamics.target_speed['z'] = z_speed  # z轴负方向为下降
            else:
                # 计算当前y和z方向的位移比例
                y_error = self.controller.position_error[1]
                z_error = self.controller.position_error[2]
                
                # 确保不会除以零
                if z_error < 0.01:
                    y_speed = 0.0
                    z_speed = 0.0
                else:
                    # 计算速度比例因子，确保x和z速度分量保持45度比例
                    ratio = y_error / z_error
                    
                    # 接近地面时减速
                    if z_error < self.safety_threshold:
                        speed_scale = z_error / self.safety_threshold  # 0~1之间的缩放因子
                    else:
                        speed_scale = 1.0  # 正常速度
                    
                    # 根据比例和总速度计算各轴速度分量
                    # 确保x和z速度分量的比值与位移比值一致，保持45度轨迹
                    if abs(ratio) > 1.0:
                        y_speed = self.speed_component * speed_scale * (y_error / abs(y_error))
                        z_speed = self.speed_component * speed_scale * (y_error / abs(y_error)) / ratio
                    else:
                        y_speed = self.speed_component * speed_scale * (z_error / abs(z_error)) * ratio
                        z_speed = self.speed_component * speed_scale * (z_error / abs(z_error))
                
                # 设置目标速度
                self.controller.dynamics.target_speed['x'] = 0.0  # 使用x轴进行水平移动
                self.controller.dynamics.target_speed['y'] = y_speed
                self.controller.dynamics.target_speed['z'] = z_speed  # z轴负方向为下降
            
            # 发布速度指令
            self.controller.publish_speed(self.controller.dynamics.target_speed)

            # 检查是否降落完成
            if self.controller.dynamics.current_pose['z'] < 0.07:
                # 停止所有运动
                self.controller.dynamics.target_speed['x'] = 0.0
                self.controller.dynamics.target_speed['z'] = 0.0
                self.controller.publish_speed(self.controller.dynamics.target_speed)
                
                self.controller.state = state.Finish
                self.get_logger().info("降落完成")

                # 发布上锁指令
                data = 'lock'
                msg = String()
                msg.data = data
                self.lock_pub.publish(msg)
                self.get_logger().info("发布上锁")
        
        else:
            pass
    
def main(args=None):
    #开启激光笔
    output_pin = 37
    GPIO.setmode(GPIO.BOARD)
    # 设置为输出模式
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)

    rclpy.init(args=args)
    main = main_ctrl()  # main_ctrl节点，内部包含Controller实例（controller）
    
    # 创建多线程执行器，同时管理main_node和其内部的controller
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(main)
    executor.add_node(main.controller)  # 将Controller节点加入执行器

    main.get_logger().info("---主控制节点已开启---")
    threading.Thread(target=main.run, daemon=True).start()  # 启动线程等待事件
    try:
        executor.spin()  # 用执行器启动事件循环，同时处理两个节点的回调
    except KeyboardInterrupt:
        main.get_logger().info("主控制节点已安全关闭")
    finally: 
        executor.shutdown()
        main.destroy_node()
        main.controller.destroy_node()  # 销毁Controller节点
        rclpy.shutdown()
        GPIO.output(output_pin, GPIO.LOW)
        GPIO.cleanup()

if __name__ == '__main__':
    main()
