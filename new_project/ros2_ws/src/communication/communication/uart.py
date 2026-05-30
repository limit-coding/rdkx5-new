# serial_comm_node.py
from interfaces.msg import Position
import rclpy
from rclpy.node import Node
from std_msgs.msg import  Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String
import serial
import struct
import threading
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
import math

class SerialCommNode(Node):
    def __init__(self):
        super().__init__('serial_comm_node')
        
        # 串口配置（根据实际情况修改）
        self.ser = serial.Serial(
            port='/dev/ttyS1',
            baudrate=115200,
            timeout=1
        )
        
        # 存储最新坐标差值数据（初始化为0）
        self.last_coords = [0.0, 0.0, 0.0, 0.0]
        # 存储最新速度数据（初始化为0）
        self.speed_coords = [0, 0, 0]

        self.coords_lock = threading.Lock()  # 坐标数据访问锁
        self.speed_lock = threading.Lock()  # 坐标数据访问锁
        self.serial_write_lock = threading.Lock()
        self.task_status_lock = threading.Lock()
        self.task_state = 0x01
        self.landing_state = 0x01
        
        self.type_list=[1,4,6,18]

        # 订阅坐标差值数据
        self.create_subscription(
            Position,
            '/pos_pub',
            self.coord_callback,
            10)

        # 订阅坐标数据
        self.create_subscription(
            Position,
            '/speed_pub',
            self.speed_callback,
            10)
        
        #订阅上锁
        self.create_subscription(
            String,
            '/lock',
            self.lock_callback,
            10)

        self.create_subscription(
            Int32MultiArray,
            '/task_status',
            self.task_status_callback,
            10)
        self.task_status_timer = self.create_timer(0.1, self.send_task_status_frame)
        
        # 发布高度数据
        self.height_pub = self.create_publisher(Int32, '/height', 10)#cm   飞控2上位机数据
        #发布解锁
        self.unlock_pub = self.create_publisher(String,'/unlock',10)
        #发布imu
        self.imu_pub = self.create_publisher(Imu,'/IMU',10)
        #发布程序选择
        self.prselect_pub = self.create_publisher(Int32,'/pr_select',10)

        self.buffer = bytearray()

        # 启动串口接收线程
        self.rx_thread = threading.Thread(target=self.serial_rx_task)
        self.rx_thread.daemon = True
        self.rx_thread.start()
        
        self.accel_scale=0.001
        self.gyro_scale=180.0/2000.0

        self.init_yaw=None

        self.get_logger().info("串口通信节点已启动")
#上位机2飞控
    def coord_callback(self, msg):
        """更新最新坐标数据"""
        try:
            with self.coords_lock:
                self.last_coords = [msg.x,msg.y,msg.z,msg.yaw]
                # self.get_logger().info(f"坐标接收: {self.last_coords}")
        except Exception as e:
            self.get_logger().error(f"坐标处理错误: {str(e)}")

        try:
            with self.coords_lock:
                current_coords = self.last_coords.copy()
            
            # 构建基础帧（不含校验码）
            base_frame = bytes([
                0xAA,   # 帧头
                0xFF,   # 目标地址
                0x80,   # 功能码
                16      # 数据长度（4个float32）
            ])
            
            # 打包浮点数据
            data_bytes = struct.pack('<ffff', 
                current_coords[0], 
                current_coords[1], 
                current_coords[2],
                current_coords[3])
            
            # 计算校验码（基础帧 + 数据的累加和）
            checksum = sum(base_frame + data_bytes) & 0xFF  # 取低8位
            
            # 完整帧 = 基础帧 + 数据 + 校验码
            frame = base_frame + data_bytes + bytes([checksum])
            
            # 发送数据
            self.write_serial_frame(frame)
            # self.get_logger().info(
            #     f"定时发送坐标: [{current_coords[0]:.2f}, "
            #     f"{current_coords[1]:.2f}, {current_coords[2]:.2f}]"
            # )
        except Exception as e:
            self.get_logger().error(f"定时发送失败: {str(e)}")

#上位机2飞控
    def speed_callback(self, msg):
        """更新最新坐标数据"""
        try:
            with self.speed_lock:
                self.speed_coords = [msg.x,msg.y,msg.z]
                #self.get_logger().info(f"速度接收: {self.speed_coords}")
        except Exception as e:
            self.get_logger().error(f"速度处理错误: {str(e)}")

        try:
            with self.speed_lock:
                current_coords = self.speed_coords.copy()
            
            # 构建基础帧（不含校验码）
            base_frame = bytes([
                0xAA,   # 帧头
                0xFF,   # 目标地址
                0x83,   # 功能码
                6      # 数据长度（3个s16）
            ])
            
            # 打包数据
            data_bytes = struct.pack('<hhh', 
                round(current_coords[0]), 
                round(current_coords[1]),
                round(current_coords[2]))
            
            # 计算校验码（基础帧 + 数据的累加和）
            checksum = sum(base_frame + data_bytes) & 0xFF  # 取低8位
            
            # 完整帧 = 基础帧 + 数据 + 校验码
            frame = base_frame + data_bytes + bytes([checksum])
            
            # 发送数据
            # self.ser.write(frame)
            # self.get_logger().info(
            #     f"定时发送速度: [{current_coords[0]:.2f}
            #     , {current_coords[1]:.2f},{current_coords[2]:.2f}"
            # )
        except Exception as e:
            self.get_logger().error(f"速度发送失败: {str(e)}")
#上位机2飞控
    def lock_callback(self,msg):
        """订阅上锁"""
        command = msg.data.strip().lower()  # 清理指令
        
        if command == "lock":
            # 构造数据帧（自动处理长度）
            try:
                frame = self._build_frame("lock")  # 即使输入带\0，也会被截断为4字节
                self.write_serial_frame(frame)
                self.get_logger().info(f"发送上锁: {frame.hex(' ')}")
            except Exception as e:
                self.get_logger().error(f"上锁发送错误: {str(e)}")

    def task_status_callback(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warn(f"Invalid task status payload: {list(msg.data)}")
            return

        with self.task_status_lock:
            self.task_state = int(msg.data[0]) & 0xFF
            self.landing_state = int(msg.data[1]) & 0xFF

    def send_task_status_frame(self):
        with self.task_status_lock:
            task_state = self.task_state
            landing_state = self.landing_state

        frame = bytes([0xAA, 0xFF, 0x02, 0x02, task_state, landing_state])
        checksum = sum(frame) & 0xFF
        self.write_serial_frame(frame + bytes([checksum]))

    def write_serial_frame(self, frame):
        with self.serial_write_lock:
            self.ser.write(frame)
        
#飞控2上位机
    def serial_rx_task(self):
        """持续接收串口数据（原有逻辑保持不变）"""
        
        while rclpy.ok():
            try:    
                # 读取所有可用数据
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:       
                    self.buffer += data
                    # print(self.buffer)
                    self.parse_buffer()
                    
            except Exception as e:
                self.get_logger().error(f"串口读取错误: {str(e)}")
                #print(self.buffer)

#解析
    def parse_buffer(self):
        while True:
            start = self.find_header()
            if start < 0:
                if len(self.buffer) > 100:
                    self.buffer = self.buffer[-3:]
                return
            
            if start > 0: 
                self.buffer = self.buffer[start:]
                continue
            
            # 最小帧长度 = 帧头(2) + 类型(1) + 数据长度(1) + 数据(n) + 校验码(1)
            if len(self.buffer) < 4:  # 至少能容纳帧头(2)、类型(1)、数据长度(1)、校验码(1)
                return
            
            # 数据长度 = 第3字节（索引2），总帧长 = 2（固定头） + 数据长度 + 1（校验码）
            data_length = self.buffer[2] 
            total_length = 3 + data_length + 1  # 3=帧头(2)+数据长度(1)
            
            if len(self.buffer) < total_length:
                return
            
            # 提取完整帧（包含校验码）
            packet = self.buffer[:total_length]
            
            # 验证校验码
            frame_without_checksum = packet[:-1]  # 取除校验码外的所有字节
            received_checksum = packet[-1]
            calculated_checksum = sum(frame_without_checksum) & 0xFF
            
            if calculated_checksum != received_checksum:
                self.get_logger().error(f"校验失败: 计算值={calculated_checksum:02X}, 接收值={received_checksum:02X}")
                self.buffer = self.buffer[total_length:]  # 丢弃错误帧
                continue
            
            # 校验通过，处理数据包
            self.handle_packet(packet[:-1])  # 传入不含校验码的帧
            self.buffer = self.buffer[total_length:]
    # def parse_buffer(self):
    #     """解析接收缓冲区"""
    #     while True:
    #         start = self.find_header()
    #         if start < 0:
    #             # 没有找到完整帧头，保留缓冲区
    #             if len(self.buffer) > 100:  # 防止缓冲区过大
    #                 self.buffer = self.buffer[-3:]  # 保留最后两个字节
    #             return
            
    #         # 2. 移除无效数据
    #         if start > 0: 
    #             self.buffer = self.buffer[start:]
    #             continue
            
    #         # 3. 检查最小长度
    #         if len(self.buffer) < 3: return
            
    #         # 4. 获取数据长度
    #         data_length = self.buffer[2]
    #         total_length = 3 + data_length  # 帧头(2)  + 长度(1) + 数据
            
    #         # 5. 检查完整长度
    #         if len(self.buffer) < total_length: return
            
    #         # 6. 提取并处理数据包
    #         packet = self.buffer[:total_length]
    #         self.handle_packet(packet)
            
    #         # 7. 移除已处理数据
    #         self.buffer = self.buffer[total_length:]

    def find_header(self):
        """查找帧头的位置"""
        for i in range(len(self.buffer) - 2):
            if self.buffer[i] == 0xAA and self.buffer[i + 1] == 0xFF and self.buffer[i + 2] in self.type_list:
                return i
        return -1

    def handle_packet(self, packet):
        # hex_str = ' '.join(f'{byte:02X}' for byte in packet)
        # print(hex_str)
        data_type = packet[2]
        data_content = packet[3:]
        
        if data_type == 4 and len(data_content) == 4:  # 高度数据
            try:
                # 解析int32数据（大端格式）
                height = struct.unpack('>i', data_content)[0]
                msg = Int32()
                msg.data = height
                self.height_pub.publish(msg)
                # self.get_logger().info(f"Published height: {height}")
            except Exception as e:
                self.get_logger().error(f"Height parse error: {str(e)}")
                
        elif data_type == 6 and len(data_content) == 6:  # 解锁数据
            try:
                # 解析字符串数据
                unlock_str = data_content.decode('ascii').strip('\x00')
                msg = String()
                msg.data = unlock_str
                self.unlock_pub.publish(msg)
                self.get_logger().info(f"Published unlock: {unlock_str}")
            except Exception as e:
                self.get_logger().error(f"Unlock parse error: {str(e)}")
        elif data_type == 18 and len(data_content) == 18:  # imu数据
            #u8 imu[14];
            # imu[0]=0xFF;
            # imu[1]=0x0C;
            # imu[2]=BYTE0(imu_data.acc_x);
            # imu[3]=BYTE1(imu_data.acc_x);
            # imu[4]=BYTE0(imu_data.acc_y);
            # imu[5]=BYTE1(imu_data.acc_y);
            # imu[6]=BYTE0(imu_data.acc_z);
            # imu[7]=BYTE1(imu_data.acc_z);
            # imu[8]=BYTE0(imu_data.gyr_x);
            # imu[9]=BYTE1(imu_data.gyr_x);
            # imu[10]=BYTE0(imu_data.gyr_y);
            # imu[11]=BYTE1(imu_data.gyr_y);
            # imu[12]=BYTE0(imu_data.gyr_z);
            # imu[13]=BYTE1(imu_data.gyr_z);
            # 	DrvUart3SendBuf(imu,14);
            try:
                # 解析字符串数据
                values = struct.unpack('<9h', data_content)
                msg=Imu()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'imu_link'  # 根据实际坐标系修改
                
                # 填充加速度数据 (转换为m/s²)
                msg.linear_acceleration = Vector3()
                msg.linear_acceleration.x = values[0] * self.accel_scale
                msg.linear_acceleration.y = values[1] * self.accel_scale
                msg.linear_acceleration.z = values[2] * self.accel_scale
                
                # 填充角速度数据 (转换为rad/s)
                # 注意：先转换为度/秒，再转换为弧度/秒
                dps_to_radps = 0.0174533  # π/180
                msg.angular_velocity = Vector3()
                msg.angular_velocity.x = values[3] * self.gyro_scale * dps_to_radps
                msg.angular_velocity.y = values[4]  * self.gyro_scale * dps_to_radps
                msg.angular_velocity.z = values[5]  * self.gyro_scale * dps_to_radps

                orient_pit=values[6]/100
                orient_rol=values[7]/100
                if (self.init_yaw==None):
                    self.init_yaw=values[8]/100
                orient_yaw=values[8]/100-self.init_yaw
                #self.get_logger().info(f"Yaw:{values[8]/100}")
                #self.get_logger().info(f"IMU:{values}",throttle_duration_sec=1)
                msg.orientation=self.euler_to_quaternion(orient_rol,orient_pit,orient_yaw)

                self.imu_pub.publish(msg)
                #self.get_logger().info(f"Published IMU: {msg}")
            except Exception as e:
                self.get_logger().error(f"IMU parse error: {str(e)}")

        elif data_type == 1 :  # 程序数据
            try:
                msg = Int32()
                msg.data = data_content[0]
                self.prselect_pub.publish(msg)
                self.get_logger().info(f"Published programme: {data_content[0]}")
            except Exception as e:
                self.get_logger().error(f"Select parse error: {str(e)}")
        else:
            self.get_logger().warn(f"Unknown data type/length: {data_type}/{len(data_content)},{packet}")

#上位机2飞控
    def _build_frame(self, data_str):
        # 帧头 | 目标地址 | 功能码 | 数据长度 | 数据
        frame_header = 0xAA
        target_address = 0xFF
        func_code = 0x81
        data_part = data_str.encode('ascii')
        data_length = len(data_part)
        
        # 基础帧（不含校验码）
        base_frame = struct.pack('4B', 
            frame_header, 
            target_address, 
            func_code, 
            data_length) + data_part
        
        # 计算校验码
        checksum = sum(base_frame) & 0xFF
        
        # 返回带校验码的完整帧
        return base_frame + bytes([checksum])
    
    def euler_to_quaternion(self,roll_deg, pitch_deg, yaw_deg):
        """
        将欧拉角(roll, pitch, yaw)转换为四元数(ROS格式)
        
        参数:
            roll_deg: 绕X轴的旋转角度(度)
            pitch_deg: 绕Y轴的旋转角度(度)
            yaw_deg: 绕Z轴的旋转角度(度)
        
        返回:
            geometry_msgs.msg.Quaternion 对象
        """
        # 将角度转换为弧度
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)
        
        # 计算半角
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        # 计算四元数分量
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        # 创建并返回ROS四元数对象
        quat = Quaternion()
        quat.x = x
        quat.y = y
        quat.z = z
        quat.w = w
        
        return quat

def main(args=None):
    rclpy.init(args=args)
    node = SerialCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
