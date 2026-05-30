import rclpy
from rclpy.node import Node
import serial
import threading
from interfaces.msg import PairArray
from interfaces.msg import Pair
from interfaces.msg import Animalcnt
from interfaces.srv import Pathplan
import struct
import time

class BluetoothNode(Node):
    def __init__(self):
        super().__init__('bluetooth_node')
        
        # 串口配置（根据实际情况修改）
        self.ser = serial.Serial(
            port='/dev/ttyS7',
            baudrate=57600,
            timeout=1
        )

        # self.ban_publisher=self.create_publisher(
        #     PairArray,
        #     '/block_pairs',
        #     10)
        self.ban_client = self.create_client(Pathplan, '/pathplan_srv')

        self.animal_sub=self.create_subscription(
            Animalcnt,
            '/animals',
            self.animal_callback,
            10)

        self.start_sub = self.create_subscription(String,'/start',start_callback,10)

        self.buffer = bytearray()

        self.banlist = PairArray()
        self.unsend = True

        self.type_list=[2]

        self.rx_thread = threading.Thread(target=self.serial_rx_task)
        self.rx_thread.daemon = True
        self.rx_thread.start()
        
        self.start_timer=None
        self.timer=None
        self.msg=None
        self.pathcnt=0
        self.get_logger().info("蓝牙节点已启动")

    def traj_callback(self, future):
        try:
            response = future.result()
            if response.if_succeed:
                self.msg=response.path
                self.timer=self.create_timer(0.5,self.send_callback)
                # for i in msg:
                #     try:
                #         # 构建基础帧（不含校验码）
                #         base_frame = bytes([
                #             0xAA,   # 帧头
                #             0xFF,   # 目标地址
                #             0x00,   # 功能码?
                #         ])
                        
                #         # 打包数据
                #         data_bytes = struct.pack('2B', 
                #             i.first, 
                #             i.second)
                        
                #         # 计算校验码（基础帧 + 数据的累加和）
                #         checksum = sum(base_frame + data_bytes) & 0xFF  # 取低8位
                        
                #         # 完整帧 = 基础帧 + 数据 + 校验码
                #         frame = base_frame + data_bytes + bytes([checksum])
                        
                #         # 发送数据
                #         self.ser.write(frame)
                        # self.get_logger().info(
                        #     f"定时发送坐标: [{i.first}, "
                        #     f"{i.second}]"
                        # )
                    # except Exception as e:
                    #     self.get_logger().error(f"航点发送失败: {str(e)}")
            else:
                self.get_logger().warn("路径规划失败")
        except Exception as e:
            self.get_logger().error(f"路径规划服务调用失败: {str(e)}")

    def send_callback(self):
        try:
            if self.pathcnt < len(self.msg):
                # 构建基础帧（不含校验码）
                base_frame = bytes([
                    0xAA,   # 帧头
                    0xFF,   # 目标地址
                    0x00,   # 功能码?
                ])
                
                # 打包数据
                data_bytes = struct.pack('2B', 
                    self.msg[self.pathcnt].first, 
                    self.msg[self.pathcnt].second)
                
                # 计算校验码（基础帧 + 数据的累加和）
                checksum = sum(base_frame + data_bytes) & 0xFF  # 取低8位
                
                # 完整帧 = 基础帧 + 数据 + 校验码
                frame = base_frame + data_bytes + bytes([checksum])
                
                # 发送数据
                self.ser.write(frame)
                self.get_logger().info(
                    f"定时发送坐标: [A{self.msg[self.pathcnt].first+1},B{self.msg[self.pathcnt].second+1}]"
                )

                self.pathcnt+=1
            else:
                if self.timer is not None:
                    self.destroy_timer(self.timer)
                    self.timer = None  # 标记为已销毁，避免重复操作
                    self.get_logger().info("所有航点发送完成，定时器已销毁")
        except Exception as e:
            self.get_logger().error(f"航点发送失败: {str(e)}")

    def animal_callback(self,msg):
        try:
            # 构建基础帧（不含校验码）
            base_frame = bytes([
                0xAA,   # 帧头
                0xFF,   # 目标地址
                0x01,   # 功能码?
            ])
            
            # 打包数据
            data_bytes = struct.pack('7B', 
                msg.pos.first, 
                msg.pos.second,
                msg.animals.data[0],
                msg.animals.data[1],
                msg.animals.data[2],
                msg.animals.data[3],
                msg.animals.data[4])
            
            # 计算校验码（基础帧 + 数据的累加和）
            checksum = sum(base_frame + data_bytes) & 0xFF  # 取低8位
            
            # 完整帧 = 基础帧 + 数据 + 校验码
            frame = base_frame + data_bytes + bytes([checksum])
            
            # 发送数据
            self.ser.write(frame)
            # self.get_logger().info(
            #     f"定时发送坐标: [{i.first}, "
            #     f"{i.second}]"
            # )
        except Exception as e:
            self.get_logger().error(f"动物数量发送失败: {str(e)}")

    def start_callback(self,msg):
        self.start_timer=self.create_timer(1,self.start_timecallback)

    def start_timecallback(self):
        try:
            # 构建基础帧（不含校验码）
            base_frame = bytes([
                0xAA,   # 帧头
                0xFF,   # 目标地址
                0x02,   # 功能码?
            ])
            
            # 计算校验码（基础帧 + 数据的累加和）
            checksum = sum(base_frame) & 0xFF  # 取低8位
            
            # 完整帧 = 基础帧 + 数据 + 校验码
            frame = base_frame + bytes([checksum])
            
            # 发送数据
            self.ser.write(frame)
            self.get_logger().info(f"启动完成...")
        except Exception as e:
            self.get_logger().error(f"启动发送失败: {str(e)}")


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
                self.get_logger().error(f"蓝牙串口读取错误: {str(e)}，尝试重新连接...")
                try:
                    self.ser.close()
                    self.ser.open()  # 尝试重新打开串口
                except Exception as re:
                    self.get_logger().error(f"串口重连失败: {str(re)}")
#解析
    def parse_buffer(self):
        while True:
            start = self.find_header()
            if start < 0:
                if len(self.buffer) > 100:
                    self.buffer = self.buffer[-20:]
                return
            
            if start > 0: 
                self.buffer = self.buffer[start:]
                continue
            
            # 最小帧长度 = 帧头(2) + 数据长度(1) + 数据(n) + 校验码(1)
            if len(self.buffer) < 4:  # 至少能容纳帧头(2)、数据长度(1)、校验码(1)
                return
            
            # 数据长度 = 第3字节（索引2），总帧长 = 2（固定头） + 数据长度 + 1（校验码）
            data_length = self.buffer[2] 
            total_length = 3 + data_length + 1  # 3=帧头(2) +数据长度(1)
            
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
        
        if data_type == 2 and len(data_content) == 2:  # 禁飞区数据
            try:
                # 解析u8数据（大端格式）
                xy = struct.unpack('2B', data_content)

                newban = Pair()
                newban.first = xy[0]
                newban.second = xy[1]
                self.banlist.pairs.append(newban)
                self.get_logger().info(f"收到禁飞区点: ({newban.first}, {newban.second})，当前总数: {len(self.banlist.pairs)}")
                
                # 收集到3个禁飞区点后，发送服务请求
                if len(self.banlist.pairs) >= 3 and self.unsend:
                    self.get_logger().info("已收集3个禁飞区点，发送路径规划请求...")
                    self.send_pathplan_request()
                    self.banlist = PairArray()  # 重置列表
                    self.unsend = False

                    self.destroy_timer(self.start_timer)
                    self.start_timer=None


            except Exception as e:
                self.get_logger().error(f"Block parse error: {str(e)}")

        else:
            self.get_logger().warn(f"Unknown data type/length: {data_type}/{len(data_content)},{packet}")

    def send_pathplan_request(self, retry=False):
        """发送路径规划服务请求"""
        # 非重试调用时重置重试计数器
        if not retry:
            self.current_retry = 0
        
        # 检查服务是否就绪
        if not self.ban_client.service_is_ready():
            self.get_logger().info("等待路径规划服务启动...")
            
            # 循环等待服务启动
            while not self.ban_client.service_is_ready():
                time.sleep(0.5)
        
        # 服务已就绪，创建请求对象
        request = Pathplan.Request()
        request.blocks = self.banlist.pairs
        
        # 发送异步请求
        try:
            future = self.ban_client.call_async(request)
            future.add_done_callback(self.traj_callback)
            self.get_logger().info(f"路径规划请求已发送")
        except Exception as e:
            self.get_logger().error(f"发送请求失败: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = BluetoothNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        self.ser.close()
        self.get_logger().info("蓝牙串口已关闭")

if __name__ == "__main__":
    main()