import pymssql
import rclpy
import time
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage,Image   # 图像消息类型
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu

def send_to_sql(data, table_name):
    conn = pymssql.connect(server='192.168.100.24', user='sa', password='pass', database='ROS2')
    cursor = conn.cursor()
    #sql = f"UPDATE {table_name} SET position_x = '(%s)', position_y = '(%s)', position_z = '(%s)',position_pitch = '(%s)',position_yaw = '(%s)',position_roll = '(%s)',T99 = '(%s)'"
    """sql = fUPDATE {table_name} SET position_x = '{data['position_x']}',position_y = '{data['position_y']}',position_z =     '{data['position_z']}',position_pitch='{data['position_pitch']}',position_yaw='{data['position_yaw']}',position_roll='{data['position_roll']}',
    linear_x = '{data['linear_x']}',
    linear_y = '{data['linear_y']}',
    linear_z = '{data['linear_z']}',
    angular_x = '{data['angular_x']}',
    angular_y = '{data['angular_y']}',
    angular_z = '{data['angular_z']}',
    image_data = (%s),T99 = '{time.strftime('%Y-%m-%d %H:%M:%S')}';"""
   
    sql = f"""INSERT INTO {table_name} (position_x, position_y, position_z, position_pitch, position_yaw, position_roll,
                                           linear_x, linear_y, linear_z, angular_x, angular_y, angular_z,angular_velocity_x,angular_velocity_y,angular_velocity_z,right_front_wheel,left_front_wheel,left_rear_wheel,right_rear_wheel, T99)
                  VALUES ('{data['position_x']}', '{data['position_y']}', '{data['position_z']}',
                          '{data['position_pitch']}', '{data['position_yaw']}', '{data['position_roll']}',
                          '{data['linear_x']}', '{data['linear_y']}', '{data['linear_z']}',
                          '{data['angular_x']}', '{data['angular_y']}', '{data['angular_z']}',
                          {data['angular_velocity_x']},{data['angular_velocity_y']},{data['angular_velocity_z']},
                          {data['right_front_wheel']},{data['left_front_wheel']},{data['left_rear_wheel']},{data['right_rear_wheel']},
                          '{time.strftime('%Y-%m-%d %H:%M:%S')}');"""
    print(sql)
    #cursor.execute(sql,(data['position_x'],data['position_y'],data['position_z'],time.strftime('%Y-%m-%d %H:%M:%S')))
    cursor.execute(sql)
    conn.commit()
    conn.close()

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        #self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        #self.odom_data = None        
        #self.timer = self.create_timer(0.5, self.timer_callback)
        
        self.subscriptions_topic = []
        self.data_store = {}
        self.bridge = CvBridge()
        #topics = ['/odom', 'image_raw/compressed','/cmd_vel']  # 訂閱的topics列表
        topics = ['/odom','/cmd_vel','/mobile_base/sensors/imu_data_raw','/robot/MotoEncoder']  # 訂閱的topics列表
        for topic in range(len(topics)):
            if topic == 0:
                sub = self.create_subscription(Odometry, topics[topic],lambda msg,topic=topics[topic]: self.odom_callback(msg, topic), 10)
                self.subscriptions_topic.append(sub)
            
            elif topic == 1:
                sub = self.create_subscription(Twist,topics[topic],lambda msg,topic=topics[topic]: self.twist_callback(msg, topic), 10)
                self.subscriptions_topic.append(sub)
            elif topic == 2:
                sub = self.create_subscription(Imu,topics[topic],lambda msg,topic=topics[topic]: self.imu_callback(msg, topic), 10)
                self.subscriptions_topic.append(sub)
            elif topic == 3:
                sub = self.create_subscription(Float32MultiArray,topics[topic],lambda msg,topic=topics[topic]: self.encoder_callback(msg, topic), 10)
                self.subscriptions_topic.append(sub)
            elif topic == 4:
                sub = self.create_subscription(CompressedImage,topics[topic],lambda msg,topic=topics[topic]: self.listener_callback(msg, topic), 10)
                self.subscriptions_topic.append(sub)
                
        self.timer = self.create_timer(1, self.timer_callback)
        

    def odom_callback(self, msg,topic):
        self.data_store[topic] = msg
        #print(msg)
    	#self.odom_data = msg
        #data = {
        #    'position_x': msg.pose.pose.position.x,
        #    'position_y': msg.pose.pose.position.y,
        #    'position_z': msg.pose.pose.position.z
        #}
        #send_to_sql(data, 'ODOM_LOG')
        self.get_logger().info(f'position_x: {msg.pose.pose.position.x},position_y : {msg.pose.pose.position.y},position_z : {msg.pose.pose.position.z}') 
    
    def twist_callback(self, msg,topic):
        self.data_store[topic] = msg
        #linear_x = msg.linear.x
        #linear_y = msg.linear.y
        #linear_z = msg.linear.z
        self.get_logger().info(f'linear_x: {msg.linear.x},linear_y : {msg.linear.y},linear_z : {msg.linear.z}')

    def listener_callback(self, msg,topic):
        # 將ROS消息中的二進制圖像數據保存到數據庫中
        self.data_store[topic] = np.frombuffer(msg.data, np.uint8).tobytes()
        #print(np.frombuffer(msg.data, np.uint8).tobytes())
        #np_arr = np.frombuffer(msg.data, np.uint8)
    	# 解碼圖像數據

        #binary_data = np_arr.tobytes()
        #self.data_store[topic] = binary_data
    def imu_callback(self, msg,topic):
        self.data_store[topic] = msg
        # 提取 angular_velocity 的 x, y, z 值
        #angular_velocity_x = msg.angular_velocity.x
        #angular_velocity_y = msg.angular_velocity.y
        #angular_velocity_z = msg.angular_velocity.z

        # 輸出 x, y, z 的角速度數據
        #self.get_logger().info(f'Angular Velocity X: {angular_velocity_x}')
        #self.get_logger().info(f'Angular Velocity Y: {angular_velocity_y}')
        #self.get_logger().info(f'Angular Velocity Z: {angular_velocity_z}')

    def encoder_callback(self, msg,topic):
        self.data_store[topic] = msg
        # 假設數據結構為 [右前輪, 左前輪, 左後輪, 右後輪]
        right_front_wheel = msg.data[0]
        left_front_wheel = msg.data[1]
        left_rear_wheel = msg.data[2]
        right_rear_wheel = msg.data[3]

        # 輸出輪子數值
        #self.get_logger().info(f'Right Front Wheel: {right_front_wheel}')
        #self.get_logger().info(f'Left Front Wheel: {left_front_wheel}')
        #self.get_logger().info(f'Left Rear Wheel: {left_rear_wheel}')
        #self.get_logger().info(f'Right Rear Wheel: {right_rear_wheel}')
    def timer_callback(self):
        print("g0 j2")
        # 每0.5秒處理一次數據
        if self.data_store != {}:
            #print(self.data_store)
            
            orientation_q = self.data_store['/odom'].pose.pose.orientation
            quaternion = (
                orientation_q.x,
                orientation_q.y,
                orientation_q.z,
                orientation_q.w
            )

            # 将四元数转换为欧拉角
            euler = R.from_quat(quaternion).as_euler('xyz', degrees=False)
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]
            
            data = {
                'position_x': self.data_store['/odom'].pose.pose.position.x,
                'position_y': self.data_store['/odom'].pose.pose.position.y,
                'position_z': self.data_store['/odom'].pose.pose.position.z,
                'position_pitch' : pitch,
                'position_yaw'   : yaw,
                'position_roll'   : roll,
                'linear_x': self.data_store['/cmd_vel'].linear.x if '/cmd_vel' in self.data_store and self.data_store['/cmd_vel'] is not None else 0.0,
                'linear_y': self.data_store['/cmd_vel'].linear.y if '/cmd_vel' in self.data_store and self.data_store['/cmd_vel'] is not None else 0.0,
                'linear_z': self.data_store['/cmd_vel'].linear.z if '/cmd_vel' in self.data_store and self.data_store['/cmd_vel'] is not None else 0.0,
                'angular_x': self.data_store['/cmd_vel'].angular.x if '/cmd_vel' in self.data_store and self.data_store['/cmd_vel'] is not None else 0.0,
                'angular_y': self.data_store['/cmd_vel'].angular.y if '/cmd_vel' in self.data_store and self.data_store['/cmd_vel'] is not None else 0.0,
                'angular_z': self.data_store['/cmd_vel'].angular.z if '/cmd_vel' in self.data_store and self.data_store['/cmd_vel'] is not None else 0.0,
                'angular_velocity_x':self.data_store['/mobile_base/sensors/imu_data_raw'].angular_velocity.x,
                'angular_velocity_y':self.data_store['/mobile_base/sensors/imu_data_raw'].angular_velocity.y,
                'angular_velocity_z':self.data_store['/mobile_base/sensors/imu_data_raw'].angular_velocity.z,
                'right_front_wheel':self.data_store['/robot/MotoEncoder'].data[0],
                'left_front_wheel':self.data_store['/robot/MotoEncoder'].data[2],
                'left_rear_wheel':self.data_store['/robot/MotoEncoder'].data[4],
                'right_rear_wheel':self.data_store['/robot/MotoEncoder'].data[6]

                #'image_data': self.data_store['image_raw/compressed']
            }
            send_to_sql(data, 'ODOM_LOG')
            self.get_logger().info(f"position_x: {data['position_x']}, position_y: {data['position_y']}, position_z: {data['position_z']}")
            # 處理完後清除數據
            self.odom_data = None

def main(args=None):
    rclpy.init(args=args)
    data_collector = DataCollector()
    rclpy.spin(data_collector)
    data_collector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
