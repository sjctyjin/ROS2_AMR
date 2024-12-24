import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile


class Nav2GoalPublisher(Node):
    def __init__(self):
        super().__init__('nav2_goal_publisher')
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', qos_profile)
        self.timer = self.create_timer(2.0, self.publish_goal)  # 设定时间间隔

    def publish_goal(self):
        goal_pose = PoseStamped()
        
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = 'map'  # 确保使ls用的坐标系正确
        goal_pose.pose.position.x = 2.0  # 指定x坐标
        goal_pose.pose.position.y = 3.0  # 指定y坐标
        goal_pose.pose.orientation.z = 0.0  # 方向朝北
        goal_pose.pose.orientation.w = 1.0  # 无旋转

        self.publisher.publish(goal_pose)
        self.get_logger().info('Publishing: "%s"' % goal_pose)
        
def main(args=None):
    rclpy.init(args=args)
    nav2_goal_publisher = Nav2GoalPublisher()
    rclpy.spin(nav2_goal_publisher)
    nav2_goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

