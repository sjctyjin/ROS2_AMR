from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
        	parameters=[
        		get_package_share_directory("huanyu_robot_start") + '/param/mapper_params_online_sync.yaml'
        	],
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
        )
    ])
