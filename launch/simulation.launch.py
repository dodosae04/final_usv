from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    world = os.path.join(
        get_package_share_directory('final_project'), 'worlds', 'my_world.sdf')
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', world],
            output='screen'),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/my_world/model/box_usv/link/camera_link/sensor/camera_sensor/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/box_usv/thruster_R@std_msgs/msg/Float64@gz.msgs.Double',
                '/box_usv/thruster_L@std_msgs/msg/Float64@gz.msgs.Double'],
            output='screen'),
        Node(
            package='final_project',
            executable='red_circle_avoider.py',
            output='screen'),
    ])
