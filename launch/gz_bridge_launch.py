from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # ros_gz_bridge 노드 실행
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='thruster_bridge',
            arguments=[
                '/box_usv/thruster_L@std_msgs/msg/Float64@ignition.msgs.Double',
                '/box_usv/thruster_R@std_msgs/msg/Float64@ignition.msgs.Double',
                '/world/my_world/model/box_usv/link/camera_link/sensor/camera_sensor/image@sensor_msgs/msg/Image@ignition.msgs.Image'
            ],
            output='screen'
        ),

        # obstacle 노드 실행
        Node(
            package='final_project',
            executable='obstacle',
            name='obstacle',
            output='screen'
        ),

        # 이그니션(Gazebo) 시뮬레이터 실행
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '4', 'my_world.sdf'],
            output='screen'
        )
    ])
