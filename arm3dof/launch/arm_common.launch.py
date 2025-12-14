# launch/arm_common.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('arm3dof')
    
    # 1. Gazebo'yu başlatma
    # arm_world.sdf dosyanızın models/ klasörünüzle aynı seviyede olduğu varsayılır.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 
                         'launch', 'gz_sim.launch.py')
        ),
        # world/arm_world.sdf dosyanızın yolu
        launch_arguments={'gz_args': [' -r ', os.path.join(pkg_share, 'worlds', 'arm_world.sdf')]}.items()
    )

    # 2. Gazebo (Ignition) - ROS 2 Bridge (Köprü)
    # Köprüleri manuel olarak tanımlar.
    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Robotun kontrolü için (ROS -> GZ):
            # ROS Topic: /cmd_pos_j1 (std_msgs/msg/Float64)
            # GZ Topic:  /cmd_pos_j1 (gz.msgs.Double)
            '/cmd_pos_j1@std_msgs/msg/Float64[gz.msgs.Double',
            '/cmd_pos_j2@std_msgs/msg/Float64[gz.msgs.Double',
            '/cmd_pos_j3@std_msgs/msg/Float64[gz.msgs.Double',
            
            # Gazebo'dan eklem durumunu almak için (GZ -> ROS):
            # GZ Topic:  /world/arm_world/model/arm3dof/joint_state (gz.msgs.Model)
            # ROS Topic: /joint_states (sensor_msgs/msg/JointState)
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            
            # Robotun konumunu ROS'a yayınlamak için (GZ -> ROS):
            '/model/arm3dof/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose',
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        parameter_bridge,
    ])