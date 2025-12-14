import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'arm3dof'
    pkg_share = get_package_share_directory(pkg_name)



    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )
    

    bridge = Node(
        package='ros_gz_bridge',
        executable= 'parameter_bridge',
        arguments=[
          '/cmd_pos_j1@std_msgs/msg/Float64]gz.msgs.Double',
          '/cmd_pos_j2@std_msgs/msg/Float64]gz.msgs.Double',
          '/cmd_pos_j3@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output ='screen',
        remappings=[
          # ROS Topic -> GZ Topic
          ('/cmd_pos_j1', '/model/arm3dof/joint/j1_base_yaw/cmd_pos'),
          ('/cmd_pos_j2', '/model/arm3dof/joint/j2_base_shoulder/cmd_pos'),
          ('/cmd_pos_j3', '/model/arm3dof/joint/j3_base_elbow/cmd_pos'),
        ]
    )


    controller_node = Node( 
        package='arm3dof',
        executable='move_demo.py',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        bridge,
        controller_node
    ])