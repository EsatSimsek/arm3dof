# launch/arm_load_xacro.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('arm3dof')
    
    # 1. XACRO DOSYASININ YOLU
    xacro_file = os.path.join(pkg_share, 'models', 'arm3dof', 'model.urdf.xacro')

    # 2. ROBOT STATE PUBLISHER (Xacro'yu işler)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            # Xacro'yu Command ile işleyip sonucu robot_description topic'ine yayınlar
            'robot_description': Command(['xacro ', xacro_file]) 
        }]
    )

    # 3. ROBOTU SPAWN ET (Topic üzerinden)
    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', # RSP'den gelen URDF'yi kullan
            '-name', 'arm3dof',
            '-z', '0.05' 
        ], 
        output='screen',
    )
    
    # 4. Kontrol Düğümü
    # move_demo.py'nin src/arm3dof/lib/arm3dof içinde çalışır durumda olması gerekir.
    controller_node = Node(
        package='arm3dof',
        executable='move_demo.py',
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        spawn_model,
        controller_node
    ])