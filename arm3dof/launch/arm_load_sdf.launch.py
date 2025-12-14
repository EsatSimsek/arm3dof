# launch/arm_load_sdf.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('arm3dof')
    
    # 1. ROBOTU SPAWN ET (Dosya üzerinden)
    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            # DÜZELTME: Yolu, GZ_SIM_RESOURCE_PATH (models/ klasörü) içinden göreli veriyoruz.
            '-file', os.path.join('arm3dof', 'model.sdf'), 
            '-name', 'arm3dof',
            '-z', '0.05'
        ], 
        output='screen',
    )
    
    return LaunchDescription([
        spawn_model
    ])