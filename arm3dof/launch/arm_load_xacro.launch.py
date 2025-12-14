# launch/arm_load_xacro.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('arm3dof')

    # 1. XACRO DOSYASININ YOLU
    # DÜZELTME: CMake'in 'models/arm3dof/' klasörünü kopyalarken klasör yapısını bozduğu varsayılır.
    # En olası durum, Xacro dosyasının 'pkg_share/models/arm3dof/model.urdf.xacro' yerine 
    # 'pkg_share/model.urdf.xacro' veya 'pkg_share/models/model.urdf.xacro' olmasıdır.
    # En güvenli, en basit yolu deneyelim:
    xacro_file = os.path.join(pkg_share, 'models', 'arm3dof', 'model.urdf.xacro')

    # **ÖNEMLİ:** Eğer bu yol hata vermeye devam ederse, yukarıdaki satırı şununla değiştirin:
    # xacro_file = os.path.join(pkg_share, 'model.urdf.xacro')
    
    # 2. ROBOT STATE PUBLISHER (Xacro'yu işler)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
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
    
    return LaunchDescription([
        robot_state_publisher_node,
        spawn_model
    ])