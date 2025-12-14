# launch/arm_xacro_run.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('arm3dof')
    
    # 1. 🚨 KRİTİK: Gazebo'ya model ve kaynak yolunu bildir
    # Bu, Gazebo'nun model dosyasını (içindeki texture, mesh vb.) bulmasını sağlar.
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_share, 'models')]
    )

    # 2. Ortak bileşenleri yükle (Gazebo ve Bridge)
    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'arm_common.launch.py')) 
    )

    # 3. Xacro yüklemesini, spawn etmeyi ve kontrolcüyü yükle
    xacro_load_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'arm_load_xacro.launch.py'))
    )

    return LaunchDescription([
        set_resource_path, 
        common_launch,
        xacro_load_launch
    ])