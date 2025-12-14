from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('arm3dof')
    
    # 1. Gazebo'ya model yolunu bildir
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_share, 'models')]
    )

    # 2. Ortak bileşenleri yükle
    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'arm_common.launch.py'))
    )

    # 3. SDF yükle ve robotu spawn et
    sdf_load_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'arm_load_sdf.launch.py'))
    )

    return LaunchDescription([
        set_resource_path, # Çevresel değişkeni tanımla
        common_launch,
        sdf_load_launch
    ])