# launch/arm_sdf_run.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('arm3dof')

    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'arm_common.launch.py'))
    )

    sdf_load_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'arm_load_sdf.launch.py'))
    )

    return LaunchDescription([
        common_launch,
        sdf_load_launch
    ])