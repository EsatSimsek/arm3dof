import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
  pkg_share = get_package_share_directory('arm3dof')

  os.environ["GZ_SIM_RESOURCE_PATH"] = os.path.join(pkg_share, 'models')

  gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )
  
  spawn_model = Node(
    package='ros_gz_sim',
    executable= 'create',
    arguments=[
      '-name', 'arm3dof', '-file', 
      os.path.join(pkg_share, 'models', 'arm3dof', 'model.sdf'),
      '-z', '0.0'
    ], output = 'screen',
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
      ('/model/arm3dof/joint/j1_base_yaw/cmd_pos', '/arm/j1/cmd_pos'),
      ('/model/arm3dof/joint/j2_shoulder/cmd_pos', '/arm/j2/cmd_pos'),
      ('/model/arm3dof/joint/j3_elbow/cmd_pos',    '/arm/j3/cmd_pos'),
    ]
  )

  controller_node = Node( 
    package='arm3dof',
    executable='move_demo.py',
    output='screen'
  )
  
  return LaunchDescription([
    gazebo,
    spawn_model,
    bridge,
    controller_node
  ])