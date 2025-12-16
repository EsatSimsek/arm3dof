import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'arm3dof'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Kaynak Yolu Ayarı (Mesh ve Xacro dosyaları için)
    # ls çıktısına göre dosya yapısı: install/arm3dof/share/arm3dof/arm3dof/...
    # Bu yüzden GZ_SIM_RESOURCE_PATH'e 'arm3dof' alt klasörünü ekliyoruz.
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_share, 'arm3dof')]
    )

    # 2. Gazebo'yu Başlat (Boş Dünya ile)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 
                         'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 3. Robot State Publisher (Xacro İşleme)
    # Dosya yolu daha önceki ls çıktına göre doğrulandı: share/arm3dof/arm3dof/model.urdf.xacro
    xacro_file = os.path.join(pkg_share, 'arm3dof', 'model.urdf.xacro')
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', xacro_file])}]
    )

    # 4. Gazebo - ROS Köprüsü (Bridge)
    # Jazzy/Harmonic için en kararlı formatı kullanıyoruz.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        arguments=[
            # --- KOMUTLAR (ROS -> Gazebo) ---
            # std_msgs/Float64 -> gz.msgs.Double
            '/cmd_pos_j1@std_msgs/msg/Float64]gz.msgs.Double',
            '/cmd_pos_j2@std_msgs/msg/Float64]gz.msgs.Double',
            '/cmd_pos_j3@std_msgs/msg/Float64]gz.msgs.Double',

            # --- GERİ BİLDİRİM (Gazebo -> ROS) ---
            # gz.msgs.Model -> sensor_msgs/JointState (JointState hatasını aşmak için Model kullanıyoruz)
            # Not: Tam JointState eşleşmesi hata veriyorsa, en azından saati köprülemek gerekir.
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    # 5. Robotu Spawn Et
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'arm3dof',
            '-z', '0.1'
        ],
        output='screen'
    )

    # 6. Hareket Kontrol Düğümü (Senin python kodun)
    move_demo = Node(
        package='arm3dof',
        executable='move_demo.py',
        name='move_demo',
        output='screen'
    )

    return LaunchDescription([
        set_resource_path,
        gazebo,
        robot_state_publisher,
        bridge,
        spawn,
        move_demo
    ])