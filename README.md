# arm3dof — Quick run (Gazebo Harmonic + ROS 2 Jazzy)

## Önkoşullar
- ROS 2 Jazzy (ros2)
- ros_gz_sim, ros_gz_bridge (kurulu ve PATH’te)
- colcon, ament_cmake

## Klon / build
# (repo'yu buraya yerleştir veya kopyala)
# örn: git clone <repo-url> .
cd ~/arm_ws
colcon build
source install/setup.bash

## Başlatma
# 1) Launch dosyasını çalıştır
ros2 launch arm3dof arm.launch.py

# Bu launch:
# - gz sim başlatır (empty world)
# - modeli spawn eder (models/arm3dof/model.sdf)
# - ros_gz_bridge ile /cmd_pos_j1, /cmd_pos_j2, /cmd_pos_j3 topiclerini köprüler
# - move_demo.py node'u başlatır (otomatik olarak A->B->C pozlarına geçer)
