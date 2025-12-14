cmake_minimum_required(VERSION 3.16)
project(arm3dof)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# 🚨 DÜZELTME: Klasör içeriğinin doğru kopyalanması için sonda eğik çizgi (/) eklenmelidir.
# Bu, 'models/' ve 'launch/' klasörlerinin içeriğini koruyarak 'share/arm3dof/' altına kopyalar.
install(DIRECTORY
  models/
  launch/
  DESTINATION share/${PROJECT_NAME}/
  PATTERN ".git" EXCLUDE # .git gibi dosyaları hariç tutar
)

# Python scripti için doğru kurulum
install(PROGRAMS
  src/move_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()