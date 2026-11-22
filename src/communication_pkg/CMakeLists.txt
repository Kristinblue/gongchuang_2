cmake_minimum_required(VERSION 3.8)
project(communication_pkg)
set(CMAKE_CXX_STANDARD 17)
# ament_cmake 是 ROS 2 的 CMake 基础设施；ament_cmake_python 用来安装 Python 包。
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# rclcpp 是 C++ 的 ROS 客户端库；robot_interfaces 是本仓自定义的 Command/Feedback 等消息。
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(serial_driver REQUIRED)
# 串口节点由三个 .cpp 组成：节点本身、串口封装、协议打包。
add_executable(serial_node src/serial_node.cpp src/my_uart.cpp src/my_ct_protocol.cpp)
# include/ 目录里放 my_uart.hpp 和 my_ct_protocol.hpp。
target_include_directories(serial_node PRIVATE include)
ament_target_dependencies(serial_node rclcpp robot_interfaces serial_driver)
# 可执行文件装到 install/communication_pkg/lib/communication_pkg/，ros2 run 从这里找。
install(TARGETS serial_node DESTINATION lib/${PROJECT_NAME})
# 安装 Python 包（my_cmd.py），任务节点用 from communication_pkg.my_cmd import MyCMD 引用。
ament_python_install_package(${PROJECT_NAME})
ament_package()
