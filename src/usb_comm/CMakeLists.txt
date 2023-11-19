cmake_minimum_required(VERSION 3.8)
project(usb_comm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(usb_comm src/serial_comm_node.cpp)
ament_target_dependencies(usb_comm rclcpp)

install(TARGETS
  usb_comm
  DESTINATION lib/${PROJECT_NAME})

ament_package()
