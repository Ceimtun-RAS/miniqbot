cmake_minimum_required(VERSION 3.8)
project(dynamixel_control_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Good warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1) Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

set(DYNAMIXEL_CPP_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

# 2) Build executables from your sources
add_executable(publisher_node src/publisher.cpp)
ament_target_dependencies(publisher_node rclcpp std_msgs)
target_include_directories(publisher_node PUBLIC include)

add_executable(subscriber_node src/subscriber.cpp)
ament_target_dependencies(subscriber_node rclcpp std_msgs)
target_include_directories(subscriber_node PUBLIC include)

add_executable(demo_ax12 src/demo_ax12.cpp)
ament_target_dependencies(demo_ax12 dynamixel_sdk)

add_executable(monitor src/monitor.cpp)
ament_target_dependencies(monitor dynamixel_sdk)

# --- Nodos ROS 2 ---
add_executable(dynamixel_node src/dynamixel_node.cpp)
ament_target_dependencies(dynamixel_node rclcpp sensor_msgs dynamixel_sdk)
target_include_directories(dynamixel_node PRIVATE ${DYNAMIXEL_CPP_INCLUDE_DIR})

add_executable(monitor_node src/monitor_node.cpp)
ament_target_dependencies(monitor_node rclcpp sensor_msgs)
target_include_directories(monitor_node PRIVATE ${DYNAMIXEL_CPP_INCLUDE_DIR})

add_executable(demo_node src/demo_node.cpp)
ament_target_dependencies(demo_node rclcpp sensor_msgs)
target_include_directories(demo_node PRIVATE ${DYNAMIXEL_CPP_INCLUDE_DIR})

# 3) Install
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

install(TARGETS
  demo_ax12
  monitor
  dynamixel_node
  monitor_node
  demo_node
  publisher_node
  subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

# 4) Tests (core logic in headers; no sleeps)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ax12_model test/test_ax12_model.cpp)
  target_include_directories(test_ax12_model PRIVATE ${DYNAMIXEL_CPP_INCLUDE_DIR})
  ament_target_dependencies(test_ax12_model sensor_msgs)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
