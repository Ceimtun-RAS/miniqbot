cmake_minimum_required(VERSION 3.8)
project(dynamixel_control_cpp)

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

# 2) Build executables from your sources
add_executable(publisher_node src/publisher.cpp)
ament_target_dependencies(publisher_node rclcpp std_msgs)

add_executable(subscriber_node src/subscriber.cpp)
ament_target_dependencies(subscriber_node rclcpp std_msgs)

add_executable(demo_ax12 src/demo_ax12.cpp)
ament_target_dependencies(demo_ax12 dynamixel_sdk)

add_executable(monitor src/monitor.cpp)
ament_target_dependencies(monitor dynamixel_sdk)

# --- Nodos ROS 2 ---
add_executable(dynamixel_node src/dynamixel_node.cpp)
ament_target_dependencies(dynamixel_node rclcpp sensor_msgs dynamixel_sdk)

add_executable(monitor_node src/monitor_node.cpp)
ament_target_dependencies(monitor_node rclcpp sensor_msgs)

add_executable(demo_node src/demo_node.cpp)
ament_target_dependencies(demo_node rclcpp sensor_msgs)

# 3) (Optional) If you later add headers under include/dynamixel_control_cpp/*.hpp
#    this tells targets where to look for your headers
target_include_directories(publisher_node PUBLIC include)
target_include_directories(subscriber_node PUBLIC include)

# 4) Install binaries so `ros2 run` can find them
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

# 5) Testing/linting (safe to keep as-is)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 6) Register with ament
ament_package()
