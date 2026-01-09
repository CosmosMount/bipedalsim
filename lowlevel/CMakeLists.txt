cmake_minimum_required(VERSION 3.8)
project(lowlevel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find message generation dependencies
find_package(rosidl_default_generators REQUIRED)

# Include directories
include_directories(include/lowlevel)

# Message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorFdb.msg"
  "msg/RobotFdb.msg"
  "msg/IMU.msg"
  DEPENDENCIES std_msgs
)

# Executable targets
add_executable(keyboard_controller src/keyboard_controller.cpp)
ament_target_dependencies(keyboard_controller
  rclcpp
  std_msgs
)

add_executable(lowlevel_msg src/lowlevel.cpp)
ament_target_dependencies(lowlevel_msg
  rclcpp
  std_msgs
  sensor_msgs
  message_filters
)

# Link custom messages to executables
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(lowlevel_msg "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  keyboard_controller
  lowlevel_msg
  DESTINATION lib/${PROJECT_NAME}
)

# # Install launch files
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
