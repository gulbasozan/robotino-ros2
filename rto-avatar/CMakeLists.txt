cmake_minimum_required(VERSION 3.8)
project(rto-avatar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


list(APPEND CMAKE_MODULE_PATH "/usr/local/robotino/api2/cmake" "/opt/robotino/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(RobotinoAPI2 REQUIRED)
find_package(rto_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include/rto-avatar
  /usr/local/robotino/api2
  ../rto_node/include/rto_node
)

add_executable(rto_avatar_safe_teleop src/SafeKeyboardTeleop.cpp)
add_executable(rto_avatar_safe_omnidrive src/SafeOmniDriveNode.cpp ../rto_node/src/ComROS.cpp src/SafeOmniDrive.cpp)

ament_target_dependencies(rto_avatar_safe_teleop rclcpp geometry_msgs)
ament_target_dependencies(rto_avatar_safe_omnidrive rclcpp geometry_msgs rto_msgs std_msgs builtin_interfaces sensor_msgs)

target_link_libraries(
  rto_avatar_safe_omnidrive
  ${REC_ROBOTINO_API2_LIBRARY}
)
install(TARGETS
  rto_avatar_safe_teleop
  rto_avatar_safe_omnidrive
  DESTINATION lib/${PROJECT_NAME}
)

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
