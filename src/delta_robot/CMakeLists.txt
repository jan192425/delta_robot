cmake_minimum_required(VERSION 3.8)
project(delta_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(delta_robot_interfaces REQUIRED)
find_package(geometry_msgs)

#executables
add_executable(commot_node src/commot_node.cpp)
ament_target_dependencies(commot_node delta_robot_interfaces rclcpp dynamixel_sdk dynamixel_sdk_custom_interfaces)

add_executable(opmod_node src/opmod_node.cpp src/kinematics.cpp)
ament_target_dependencies(opmod_node delta_robot_interfaces rclcpp dynamixel_sdk dynamixel_sdk_custom_interfaces geometry_msgs)

# Install targets
install(TARGETS commot_node opmod_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files (falls nötig)
install(DIRECTORY include/
  DESTINATION include
)

target_include_directories(opmod_node 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(commot_node 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
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
