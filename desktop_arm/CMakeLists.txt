cmake_minimum_required(VERSION 3.22)
project(desktop_arm)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(moveit_common REQUIRED)
moveit_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  control_toolbox
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  pluginlib
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  tf2_eigen
  trajectory_msgs
)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)
ament_export_include_directories(include)

# Install include, launch, config directories
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY resource DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})


ament_python_install_package(${PROJECT_NAME})

# Install python executables
install(PROGRAMS
    desktop_arm/arduino_driver.py
    desktop_arm/robot_controller.py
    desktop_arm/wireless_ros_server.py
    DESTINATION lib/${PROJECT_NAME}    
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
