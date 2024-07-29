cmake_minimum_required(VERSION 3.8)
project(viro_simple_fleet)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vda5050_msgs REQUIRED)
find_package(vda5050_connector REQUIRED)

install(
  DIRECTORY
    config
    maps
    launch
    scripts
  DESTINATION
    share/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables     --- make sure to chmod +x XYZ.py
install(
  PROGRAMS
    scripts/fleet_mngr_main.py
    scripts/fleet_client_main.py
    scripts/fleet_client_monitor.py
    scripts/fleet_client_navigator.py
  DESTINATION
    lib/${PROJECT_NAME})

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
