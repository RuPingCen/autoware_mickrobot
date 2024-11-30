cmake_minimum_required(VERSION 3.8)
project(mick_robot_vehicle_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(CMAKE_INSTALL_RPATH /usr/local/lib)
find_package(serial REQUIRED)

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


ament_auto_add_executable(mick_robot_vehicle_interface
  src/mick_robot_vehicle_interface/mick_robot_vehicle_interface.cpp
  src/mick_robot_vehicle_interface/mick_robot_vehicle_interface_node.cpp
  src/mick_robot_vehicle_interface/mick_chassis_protocol.cpp
  # src/async_serial/AsyncSerial.cpp
  # src/async_serial/BufferedAsyncSerial.cpp
)

# ament_package()
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)

