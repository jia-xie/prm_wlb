cmake_minimum_required(VERSION 3.5)
project(prm_hardware)

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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) # ROS C++
find_package(prm_interfaces REQUIRED) # purduerm interface (including srv msgs)

include_directories(include)
link_directories(lib)

add_executable(MotorCtrlNode 
  src/MotorCtrlNode.cpp
  src/UARTPort.cpp
  src/RemoteDR16Driver.cpp
)

target_link_libraries(MotorCtrlNode libUnitreeMotorSDK_Arm64.so)

ament_target_dependencies(MotorCtrlNode
  rclcpp
  prm_interfaces
)

set_target_properties(MotorCtrlNode PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH TRUE
)

install (
  TARGETS MotorCtrlNode
  DESTINATION lib/${PROJECT_NAME}
)
install(
  FILES lib/libUnitreeMotorSDK_Arm64.so
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
