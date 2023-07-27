cmake_minimum_required(VERSION 3.5)
project(robot_control)

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

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(geometry_msgs REQUIRED)
find_package(Threads REQUIRED) # Find pthreads

# Include directories
include_directories(include ${Boost_INCLUDE_DIRS})

add_executable(motor_controller src/motor_controller.cpp)
ament_target_dependencies(motor_controller rclcpp std_msgs geometry_msgs)

add_executable(motor_datasender src/motor_datasender.cpp)
target_link_libraries(motor_datasender ${CMAKE_THREAD_LIBS_INIT}) # Link pthreads

target_link_libraries(motor_controller ${Boost_LIBRARIES})

install(TARGETS
  motor_controller
  motor_datasender
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
