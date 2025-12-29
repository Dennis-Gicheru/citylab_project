cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find the required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 2. Add the executable (ensure your file is in src/patrol.cpp)
add_executable(patrol_node src/patrol.cpp)

# 3. Link the dependencies to the executable
ament_target_dependencies(patrol_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs
)

# 4. Install the executable so 'ros2 run' can find it
install(TARGETS
  patrol_node
  DESTINATION lib/${PROJECT_NAME}
)

# 5. Install the launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()