cmake_minimum_required(VERSION 2.8.3)
project(academy_tools)

find_package(catkin REQUIRED COMPONENTS academy_lib academy_msgs roscpp sensor_msgs visualization_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_joint_marker src/joint_marker.cpp)
target_link_libraries(${PROJECT_NAME}_joint_marker ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_joint_marker ${catkin_EXPORTED_TARGETS})
set_target_properties(${PROJECT_NAME}_joint_marker PROPERTIES OUTPUT_NAME joint_marker PREFIX "")
