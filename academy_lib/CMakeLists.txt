cmake_minimum_required(VERSION 2.8.3)
project(academy_lib)

find_package(catkin REQUIRED COMPONENTS angles std_msgs)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_colors
  CATKIN_DEPENDS
    std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_colors  src/colors.cpp)
target_link_libraries(${PROJECT_NAME}_colors
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_colors ${catkin_EXPORTED_TARGETS})
