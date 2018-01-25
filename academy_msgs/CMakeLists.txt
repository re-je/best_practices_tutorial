cmake_minimum_required(VERSION 2.8.3)
project(academy_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation)


add_service_files(DIRECTORY srv
  FILES
  SelectJoint.srv
)

generate_messages(DEPENDENCIES)

catkin_package(CATKIN_DEPENDS message_runtime)
