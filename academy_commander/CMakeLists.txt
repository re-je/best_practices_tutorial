cmake_minimum_required(VERSION 2.8.3)
project(academy_commander)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/commander
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY resource
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
