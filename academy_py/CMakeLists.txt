cmake_minimum_required(VERSION 2.8.3)
project(academy_py)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(academy_msgs REQUIRED)
  catkin_add_nosetests(test/test_is_success.py DEPENDENCIES ${academy_msgs_EXPORTED_TARGETS})
endif()
