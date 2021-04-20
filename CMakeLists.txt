cmake_minimum_required(VERSION 3.0.2)
project(python_op3)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

catkin_install_python(PROGRAMS scripts/op3_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
