cmake_minimum_required(VERSION 3.0.2)
project(code_coverage)

find_package(catkin REQUIRED)

catkin_package(CFG_EXTRAS code_coverage-extras.cmake)

## Install all cmake files
install(DIRECTORY cmake/Modules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)
