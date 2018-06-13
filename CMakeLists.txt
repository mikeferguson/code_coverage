cmake_minimum_required(VERSION 2.8.3)
project(code_coverage)

find_package(catkin REQUIRED)

option(ENABLE_COVERAGE_TESTING "Turn on coverage testing" OFF)
catkin_package(CFG_EXTRAS code_coverage-extras.cmake)

if (ENABLE_COVERAGE_TESTING)
  message(STATUS "Using ENABLE_COVERAGE_TESTING: ${ENABLE_COVERAGE_TESTING}")
endif()

## Install all cmake files
install(DIRECTORY cmake/Modules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)
