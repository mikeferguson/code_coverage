# code_coverage

ROS package to run coverage testing

## Usage
To use this with your ROS package:

 * Add code_coverage as a test depend in your package.xml
 * Update your CMakeLists.txt, in the testing section add:
```
if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

# Add your targets here

if (CATKIN_ENABLE_TESTING)
  # Add your tests here

  # Create a target ${PROJECT_NAME}_coverage_report
  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*" "*/${PROJECT_NAME}/other_dir_i_dont_care_about*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage_report
      DEPENDENCIES tests
    )
  endif()
endif()
```

* Now you can build and run the tests (you need a debug build to get reasonable coverage numbers):

  - if using CATKIN_MAKE:
  ```
      catkin_make -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug package_name_coverage
  ```
  - if using CATKIN_TOOLS:
  ```
    catkin config --cmake-args -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug
    catkin build
    catkin build PACKAGE_NAME -v --no-deps --catkin-make-args PACKAGE_NAME_coverage 
  ```

* The output will print where the coverage report is located
