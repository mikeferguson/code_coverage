# code_coverage

ROS package to run coverage testing

## Examples

 * The [robot_calibration](https://github.com/mikeferguson/robot_calibration) package uses this to generate coverage for C++ code using Travis-CI and Codecov.io

## Usage
To use this with your ROS package:

 * Add code_coverage as a test depend in your package.xml
 * Update your CMakeLists.txt, in the testing section add the following. **NOTE** the order of test targets and coverage macros:
   ```cmake
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
         DEPENDENCIES _run_tests_${PROJECT_NAME}
       )
     endif()
   endif()
   ```
* **Note**: The variable `COVERAGE_EXCLUDES` must have some content!
* Now you can build and run the tests (you need a debug build to get reasonable coverage numbers):

  - if using CATKIN_MAKE:
  ```
      catkin_make -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug
      catkin_make -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug PACKAGE_NAME_coverage_report
  ```
  - if using CATKIN_TOOLS:
  ```
    catkin config --cmake-args -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug
    catkin build
    catkin build PACKAGE_NAME -v --no-deps --catkin-make-args PACKAGE_NAME_coverage_report 
  ```

* The output will print where the coverage report is located

## Python rostest Support

While the C++ interface and Python-based unit tests require no
modification to get coverage information, Python-based nodes
run from rostest launch files need a bit of additional
instrumentation turned on:

```xml
<launch>

    <!-- Add an argument to the launch file to turn on coverage -->
    <arg name="coverage" default="false"/>

    <!-- This fancy line forces nodes to generate coverage -->
    <arg name="pythontest_launch_prefix" value="$(eval 'python-coverage run -p' if arg('coverage') else '')"/>

    <!-- This node will NOT generate coverage information -->
    <node pkg="example_pkg" name="publisher_node" type="publisher_node.py" />

    <!-- But this node WILL generate coverage -->
    <node pkg="example_pkg" name="subscriber_node" type="subscriber_node.py"
          launch-prefix="$(arg pythontest_launch_prefix)" />

    <!-- The test can also generate coverage information if you include the launch-prefix -->
    <test time-limit="10" test-name="sample_rostest" pkg="example_pkg" type="sample_rostest.py"
          launch-prefix="$(arg pythontest_launch_prefix)" />

</launch>
```

In the CMakeLists, you will need to pass this argument:

```cmake
add_rostest(example_rostest.test ARGS coverage:=ENABLE_COVERAGE_TESTING)
```
