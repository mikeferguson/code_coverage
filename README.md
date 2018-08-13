# code_coverage

ROS package to run coverage testing

## Usage
To use this with your ROS package:

 * Add code_coverage as a build depend in your package.xml
 * Update your CMakeLists.txt:
```
    find_package(catkin
      REQUIRED
        code_coverage
        ...
    )
    catkin_package(
        ...
    )

    if(ENABLE_COVERAGE_TESTING)
      include(CodeCoverage)
      APPEND_COVERAGE_COMPILER_FLAGS()
    endif()

    # Add your targets here

    if (CATKIN_ENABLE_TESTING)
      
      # Add your tests here

      if(ENABLE_COVERAGE_TESTING)
        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*" "*/${PROJECT_NAME}/other_dir_i_dont_care_about*")
        add_code_coverage(
          NAME ${PROJECT_NAME}_coverage
          DEPENDS tests
        )
      endif()
    endif()
```
 * Now you can build and run the tests:
```
    IF USING CATKIN_MAKE:
    catkin_make -DENABLE_COVERAGE_TESTING=ON package_name_coverage
    
    IF USING CATKIN_TOOLS:
    catkin config --cmake-args -DENABLE_COVERAGE_TESTING=ON
    catkin build
    catkin build PACKAGE_NAME -v -no-deps --catkin-make-args PACKAGE_NAME_coverage 
```

 * The output will print where the coverage report is located
