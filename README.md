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
        add_code_coverage(
          NAME ${PROJECT_NAME}_coverage
          DEPENDS tests
        )
      endif()
    endif()
```
 * Now you can run:
```
 	catkin_make -DENABLE_COVERAGE_TESTING=ON package_name_coverage
```
 * The output will print where the coverage report is located
