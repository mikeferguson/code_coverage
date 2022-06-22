# Copyright (c) 2012 - 2017, Lars Bilke
# All rights reserved.
# From: https://github.com/bilke/cmake-modules/blob/master/CodeCoverage.cmake
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software without
#    specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# CHANGES:
#
# 2012-01-31, Lars Bilke
# - Enable Code Coverage
#
# 2013-09-17, Joakim Söderberg
# - Added support for Clang.
# - Some additional usage instructions.
#
# 2016-02-03, Lars Bilke
# - Refactored functions to use named parameters
#
# 2017-06-02, Lars Bilke
# - Merged with modified version from github.com/ufz/ogs
#
# 2018-06-12, Michael Ferguson
# - Forked for ROS support
#
# 2020-10-28, Stefan Fabian
# - Added python3-coverage support
#

include(CMakeParseArguments)

# Check prereqs
find_program( GCOV_PATH gcov )
find_program( LCOV_PATH  NAMES lcov lcov.bat lcov.exe lcov.perl)
find_program( GENHTML_PATH NAMES genhtml genhtml.perl genhtml.bat )
find_program( GCOVR_PATH gcovr PATHS ${CMAKE_SOURCE_DIR}/scripts/test)
find_program( SIMPLE_PYTHON_EXECUTABLE python )
find_program( PYTHON_COVERAGE_PATH python-coverage )

if (NOT PYTHON_COVERAGE_PATH)
    find_program( PYTHON_COVERAGE_PATH python3-coverage )
endif()

if(NOT GCOV_PATH)
    message(FATAL_ERROR "gcov not found! Aborting...")
endif() # NOT GCOV_PATH

if (NOT PYTHON_COVERAGE_PATH)
    message(FATAL_ERROR "Neither python3-coverage nor python-coverage not found! Aborting...")
endif()

if("${CMAKE_CXX_COMPILER_ID}" MATCHES "(Apple)?[Cc]lang")
    if("${CMAKE_CXX_COMPILER_VERSION}" VERSION_LESS 3)
        message(FATAL_ERROR "Clang version must be 3.0.0 or greater! Aborting...")
    endif()
elseif(NOT CMAKE_COMPILER_IS_GNUCXX)
    message(FATAL_ERROR "Compiler is not GNU gcc! Aborting...")
endif()

set(COVERAGE_COMPILER_FLAGS "-g -O0 --coverage -fprofile-arcs -ftest-coverage"
    CACHE INTERNAL "")

set(CMAKE_CXX_FLAGS_COVERAGE
    ${COVERAGE_COMPILER_FLAGS}
    CACHE STRING "Flags used by the C++ compiler during coverage builds."
    FORCE )
set(CMAKE_C_FLAGS_COVERAGE
    ${COVERAGE_COMPILER_FLAGS}
    CACHE STRING "Flags used by the C compiler during coverage builds."
    FORCE )
set(CMAKE_EXE_LINKER_FLAGS_COVERAGE
    ""
    CACHE STRING "Flags used for linking binaries during coverage builds."
    FORCE )
set(CMAKE_SHARED_LINKER_FLAGS_COVERAGE
    ""
    CACHE STRING "Flags used by the shared libraries linker during coverage builds."
    FORCE )
mark_as_advanced(
    CMAKE_CXX_FLAGS_COVERAGE
    CMAKE_C_FLAGS_COVERAGE
    CMAKE_EXE_LINKER_FLAGS_COVERAGE
    CMAKE_SHARED_LINKER_FLAGS_COVERAGE )

if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(WARNING "Code coverage results with an optimised (non-Debug) build may be misleading")
endif() # NOT CMAKE_BUILD_TYPE STREQUAL "Debug"

if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    link_libraries(gcov)
else()
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
endif()

# Defines a target for running and collection code coverage information
# Builds dependencies, runs all the ROS tests and outputs reports.
# NOTE! The executable should always have a ZERO as exit code otherwise
# the coverage generation will not complete.
#
# ADD_CODE_COVERAGE(
#     NAME testrunner_coverage                    # New target name
#     DEPENDENCIES testrunner                     # Dependencies to build first
# )
function(ADD_CODE_COVERAGE)

    set(options NONE)
    set(oneValueArgs NAME)
    set(multiValueArgs DEPENDENCIES)
    cmake_parse_arguments(Coverage "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT LCOV_PATH)
        message(FATAL_ERROR "lcov not found! Aborting...")
    endif() # NOT LCOV_PATH

    if(NOT GENHTML_PATH)
        message(FATAL_ERROR "genhtml not found! Aborting...")
    endif() # NOT GENHTML_PATH

    # Determine directory to store python coverage files
    set(COVERAGE_DIR $ENV{HOME}/.ros)
    if(DEFINED ENV{ROS_HOME})
        set(COVERAGE_DIR $ENV{ROS_HOME})
    endif()

    # Cleanup C++ counters
    add_custom_target(${Coverage_NAME}_cleanup_cpp
        # Cleanup lcov
        COMMAND ${LCOV_PATH} --directory . --zerocounters
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
        DEPENDS ${Coverage_DEPENDENCIES}
        COMMENT "Resetting CPP code coverage counters to zero."
    )

    # Cleanup python counters
    add_custom_target(${Coverage_NAME}_cleanup_py
        COMMAND ${PYTHON_COVERAGE_PATH} erase
        WORKING_DIRECTORY ${COVERAGE_DIR}
        COMMENT "Resetting PYTHON code coverage counters to zero."
    )

    # Cleanup before we run tests (attach to the tests target, which is completed before any test is run)
    add_dependencies(tests ${Coverage_NAME}_cleanup_cpp)
    add_dependencies(tests ${Coverage_NAME}_cleanup_py)

    # Create C++ coverage report
    add_custom_target(${Coverage_NAME}_cpp
        COMMAND export PYTHONIOENCODING=UTF-8
        # Create baseline to make sure untouched files show up in the report
        COMMAND ${LCOV_PATH} -c -i -d . -o ${PROJECT_BINARY_DIR}/${Coverage_NAME}.base
        # Capturing lcov counters and generating report
        COMMAND ${LCOV_PATH} --directory . --capture --output-file ${PROJECT_BINARY_DIR}/${Coverage_NAME}.info
        # add baseline counters
        COMMAND ${LCOV_PATH} -a ${PROJECT_BINARY_DIR}/${Coverage_NAME}.base -a ${PROJECT_BINARY_DIR}/${Coverage_NAME}.info --output-file ${PROJECT_BINARY_DIR}/${Coverage_NAME}.total || (exit 0)
        COMMAND ${LCOV_PATH} --remove ${PROJECT_BINARY_DIR}/${Coverage_NAME}.total ${COVERAGE_EXCLUDES} --output-file ${PROJECT_BINARY_DIR}/${Coverage_NAME}.info.removed || (exit 0)
        COMMAND ${LCOV_PATH} --extract ${PROJECT_BINARY_DIR}/${Coverage_NAME}.info.removed "'*/${PROJECT_NAME}/*'" --output-file ${PROJECT_BINARY_DIR}/${Coverage_NAME}.info.cleaned || (exit 0)
        COMMAND ${GENHTML_PATH} ${GENHTML_EXTRA_FLAGS} -o ${Coverage_NAME} ${PROJECT_BINARY_DIR}/${Coverage_NAME}.info.cleaned || (exit 0)
        COMMAND ${CMAKE_COMMAND} -E remove ${PROJECT_BINARY_DIR}/${Coverage_NAME}.base ${PROJECT_BINARY_DIR}/${Coverage_NAME}.total || (exit 0)
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
        DEPENDS _run_tests_${PROJECT_NAME}
    )

    # Ensure that the include component uses the correct path for symlinked source directories
    get_filename_component(REAL_SOURCE_DIR ${PROJECT_SOURCE_DIR} REALPATH)

    # Create Python coverage report
    add_custom_target(${Coverage_NAME}_py
        # Rename .coverage file generated by nosetests to avoid overwriting during combine step
        COMMAND if [ -f ${PROJECT_BINARY_DIR}/.coverage ]\; then mv ${PROJECT_BINARY_DIR}/.coverage ${PROJECT_BINARY_DIR}/.coverage.nosetests\; fi
        COMMAND cp ${PROJECT_BINARY_DIR}/.coverage* ${COVERAGE_DIR}/ || echo "WARNING: No python coverage!"
        COMMAND ${PYTHON_COVERAGE_PATH} combine || echo "WARNING: No python coverage to combine!"
        COMMAND ${PYTHON_COVERAGE_PATH} report --include "*${REAL_SOURCE_DIR}*" --omit ${COVERAGE_EXCLUDES} || echo "WARNING: no python report to output"
        COMMAND ${PYTHON_COVERAGE_PATH} xml    --include "*${REAL_SOURCE_DIR}*" --omit ${COVERAGE_EXCLUDES} || echo "WARNING: No python xml to output"
        COMMAND ${PYTHON_COVERAGE_PATH} html   --include "*${REAL_SOURCE_DIR}*" --omit ${COVERAGE_EXCLUDES} || echo "WARNING: No python html to output"
        WORKING_DIRECTORY ${COVERAGE_DIR}
        DEPENDS _run_tests_${PROJECT_NAME}
    )

    add_custom_target(${Coverage_NAME}
        DEPENDS ${Coverage_NAME}_cpp
        DEPENDS ${Coverage_NAME}_py
        COMMENT "Processing code coverage counters and generating report."
    )

    # Show where to find the lcov info report
    add_custom_command(TARGET ${Coverage_NAME} POST_BUILD
        COMMAND ;
        COMMENT "Lcov code coverage info report saved in ${PROJECT_BINARY_DIR}/${Coverage_NAME}.info."
    )

    # Show info where to find the C++ report
    add_custom_command(TARGET ${Coverage_NAME} POST_BUILD
        COMMAND ;
        COMMENT "Open ${PROJECT_BINARY_DIR}/${Coverage_NAME}/index.html in your browser to view the coverage report."
    )

    # Show info where to find the Python report
    add_custom_command(TARGET ${Coverage_NAME} POST_BUILD
        COMMAND ;
        COMMENT "Python code coverage info saved in ${COVERAGE_DIR} directory."
        COMMENT "Python code coverage html-format: ${COVERAGE_DIR}/htmlcov/index.html."
    )

endfunction() # SETUP_TARGET_FOR_COVERAGE

function(APPEND_COVERAGE_COMPILER_FLAGS)
    # Set flags for all C++ builds
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COVERAGE_COMPILER_FLAGS}" PARENT_SCOPE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COVERAGE_COMPILER_FLAGS}" PARENT_SCOPE)
    # Turn on coverage in python nosetests (see README for requirements on rostests)
    set(ENV{CATKIN_TEST_COVERAGE} "1")
    message(STATUS "Appending code coverage compiler flags: ${COVERAGE_COMPILER_FLAGS}")
endfunction() # APPEND_COVERAGE_COMPILER_FLAGS

option(ENABLE_COVERAGE_TESTING "Turn on coverage testing" OFF)
