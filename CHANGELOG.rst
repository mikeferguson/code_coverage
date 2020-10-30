^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package code_coverage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.4 (2020-10-29)
------------------
* Added python3-coverage support and error if python*-coverage is missing. (`#27 <https://github.com/mikeferguson/code_coverage/issues/27>`_)
* Contributors: Stefan Fabian

0.4.3 (2020-07-30)
------------------
* Use multiple python coverage files (`#23 <https://github.com/mikeferguson/code_coverage/issues/23>`_)
  Co-authored-by: hslusarek <h.slusarek@pilz.de>
* add note that robot_calibration uses this package
* fix `#25 <https://github.com/mikeferguson/code_coverage/issues/25>`_
* Add new report formats (`#24 <https://github.com/mikeferguson/code_coverage/issues/24>`_)
  * Add html report format
  * Add console report format
  * Add '--include'  and '--omit' flag to python-coverage commands
* Contributors: Alexander Gutenkunst, Michael Ferguson, hslusarek

0.4.2 (2020-05-05)
------------------
* Add option for specifying extra flags to genhtml (`#20 <https://github.com/mikeferguson/code_coverage/issues/20>`_)
  This modification allows you to add flags to the genhtml step so that you can do things like output the lcov report with demangled C++ function names, e.g.:
  catkin_make -DGENHTML_EXTRA_FLAGS="--demangle-cpp" -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug test1_coverage_report
* Add catkin_make build step in usage example (`#19 <https://github.com/mikeferguson/code_coverage/issues/19>`_)
* bump cmake version for noetic
* Contributors: Immanuel Martini, Michael Ferguson, mschickler

0.4.1 (2020-04-05)
------------------
* update package.xml for noetic
* Contributors: Michael Ferguson

0.4.0 (2020-04-02)
------------------
* add support for python-based coverage (`#18 <https://github.com/mikeferguson/code_coverage/issues/18>`_)
* Contributors: Michael Ferguson

0.3.0 (2019-11-18)
------------------
* update target name in readme
* Merge pull request `#15 <https://github.com/mikeferguson/code_coverage/issues/15>`_ from rhaschke/master
  Simplify + clarify usage
* add link to original source
* simplify usage
  - automatically include(CodeCoverage)
  - clarify that APPEND_COVERAGE_COMPILER_FLAGS() needs to be called before defining any target
* Merge pull request `#13 <https://github.com/mikeferguson/code_coverage/issues/13>`_ from leunMar/fix/usage_docu
  Fix example of add_code_coverage() in README.md
* Fix example of add_code_coverage() in README.md
* Contributors: Immanuel Martini, Michael Ferguson, Robert Haschke

0.2.4 (2019-07-18)
------------------
* Merge pull request `#11 <https://github.com/mikeferguson/code_coverage/issues/11>`_ from agutenkunst/master
  Keep *.info.cleaned. Closes `#10 <https://github.com/mikeferguson/code_coverage/issues/10>`_
* Keep *.info.cleaned. Closes `#10 <https://github.com/mikeferguson/code_coverage/issues/10>`_
* Merge pull request `#8 <https://github.com/mikeferguson/code_coverage/issues/8>`_ from jschleicher/documentation
  documentation: degrade to test_depend
* documentation degrade to test_depend
  and note that build type should be set to Debug
* Contributors: Alexander Gutenkunst, Joachim Schleicher, Michael Ferguson

0.2.3 (2018-08-24)
------------------
* Merge pull request `#7 <https://github.com/mikeferguson/code_coverage/issues/7>`_ from jschleicher/fix/installed_package
  fix search path for installed package
* fix search path for installed package
  Closes `#6 <https://github.com/mikeferguson/code_coverage/issues/6>`_
* Contributors: Joachim Schleicher, Michael Ferguson

0.2.2 (2018-08-21)
------------------
* fix name of installspace file
* Contributors: Michael Ferguson

0.2.1 (2018-08-13)
------------------
* Merge pull request `#3 <https://github.com/mikeferguson/code_coverage/issues/3>`_ from mikeferguson/catkin_build_fix
  Add support for catkin_tools (fixes `#2 <https://github.com/mikeferguson/code_coverage/issues/2>`_)
* add information on how to run with catkin_tools
* minor escaping patch to work with catkin_tools
* Contributors: Michael Ferguson

0.2.0 (2018-08-01)
------------------
* First release
* Contributors: Michael Ferguson
