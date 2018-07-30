# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(task_compiler)
find_package(catkin REQUIRED COMPONENTS
  roseus_smach)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
  add_rostest(test/test_task_compiler.test)
  roslaunch_add_file_check(test/execute-pddl.test.xml)
endif()

install(DIRECTORY launch euslisp samples
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)
