# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(task_compiler)
find_package(catkin REQUIRED COMPONENTS)

catkin_package(
    DEPENDS #
    CATKIN-DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  if($ENV{ROS_DISTRO} STREQUAL "indigo") # smach_viewer only available on indigo (https://github.com/jbohren/executive_smach_visualization-release/issues/1)
    roslaunch_add_file_check(test/execute-pddl.test.xml)
  endif()

  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/test_task_compiler.test)

endif()

install(DIRECTORY launch euslisp samples
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)
