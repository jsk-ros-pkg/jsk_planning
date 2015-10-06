# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(task_compiler)
find_package(catkin REQUIRED COMPONENTS roseus_smach roseus)

catkin_package(
    DEPENDS #
    CATKIN-DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(DIRECTORY launch euslisp samples
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/test_task_compiler.test)
endif()