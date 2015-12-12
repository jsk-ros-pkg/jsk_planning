cmake_minimum_required(VERSION 2.8.3)
project(pddl_planner)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
    DEPENDS pddl_msgs #
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(DIRECTORY launch demos src samples test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/test-simple-failure-torelant.test)
  add_rostest(test/test-sample-pddl.test)
endif()