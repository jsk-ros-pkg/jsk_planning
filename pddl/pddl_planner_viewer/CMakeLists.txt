cmake_minimum_required(VERSION 2.8.3)
project(pddl_planner_viewer)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY sample src
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)


