cmake_minimum_required(VERSION 2.8.3)
project(pddl_msgs)

find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs message_generation)

add_action_files(
  DIRECTORY action
  FILES PDDLPlanner.action
)
add_message_files(
  DIRECTORY msg
  FILES PDDLAction.msg PDDLDomain.msg PDDLProblem.msg PDDLActionArray.msg PDDLObject.msg PDDLStep.msg
)

generate_messages(
  DEPENDENCIES actionlib_msgs
)

catkin_package(
    DEPENDS actionlib actionlib_msgs message_runtime
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
