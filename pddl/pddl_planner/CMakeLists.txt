cmake_minimum_required(VERSION 2.8.3)
project(pddl_planner)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY launch demos src samples test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test-simple-failure-torelant.test)
  add_rostest(test/test-sample-pddl.test)
  add_rostest(test/test-sample-derived-predicate.test)
  add_rostest(test/test-durative-two-robot-tablecloth.test)
  add_rostest(test/2008_okada_ias.test)
  add_rostest(test/2011_kakiuchi.test)
  add_rostest(test/2011_saito_knock_door.test)
  add_rostest(test/2011_saito_simple.test)
  add_rostest(test/2013_fridge_demo.test)
  ## add_rostest(test/2016_kamada_durative.test) ## this requries lpg_planner, which is needs source install
  add_rostest(test/demos_hanoi.test)
  add_rostest(test/search_object.test)
  add_rostest(test/simple_failure_torelant.test)
  add_rostest(test/simple_metric.test)
endif()
