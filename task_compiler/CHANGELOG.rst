^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package task_compiler
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.7 (2016-05-28)
------------------
* [task_compiler/euslisp/execute-pddl-core.l] add hook function call on execution ( `#44 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/44>`_ )
* Contributors: Yuki Furuta

0.1.6 (2015-12-15)
------------------
* [task_compiler/CMakeLists.txt&package.xml] add roseus_smach to build_depend
* [task_compiler] add task_compiler test for solving plan and manage actions
* Contributors: Yuki Furuta

0.1.5 (2015-11-26)
------------------
* CMakeLists.txt: test execute-pddl-test.xml only for indigo
* [pckage.xml] add run_depend to the smach_viewer
* [test/{check-execute-pddl.test, execute-pddl.test.xml}: add test code to check execute-pddl.launch
* [task_compiler] Define *failed-nodes* on toplevel
* [task_compiler/euslisp/execute-pddl-core.l] fix undefined variable
* Contributors: Yuki Furuta, Kei Okada, Ryohei Ueda

0.1.4 (2015-06-11)
------------------

0.1.3 (2015-01-31)
------------------
* remove rosbuild stuff, change to pure catkin packages
* remove rosbuild stuff, change to pure catkin packages
* also removed from build_depend
* pddl_planner and roseus_smach is not used in cmake
* add option result success and fail
* support failure plan
* add planner option for downward
* Contributors: Yuki Furuta, Kei Okada

0.1.2 (2014-05-06)
------------------

0.1.1 (2014-05-05)
------------------
* catkinize jsk_planning
* add argument for debug
* fix minor bug
* update depend on roseus_smach
* forgot to add romeo_action.l
* added sample programs for romeo
* removed pr2eus_openrave dependency
* fixed pr2eus_openrave method name
* remove mismatched parentheses
* object in ADL typing may be keyword, should not use??
* dump generated state machine to /tmp/action_state_machine.l for reuse
* add room cleaning sample of task_compiler
* remove empty file
* add scripts for room cleaning planning sample, but it will work tomorrow
* change debug print
* fix smach_structure publish properly timing, add user input action to task_compiler
* add level 0 example and fix bug in launch syntax
* add new package task_compiler, which is a converter from PDDL description to SMACH executable graph.
* Contributors: Kei Okada, Manabu Saito, Hiroyuki Mikita, Youhei Kakiuchi
