^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pddl_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.10 (2017-03-03)
-------------------
* [task_compiler] fix: symbol is compared with string (`#60 <https://github.com/jsk-ros-pkg/jsk_planning/issues/60>`_)
* Contributors: Yuki Furuta

0.1.9 (2017-03-01)
------------------

0.1.8 (2017-02-17)
------------------
* fix for  kinetic (`#52  <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/52>`_ )
  * CMakeLists.txt : clean up catkin_package() command
  * add lpg_planner to run_depends and remove planners from build_depends
* [pddl_planner] add relationship graph  (`#51  <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/51>`_ )
* add durative-action graph  (`#48  <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/48>`_ )
* make graph for durative action (`#47  <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/47>`_ )
* add durative action mode (`#46  <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/46>`_ )
* [pddl_planner&task_compiler] add test for task_compiler hook
  functions (`#45  <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/45>`_ )
* Contributors: Kamada Hitoshi, Kei Okada, Yuki Furuta

0.1.7 (2016-05-28)
------------------
* [pddl_planner/demos/2013_fridge_demo/solve-bring-can.l] comment in recovery motion ( `#43  <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/43>`_ )
* [pddl_planner/README] fix typo  ( `#42  <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/42>`_ )
* [pddl_planner/README] Update README.md, Add search option and plan file path to bare downward example ( `#38  <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/38>`_ )
* Contributors: Grollo, Kamada Hitoshi, Yuki Furuta

0.1.6 (2015-12-15)
------------------
* pddl_planner: mv demos/sample-pddl/README README.md
* pddl_planner/demos/sample-pddl: add sample-client.py and its test to test-sample-pddl.test
* demos/sample-pddl/{sample-problem.pddl, README}: fix problem.pddl which fails on downward, and added to README
* add test for demos/sample-pddl directory
* [pddl_planner/CMakeLists.txt] add test to install
* [pddl/pddl_planner/package.xml] add time to run_depend for downward on hydro
* [pddl_planner] add test for pddl_planner
* Contributors: Yuki Furuta, Kei Okada

0.1.5 (2015-11-26)
------------------

0.1.4 (2015-06-11)
------------------

0.1.3 (2015-01-31)
------------------
* remove rosbuild stuff, change to pure catkin packages
* use rosrun instead of find_package to search pddl planner
* not use roslib in hydro
* add planner option for downward
* Contributors: Yuki Furuta, Kei Okada

0.1.2 (2014-05-06)
------------------

0.1.1 (2014-05-05)
------------------
* pddl_planner: add samples
* catkinize jsk_planning
* add solved-fridge-graph.l
* update step-state in a while loop
* merge ffha and downward clients
* suport fastdownward
* added metric plan sample
* fix: using single state in a node
* fix for metric plan
* rename action grasp-can -> grasp-object
* fix global variable name
* add comments to search_object plan
* udpate action name
* fix typo
* add pddl-plan-to-graph function for creating plan graph
* use require and provide in pddl_planner
* add pddl demo for searching an object where it is
* make enable to use specific failed action name
* fix waring message
* add keyword for using copy
* fix typo
* remove negative precondition keyword
* fix order of pddl effects
* update return value
* add planning domain for fridge demo
* read-from-string except ff:
* add :durative-actions examples
* set default display_graph value to true
* revert wrong fommit r4686 and fix when ~display_graph is not set
* mv samples/agentsystem.py  demos/hanoi/solve-hanoi.py
* delete eus-sample.l, this is duplicate of demos/hanoi/solve-hanoi.l
* fix ffha.launch to show final domain representation, and fix pddl.py to check if final rep. is showen in the output
* delete debug files
* add comment to samples/agentsystem.py
* add sample-pddl
* support metrics and functions, [`#89 <https://github.com/jsk-ros-pkg/jsk_planning/issues/89>`_]
* use default variables, see [`#89 <https://github.com/jsk-ros-pkg/jsk_planning/issues/89>`_]
* add comment -g 6 -h 2 sometimes does not returns result
* ff does not have :data
* use append instaed of push-back
* add comment
* fix, old api?
* remove load command for irtgraph.l
* do not add the condition(state) already exists, and state compare test 'eq'->'not xor'
* changed the end condition in add-failed-nodes
* fix bug in sort-condition
* sort compare function should be <= or >=
* changed append -> union in apply-act function
* changed to use unreviewed version of irtgraph.l
* change the loop condition to make correct plan graph. (ex. Act1 is needed only after Act2 is failed)
* move some sample scripts to new package, task_compiler
* add level argument in demo-failure-recovery-task.launch
* add sample script for pddl->smach
* change sorting method to ignore negation of ffha-result conditions
* remove space from name of pddl-state, and make-readable-graph method
* move convert script from pddl to smach
* fix, add additional(fixed) condition to solved result
* add simple sample for PDDL->SMACH
* change name of predicates
* set 3 goals in pddl/2011_saito
* add goal nodes once
* add convert function from domain to eus script template
* fix add-failed-nodes for multiple results
* add another goal condition in one PDDL domain
* update PDDL-SMACH converter, I want to patch smach_viewer
* add smach convert sample
* dump :functions if functions slot is specified
* add additional-conditions for constant condition
* change for using REACHABLE
* add debug keyword for pddl-planning and fix minor bug
* delete REACHABLE predicates
* spell sepalate -> separate
* add knock door navigation problem
* add launch files for making graph pdf file
* add result parser and pddl samples
* add eus-pddl-client program
* update parser for pddl result
* add support constants for pddl-domain
* fix sample for using result parser
* add ffha-result-parser.l for making conditions of each step
* uncomment data valiable in pddl action and fix launch files
* fix, allow null parameters
* add ffha to the dependency
* add ffha (ff like pddl solver)
* fix: action parse when using typing
* fix: parse properly for more than 10 results
* update for latest roseus format
* move 3rdparty/pddl to jsk-ros-pkg/pddl, because pddl stack except ff is developed by R.Ueda and JSK, now 3rdparty
* mv jtalk and pddl to 3rdparty directory
* add pddl stack
* Contributors: Kei Okada, Yuki Furuta, Manabu Saito, Hiroyuki Mikita, Ryohei Ueda, Youhei Kakiuchi
