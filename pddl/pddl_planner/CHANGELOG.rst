^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pddl_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
