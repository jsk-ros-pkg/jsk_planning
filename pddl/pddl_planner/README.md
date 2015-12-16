## pddl_planner

This package provides ROS interface for ff/ffha/downward planners

- ff: [J. Hoffmann, B. Nebel, "The FF Planning System: Fast Plan Generation Through Heuristic Search"](http://www.ai.mit.edu/courses/16.412J/ff.ps), in: Journal of Artificial Intelligence Research, Vol. 14, 2001, Pages 253-302.
- ffha: [Emil Keyder, Hector Geffner, "The FF(ha) Planner for Planning with Action Costs"](http://ipc.informatik.uni-freiburg.de/Planners?action=AttachFile&do=view&target=ffha.pdf)
- downward: http://www.fast-downward.org/

### demos/sample-pddl

- bare example

 - ff example
    ```
roscd pddl_planner/demos/sample-pddl/
rosrun ff ff -f ./sample-problem.pddl -o sample-domain.pddl
    ```

 - ffha example
    ```
roscd pddl_planner/demos/sample-pddl/
rosrun ffha ffha -f ./sample-problem.pddl -o sample-domain.pddl
    ```

 - downward example
    ```
roscd pddl_planner/demos/sample-pddl/
rosrun downward plan sample-domain.pddl ./sample-problem.pddl ipc seq-sat-lama-2011 --plan-file sample.plan
    ```

- ros example

  This package provides ROS actionlib interface for planners, using API defined in [pddl_msgs](https://github.com/jsk-ros-pkg/jsk_planning/blob/master/pddl/pddl_msgs/action/PDDLPlanner.action), see [pddl_planner/demos/sample-pddl/sample-client.py](https://github.com/jsk-ros-pkg/jsk_planning/blob/master/pddl/pddl_planner/demos/sample-pddl/sample-client.py) for cliet example.

 - ff example
    ```
roslaunch pddl_planner pddl_ff.launch
rosrun pddl_planner sample-client.py --text
   ```

 - ffha example
   ```
roslaunch pddl_planner pddl_ffha.launch
rosrun pddl_planner sample-client.py --text
   ```

 - downward example
    ```
roslaunch pddl_planner pddl_downward.launch
rosrun pddl_planner sample-client.py --text
   ```

