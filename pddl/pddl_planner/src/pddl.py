#!/usr/bin/env python

import rospy
import os
import re
import commands
import tempfile

try:
    from pddl_msgs.msg import *
except:
    import roslib; roslib.load_manifest('pddl_planner')
    from pddl_msgs.msg import *
import actionlib


class PDDLPlannerActionServer(object):
    _result = PDDLPlannerResult()
    _feedback = PDDLPlannerFeedback()
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                PDDLPlannerAction,
                                                self.execute_cb)
        # resolve rosparam
        self._planner_path = rospy.get_param('~pddl_planner_path')
        if rospy.has_param('~pddl_search_option'):
            self._search_option = rospy.get_param('~pddl_search_option')

    def execute_cb(self, goal):
        problem = goal.problem
        domain = goal.domain
        rospy.loginfo("take a message")
        (problem_path, domain_path) = self.gen_tmp_pddl_file(problem, domain)
        rospy.loginfo("problem_path => %s" % problem_path)
        rospy.loginfo("domain_path => %s" % domain_path)
        result = self.call_pddl_planner(problem_path, domain_path)
        if result:
            self._result.sequence = [PDDLStep(action = x.split(' ')[0],
                                              args = x.split(' ')[1:])
                                     for x in result]
            self._as.set_succeeded(self._result)
        else:
            self._as.set_aborted()

    def parse_pddl_result(self, output):
        rospy.loginfo(output)
        # dirty implementation
        step_before_after = output.split("step")
        if len(step_before_after) == 2:
            results = [re.sub("\s*$", "", re.sub(r'^\s*\d+:\s*', "" , x))
                       for x in step_before_after[1].split("time spent")[0].split("\n")]
            rospy.loginfo("result => %s" % results)
            return filter(lambda x: x != "", results)
        else:
            return False
    def parse_pddl_result_ffha(self, output):
        rospy.loginfo(output)
        # dirty implementation
        step_before_after = output.split("step")
        if len(step_before_after) == 2:
            results = [y.group(0)
                       for y in [re.search("\([^\)]+\)", x)
                                 for x in step_before_after[1].split("Total cost")[0].split("\n")[1:]]
                       if y != None]
            rospy.loginfo("result => %s" % results)
            return results
        else:
            return False

    def parse_pddl_result_downward(self, path_name):
        plan_path = path_name
        i = 1
        while os.path.exists(path_name + "." + str(i)):
            plan_path = path_name + "." + str(i)
            i += 1
        rospy.loginfo("plan_path => %s" % plan_path)

        with open(plan_path) as f:
            plan = f.read().split("\n")

        plan.remove("")
        results = [re.sub(" \)$", ")", x)
                   for x in plan]
        rospy.loginfo(results)

        return results

    def call_pddl_planner(self, problem, domain):

        if re.search("/bin/ff", self._planner_path):
            # -f problem -o domain
            output = commands.getoutput("%s -f %s -o %s" % (self._planner_path,
                                                            problem,
                                                            domain))
            # ffha
            if re.search("/ffha", self._planner_path):
                if re.search("final domain representation is:", output):
                    tmp = output.split("metric:")
                    if len(tmp) > 1:
                        output = tmp[1];
                    self._result.data = tmp;
                return self.parse_pddl_result_ffha(output)
            # ff
            return self.parse_pddl_result(output)

        if re.search("downward", self._planner_path):
            (fd, path_name) = tempfile.mkstemp(text=True, prefix='plan_')
            (status, output) = commands.getstatusoutput("%s %s %s %s --plan-file %s" % (self._planner_path,
                                                                                        domain,
                                                                                        problem,
                                                                                        self._search_option,
                                                                                        path_name))
            rospy.loginfo(output)
            if status == 0:
                self._result.data = output
                return self.parse_pddl_result_downward(path_name)
            else:
                return False

    def gen_tmp_pddl_file(self, problem, domain):
        problem_file = self.gen_tmp_problem_pddl_file(problem)
        domain_file = self.gen_tmp_domain_pddl_file(domain)
        return (problem_file, domain_file)
    def gen_problem_objects_strings(self, objects):
        # objects = list of PDDLObject
        # PDDLObject has name and type
        # collect PDDLObjects which has the same type
        grouped_objects = {}
        # grouped_objects := (type_and_objects ...)
        # type_and_objects := (type_name object_a object_b ...)
        # destructively change grouped_objects
        for o in objects:
            object_name = o.name
            # find same object_type in grouped_objects
            if grouped_objects.has_key(o.type):
                grouped_objects[o.type].append(o.name)
            else:
                grouped_objects[o.type] = [o.name]
        return [" ".join(grouped_objects[t]) + " - " + t
                for t in grouped_objects.keys()]
    
    def gen_tmp_problem_pddl_file(self, problem):
        (fd, path_name) = tempfile.mkstemp(text=True, prefix='problem_')
        path = os.fdopen(fd, 'w')
        path.write("""(define (problem %s)
(:domain %s)
(:objects %s)
(:init %s)
(:goal %s)
""" % (problem.name, problem.domain,
       "\n".join(self.gen_problem_objects_strings(problem.objects)),
       ' '.join(problem.initial), problem.goal))
        if problem.metric:
            path.write("""(:metric %s)""" % problem.metric)
        path.write(""")""")
        return path_name
    def gen_tmp_domain_pddl_file(self, domain):
        (fd, path_name) = tempfile.mkstemp(text=True, prefix='domain_')
        path = os.fdopen(fd, 'w')
        path.write("(define (domain %s)\n" % domain.name)
        path.write("(:requirements %s)\n" % domain.requirements)
        path.write("(:types \n")
        for i in domain.types:
            path.write(i + " ")
        path.write(")\n")
        if len(domain.constants) > 0:
            path.write("(:constants \n")
            for i in domain.constants:
                path.write(i + " ")
            path.write(")\n")
        path.write("(:predicates\n")
        for i in domain.predicates:
            path.write(i + " ")
        path.write(")\n")
        if domain.functions:
            path.write("(:functions\n")
            for i in domain.functions:
                path.write(i + " ")
            path.write(")\n")
        for action in domain.actions:
            path.write("(:action %s\n" % action.name)
            path.write(":parameters %s\n" % action.parameters)
            path.write(":precondition %s\n" % action.precondition)
            path.write(":effect %s\n" % action.effect)
            path.write(")\n")               # (:action
        path.write(")\n")               # (define
        return path_name
    

if __name__ == '__main__':
    rospy.init_node('pddl_planner')
    PDDLPlannerActionServer(rospy.get_name())
    rospy.spin()
