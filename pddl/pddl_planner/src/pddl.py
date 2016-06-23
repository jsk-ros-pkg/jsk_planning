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
        self._planner_name = rospy.get_param('~pddl_planner', 'downward')
        self._search_option = rospy.get_param('~pddl_search_option', '')

    def execute_cb(self, goal):
        problem = goal.problem
        domain = goal.domain
        rospy.loginfo("take a message")
        (problem_path, domain_path) = self.gen_tmp_pddl_file(problem, domain)
        rospy.loginfo("problem_path => %s" % problem_path)
        rospy.loginfo("domain_path => %s" % domain_path)
        result = self.call_pddl_planner(problem_path, domain_path)
        if result:
            if self._planner_name == "lpg":
                self._result.sequence = [PDDLStep(action = x.split(' ')[1].lstrip("\("),
                                                  args = re.search("\([^\)]+\)", x).group(0).lstrip("\(").rstrip("\)").split(' ')[1:],
                                                  start_time = re.search("[0-9]+.[0-9]+:" , x).group(0).rstrip(":"),
                                                  action_duration = re.search("\[D[^\)]+;", x).group(0).lstrip("[D:").rstrip(";")
                                                  )
                                         for x in result]
                self._result.use_durative_action = True
            else:
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

    def parse_pddl_result_lpg(self, output):
        rospy.loginfo(output)
        #dirty implementation
        duration_before_after = output.split("action Duration")
        if len(duration_before_after) == 2:
            results = [y.group(0)
                       for y in [re.search("[0-9][^\]]+\]", x)
                                 for x in duration_before_after[1].split("Solution number")[0].split("\n")[1:]]
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

        if  self._planner_name == "ff":
            # -f problem -o domain
            output = commands.getoutput("rosrun ff ff -f %s -o %s" % (problem, domain))
            return self.parse_pddl_result(output)

        # ffha
        elif self._planner_name == "ffha":
            output = commands.getoutput("rosrun ffha ffha %s -f %s -o %s" % (self._search_option, problem, domain))
            if re.search("final domain representation is:", output):
                tmp = output.split("metric:")
                if len(tmp) > 1:
                    output = tmp[1];
                self._result.data = tmp;
            return self.parse_pddl_result_ffha(output)

        elif self._planner_name == "downward":
            (fd, path_name) = tempfile.mkstemp(text=True, prefix='plan_')
            (status, output) = commands.getstatusoutput("rosrun downward plan %s %s %s --plan-file %s" % (domain,
                                                                                                          problem,
                                                                                                          self._search_option,
                                                                                                          path_name))
            rospy.loginfo(output)
            if status == 0:
                self._result.data = output
                return self.parse_pddl_result_downward(path_name)
            else:
                return False

        elif self._planner_name == "lpg":
            # temporary
            rospy.loginfo ("lpg")
            output = commands.getoutput("rosrun lpg_planner lpg-1.2 %s -o %s -f %s" % (self._search_option, domain, problem))
            return self.parse_pddl_result_lpg(output)

        else:
            rospy.logfatal("set invalid planner: %s !" % self._planner_name)
            return

    def gen_tmp_pddl_file(self, problem, domain):
        search_durative = re.search("durative", domain.requirements)
        rospy.loginfo ("gen_tmp_pddl_file: requirements:%s" % domain.requirements)
        if search_durative == None:
            problem_file = self.gen_tmp_problem_pddl_file(problem)
            domain_file = self.gen_tmp_domain_pddl_file(domain)
            return (problem_file, domain_file)
        else:
            problem_file = self.gen_tmp_problem_pddl_file(problem)
            domain_file = self.gen_tmp_durative_domain_pddl_file(domain)
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
    
    def gen_tmp_durative_domain_pddl_file(self, domain):
        rospy.loginfo("domain.actions:%s" % domain.actions)
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
            path.write("(:durative-action %s\n" % action.name)
            path.write(":parameters %s\n" % action.parameters)
            path.write(":duration %s\n" % action.action_duration)
            path.write(":condition %s\n" % action.precondition)
            path.write(":effect %s\n" % action.effect)
            path.write(")\n")               # (:action
        path.write(")\n")               # (define
        return path_name
    

if __name__ == '__main__':
    rospy.init_node('pddl_planner')
    PDDLPlannerActionServer(rospy.get_name())
    rospy.spin()
