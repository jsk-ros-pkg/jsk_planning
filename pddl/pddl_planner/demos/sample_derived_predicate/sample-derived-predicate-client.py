#!/usr/bin/env python
import roslib
import rospy
import os

import actionlib
import pddl_msgs
from pddl_msgs.msg import *

import sys
import unittest

class TestDerivedPredicate(unittest.TestCase):

    def test_derived_predicate(self):
        client = actionlib.SimpleActionClient('pddl_planner',
                                              PDDLPlannerAction)
        client.wait_for_server()
        goal = PDDLPlannerGoal()
        goal.domain.name = "manip"
        goal.domain.requirements = ":typing :derived-predicates"
        goal.domain.types = "object"
        goal.domain.predicates = ["(on ?obj0 ?obj1 - object)",
                                  "(clear ?obj - object)",
                                  "(ontable ?obj - object)",
                                  "(holding ?obj - object)",
                                  "(handempty)"]
        pickup = PDDLAction()
        pickup.name = "pickup"
        pickup.parameters = "(?obj - object)"
        pickup.precondition = "(and (ontable ?obj) (clear ?obj) (handempty))"
        pickup.effect = """(and (not (ontable ?obj)) (holding ?obj))"""
        putdown = PDDLAction()
        putdown.name = "putdown"
        putdown.parameters = "(?obj - object)"
        putdown.precondition = "(and (holding ?obj))"
        putdown.effect = """(and
                                (not (holding ?obj))
                                (ontable ?obj))"""
        stack = PDDLAction()
        stack.name = "stack"
        stack.parameters = "(?obj0 ?obj1 - object)"
        stack.precondition = """(and 
                                        (holding ?obj0)
                                        (clear ?obj1))"""
        stack.effect = """(and
                                (not (holding ?obj0))
                                (on ?obj0 ?obj1))"""
        unstack = PDDLAction()
        unstack.name = "unstack"
        unstack.parameters = "(?obj0 ?obj1 - object)"
        unstack.precondition = """(and 
                                        (handempty)
                                        (on ?obj0 ?obj1)
                                        (clear ?obj0))"""
        unstack.effect = """(and
                                 (not (on ?obj0 ?obj1))
                                 (holding ?obj0))"""
        goal.domain.actions = [pickup, unstack, stack, putdown]
        clear = PDDLDerived()
        clear.predicate = "(clear ?obj0 - object)"
        clear.formula = "(and (not (holding ?obj0)) (forall (?obj1 - object) (not (on ?obj1 ?obj0))))"
        handempty = PDDLDerived()
        handempty.predicate = "(handempty)"
        handempty.formula = "(forall (?obj - object) (not (holding ?obj)))"
        goal.domain.deriveds = [clear, handempty]
        goal.problem.name = "sample"
        goal.problem.domain = "manip"
        goal.problem.objects = [PDDLObject(name="a", type="object"),
                                PDDLObject(name="b", type="object"),
                                PDDLObject(name="c", type="object")]
        goal.problem.initial = ["(on c a)", 
                                "(ontable a)",
                                "(ontable b)"]
        goal.problem.goal = "(and (on a b) (on b c))"
        rospy.loginfo(str(goal))
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_result()
        rospy.logdebug(str(result))
        rospy.loginfo(str(result.sequence))
        self.assertEquals(len(result.sequence), 6, "result sequence is 6")

if __name__ == '__main__':
    import rosunit
    rospy.init_node('pddl_planner_client', anonymous=True)
    rosunit.unitrun('pddl_planner', 'test_derived_predicate', TestDerivedPredicate)
