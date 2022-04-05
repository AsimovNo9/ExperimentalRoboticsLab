#! /usr/bin/env python

import rospy

import smach
import smach_ros
import random

from experimental_robotics.srv import Nav, NavRequest
from experimental_robotics.srv import Hints, HintsRequest
from experimental_robotics.srv import Oracle, OracleRequest
from experimental_robotics.srv import Knowledge, KnowledgeRequest

###++++++++++++++++ Navigate ++++++++++++++++###


class Navigate(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["Reached Room", "Room Unreached"],
            # outpuy_keys = [],
            # input_keys = []
        )

    def navigation(self, req):
        rospy.wait_for_service("nav_to_service")
        try:
            Nav_serv = rospy.ServiceProxy("nav_to_service", Nav)
            return Nav_serv(req)
        except rospy.ServiceException as exc:
            print(f"Service call failed: {exc}")

    def execute(self, userdata):
        req = NavRequest()
        req.goal = random.choice(
            ["kitchen", "bedroom", "bathroom", "library", "garage", "living_room"]
        )
        response = self.navigation(req)
        if response.result == "Reached Room":
            return "Reached Room"
        else:
            return "Room Unreached"


###++++++++++++++++ Gather Hints ++++++++++++++++###


class GatherHints(smach.State):
    guess_hint = ""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["Hint Obtained", "No Hint"],
            output_keys=["guess_out"],
            # input_keys = []
        )

    def guess(self, req):
        rospy.wait_for_service("hint_generator")
        try:
            hint_serv = rospy.ServiceProxy("hint_generator", Hints)
            return hint_serv(req)
        except rospy.ServiceException as exc:
            print(f"Service call failed: {exc}")

    def execute(self, userdata):
        req = HintsRequest()
        req.empty = ""
        GatherHints.guess_hint = self.guess(req)
        userdata.guess_out = GatherHints.guess_hint.hint
        if GatherHints.guess_hint.hint != "":
            return "Hint Obtained"
        else:
            return "No Hint"


###++++++++++++++++ Check Knowledge ++++++++++++++++###


class KnowledgeBase(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["Consistent", "Inconsistent"],
            # output_keys = [],
            input_keys=["guess_hint"],
        )

    def findConsistency(self, req):
        rospy.wait_for_service("knowledge_base")
        try:
            Knowledge_serv = rospy.ServiceProxy("knowledge_base", Knowledge)
            return Knowledge_serv(req)
        except rospy.ServiceException as exc:
            print(f"Service call failed: {exc}")

    def execute(self, userdata):
        req = KnowledgeRequest()
        req.guess = "Kitchen Mark Broom"
        response = self.findConsistency(req)
        if response.result == "1":
            print("It is complete and consistent")
            return "Consistent"
        elif response.result == "2" or response.result == "3":
            if response.result == "2":
                print("It is incomplete and inconsistent")
            elif response == "3":
                print("It is complete and inconsistent")
            return "Inconsistent"


###++++++++++++++++ Check Win with Oracle ++++++++++++++++###


class OracleCheck(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["Game Won", "Game Lost"],
            # output_keys = [],
            input_keys=["guess_hint"],
        )

    def oracle_check(self, req):
        rospy.wait_for_service("oracle")
        try:
            oracle_serv = rospy.ServiceProxy("oracle", Oracle)
            return oracle_serv(req)
        except rospy.ServiceException as exc:
            print(f"Service call failed: {exc}")

    def execute(self, userdata):
        req = OracleRequest()
        req.guess = userdata.guess_hint
        response = self.oracle_check(req)
        if response.win_status == "Game Won":
            return "Game Won"
        elif response.win_status == "Game Lost":
            return "Game Lost"


if __name__ == "__main__":
    rospy.init_node("State_machine")

    cluedo = smach.StateMachine(outcomes=["Game Won!!!"])

    with cluedo:

        smach.StateMachine.add(
            "Navigate",
            Navigate(),
            transitions={
                "Reached Room": "Gather Hints",
                "Room Unreached": "Navigate",
            },
        )

        smach.StateMachine.add(
            "Gather Hints",
            GatherHints(),
            transitions={
                "Hint Obtained": "Knowledge Base",
                "No Hint": "Gather Hints",
            },
            remapping={"guess_out": "guess_hint"},
        )

        smach.StateMachine.add(
            "Knowledge Base",
            KnowledgeBase(),
            transitions={
                "Consistent": "Oracle Check",
                "Inconsistent": "Navigate",
            },
        )

        smach.StateMachine.add(
            "Oracle Check",
            OracleCheck(),
            transitions={
                "Game Won": "Game Won!!!",
                "Game Lost": "Navigate",
            },
        )

        sis = smach_ros.IntrospectionServer(
            "server_name", cluedo, "Cluedo Robotics State Machine"
        )
        sis.start()

        outcome = cluedo.execute()

        rospy.spin()
        sis.stop()
