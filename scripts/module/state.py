from asyncio.windows_events import NULL
import rospy

import smach

import smach_ros

from experimental_robotics.srv import nav, nevResponse

###++++++++++++++++ Navigate ++++++++++++++++###


class Navigate(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["Reached Room", "Room Unreached"],
            # outpuy_keys = [],
            # input_keys = []
        )

    def navigation(req):
        rospy.wait_for_service("nav_to-service")
        try:
            Nav_serv = rospy.ServiceProxy("nav_to_service", Nav)
            return Nav_serv(req)
        except rospy.ServiceException as exc:
            print(f"Service call failed: {exc}")

    def execute(self, userdata):
        response = navigation("Living room")
        if response == "Reached Room":
            return "Reached Room"
        else:
            return "Room Unreached"


###++++++++++++++++ Gather Hints ++++++++++++++++###


class GatherHints(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["Hint Obtained", "No Hint"],
            # outpuy_keys = [],
            # input_keys = []
        )

    def guess(req):
        rospy.wait_for_service("hint_generator")
        try:
            hint_serv = rospy.ServiceProxy("hint_generator", Hints)
            return hint_serv(req)
        except rospy.ServiceException as exc:
            print(f"Service call failed: {exc}")

    def execute(self, userdata):
        response = guess(req)
        if response != NULL:
            return "Hint Obtained"
        else:
            return "No Hint"


###++++++++++++++++ Check Knowledge ++++++++++++++++###


class Knowledge(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["Consistent", "Inconsistent"],
            # outpuy_keys = [],
            # input_keys = []
        )

    def FindConsistency(req):
        rospy.wait_for_service("knowledge_base")
        try:
            Knowledge_serv = rospy.ServiceProxy("knowledge_base", Knowledge)
            return Knowledge_serv(req)
        except rospy.ServiceException as exc:
            print(f"Service call failed: {exc}")

    def execute(self, userdata):
        response = FindConsistency(req)
        if response == "1":
            print("It is complete and consistent")
            return "Consistent"
        elif response == "2":
            print("It is incomplete and inconsistent")
            return "Inconsistent"
        elif response == "3":
            print("It is complete and inconsistent")
            return "Inconsistent"


###++++++++++++++++ Check Win with Oracle ++++++++++++++++###


class Oracle(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["Game Won", "Game Lost"],
            # outpuy_keys = [],
            # input_keys = []
        )

    def oracle_check(req):
        rospy.wait_for_service("oracle")
        try:
            oracle_serv = rospy.ServiceProxy("oracle", Oracle)
            return oracle_serv(req)
        except rospy.ServiceException as exc:
            print(f"Service call failed: {exc}")

    def execute(self, userdata):
        response = oracle_check(req)
        if response == "Game Won":
            return "Game Won"
        elif response == "Game Lost":
            return "Game Lost"
