import rospy

import smach
import smach_ros

from experimental_robotics.srv import Nav, NavResponse
from experimental_robotics.srv import Hints, NavResponse
from experimental_robotics.srv import Oracle, NavResponse
from experimental_robotics.srv import Knowledge, NavResponse

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
        response = self.navigation("living_room")
        if response == "Reached Room":
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
        GatherHints.guess_hint = self.guess("")
        userdata.guess_out = GatherHints.guess_hint
        if GatherHints.guess_hint != "":
            return "Hint Obtained"
        else:
            return "No Hint"


###++++++++++++++++ Check Knowledge ++++++++++++++++###


class Knowledge(smach.State):
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
        response = self.findConsistency(userdata.guess_hint)
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
        response = self.oracle_check(userdata.guess_hint)
        if response == "Game Won":
            return "Game Won"
        elif response == "Game Lost":
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
            Knowledge(),
            transitions={
                "Consistent": "Oracle Check",
                "Inconsistent": "Knowledge Base",
            },
        )

        smach.StateMachine.add(
            "Oracle Check",
            Oracle(),
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
