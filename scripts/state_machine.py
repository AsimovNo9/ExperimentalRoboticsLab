import rospy

import smach
import smach_ros
from module.state import *

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
            trasnsitions={
                "Hint Obtained": "Knowledge Base",
                "No Hint": "Gather Hints",
            },
        )

        smach.StateMachine.add(
            "Knowledge Base",
            Knowledge(),
            trasnsitions={
                "Consistent": "Oracle Check",
                "Inconsistent": "Knowledge Base",
            },
        )

        smach.StateMachine.add(
            "Oracle Check",
            Oracle(),
            trasnsitions={
                "Game Won": "Game Won!!!",
                "Game Lost": "Navigate",
            },
        )

        sis = smach_ros.IntrospectionServer(
            "server_name", Cluedo, "Cluedo Robotics State Machine"
        )
        sis.start()

        outcome = Cluedo.execute()

        rospy.spin()
        sis.stop()
