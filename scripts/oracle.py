#!/usr/bin/env python

import sys
import rospy
import time
import random

from experimental_robotics.srv import Oracle, OracleResponse


def win_status(guess):
    if guess == ["Joseph", "Bathroom", "Knife"]:
        return OracleResponse("Game Won")
    else:
        return OracleResponse("Game Lost")


def main():

    rospy.init_node("Oracle", anonymous=False)
    print("The service is about to start")

    rospy.Service("oracle", Oracle, win_status)
    print("The service has started")

    rospy.spin()


if __name__ == "__main__":
    main()
