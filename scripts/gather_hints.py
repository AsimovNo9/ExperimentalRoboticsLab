#!/usr/bin/env python

import sys
import rospy
import random

from experimental_robotics.srv import Hints, HintsResponse


def generate_hints(req):
    where = ["Kitchen", "Bedroom", "Bathroom", "Library", "Garage", "Living Room"]
    who = ["Joseph", "Mark", "Mabel", "Romero", "Almate", "Bruno"]
    what = ["Broom", "Stick", "Knife", "Bucket", "Gun", "Tissue", "Slippers"]

    choice = random.choice([0, 1])
    if choice == 0:
        guess = random.choice(where) + random.choice(who) + random.choice(what)
    elif choice == 1:
        guess = (
            random.choice(where)
            + random.choice(who)
            + random.choice(who)
            + random.choice(what)
        )
    return guess


def main():

    rospy.init_node("hint_generator", anonymous=False)
    print("The service is about to start")

    rospy.Service("hint_generator", Hints, generate_hints)
    print("The service has started")

    rospy.spin()


if __name__ == "__main__":
    main()
