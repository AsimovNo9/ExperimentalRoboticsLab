#!/usr/bin/env python

import sys
from tkinter import Place
import rospy
import time
import random

from experimental_robotics.srv import Hints, HintsResponse


def generate_hints():
    where = ["Kitchen", "Bedroom", "Bathroom", "Library", "Garage", "Living Room"]
    who = ["Joseph", "Mark", "Mabel", "Romero", "Almate", "Bruno"]
    what = ["Broom", "Stick", "Knife", "Bucket", "Gun", "Tissue", "Slippers"]


def clbk_function(req):
    if rospy.has_param(f"/locations/{req.goal}"):
        result = destination(req.goal)
        return NavResponse(result)
    else:
        return NavResponse("This Room doesn't exist")


def main():

    rospy.init_node("Navigator", anonymous=True)
    print("The service is about to start")

    rospy.Service("nav_to_service", Nav, clbk_function)
    print("The service has started")

    rospy.spin()


if __name__ == "__main__":
    main()
