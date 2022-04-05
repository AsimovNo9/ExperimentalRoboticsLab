#!/usr/bin/env python

import sys
import rospy
import random

from experimental_robotics.srv import Knowledge, KnowledgeResponse


def guess_checker(guess):

    guess = guess.guess.split(" ")
    where = ["Kitchen", "Bedroom", "Bathroom", "Library", "Garage", "Living Room"]
    who = ["Joseph", "Mark", "Mabel", "Romero", "Almate", "Bruno"]
    what = ["Broom", "Stick", "Knife", "Bucket", "Gun", "Tissue", "Slippers"]

    consistency_code = None
    count = [0, 0, 0]
    for i in guess:
        if i in where:
            count[0] += 1
        if i in who:
            count[1] += 1
        if i in what:
            count[2] += 1

    for index, value in enumerate(count):
        count[index] = str(value)
    consistency_code = "".join(count)

    if consistency_code == "111":
        return KnowledgeResponse("1")
    elif "0" in list(consistency_code):
        return KnowledgeResponse("2")
    else:
        return KnowledgeResponse("3")


def main():

    rospy.init_node("knowledge_base", anonymous=False)
    print("The service is about to start")

    rospy.Service("knowledge_base", Knowledge, guess_checker)
    print("The service has started")

    rospy.spin()


if __name__ == "__main__":
    main()
