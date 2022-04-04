#!/usr/bin/env python 

import sys 
import rospy
import time
import random

from experimental_robotics.srv import Nav, NavResponse

def destination(where_to):
	place = rospy.get_param(f"/locations/{where_to}")

	name = place["name"]
	rospy.loginfo(f'Robot trying to reach destination: {place["name"])

	rospy.loginfo(f'Robot trying to reach destination: {place["name"]} at position ({place["x"]}, {place["y"]})'))

	time.sleep((4*random.random()))
	return "Reached Room"

def clbk_function(req):
	if rospy.has_param(f"/locations/{req.goal}"):
		result = destination(req.goal)
		return NavResponse(result)
	else:
		return NavResponse("This Room doesn't exist")

def main():

	rospy.init_node('Navigator', anonymous=False)
	print("The service is about to start")

	rospy.Service('nav_to_service', Nav, clbk_function)
	print("The service has started")

	rospy.spin()

if __name__  == '__main__':
	main()