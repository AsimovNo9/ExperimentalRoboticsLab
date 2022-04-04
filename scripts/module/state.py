import rospy

import smach

import smach_ros

from experimental_robotics.srv import nav, nevResponse

###++++++++++++++++ Navigate ++++++++++++++++###

class Navigate(smach.State):
	
	def__init__(self):
		smach.State.__init__(
			self,
			outcomes = ["Reached Room", "Room Unreached"],
			# outpuy_keys = [],
			# input_keys = []
		)

	def navigation(req):
		rospy.wait_for_service('nav_to-service')
		try:
			Nav_serv = rospy.ServiceProxy('nav_to_service', Nav)
			return Nav_serv(req)
		except rospy.ServiceException as exc:
			print(f"Service call failed: {exc}")
		
	def execute(self, userdata):
		response = navigation("Living room")
		if response == 'Reached Room':
			return "Reached Room"
		else:
			return "Room Unreached"
		pass

###++++++++++++++++ Gather Hints ++++++++++++++++###

class GatherHints(smach.State):
	
	def__init__(self):
		smach.State.__init__(
			self,
			outcomes = ["Hint Obtained", "No Hint"].
			# outpuy_keys = [],
			# input_keys = []
		)
	

		
	def execute(self, userdata):
		# Get hints msgs
		# Save messages and send to
		# Obtain result
		# if positive result is obtained
			# rospy.loginfo(f"Navigation complete")
			# return "reached room"
		pass
	
###++++++++++++++++ Learn Knowledge ++++++++++++++++###

class GainKnowledge(smach.State):
	
	def__init__(self):
		smach.State.__init__(
			self,
			outcomes = ["Learnt Something", "Learnt Nothing"].
			# outpuy_keys = [],
			# input_keys = []
		)
		
	def execute(self, userdata):
		# Get hints msgs
		# Save messages and send to
		# Obtain result
		# if positive result is obtained
			# rospy.loginfo(f"Navigation complete")
			# return "reached room"
		pass

###++++++++++++++++ Check Consistency ++++++++++++++++###

class GainKnowledge(smach.State):
	
	def__init__(self):
		smach.State.__init__(
			self,
			outcomes = ["Consistent", "Inconsistent"].
			# outpuy_keys = [],
			# input_keys = []
		)
		
	def execute(self, userdata):
		# Get hints msgs
		# Save messages and send to
		# Obtain result
		# if positive result is obtained
			# rospy.loginfo(f"Navigation complete")
			# return "reached room"
		pass

###++++++++++++++++ Check Consistency ++++++++++++++++###

class GainKnowledge(smach.State):
	
	def__init__(self):
		smach.State.__init__(
			self,
			outcomes = ["Consistent", "Inconsistent"].
			# outpuy_keys = [],
			# input_keys = []
		)
		
	def execute(self, userdata):
		# Get hints msgs
		# Save messages and send to
		# Obtain result
		# if positive result is obtained
			# rospy.loginfo(f"Navigation complete")
			# return "reached room"
		pass
