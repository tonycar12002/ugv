#!/usr/bin/env python
import rospy
import numpy as np



class task3_node():
	def __init__(self):
		self.fsm_state = 0
		self.start_point = [0, 0]
		self.end_point = [0, 0]
	def fsm_transit(self, state_to_transit):
		state_to_transit = self.fsm_state


	def process(self):
		if self.fsm_state == 0:
			# wait for clicked_points (start and end points)
			self.fsm_transit(1)
		if self.fsm_state == 1:
			# travel to start
			self.fsm_transit(2)
		if self.fsm_state(2):
			# travel to end
			self.fsm_transit(3)



def main(args):
	rospy.init_node('task3_node', anonymous = True)
	ic = task3_node()
	while(1):
		ic.process()
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)
