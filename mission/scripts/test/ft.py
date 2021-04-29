#!/usr/bin/env python

import rospy
import actionlib
import json
import ast
from collections import OrderedDict
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseResult
from move_base_msgs.msg import MoveBaseFeedback
from actionlib_msgs.msg import GoalStatus
from yeon_robot_msgs.srv import SendAGoal

class FollowPath():
	
	def __init__(self):

		self.mission_dict = self.load_mission_database()
		self.sub_mission_dict = self.load_sub_mission_database()

		self._goal_service = rospy.Service("/follow_path/send_a_goal", SendAGoal, self.handle_send_a_goal)
		self.goal_cnt = 0
		
	def handle_send_a_goal(self, req):

		rospy.loginfo(self.mission_dict.keys())

		if req.Mission_Name in self.mission_dict.keys():

			self.sub_mission_dict.update(self.mission_dict[req.Mission_Name])
			rospy.loginfo(self.sub_mission_dict)
			self.dump_sub_mission_database(self.sub_mission_dict)
			return "Processing goal location..."
		else:
			return "Wrong Mission Name. Please check again!"



	def load_mission_database(self):
		with open('/home/jetson/nprobots_ws/src/navigation_yeon/yeon/database/mission.json', 'r') as f:
			mdict = json.load(f, object_pairs_hook=OrderedDict)
		return mdict

	def load_sub_mission_database(self):
		with open('/home/jetson/nprobots_ws/src/navigation_yeon/yeon/database/sub_mission.json', 'r') as f:
			pdict = json.load(f, object_pairs_hook=OrderedDict)
		return pdict

	def dump_sub_mission_database(self, pdata):
		with open('/home/jetson/nprobots_ws/src/navigation_yeon/yeon/database/sub_mission.json', 'w') as f:
			json.dump(OrderedDict(pdata), f, indent=4, sort_keys=False)

if __name__ == '__main__':

	rospy.init_node('follow_path')

	FollowPath()
	rospy.spin()
