#!/usr/bin/env python

import rospy
import actionlib
import json
from json import JSONDecoder
from collections import OrderedDict
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
from navigation_yeon.srv import PathSequence
from navigation_yeon.srv import PathPauseTime
from navigation_yeon.srv import ManagePath


class PathManager():
	
	def __init__(self):
		self._waypoints_database = '/home/jetson/nprobots_ws/src/navigation_yeon/yeon/database/waypoints.json'
		self._path_database = '/home/jetson/nprobots_ws/src/navigation_yeon/yeon/database/path.json'
		self._path_sequence_service = rospy.Service("path_manager/set_path_sequence", PathSequence, self.handle_set_path_sequence)
		self._path_pause_time_service = rospy.Service("path_manager/set_pause_time", PathPauseTime, self.handle_pause_time)
		self._path_manager_service = rospy.Service("path_manager/manage_path", ManagePath, self.handle_manage_path)

	def handle_set_path_sequence(self, req):	#sequence, loops

		mywaypoints_dict = self.load_waypoints_database()
		mypath_dict = self.load_path_database()
		w_dict = mywaypoints_dict.keys()
		p_dict = mypath_dict.keys()
		
		rospy.loginfo(w_dict)
		rospy.loginfo(req.locations)

		result = all(elem in w_dict for elem in req.locations)
	
		if result:
			mypath_dict.clear()
			i=0
			for i in range(len(req.locations)):
				mypath_dict[req.locations[i]] = {"x": mywaypoints_dict[req.locations[i]]["x"], "y": mywaypoints_dict[req.locations[i]]["y"], "w": mywaypoints_dict[req.locations[i]]["w"]}
				self.dump_p_database(mypath_dict)
				rospy.loginfo(req.locations[i])
				i+=1
			return "You have successfully set your path sequence!"
		else: 
			return "Some of the waypoints are not registered! Please check your saved waypoints"

	def handle_pause_time(self, req):		#pause_time, loops
		
		mywaypoints_dict = self.load_waypoints_database()
		mypath_dict = self.load_path_database()
		w_dict = mywaypoints_dict.keys()
		p_dict = mypath_dict.keys()
		
		rospy.loginfo(w_dict)
		rospy.loginfo(p_dict)

		if len(mypath_dict) == 0:
			i=0
			for i in range(len(w_dict)):
				mywaypoints_dict[w_dict[i]]["Pause Time"] = req.time
				self.dump_w_database(mywaypoints_dict)
				i+=1
			return "There is no existing path set up. Pause Time is re-directed to Waypoints system."
		else:		
			i=0
			for i in range(len(p_dict)):
				mypath_dict[p_dict[i]]["Pause Time"] = req.time
				self.dump_p_database(mypath_dict)
				i+=1
			return "Pause Time is successfully assigned for all locations!"

	
	def handle_manage_path(self, req):		#sequence, pasue_time, loops

		mywaypoints_dict = self.load_waypoints_database()
		mypath_dict = self.load_path_database()
		w_dict = mywaypoints_dict.keys()
		p_dict = mypath_dict.keys()

		rospy.loginfo(w_dict)
		rospy.loginfo(req.sequence)

		result = all(elem in w_dict for elem in req.sequence)
	
		if result:
			mypath_dict.clear()
			i=0
			for i in range(len(req.sequence)):
				mypath_dict[req.sequence[i]] = {"x": mywaypoints_dict[req.sequence[i]]["x"], "y": mywaypoints_dict[req.sequence[i]]["y"], "w": mywaypoints_dict[req.sequence[i]]["w"], "Pause Time": req.time[i]}
				self.dump_p_database(mypath_dict)
				rospy.loginfo(req.sequence[i])
				i+=1
			return "You have successfully set your Path Sequence and Pause Time!"
		else: 
			return "Some of the waypoints are not registered! Please check your saved waypoints"
		
		

	def load_waypoints_database(self):
		with open(self._waypoints_database, 'r') as f:
			wp_dict = json.load(f)
		return wp_dict

	def load_path_database(self):
		with open(self._path_database, 'r') as f:
			path_dict = json.load(f, object_pairs_hook=OrderedDict)
		return path_dict

	def clear_database(self):
		with open(self._path_database, 'w') as f:
			f.write("")

	def dump_w_database(self, wdata):
		with open(self._waypoints_database, 'w') as f:
			json.dump(OrderedDict(wdata), f, indent=4, sort_keys=False)

	def dump_p_database(self, pdata):
		with open(self._path_database, 'w') as f:
			json.dump(OrderedDict(pdata), f, indent=4, sort_keys=False)


if __name__ == "__main__":
	rospy.init_node("path_manager")

	client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

	PathManager()

	rospy.spin()
