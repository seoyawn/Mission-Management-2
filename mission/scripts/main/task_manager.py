#!usr/bin/env python

import rospy
import json
import ast
from json import JSONDecoder
from collections import OrderedDict
from yeon_robot_msgs.srv import AssignATask
from yeon_robot_msgs.srv import AssignTasks
#from yeon_robot_msgs.srv import GetAllTasks

class TaskManager():
	
	def __init__(self):
		self._path_database = '/home/jetson/nprobots_ws/src/navigation_yeon/yeon/database/path.json'
		self._task_database = '/home/jetson/nprobots_ws/src/navigation_yeon/yeon/database/task.json'
		self._assign_a_task = rospy.Service("/task_manager/assign_a_task", AssignATask, self.handle_assign_a_task)
		self._assign_tasks = rospy.Service("/task_manager/assign_tasks", AssignTasks, self.handle_assign_tasks)
		#self._retrieve_all_tasks = rospy.Service("/task_manager/get_all_tasks", GetAllTasks, self.handle_get_all_tasks)

		self.path_dict = self.load_path_database()
		self.p_dict = ast.literal_eval(json.dumps(self.path_dict.keys()))
		self.task_dict = self.load_task_database()
		self.t_dict = ast.literal_eval(json.dumps(self.task_dict.keys()))
	
	def handle_assign_a_task(self, req):
		
		rospy.loginfo(self.p_dict)		
		
		if req.location in self.p_dict:
			if req.task in self.t_dict:
				self.path_dict[req.location]["Task"] = req.task
				self.dump_path_database(self.path_dict)
				rospy.loginfo(req.location)
				rospy.loginfo(req.task)
			return "You have successfully assigned Task: "+req.task
		else:
			return req.location+" does not exist"
	
	def handle_assign_tasks(self, req):

		i=0
		for i in range(len(req.locations)):
			self.path_dict[req.locations[i]]["Task"] = req.task[i]
			self.dump_path_database(self.path_dict)
			rospy.loginfo(req.locations[i])
			rospy.loginfo(req.task[i])
			i+=1
		return "You have successfully assigned tasks at multiple locations!"
	
	
	#def handle_get_all_tasks(self, req):

		#i=0
		#task_list = []
		#for i in range(len(self.t_dict)):
		#	task_list[i] = self.t_dict[i]
		#	task_list.append

	def load_path_database(self):
		with open(self._path_database, 'r') as f:
			path_dict = json.load(f, object_pairs_hook=OrderedDict)
		return path_dict
	
	def load_task_database(self):
		with open(self._task_database, 'r') as f:
			task_dict = json.load(f, object_pairs_hook=OrderedDict)
		return task_dict

	def dump_path_database(self, pdata):
		with open(self._path_database, 'w') as f:
			json.dump(OrderedDict(pdata), f, indent=4, sort_keys=False)

if __name__ == "__main__":
	rospy.init_node("task_manager")
	
	TaskManager()

	rospy.spin()
		
