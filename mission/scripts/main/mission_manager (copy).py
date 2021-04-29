#!/usr/bin/env python

import ast
import rospy
import json
from collections import OrderedDict

from navigation_yeon.msg import Mission
from navigation_yeon.msg import RobotPose
from navigation_yeon.msg import AllWaypoint

from navigation_yeon.srv import ManipulateWaypoint
from navigation_yeon.srv import RemoveWaypoint
from navigation_yeon.srv import RemoveAllWaypoint
from navigation_yeon.srv import GetWaypoint
from navigation_yeon.srv import GetAllWaypoint
from navigation_yeon.msg import AllWaypoint

from navigation_yeon.srv import PathSequence
from navigation_yeon.srv import PathPauseTime
from navigation_yeon.srv import ManagePath

from navigation_yeon.srv import AddAMission
from navigation_yeon.srv import RemoveAMission
from navigation_yeon.srv import RemoveAllMissions
from navigation_yeon.srv import RetrieveAMission
from navigation_yeon.srv import RetrieveAllMissions


class MissionManager():
	
	def __init__(self):

		directory = '/home/jetson/nprobots_ws/src/navigation_yeon/yeon/database'

		#waypoint
		self._add_a_waypoint_service = rospy.Service("/mission_manager/add_waypoint", ManipulateWaypoint, self.handle_add_waypoint)
		self._remove_a_waypoint_service = rospy.Service("/mission_manager/remove_waypoint", RemoveWaypoint, self.handle_remove_waypoint)
		self._remove_all_waypoints_service = rospy.Service("/mission_manager/remove_all_waypoints", RemoveAllWaypoint, self.handle_remove_all_waypoint)
		self._get_waypoint_service = rospy.Service("/mission_manager/get_waypoint", GetWaypoint, self.handle_get_waypoint)
		self._get_allwaypoints_service = rospy.Service("/mission_manager/get_all_waypoints", GetAllWaypoint, self.handle_get_all_waypoints)

		#path 
		self._path_database = directory + '/path.json'

		#mission
		self._waypoints_database = directory + '/waypoints.json'
		print(self._waypoints_database)
		self._sub_mission_database = directory + '/sub_mission.json'
		self._mission_database = directory + '/mission.json'

		self.waypoints_dict = self.load_waypoints_database()
		self.waypoints_keys = ast.literal_eval(json.dumps(self.waypoints_dict.keys()))
		self.sub_mission_dict = self.load_sub_mission_database()
		self.sub_mission_keys = ast.literal_eval(json.dumps(self.sub_mission_dict.keys()))
		self.mission_dict = self.load_mission_database()

		self._add_a_mission = rospy.Service("/mission_manager/add_a_mission", AddAMission, self.handle_add_a_mission)
		self._retrieve_a_mission = rospy.Service("/mission_manager/retrieve_a_mission", RetrieveAMission, self.handle_retrieve_a_mission)
		self._retrieve_all_missions = rospy.Service("mission_manager/retrieve_all_missions", RetrieveAllMissions, self.handle_retrieve_all_missions)
		self._remove_a_mission = rospy.Service("/mission_manager/remove_a_mission", RemoveAMission, self.handle_remove_a_mission)
		self._remove_all_mission = rospy.Service("/mission_manager/remove_all_missions", RemoveAllMissions, self.handle_remove_all_missions)
	
	#waypoint_functions
	def handle_add_waypoint(self, req):

		if req.name in self.waypoints_dict:
			return False, "Failed to add the waypoint due to duplicate name!!"
		else: 
			self.waypoints_dict[req.name] = {"x": req.x, "y":req.y , "w": req.w }
			self.dump_waypoints_database(self.waypoints_dict)
			return True, "Successfully added one waypoint: " + req.name

	def handle_remove_waypoint(self, req):

		if req.name in self.waypoints_dict:
			del self.waypoints_dict[req.name]
			self.dump_waypoints_database(self.waypoints_dict)
			return True, "Successfully removed " + req.name
		else: 
			return False, "No " + req.name + " found!"

	def handle_remove_all_waypoint(self, req):

		self.waypoints_dict.clear()
		self.dump_waypoints_database(self.waypoints_dict)
		return True, "All waypoints have been removed successfully!"

	def handle_get_waypoint(self, req):

		rospy.loginfo(req.name)
		rospy.loginfo(self.waypoints_dict.keys())

		if req.name in self.waypoints_dict:
			x = self.waypoints_dict.get(req.name)
			 # temporary solution 
			return True, req.name + " is saved in waypoint file.",self.waypoints_dict[req.name]["x"], self.waypoints_dict[req.name]["y"], self.waypoints_dict[req.name]["w"] 
		else:
			return False, "No such waypoint saved in the file!", -1, -1, -1

	def handle_get_all_waypoints(self, req):

		robot_pose_array = []
		i = 0
		for k, v in self.waypoints_dict.items():
			rospy.loginfo(k)
			myPose = AllWaypoint()
			myPose.name = k
			myPose.pose.x = v["x"]
			myPose.pose.y = v["y"]
                        myPose.pose.z = v["z"]
                        myPose.pose.w = v["w"]

			
			robot_pose_array.append(myPose)
			
		return True, "Successfully retrieved. Followings are all the waypoints: ", robot_pose_array

	#path_functions

		
	#mission_functions
	def handle_add_a_mission(self, req):

		result_len = self.check_len(req.path, req.task)
		result_loc = self.check_loc(self.waypoints_keys, req.path)

		if req.mission_name == "":
			return False, "Path Name is not given."
		else:
			if result_len == True and result_loc == True:
				self.mission_dict[req.mission_name] = {}
				self.dump_mission_database(self.mission_dict)

				i=0
				self.sub_mission_dict.clear()
				self.dump_sub_mission_database(self.sub_mission_dict)
				for i in range(len(req.path)):
					self.sub_mission_dict[req.path[i]] = {"x": self.waypoints_dict[req.path[i]]["x"], "y": self.waypoints_dict[req.path[i]]["y"], "w": self.waypoints_dict[req.path[i]]["w"], "task": req.task[i], "pause time": req.time[i]}
					i+=1
					self.dump_sub_mission_database(self.sub_mission_dict) 

				self.mission_dict[req.mission_name]=self.sub_mission_dict.copy()
				rospy.loginfo(self.mission_dict)
				self.dump_mission_database(self.mission_dict)
				return True, "Mission: "+req.mission_name+" has been set successfully"
			else:
				return False, "[Invalid] 2 Possible Errors: Given path does not exist in waypoints OR Number of tasks assigned does not tally with Number of Path given!"
				

	def handle_retrieve_a_mission(self, req):

		if req.mission_name in self.mission_dict.keys():
			
			mission_array = []

			i=0
			for k, v in self.mission_dict[req.mission_name].items():
				rospy.loginfo(k)
				missionPose = Mission()
				missionPose.name = k
				missionPose.x = v["x"]
				missionPose.y = v["y"]
                        	missionPose.w = v["w"]
				missionPose.task = v["task"]
				missionPose.time = v["pause time"]


				mission_array.append(missionPose)

			return "Successfully retrieved. Followings are all the waypoints: ", req.mission_name, mission_array

		else:
			empty_array = [Invalid]
			return "Invalid. No Mission named: ", req.mission_name, empty_array
			
	def handle_retrieve_all_missions(self, req):	
	
		if self.mission_dict == "":

			empty_array = [Invalid]
			return "No existing mission. Please add a mission.", empty_array

		else:
			missions = []
			mission_keys = ast.literal_eval(json.dumps(self.mission_dict.keys()))	
			rospy.loginfo(mission_keys)
			missions.append(str(mission_keys))
			rospy.loginfo(missions)		

			#i=0
			#for i in range(len(self.mission_dict.keys())):
				#missions[i] = mission_keys[i]
				#rospy.loginfo(missions)
				#i+=1

			return "You have successfully retrieved all missions.", missions

	def handle_remove_a_mission(self, req):

		if req.mission_name in self.mission_dict.keys():
			del self.mission_dict[req.mission_name]
			self.dump_mission_database(self.mission_dict)
			self.sub_mission_dict.clear()
			self.dump_sub_mission_database(self.sub_mission_dict)
			return True, "Successfully removed " + req.mission_name
		else: 
			return False, "No " + req.name + " found! Please check your Path Name"
	

	def handle_remove_all_missions(self, req):

		self.mission_dict.clear()
		self.dump_mission_database(self.mission_dict)
		return True, "You have successfully cleared ALL missions!"


	def check_loc(self, a, b):
		result = all(elem in a for elem in b)
		if result:
			return True
		else:
			return False
				
	def check_len(self, a, b):
		if len(a) == len(b):
			return True
		else:		
			return False
	
	def load_waypoints_database(self):
		with open(self._waypoints_database, 'r') as f:
			wp_dict = json.load(f)
		return wp_dict

	def load_path_database(self):
		with open(self._path_database, 'r') as f:
			path_dict = json.load(f, object_pairs_hook=OrderedDict)
		return path_dict

	def load_sub_mission_database(self):
		with open(self._sub_mission_database, 'r') as f:
			pdict = json.load(f, object_pairs_hook=OrderedDict)
		return pdict

	def load_mission_database(self):
		with open(self._mission_database, 'r') as f:
			mission_dict = json.load(f, object_pairs_hook=OrderedDict)
		return mission_dict

	def dump_waypoints_database(self, wdata):
		with open(self._waypoints_database, 'w') as f:
			json.dump(OrderedDict(wdata), f, indent=4, sort_keys=False)

	def dump_p_database(self, pdata):
		with open(self._path_database, 'w') as f:
			json.dump(OrderedDict(pdata), f, indent=4, sort_keys=False)

	def dump_sub_mission_database(self, pdata):
		with open(self._sub_mission_database, 'w') as f:
			json.dump(OrderedDict(pdata), f, indent=4, sort_keys=False)

	def dump_mission_database(self, mdata):
		with open(self._mission_database, 'w') as f:
			json.dump(OrderedDict(mdata), f, indent=4, sort_keys=False)

if __name__ == "__main__":

	rospy.init_node("mission_manager")

	MissionManager()

	rospy.spin()
			
