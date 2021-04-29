#!/usr/bin/env python

import rospy
import actionlib
import json
from json import JSONDecoder
from collections import OrderedDict
from yeon_robot_msgs.srv import ManipulateWaypoint
from yeon_robot_msgs.srv import RemoveWaypoint
from yeon_robot_msgs.srv import RemoveAllWaypoint
from yeon_robot_msgs.srv import GetWaypoint
from yeon_robot_msgs.srv import GetAllWaypoint
from yeon_robot_msgs.msg import AllWaypoint, RobotPose

class WayPointsManager():
	def __init__(self): 
		self._waypoints_database = "/home/jetson/nprobots_ws/src/navigation_yeon/yeon/database/waypoints.json"
		self._add_a_waypoint_service = rospy.Service("/waypoints_manager/add_waypoint", ManipulateWaypoint, self.handle_add_waypoint)
		self._remove_a_waypoint_service = rospy.Service("/waypoints_manager/remove_waypoint", RemoveWaypoint, self.handle_remove_waypoint)
		self._remove_all_waypoints_service = rospy.Service("/waypoints_manager/remove_all_waypoints", RemoveAllWaypoint, self.handle_remove_all_waypoint)
		self._get_waypoint_service = rospy.Service("/waypoints_manager/get_waypoint", GetWaypoint, self.handle_get_waypoint)
		self._get_allwaypoints_service = rospy.Service("/waypoints_manager/get_all_waypoints", GetAllWaypoint, self.handle_get_all_waypoints) 

	# callback functions

	def handle_add_waypoint(self, req):

       		mydata_dict = self.load_database()	

		if req.name in mydata_dict:
			return False, "Failed to add the waypoint due to duplicate name!!"
		else: 
			mydata_dict[req.name] = {"x": req.x, "y":req.y , "w": req.w }
			self.clear_database()
			self.dump_database(mydata_dict)
			return True, "Successfully added one waypoint: " + req.name

	def handle_remove_waypoint(self, req):
		
		mydata_dict = self.load_database()

		if req.name in mydata_dict:
			del mydata_dict[req.name]
			self.dump_database(mydata_dict)
			return True, "Successfully removed " + req.name
		else: 
			return False, "No " + req.name + " found!"

	def handle_remove_all_waypoint(self, req):
		
		mydata_dict = self.load_database()
		mydata_dict.clear()
		self.dump_database(mydata_dict)
		return True, "All waypoints have been removed successfully!"

	def handle_get_waypoint(self, req):

		mydata_dict = self.load_database()
		rospy.loginfo(req.name)
		rospy.loginfo(mydata_dict.keys())

		if req.name in mydata_dict:
			x = mydata_dict.get(req.name)
					 # temporary solution 
			return True, req.name + " is saved in waypoint file.", mydata_dict[req.name]["x"], mydata_dict[req.name]["y"], mydata_dict[req.name]["w"] 
		else:
			return False, "No such waypoint saved in the file!", -1, -1, -1

	def handle_get_all_waypoints(self, req):
		
		mydata_dict = self.load_database()

		robot_pose_array = []
		i = 0
		for k, v in mydata_dict.items():
			rospy.loginfo(k)
			myPose = AllWaypoint()
			myPose.name = k
			myPose.pose.x = v["x"]
			myPose.pose.y = v["y"]
                        myPose.pose.w = v["w"]

			
			robot_pose_array.append(myPose)
			
		return True, "Successfully retrieved. Followings are all the waypoints: ", robot_pose_array
		

	def load_database(self):
		with open(self._waypoints_database, 'r') as f:
			data_dict = json.load(f, object_pairs_hook=OrderedDict)
		return data_dict 

	def clear_database(self):
		with open(self._waypoints_database, 'w') as f:
			f.write("")

	def dump_database(self, data):
		with open(self._waypoints_database, 'w') as f: 
			json.dump(OrderedDict(data), f, indent=4, sort_keys=False)

if __name__ == "__main__":
	rospy.init_node("waypoints_manager")

	WayPointsManager()
	
	rospy.spin()	
	
