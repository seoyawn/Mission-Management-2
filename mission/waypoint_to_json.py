#! /usr/bin/env python

import rospy
import ast
from nav_msgs.msg import Path
from collections import OrderedDict

def __init__(self): 
	self._raw_waypoints_database = "/home/jetson/nprobots_ws/src/navigation_yeon/yeon/database/raw_waypoints.json"
	self.raw_waypoints_dict = self.load_database()

def callback(self, msg):

	for i in range(len(msg.poses)):
		x = msg.poses[i].pose.position.x
		y = msg.poses[i].pose.position.y
		z = msg.poses[i].pose.orientation.z
		w = msg.poses[i].pose.orientation.w

		print "Waypoint" + str(i+1) + "," +str(x) +"," + str(y) +"," + str(z) +"," + str(w)

		self.raw_waypoints_dict["Waypoint"+ str(i+1)] = {"x": x, "y": y, "z": z, "w": w}
		self.dump_raw_waypoints_database(self.raw_waypoints_dict)

def load_database(self):
	with open(self._raw_waypoints_database, 'r') as f:
		data_dict = json.load(f, object_pairs_hook=OrderedDict)
	return data_dict 

def dump_raw_waypoints_database(self, wdata):
	with open(self._raw_waypoints_database, 'w') as f:
		json.dump(OrderedDict(wdata), f, indent=4, sort_keys=False)

if __name__ == "__main__":
	rospy.init_node('read')

	sub = rospy.Subscriber("/waypoints", Path, callback)

	rospy.spin()
