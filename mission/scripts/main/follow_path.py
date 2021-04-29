#!/usr/bin/env python

import rospy
import actionlib
import json
import ast
from functionlib import *
from collections import OrderedDict
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseResult
from move_base_msgs.msg import MoveBaseFeedback
from actionlib_msgs.msg import GoalStatus
from yeon_robot_msgs.srv import SendAGoal
from yeon_robot_msgs.srv import SendAMission

class FollowPath():
	
	def __init__(self):

		directory = '/home/jetson/nprobots_ws/src/navigation_yeon/yeon/database'
		
		self.mission_dict = self.load_mission_database()
		self.sub_mission_dict = self.load_sub_mission_database()
		self.waypoints_dict = self.load_waypoints_database()

		self._mission_service = rospy.Service("/follow_path/send_a_mission", SendAMission, self.handle_send_a_mission)
		self._goal_service = rospy.Service("/follow_path/send_a_goal", SendAGoal, self.handle_send_a_goal)
		self.goal_cnt = 0

		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

	def handle_send_a_goal(self, req):

		if req.Goal_Name in self.waypoints_dict.keys():

			self.waypoint_keys = ast.literal_eval(json.dumps(self.waypoints_dict.keys()))

			rospy.loginfo("Waiting for move_base action server...")
			wait = self.client.wait_for_server(rospy.Duration(8.0))
			if not wait:
				rospy.logerr("Action server not available!")
				rospy.signal_shutdown("Action server not available!")
				return "Action server not available!"
			else:
				rospy.loginfo("Connected to move_base server")
				rospy.loginfo("Starting goals achievements...")

				goal = MoveBaseGoal()
				goal.target_pose.header.frame_id = "map"
				goal.target_pose.pose.position.x = self.waypoints_dict[req.Goal_Name]["x"]
				goal.target_pose.pose.position.y = self.waypoints_dict[req.Goal_Name]["y"]
				goal.target_pose.pose.orientation.w = self.waypoints_dict[req.Goal_Name]["w"]
				rospy.loginfo("Sending a goal pose "+ req.Goal_Name +" to Action Server")
				self.client.send_goal(goal, self.active_cb, self.feedback_cb)
				rospy.spin()


				return "Starting goals achievements..."
		else:
			return "Wrong Goal Name! Please check again"

	def handle_send_a_mission(self, req):

		if req.Mission_Name in self.mission_dict.keys():

			self.sub_mission_dict.clear()
			self.sub_mission_dict.update(self.mission_dict[req.Mission_Name])
			rospy.loginfo(self.sub_mission_dict)
			self.dump_sub_mission_database(self.sub_mission_dict)

			#list of path location names
			self.sub_mission_seq = ast.literal_eval(json.dumps(self.sub_mission_dict.keys()))
			rospy.loginfo(self.sub_mission_seq)

			rospy.loginfo("Waiting for move_base action server...")
			wait = self.client.wait_for_server(rospy.Duration(8.0))
			if not wait:
				rospy.logerr("Action server not available!")
				rospy.signal_shutdown("Action server not available!")
				return "Action server not available!"
			else:
				rospy.loginfo("Connected to move_base server")
				rospy.loginfo("Starting goals achievements...")
				self.movebase_client()
				return "Starting goals achievements..."

		else:
			return "Wrong Mission Name! Please check again"
	

	def movebase_client(self):

		
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.pose.position.x = self.sub_mission_dict[self.sub_mission_seq[self.goal_cnt]]["x"]
		goal.target_pose.pose.position.y = self.sub_mission_dict[self.sub_mission_seq[self.goal_cnt]]["y"]
		goal.target_pose.pose.orientation.w = self.sub_mission_dict[self.sub_mission_seq[self.goal_cnt]]["w"]
		pt = self.sub_mission_dict[self.sub_mission_seq[self.goal_cnt]]["pause time"]
		fn = self.sub_mission_dict[self.sub_mission_seq[self.goal_cnt]]["task"]
		rospy.loginfo(fn)
		rospy.loginfo(pt)
		rospy.loginfo("Sending a goal pose "+self.sub_mission_seq[self.goal_cnt] +" to Action Server")
		rospy.loginfo("Sleeping for "+ str(pt) +" seconds")
		self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
		exec(fn + '()')
		rospy.sleep(pt)
		rospy.spin()

	def feedback_cb(self, feedback):
		rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")
	
	def active_cb(self):
		rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being process by the Action Server") 

	def done_cb(self, status, result):
		self.goal_cnt += 1		
		rospy.loginfo(self.goal_cnt)
		if status == 2:
			rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request")
		
		if status == 3:
			rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")
	
			if self.goal_cnt< len(self.sub_mission_seq):
				next_goal = MoveBaseGoal()
				next_goal.target_pose.header.frame_id = "map"
				next_goal.target_pose.pose.position.x = self.sub_mission_dict[self.sub_mission_seq[self.goal_cnt]]["x"] 
				next_goal.target_pose.pose.position.y = self.sub_mission_dict[self.sub_mission_seq[self.goal_cnt]]["y"]
				next_goal.target_pose.pose.orientation.w = self.sub_mission_dict[self.sub_mission_seq[self.goal_cnt]]["w"]
				fn = self.sub_mission_dict[self.sub_mission_seq[self.goal_cnt]]["task"]
				rospy.loginfo(fn)
				pt = self.sub_mission_dict[self.sub_mission_seq[self.goal_cnt]]["pause time"] 
				rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server!")
				exec(fn + '()')
				rospy.sleep(pt)
				rospy.loginfo("Sleeping for "+ str(pt) +" seconds")
				self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
			else:
				rospy.loginfo("You have reached final goal pose!")
				rospy.signal_shutdown("You have reached final goal pose!")
				return

		if status == 4:
			rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" was aborted, shutting down!")
			return
		
		if status == 5:
			rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shuttig down!")
			return
	
		if status == 6:
			rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" received cancel request after execution. Shutting down!")
			return

		if status == 7:
			rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" received cancel request before execution. Shutting down!")
			return

		if status == 8:
			rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" received a cancel request, shutting down!")
			return

		if status == 9:
			rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" is LOST. Shutting down!")

	def load_waypoints_database(self):
		with open(directory + '/waypoints.json', 'r') as f:
			wp_dict = json.load(f)
		return wp_dict

	def load_mission_database(self):
		with open(directory + '/mission.json', 'r') as f:
			mdict = json.load(f, object_pairs_hook=OrderedDict)
		return mdict

	def load_sub_mission_database(self):
		with open(directory + '/sub_mission.json', 'r') as f:
			pdict = json.load(f, object_pairs_hook=OrderedDict)
		return pdict

	def dump_sub_mission_database(self, pdata):
		with open(directory + '/sub_mission.json', 'w') as f:
			json.dump(OrderedDict(pdata), f, indent=4, sort_keys=False)

if __name__ == '__main__':

	rospy.init_node('follow_path')

	FollowPath()
	rospy.spin()
	




















