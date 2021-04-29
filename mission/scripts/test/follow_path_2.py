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
		
		rospy.init_node('follow_path')

		self.mission_dict = self.load_mission_database()
		self.sub_mission_dict = self.load_sub_mission_database()

		self.goal_service = rospy.Service("/follow_path/send_a_goal", SendAGoal, self.handle_send_a_goal)

		#list of path location names
		#self._seq = ast.literal_eval(json.dumps(self.path_dict.keys()))
		#rospy.loginfo(self.path_seq)
		self.goal_cnt = 0
		
	def handle_send_a_goal(self, req):
			
		self.sub_mission_dict.update(self.mission_dict[req.Mission_Name])
		rospy.loginfo(self.sub_mission_dict)
		
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		rospy.loginfo("Waiting for move_base action server...")
		wait = self.client.wait_for_server(rospy.Duration(8.0))
		if not wait:
			rospy.logerr("Action server not available!")
			rospy.signal_shutdown("Action server not available!")
			return
		else:
			rospy.loginfo("Connected to move_base server")
			rospy.loginfo("Starting goals achievements...")
			#self.movebase_client()

	def movebase_client(self):
		
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.pose.position.x = self.path_dict[self.path_seq[self.goal_cnt]]["x"]
		goal.target_pose.pose.position.y = self.path_dict[self.path_seq[self.goal_cnt]]["y"]
		goal.target_pose.pose.orientation.w = self.path_dict[self.path_seq[self.goal_cnt]]["w"]
		rospy.loginfo("Sending a goal pose "+self.path_seq[self.goal_cnt] +" to Action Server")
		
		self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
		rospy.spin()

	def feedback_cb(self, feedback):
		rospy.loginfo("Feedback for goal pose "+self.path_seq[self.goal_cnt+1]+" received")
	
	def active_cb(self):
		rospy.loginfo("Goal pose "+self.path_seq[self.goal_cnt+1]+" is now being process by the Action Server") 

	def done_cb(self, status, result):
		self.goal_cnt += 1		
		rospy.loginfo(self.goal_cnt)
		rospy.loginfo(len(self.path_seq))
		if status == 2:
			rospy.loginfo("Goal pose "+self.path_seq[self.goal_cnt]+" received a cancel request")
		
		if status == 3:
			rospy.loginfo("Goal pose "+self.path_seq[self.goal_cnt]+" reached")
	
			if self.goal_cnt< len(self.path_seq):
				next_goal = MoveBaseGoal()
				next_goal.target_pose.header.frame_id = "map"
				next_goal.target_pose.pose.position.x = self.path_dict[self.path_seq[self.goal_cnt]]["x"] 
				next_goal.target_pose.pose.position.y = self.path_dict[self.path_seq[self.goal_cnt]]["y"]
				next_goal.target_pose.pose.orientation.w = self.path_dict[self.path_seq[self.goal_cnt]]["w"]
				rospy.loginfo("Sending goal pose "+self.path_seq[self.goal_cnt+1]+" to Action Server!")
				self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
			else:
				rospy.loginfo("You have reached final goal pose!")
				rospy.signal_shutdown("You have reached final goal pose!")
				return

		if status == 4:
			rospy.loginfo("Goal pose "+self.path_seq[self.goal_cnt]+" was aborted, shutting down!")
			rospy.signal_shutdown("Goal pose "+self.path_seq[self.goal_cnt]+" was aborted, shutting down!")
			return
		
		if status == 5:
			rospy.loginfo("Goal pose "+self.path_seq[self.goal_cnt]+" rejected, shuttig down!")
			rospy.signal_shutdown("Goal pose "+self.path_seq[self.goal_cnt]+" rejected, shuttig down!")
			return
	
		if status == 6:
			rospy.loginfo("Goal pose "+self.path_seq[self.goal_cnt]+" received cancel request after execution. Shutting down!")
			rospy.signal_shutdown("Goal pose "+self.path_seq[self.goal_cnt]+" received cancel request after execution. Shutting down!")
			return

		if status == 7:
			rospy.loginfo("Goal pose "+self.path_seq[self.goal_cnt]+" received cancel request before execution. Shutting down!")
			rospy.signal_shutdown("Goal pose "+self.path_seq[self.goal_cnt]+" received cancel request before execution. Shutting down!")
			return

		if status == 8:
			rospy.loginfo("Goal pose "+self.path_seq[self.goal_cnt]+" received a cancel request, shutting down!")
			rospy.signal_shutdown("Goal pose "+self.path_seq[self.goal_cnt]+" received a cancel request, shutting down!")
			return

		if status == 9:
			rospy.loginfo("Goal pose "+self.path_seq[self.goal_cnt]+" is LOST. Shutting down!")
			rospy.signal_shutdown("Goal pose "+self.path_seq[self.goal_cnt]+" is LOST. Shutting down!")

	def load_sub_mission_database(self):
		with open('/home/jetson/nprobots_ws/src/navigation_yeon/yeon/database/sub_mission.json', 'r') as f:
			pdict = json.load(f, object_pairs_hook=OrderedDict)
		return pdict

	def load_mission_database(self):
		with open('/home/jetson/nprobots_ws/src/navigation_yeon/yeon/database/mission.json', 'r') as f:
			tdict = json.load(f, object_pairs_hook=OrderedDict)
		return tdict

if __name__ == '__main__':
	try:
		FollowPath()
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation Finished.")




















