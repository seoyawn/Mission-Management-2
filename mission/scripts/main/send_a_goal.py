#!/usr/bin/env python 
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback 

def feedback_callback(feedback):
	print('[Feedback] going to goal pose...')


if __name__ == "__main__":
	rospy.init_node("send_a_goal")

	#connect to a action server called '/move_base'
	client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
	
	client.wait_for_server()

	my_goal = MoveBaseGoal()
	my_goal.target_pose.header.frame_id = "map"
	my_goal.target_pose.pose.position.x= 1
	my_goal.target_pose.pose.position.y= 1
	my_goal.target_pose.pose.orientation.w = 1

	client.send_goal(my_goal, feedback_cb = feedback_callback)
	rospy.loginfo("A goal has been sent.")

	client.wait_for_result()
	rospy.loginfo(client.get_state()); 

	if rospy.ROSInterruptException:
		rospy.loginfo("Robot has reached the goal")
	else:
		rospy.spin() 
