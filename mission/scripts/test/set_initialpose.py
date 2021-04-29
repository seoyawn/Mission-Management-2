#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseActionGoal 
from std_msgs.msg import String

if __name__ == '__main__':
	rospy.init_node('yeon_send_a_goal')
	pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
	       

       	myGoal = MoveBaseActionGoal()
       	myGoal.header.frame_id = "/map"
       	myGoal.header.stamp = rospy.Time.now()
       	myGoal.goal.target_pose.header.frame_id = "/map"
       	myGoal.goal.target_pose.pose.position.x= 0
       	myGoal.goal.target_pose.pose.position.y=0
       	myGoal.goal.target_pose.pose.orientation.w = 1

       	pub.Publish(myGoal)
       	rospy.loginfo("A goal has been sent.")

       	rospy.spin()

		 



#       pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size = 10)
#        pub = rospy.Publisher('/test123', String, queue_size=10)

#       pub1 = rospy.Publisher('/test1', String, queue_size = 10)

#       myGoal = MoveBaseActionGoal()
#       myGoal.header.frame_id = "/map"
#       myGoal.header.stamp = rospy.Time.now()
#       myGoal.goal.target_pose.header.frame_id = "/map"
#       myGoal.goal.target_pose.pose.position.x= 5
#       myGoal.goal.target_pose.pose.position.y=2
#       myGoal.goal.target_pose.pose.orientation.w = 1

 #       info  = String()
  #      info.data = "testing"
   #     pub.Publish(info)
    #    rospy.loginfo("A goal has been sent.")

     #   rospy.spin()

