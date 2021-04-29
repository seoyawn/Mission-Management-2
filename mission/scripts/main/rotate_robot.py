#!/usr/bin/env python 
import rospy
import time
from geometry_msgs.msg import Twist, Vector3

if __name__ == "__main__":
	
	rospy.init_node('rotate_robot')
		
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 5)

	t_end = time.time() + 11
	finish = True

	while finish and time.time() < t_end:
	
		move = Twist()

		move.linear.x = 0.0
		move.linear.y = 0.0
		move.angular.z = 0.5

		pub.publish(move)





