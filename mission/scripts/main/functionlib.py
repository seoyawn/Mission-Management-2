#!/usr/bin/env python

import rospy

def pause(self, time):
	rospy.sleep(time)
	return True

def null():
	return True

def hello():
	rospy.loginfo("Hello, nice to meet you!")
	return True

def bye():
	rospy.loginfo("Bye World!")
	return True

def smile():
	rospy.loginfo("Let's smile :-)")
	return True

def dance():
	rospy.loginfo("Let's dance together!")
	return True

