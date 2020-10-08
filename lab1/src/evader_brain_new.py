#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import random
from sensor_msgs.msg import LaserScan

class Problem1:
	def __init__(self):
		self.pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
		self.global_ranges = []
		self.global_turning_angle = -1

	def getEmptyTwist(self):
		vel_msg = Twist()
		vel_msg.linear.x = 0
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = 0
		return vel_msg

	def getRangeAv(self):
		ranges = self.global_ranges[40:321]
		cnt = len(ranges)
		total = 0
		for val in ranges:
			total += val
		total /= cnt
		return total

	def shouldTurn(self):
		rangeAverage = self.getRangeAv()
		print("Average Range: "+str(rangeAverage))
		return rangeAverage < 1.2
		
	def turn(self):
		vel_msg = self.getEmptyTwist()
		vel_msg.angular.z = self.global_turning_angle
		self.pub.publish(vel_msg)

	def goStraight(self):
		vel_msg = self.getEmptyTwist()
		vel_msg.linear.x = 2
		self.pub.publish(vel_msg)

	def mover(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			if len(self.global_ranges) != 0:
				if self.shouldTurn():
					if self.global_turning_angle == -1:
						# i.e. it has not been set in the previous iteration, then set it
						self.global_turning_angle = random.uniform(2, 3.14)
						"""neg = random(0, 1)
						if neg < 0.5:
							self.global_turning_angle *= -1"""
					else #use the already populated angle
						self.turn()
				else:
					self.global_turning_angle = -1
					self.goStraight()
			else:
				print("Global ranges len = 0 ")
			rate.sleep()

	def callback(self, msg):
		#print("Subscriber called. Updating the ranges..")
		self.global_ranges = list(msg.ranges)

if __name__ == '__main__':
	try:
		rospy.init_node('evader_brain', anonymous=True)
		prob1 = Problem1()
		sub = rospy.Subscriber("/robot_0/base_scan", LaserScan, prob1.callback)
		prob1.mover()
		rospy.spin()
     	except rospy.ROSInterruptException:
        	pass
