#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import random
from sensor_msgs.msg import LaserScan

pub = None

def callback(msg):
	vel_msg = Twist()
	vel_msg.linear.x = 2
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0

	ranges = list(msg.ranges)
	ranges = ranges[40:321]
	#cnt = len(ranges)
	#total = 0
	#for val in ranges:
	#	total += val
	#total /= cnt
	total = min(ranges)
	print("Average: "+str(total))
	if total < 1.2:
		print("Rotating")
		#angle = random.uniform(-1, 1) * 6.28
		posNeg = random.uniform(0, 1)
		angle = random.uniform(2, 3.14)
		#if posNeg < 0.5:
		#	angle *= -1
		print("Angle: "+str(angle))
		vel_msg.linear.x = 0
		vel_msg.angular.z = angle
		rate = rospy.Rate(20)
		rotationCount = 0
		while not rospy.is_shutdown():
			if rotationCount > 10: 
				#print("Rotations exceed stopping..")
				break
			#print("Rotating.."+str(rotationCount))
			pub.publish(vel_msg)
			rate.sleep()
			rotationCount += 1

		vel_msg.angular.z = 0
		vel_msg.linear.x = 2
		# now move it 10 steps stright
		stepCount = 0
		while not rospy.is_shutdown():
			if stepCount > 3:
				#print("Steps exceeded..")
				break	
			#print("Forward step: "+str(stepCount))
			pub.publish(vel_msg)
			rate.sleep()
			stepCount += 1

	pub.publish(vel_msg)

if __name__ == '__main__':
	try:
		rospy.init_node('evader_brain')
		pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
		sub = rospy.Subscriber("/robot_0/base_scan", LaserScan, callback)
		#mover()
		rospy.spin()
     	except rospy.ROSInterruptException:
        	pass
