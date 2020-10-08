#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
from geometry_msgs.msg import Twist
import turtlesim.srv
from sensor_msgs.msg import LaserScan

if __name__ == '__main__':
     rospy.init_node('pursuer')
     global_ranges = []
     listener = tf.TransformListener()
     turtle_vel = rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=1)

     #rospy.wait_for_service('spawn')
     #spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
     #spawner(4, 2, 0, 'turtle2')
     rospy.sleep(4)
     rate = rospy.Rate(10)
     while not rospy.is_shutdown():
         try:
                now = rospy.Time.now()
		past = now - rospy.Duration(1.0)
		listener.waitForTransformFull("/robot_1", now, "/robot_0", past, "/stage", rospy.Duration(1.0))
 	 	(trans,rot) = listener.lookupTransformFull('/robot_1', now, '/robot_0', past, "/stage")
 	 	print("Pursuer!")
         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	     print("Exception occured:")
             continue

         angular = 4 * math.atan2(trans[1], trans[0])
         linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
	 
         cmd = Twist()
         cmd.linear.x = linear
         cmd.angular.z = angular
         turtle_vel.publish(cmd)
         rate.sleep()
