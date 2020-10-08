#!/usr/bin/env python  
import roslib
import rospy
from nav_msgs.msg import Odometry
import tf

def handle_turtle_pose(msg):
     br = tf.TransformBroadcaster()
     br.sendTransform((msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z),
                      (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),
		       rospy.Time.now(),
                      "robot_0",
                      "stage")

if __name__ == '__main__':
     rospy.init_node('broadcast')
     #turtlename = rospy.get_param('~turtle')
     turtlename = "robot_0"
     rospy.Subscriber('/robot_0/odom', Odometry, handle_turtle_pose)
     rospy.spin()
