#!/usr/bin/env python
import rospy
from std_msgs.msg import String
   
pub = rospy.Publisher('chatter', String, queue_size=None)
rospy.init_node('demo_pub_node')
r = rospy.Rate(1) # 10hz
while not rospy.is_shutdown():
   pub.publish("hello world")
   print('sending data...')
   r.sleep()