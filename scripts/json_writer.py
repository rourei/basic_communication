#!/usr/bin/env python

'''

'''

# Import utilities
import rospy
import json
import os
from rospy_message_converter import json_message_converter
# Import necessary ROS messages
from std_msgs.msg import String



########################## Callbacks and functions ##########################
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    rospy.spin()


########################## Main ##########################
if __name__ == '__main__':
    listener()
