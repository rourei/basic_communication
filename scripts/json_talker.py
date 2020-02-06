#!/usr/bin/env python

'''
Talker node for testing usage of JSON files and format inside ROS. The node publishes two different topics:
1: json_chatter:  messages generated from a JSON-string created inside this node
2: konto_chatter: messages generated from an external .json file
'''

# Import utilities
import rospy
import json
import os
from rospy_message_converter import message_converter
from rospy_message_converter import json_message_converter
# Import necessary ROS messages
from std_msgs.msg import String
from basic_communication.msg import Kreditkarte


########################## Talker function ##########################
def talker(message_karte, message_str):
    # Create publisher objects
    pub_str = rospy.Publisher('json_chatter', String, queue_size=10)
    pub_karte = rospy.Publisher('karte_chatter', Kreditkarte, queue_size=10)
    # Initialize node and publish rate
    rospy.init_node('json_test_talker', anonymous=True)
    rate = rospy.Rate(10)

    # Publish messages
    while not rospy.is_shutdown():
        pub_karte.publish(message_karte)
        pub_str.publish(message_str)
        rate.sleep()


########################## main ##########################
if __name__ == '__main__':

    # Determine directory that contains this script
    fileDir = os.path.dirname(os.path.realpath(__file__))
    # Get .json file path in parent directory
    filename = os.path.realpath(os.path.join(fileDir, "../example.json"))

    # Open .json file that contains data => format is the same as in basic_communication/Konto.msg
    with open(filename) as input_file:
        data_from_file = json.load(input_file) # data_from_file is type dict

    # Generate message from data
    msg_karte = message_converter.convert_dictionary_to_ros_message('basic_communication/Kreditkarte', data_from_file)

    # Create string variable in JSON format
    json_str = '{"data": "Hello from the JSON string!"}'
    # Generate message from string
    msg_str = json_message_converter.convert_json_to_ros_message('std_msgs/String', json_str)

    # Run the talker node
    try:
        talker(msg_karte, msg_str)
    except rospy.ROSInterruptException:
        pass
