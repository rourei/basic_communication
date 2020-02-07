#!/usr/bin/env python

'''
Script that listens to a topic and writes the received messages into a JSON file.
Conversion: msg -> Python dictionary -> .json file

'''

# Import utilities
import rospy
import json
import os
from rospy_message_converter import message_converter
# Import necessary ROS messages
from basic_communication.msg import Kreditkarte


########################## Callbacks and functions ##########################
def callback(card):

    # Convert message data to dictionary
    json_dict = message_converter.convert_ros_message_to_dictionary(card)

    # Alter contained data
    json_dict["Herausgeber"] = "Volksbank, nicht Xema"
    json_dict["Inhaber"] = "Manuela Mustermann"
    json_dict["Deckung"] = 500

    # Determine directory that contains this script
    fileDir = os.path.dirname(os.path.realpath(__file__))
    # Get .json file path in parent directory
    filename = os.path.realpath(os.path.join(fileDir, "../example_written.json"))

    # Open .json file and write dictionary in JSON format
    with open(filename, "w") as output_file:
        json.dump(json_dict, output_file)



def listener():

    # Initialize ROS and the subscriber
    rospy.init_node('json_writer', anonymous=True)
    rospy.Subscriber("karte_chatter", Kreditkarte, callback)
    rospy.spin()


########################## Main ##########################
if __name__ == '__main__':
    listener()
