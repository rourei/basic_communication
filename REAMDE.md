# basic_communication

This package includes various nodes that for testing communication concepts for my master thesis.

---

## Nodes / Scripts

- **server.cpp and client.cpp**
 - nodes to test the [C++ client API](https://github.com/ros/dynamic_reconfigure/blob/melodic-devel/include/dynamic_reconfigure/client.h) of *dynamic_reconfigure*

- **json_talker.py**
 - script for testing the *rospy_message_converter* that can be found [here](https://github.com/uos/rospy_message_converter)
 - takes *example.json* as an input and converts it into the custom message type *Kreditkarte*
 - the message is then published via ROS topic
 - additionally it takes a JSON formatted string inside the script and publishes it on another topic
