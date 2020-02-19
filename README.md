# basic_communication

This package includes various nodes that for testing communication concepts for my master thesis.

---

## Nodes / Scripts

- **dyn_rec_server.cpp, dyn_rec_client.cpp and dyn_rec_client_double.cpp**
  - nodes to test the [C++ client API](https://github.com/ros/dynamic_reconfigure/blob/melodic-devel/include/dynamic_reconfigure/client.h) of *dynamic_reconfigure*
  - the client nodes use the [AsyncSpinner](https://roboticsbackend.com/ros-asyncspinner-example/) to handle timing of callbacks properly
  - explanation of the *level* parameter in the dyn_rec_server callback can be found [here](https://answers.ros.org/question/9883/what-are-the-semantics-of-reconfiguration-level-for-dynamic-reconfigure/)

- **json_talker.py**
  - script for testing the *rospy_message_converter* that can be found [here](https://github.com/uos/rospy_message_converter)
  - takes *example.json* as an input and converts it into the custom message type *Kreditkarte*
  - the message is then published via ROS topic
  - additionally it takes a JSON formatted string inside the script and publishes it on another topic

- **json_writer.py**
  - receives the ROS message of type *Kreditkrate* from the *json_talker.py*
  - alters the fields of the message, converts it to JSON format and saves it to *example_written.json*

- **latched_talker and latched_listener**
  - nodes to show how the concept of latching a topic works
  - the *latched_talker* publishes the latched topic *latched_chatter* which means that the last message sent will always be buffered and sent to new subscribers - even if they subscribe **after** the message has been sent
  - by starting the *latched_listener* after a message has been sent by the talker this behaviour can be observed
  - an additional `rostopic echo /latched_chatter` shows that the message is not sent continuously but is still received by the listener node
