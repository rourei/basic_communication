/*
 * Node that listenes to a latched topic for conceptual testing.
 * Date: 2020/02/19
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

// ############ Callback ############
void latchedChatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

// ############ Main ############
int main(int argc, char **argv)
{

    // Initialize ROS and create NodeHandle object
    ros::init(argc, argv, "latched_listener");
    ros::NodeHandle nh;

    // Create subscriber object
    ros::Subscriber sub = nh.subscribe("latched_chatter", 1000, latchedChatterCallback);

    // ... and run
    ros::spin();

    return 0;
}
