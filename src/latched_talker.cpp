/*
 * Node that publishes a latched topic at a rate of 0.1Hz for conceptual testing.
 * Date: 2020/02/19
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>


int main(int argc, char **argv)
{

    // Initialize ROS
    ros::init(argc, argv, "latched_talker");
    // Create NodeHandle object
    ros::NodeHandle nh;

    // Argument 'true' toggles lacthing
    ros::Publisher latched_chatter_pub = nh.advertise<std_msgs::String>("latched_chatter", 1000, true);

    ros::Rate loop_rate(0.1);

    // Publishing loop
    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "Hello World " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        latched_chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        count++;
    }


    return 0;
}
