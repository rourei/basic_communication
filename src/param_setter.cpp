/*
 * Node that updates a parameter 'test_param' on the paramater server at 0.1Hz to test parameter caching.
 * Date: 2020/02/20
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

// ############ MAIN ############
int main(int argc, char **argv)
{

    // Initialize ROS
    ros::init(argc, argv, "param_setter");
    // Create NodeHandle object
    ros::NodeHandle nh;

    // Define loop rate and counter variable
    ros::Rate loop_rate(0.1);
    int count = 0;

    // ### LOOP ###
    while (ros::ok())
    {
        // Construct parameter value
        std::stringstream ss;
        ss << "Hello World " << count;

        // Set parameter on server
        nh.setParam("test_param", ss.str());

        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

    return 0;
}
