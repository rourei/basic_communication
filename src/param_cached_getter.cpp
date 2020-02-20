/*
 * Node that reads 'test_param' from server and only re-reads it if it has been updated since the last call.
 * Date: 2020/02/20
 *
 * Info:
 * - possible via nodehandle.getParamCached() or ros::param::getCached()
 * - nodehandle calls ros::param::getCached() internally
 */

#include "ros/ros.h" // FYI: ros.h includes string already
#include <string>

// ############ MAIN ############
int main(int argc, char **argv)
{

    // Initialize ROS
    ros::init(argc, argv, "param_cached_getter");
    // Create NodeHandle object
    ros::NodeHandle nh;

    // Variable to store the read parameter value
    std::string s;

    // Define loop rate
    ros::Rate loop_rate(0.1);

    // ### LOOP ###
    while (ros::ok())
    {
        // Returns true when the parameter can be retrieved (cached or from server)
        if (ros::param::getCached("/test_param", s))
        {
            ROS_INFO("Return value: TRUE");
        }
        else
        {
            ROS_INFO("Return value: FALSE");
        }

        // Output parameter value
        ROS_INFO("Parameter is %s", s.c_str());

        // Spin and sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
