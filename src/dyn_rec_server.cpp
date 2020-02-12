/*
 * Server node that can be reconfigured by a dynamic_reconfigure client on runtime.
 * Source: http://wiki.ros.org/dynamic_reconfigure/Tutorials
 */

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <basic_communication/TutorialsConfig.h>
#include <bitset>
#include <iostream>


// Callback will be called when new configuration is received
void callback(basic_communication::TutorialsConfig &config, uint32_t level) {
    // For now, it's only printig the new configuration
    ROS_INFO("Reconfigure Request: %d %f %s %s %d",
             config.int_param, config.double_param,
             config.str_param.c_str(),
             config.bool_param?"True":"False",
             config.size);
    ROS_INFO("Level param is %d", level);
    // Convert level to bit set and display as string
    std::bitset<32> level_bitset;
    level_bitset.set(level);
    std::cout << "Level param as bitset = " << level_bitset.to_string() << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "basic_communication_server");

    // Create server object, passing the config type
    dynamic_reconfigure::Server<basic_communication::TutorialsConfig> server;
    // Creating the callback that is called upon receiving a new config
    dynamic_reconfigure::Server<basic_communication::TutorialsConfig>::CallbackType f;

    // Variable to represent callback
    f = boost::bind(&callback, _1, _2);
    // Send the variable to server so it will use this custom defined callback
    server.setCallback(f);

    // Run
    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}
