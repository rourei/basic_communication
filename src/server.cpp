#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <dynamic_tutorials/TutorialsConfig.h>


// Callback will be called when new configuration is received
void callback(dynamic_tutorials::TutorialsConfig &config, uint32_t level) {
    // For now, it's only printig the new configuration
    ROS_INFO("Reconfigure Request: %d %f %s %s %d",
             config.int_param, config.double_param,
             config.str_param.c_str(),
             config.bool_param?"True":"False", // inline function see https://stackoverflow.com/a/29798
             config.size);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamic_tutorials_server");

    // Create server object, passing the config type
    dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig> server;
    // Creating the callback that is called upon receiving a new config
    dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig>::CallbackType f;

    // Variable to represent callback
    f = boost::bind(&callback, _1, _2);
    // Send the variable to server so it will use this custom defined callback
    server.setCallback(f);

    // Run
    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}
