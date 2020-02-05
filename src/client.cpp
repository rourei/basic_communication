/*
 * Client node that reconfigures a server node via dynamic_reconfigure on runtime.
 */

#include <ros/ros.h>
#include <dynamic_reconfigure/client.h>
#include <dynamic_tutorials/TutorialsConfig.h>

dynamic_tutorials::TutorialsConfig CFG;
dynamic_reconfigure::ConfigDescription DESCRPT;


// ########### Callbacks ###########
void configurationCallback(const dynamic_tutorials::TutorialsConfig& config){
    ROS_INFO("configurationCallback: %d %f %s %s %d",
             config.int_param, config.double_param,
             config.str_param.c_str(),
             config.bool_param?"True":"False",
             config.size);

    CFG = config;
}

void descriptionCallback(const dynamic_reconfigure::ConfigDescription& description){
    ROS_INFO("New description received");
}

// ########### Main ###########

int main(int argc, char **argv) {

    // Initialize ROS, NodeHandle necessary for using
    ros::init(argc, argv, "dynamic_tutorials_client");
    ros::NodeHandle n;

    // Define loop rate
    ros::Rate loop_rate(0.1);

    // Create client and config object
    dynamic_reconfigure::Client<dynamic_tutorials::TutorialsConfig> client("/dynamic_tutorials_server", &configurationCallback, &descriptionCallback);


    // DEBUG
    ROS_INFO("Config after init of client: %d %f %s %s %d",
             CFG.int_param, CFG.double_param,
             CFG.str_param.c_str(),
             CFG.bool_param?"True":"False",
             CFG.size);


    ROS_INFO("Spinning node");

    // Read current configuration from server
    if (!client.getCurrentConfiguration(CFG, ros::Duration(10)))
    {
        ROS_INFO("Timeout on first getCurrentConfig");

    }

    // ### Loop ###
    while(ros::ok())
    {
        if (client.getCurrentConfiguration(CFG, ros::Duration(10)))
        {
            // DEBUG
            ROS_INFO("Current configuration (inside loop): %d %f %s %s %d",
                     CFG.int_param, CFG.double_param,
                     CFG.str_param.c_str(),
                     CFG.bool_param?"True":"False",
                     CFG.size);

            // Change paramter in config
            CFG.int_param = CFG.int_param + 3;
        }

        // DEBUG
        ROS_INFO("New configuration (inside loop): %d %f %s %s %d",
                 CFG.int_param, CFG.double_param,
                 CFG.str_param.c_str(),
                 CFG.bool_param?"True":"False",
                 CFG.size);

        // Sent new configuration to server
        client.setConfiguration(CFG);

        // DEBUG
        ROS_INFO("Config has been sent. Sleeping.");

        // Spin the loop
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}

