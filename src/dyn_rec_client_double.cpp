/*
 * Client node that reconfigures a server node via dynamic_reconfigure on runtime.
 * This node increments the double parameter by 0.05 at 0.1Hz.
 */

#include <ros/ros.h>
#include <dynamic_reconfigure/client.h>
#include <basic_communication/TutorialsConfig.h>


// ########### Callbacks ###########
void configurationCallback(const basic_communication::TutorialsConfig& config){
    ROS_INFO("configurationCallback: %d %f %s %s %d",
             config.int_param, config.double_param,
             config.str_param.c_str(),
             config.bool_param?"True":"False",
             config.size);
}

void descriptionCallback(const dynamic_reconfigure::ConfigDescription& description){
    ROS_INFO("New description received");
}

// ########### Main ###########

int main(int argc, char **argv) {

    // Initialize ROS, NodeHandle
    ros::init(argc, argv, "basic_communication_client_double");
    ros::NodeHandle n;

    // Define loop rate
    ros::Rate loop_rate(0.1);

    // AsyncSpinner for handling timing properly
    ros::AsyncSpinner spinner(0);
    spinner.start();


    // Create client, config object and description object
    dynamic_reconfigure::Client<basic_communication::TutorialsConfig> client("/basic_communication_server", &configurationCallback, &descriptionCallback);
    basic_communication::TutorialsConfig cfg;
    dynamic_reconfigure::ConfigDescription descrpt;


    ROS_INFO("Spinning node");

    // Read current configuration from server
    if (!client.getCurrentConfiguration(cfg, ros::Duration(1)))
    {
        ROS_INFO("Timeout on first getCurrentConfig");

    }

    // ### Loop ###
    while(ros::ok())
    {
        if (client.getCurrentConfiguration(cfg, ros::Duration(1)))
        {
            // Change parameter in config
            cfg.double_param = cfg.double_param + 0.05;
        }
        else if (!client.getCurrentConfiguration(cfg, ros::Duration(1)))
		{
			ROS_INFO("Timeout in loop.");
		}

        // Send new configuration to server
        client.setConfiguration(cfg);

        // DEBUG
        ROS_INFO("Config has been sent. Sleeping.");

        // Spin the loop
        loop_rate.sleep();
    }

    ros::waitForShutdown();
    return 0;
}

