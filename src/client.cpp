/*
 * Client node that reconfigures a server node via dynamic_reconfigure on runtime.
 */

#include <ros/ros.h>
#include <dynamic_reconfigure/client.h>
#include <basic_communication/TutorialsConfig.h>

basic_communication::TutorialsConfig CFG;
dynamic_reconfigure::ConfigDescription DESCRPT;


// ########### Callbacks ###########
void configurationCallback(const basic_communication::TutorialsConfig& config){
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
    ros::init(argc, argv, "basic_communication_client");
    ros::NodeHandle n;

    // Define loop rate
    ros::Rate loop_rate(0.1);

    // Create client and config object
    dynamic_reconfigure::Client<basic_communication::TutorialsConfig> client("/basic_communication_server", &configurationCallback, &descriptionCallback);


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
		else if (!client.getCurrentConfiguration(CFG, ros::Duration(10)))
		{
			ROS_INFO("Timeout in loop.");
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

