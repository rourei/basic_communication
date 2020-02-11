/*
 * Client node that reconfigures a server node via dynamic_reconfigure on runtime.
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

    // Initialize ROS, NodeHandle necessary for using
    ros::init(argc, argv, "basic_communication_client");
    ros::NodeHandle n;

    // Define loop rate
    ros::Rate loop_rate(0.1);

    // New Async Spinner object
    ros::AsyncSpinner spinner(0);
    spinner.start();


    // Create client, config object and description object
    dynamic_reconfigure::Client<basic_communication::TutorialsConfig> client("/basic_communication_server", &configurationCallback, &descriptionCallback);
    basic_communication::TutorialsConfig cfg;
    dynamic_reconfigure::ConfigDescription descrpt;


    // DEBUG
//    ROS_INFO("Config after init of client: %d %f %s %s %d",
//             cfg.int_param, cfg.double_param,
//             cfg.str_param.c_str(),
//             cfg.bool_param?"True":"False",
//             cfg.size);


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
            // DEBUG
            ROS_INFO("Current configuration (inside loop): %d %f %s %s %d",
                     cfg.int_param, cfg.double_param,
                     cfg.str_param.c_str(),
                     cfg.bool_param?"True":"False",
                     cfg.size);

            // Change paramter in config
            cfg.int_param = cfg.int_param + 1;
        }
        else if (!client.getCurrentConfiguration(cfg, ros::Duration(1)))
		{
			ROS_INFO("Timeout in loop.");
		}

        // DEBUG
        ROS_INFO("New configuration (inside loop): %d %f %s %s %d",
                 cfg.int_param, cfg.double_param,
                 cfg.str_param.c_str(),
                 cfg.bool_param?"True":"False",
                 cfg.size);

        // Sent new configuration to server
        client.setConfiguration(cfg);

        // DEBUG
        ROS_INFO("Config has been sent. Sleeping.");

        // Spin the loop
        loop_rate.sleep();
    }

    ros::waitForShutdown();
    return 0;
}

