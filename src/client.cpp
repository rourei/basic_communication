#include <ros/ros.h>

#include <dynamic_reconfigure/client.h>
#include <dynamic_tutorials/TutorialsConfig.h>



int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamic_tutorials_client");
    ros::NodeHandle n;

    ros::Rate loop_rate(0.1);

    // Create client object
    dynamic_reconfigure::Client<dynamic_tutorials::TutorialsConfig> client("dynamic_tutorials_server");
    dynamic_tutorials::TutorialsConfig cfg;

    ROS_INFO("cfg after init: %d %f %s %s %d",
             cfg.int_param, cfg.double_param,
             cfg.str_param.c_str(),
             cfg.bool_param?"True":"False",
             cfg.size);


    ROS_INFO("Spinning node");

    client.getCurrentConfiguration(cfg, ros::Duration(5));
    ROS_INFO("Initial configuration after first getting: %d %f %s %s %d",
             cfg.int_param, cfg.double_param,
             cfg.str_param.c_str(),
             cfg.bool_param?"True":"False",
             cfg.size);

    while(ros::ok())
    {
        if (client.getCurrentConfiguration(cfg, ros::Duration(5)))
        {
            ROS_INFO("Current configuration: %d %f %s %s %d",
                     cfg.int_param, cfg.double_param,
                     cfg.str_param.c_str(),
                     cfg.bool_param?"True":"False",
                     cfg.size);
            cfg.int_param++;
        }
        ROS_INFO("New configuration: %d %f %s %s %d",
                 cfg.int_param, cfg.double_param,
                 cfg.str_param.c_str(),
                 cfg.bool_param?"True":"False",
                 cfg.size);
        client.setConfiguration(cfg);
        ROS_INFO("Config has been sent. Sleeping.");
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}

