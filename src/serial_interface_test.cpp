/*
 * Node to test interfacing serial devices (e.g. Arduinos) using boost.
 * Date: 2020/06/14
 */

#include "ros/ros.h"
#include <iostream>
#include <stdio.h>
#include <fstream>
#include "boost/asio.hpp"

// ###### MAIN #########################################################################################################
int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "serial_interface_test");

  // AsyncSpinner for spinning enough threads
  ros::AsyncSpinner spinner(0);
  spinner.start();

  // Base serial settings
  boost::asio::serial_port_base::baud_rate BAUD(9600);
  boost::asio::serial_port_base::flow_control FLOW( boost::asio::serial_port_base::flow_control::none );
  boost::asio::serial_port_base::parity PARITY( boost::asio::serial_port_base::parity::none );
  boost::asio::serial_port_base::stop_bits STOP( boost::asio::serial_port_base::stop_bits::one );

  boost::asio::io_service io;
  boost::asio::serial_port port(io, "/dev/ttyACM0");

  // Setup port - base settings
  port.set_option( BAUD );
  port.set_option( FLOW );
  port.set_option( PARITY );
  port.set_option( STOP );

  unsigned char input;
  char c;
  while(ros::ok()){
    // Send -------------------------------------------------------------------
    // What to send
    std::cin >> input;

    // Output buffer
    unsigned char command[1] = {0};

    // Convert and send
    command[0] = static_cast<unsigned char>( input );
    boost::asio::write(port, boost::asio::buffer(command, 1));

    // Receive response -------------------------------------------------------
//    boost::asio::read(port, boost::asio::buffer(&c,1));
//    std::cout << c << std::endl;
  }

  ros::waitForShutdown();

  return 0;
}
