/*******************************************************************
 *
 * fwd_health_task_tools
 *
 * Copyright (c) 2020,
 * FORWARDttc GmbH
 * (LICENSE)
 * All rights reserved.
 *
 * https://www.forward-ttc.de/
 *
 * This software is distributed WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.
 *
 * Author: Rouven Reichert (rouven.reichert@forward-ttc.de)
 *
 ******************************************************************/

/*
 * ### INFO ###
 * https://stackoverflow.com/a/49201798
 *
 * [ ] matcht Zeichen, die in der Klammer stehen
 * [^ ] matcht Zeichen, die NICHT in der Klammer stehen
 * innerhalb dieser Klammern muessen Sonderzeichen (z.B. '.') nicht escaped werden
 * \s kennzeichnet whitespace \S alles ausser whitespace
 * \d kennzeichnet eine Zahl, \D alles ausser einer Zahl
 * \w keennzeichnet Buchstabe, \W alles ausser Buchstabe
 *
 * regex_search bricht nach dem ersten gefundenen Match ab
 * -> Abhilfe schafft ein Iterator: https://stackoverflow.com/a/49725378
 */

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <regex>


// ###### MAIN #########################################################################################################
int main(int argc, char** argv)
{
  ros::init(argc, argv, "regex_string_splitting");
  ros::NodeHandle nh("~");

  ROS_INFO("Node is spinning...\n");

  // -------------------------------------------------------------------------------------------------------------------

  // ------ MOVE_BASE_GOAL EXAMPLE ------
  const std::string position1 ("x: 1.68   y: 1.45");
  const std::string position2 ("x: 2.68 y:2.45 z: 254.12");
  const std::string orientation = "x: 0.0 y: 0.0 z: 0.0 w: 1.0";


  // Define regular expression
  std::regex regex(R"(([xyzw]):\s*(\d+\.\d+))");

  // Define matching variable
  std::smatch matches;

  // -------------------------------------------------------------------------------------------------------------------

  std::cout << "\n### ITERATOR-VERSION ###\n";
  // Findet alle Matches separat, dafür nicht gruppiert (nur x: 1.0)

  std::sregex_token_iterator a ( position1.begin(), position1.end(), regex );
  std::sregex_token_iterator b ( position2.begin(), position2.end(), regex );
  std::sregex_token_iterator c ( orientation.begin(), orientation.end(), regex );
  std::sregex_token_iterator rend; // default constructor = end-of-sequence:

  while (a != rend) std::cout << "[" << *a++ << "]";
  std::cout << std::endl;

  while (b != rend) std::cout << "[" << *b++ << "]";
  std::cout << std::endl;

  while (c != rend) std::cout << "[" << *c++ << "]";
  std::cout << std::endl;

  // -------------------------------------------------------------------------------------------------------------------

  std::cout << "\n### WHILE-VERSION ###";
  // Findet alle Matches, auch gruppiert

  // Temp-Variablen notwendig, weil das bereits gefundene Match abgeschnitten wird -> String wird verändert
  std::string temp_pos1 = position1;
  std::string temp_pos2 = position2;
  std::string temp_orientation = orientation;

  while(std::regex_search(temp_pos1, matches, regex))
  {
    std::cout << std::endl << "[POS1] size of matches is " << matches.size() << std::endl;

    for (int i = 0; i < matches.size(); i++)
    {
      std::cout << matches[i] << std::endl;
    }

    temp_pos1 = matches.suffix();
  }

  while(std::regex_search(temp_pos2, matches, regex))
  {
    std::cout << std::endl << "[POS2] size of matches is " << matches.size() << std::endl;

    for (int i = 0; i < matches.size(); i++)
    {
      std::cout << matches[i] << std::endl;
    }

    temp_pos2 = matches.suffix();
  }

  while(std::regex_search(temp_orientation, matches, regex))
  {
    std::cout << std::endl << "[OR] size of matches is " << matches.size() << std::endl;

    for (int i = 0; i < matches.size(); i++)
    {
      std::cout << matches[i] << std::endl;
    }

    temp_orientation = matches.suffix();
  }

  // -------------------------------------------------------------------------------------------------------------------

  std::cout << "\n### IF-VERSION ###";
  // Findet nur das jeweils erste Match, dafür aber auch gruppiert (einmal komplett, einmal nur die Zahl)

  if (std::regex_search(position1, matches, regex))
  {
    std::cout << std::endl << "[POS1] size of matches is " << matches.size() << std::endl;

    for (int i = 0; i < matches.size(); i++)
    {
      std::cout << matches[i] << std::endl;
    }
  }

  if (std::regex_search(position2, matches, regex))
  {
    std::cout << std::endl << "[POS2] size of matches is " << matches.size() << std::endl;

    for (int i = 0; i<matches.size(); i++)
    {
      std::cout << matches[i] << std::endl;
    }
  }

  ros::spin();

  return 0;
}
