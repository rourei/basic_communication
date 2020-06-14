/*
 * Node to test the usage of regular expressions in C++.
 * Date: 2020/05/30
 */

/*
 * ### INFO ###
 * https://stackoverflow.com/a/49201798
 *
 * [ ]  matches characters inside the brackets
 * [^ ] matches characters NOT inside the brackets
 *      -> special characters (e.g. '.') do not need to be escaped
 * \s = whitespace, \S every character except whitespace
 * \d = number,     \D every character except numbers
 * \w = letter,     \W every character except letters
 *
 * regex_search stops after first match
 * -> better to use iterators: https://stackoverflow.com/a/49725378
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
  // Finds all matches separately, but not grouped (only x: 1.0)

  std::sregex_token_iterator a ( position1.begin(), position1.end(), regex );
  std::sregex_token_iterator b ( position2.begin(), position2.end(), regex );
  std::sregex_token_iterator c ( orientation.begin(), orientation.end(), regex );
  std::sregex_token_iterator rend; // default constructor = end-of-sequence

  while (a != rend) std::cout << "[" << *a++ << "]";
  std::cout << std::endl;

  while (b != rend) std::cout << "[" << *b++ << "]";
  std::cout << std::endl;

  while (c != rend) std::cout << "[" << *c++ << "]";
  std::cout << std::endl;

  // -------------------------------------------------------------------------------------------------------------------

  std::cout << "\n### WHILE-VERSION ###";
  // Finds all matches, also grouped

  // Temporary variables necessary, because every match that has been found is cut off -> string is altered
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
  // Only finds the first match, but also grouped (one time the complete match, and once only the number)

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
