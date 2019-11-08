/**
 * Distributed under the BSD License (license terms found in LICENSE or at https://opensource.org/licenses/BSD-2-Clause)
 *
 * @file main.cpp
 *
 * @brief Main file to test talker node.
 *
 * @author Andre Ferreira
 *
 * @copyright  Andre Ferreira
 *
 */

#include <gtest/gtest.h>
#include <ros/ros.h>

/**
 * @brief main function to call all tests
 *
 * @param argc 
 * @param argv 
 *
 * @return     none
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "testTalkerNode");
  testing::InitGoogleTest(&argc, argv);

/// Creating node handle
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}

