/**
 * Distributed under the BSD License (license terms found in LICENSE or at https://opensource.org/licenses/BSD-2-Clause)
 *
 * @file test.cpp
 *
 * @brief Implementing tests of talker node
 *
 * @author Andre Ferreira
 *
 * @copyright  Andre Ferreira
 *
 */
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ros/service_client.h>
#include "beginner_tutorials/changeBaseOtput.h"

/**
 * @brief  Test made to check the service call and its funcionalitty 
 *
 *
 * @param[in]  TESTSuite
 *
 * @param[in]  testChangeString
 *
 * @return     none
 *
 */
TEST(TESTSuite, testChangeString) {
/// creating handle of the node
  ros::NodeHandle nh;

/// Creating a service client for the service
  ros::ServiceClient client = nh.serviceClient
      < beginner_tutorials::changeBaseOtput > ("changeString");

/// Creating srv object of the service
  beginner_tutorials::changeBaseOtput srv;

/// setting string to the service request
  srv.request.stringInput = "test string";

/// Test if the system is being called
  bool serviceCall = client.call(srv);
  EXPECT_TRUE(serviceCall);

/// Test if the service is behaving properly comparing input and output
  EXPECT_STREQ("test string", srv.response.stringOutput.c_str());
}
