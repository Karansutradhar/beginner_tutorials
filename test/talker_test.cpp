/**
 *  Copyright 2020 Karan Sutradhar
 *  @file talker_test.cpp
 *  @author Karan Sutradhar
 *  @date 11/06/2020
 * 
 *  @brief The Unit tests for talker node
 *
 *  @section LICENSE
 *  
 * MIT License
 * Copyright (c) 2020 Karan Sutradhar
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 *  @section DESCRIPTION
 *
 *  Source file containing unit tests for talker node
 *
 */
#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/UpdateString.h"

/**
 * Declare a test
 */
TEST(testNode, testCase){

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient
  <beginner_tutorials::UpdateString>("UpdateString");
  /**
   * test whether the client exists
   */
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5)));
  beginner_tutorials::UpdateString srv;
    srv.request.inputString = "update";
    client.call(srv);
    /**
   * test whether the correct response is output
   */
    EXPECT_EQ("update", srv.response.outputString);
}
