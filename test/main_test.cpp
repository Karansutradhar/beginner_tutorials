/**
 *  Copyright 2020 Karan Sutradhar
 *  @file main_test.cpp
 *  @author Karan Sutradhar
 *  @date 11/10/2020
 * 
 *  @brief execution of all the unit tests
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
 *  Main source file for running all unit tests
 *
 */

/**
 * Run all the tests that were declared with TEST()
 * @param none
 * @returns results
 */


#include "ros/ros.h"
#include <gtest/gtest.h>
#include "beginner_tutorials/UpdateString.h"

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test1");
  ros::NodeHandle n;
  return RUN_ALL_TESTS();
}