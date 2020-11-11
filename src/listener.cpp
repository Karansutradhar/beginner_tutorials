/**
 * @file talker.cpp
 * @author Karan Sutradhar
 * @brief The talker.cpp file for beginner_tutorials ROS package.
 * It contains code for publishing a simple message.
 * @Copyright "Copyright 2020" <Karan Sutradhar>
 */
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/UpdateString.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO_STREAM("I heard:," << msg->data.c_str());
}

int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  std::string message;
  
  /**
   * getParam functions gets the parameter fed by the user while launching the .launch file
   */

  n.getParam("message", message);
  std::cout << "\n" << message;
  std::cout << "The program has nothing to say";
  while (ros::ok()) {

    ros::Subscriber subject = n.subscribe("chatter", 1000, chatterCallback);
    /** 
   * ServiceClient creates a client of the service UpdateString
   */
    ros::ServiceClient client =
    n.serviceClient<beginner_tutorials::UpdateString>("conversation");

    /**
     * srv is a service object that contains the attributes in UpdateString.srv. The user 
     * request message with their answer to the question
     */
    beginner_tutorials::UpdateString srv;
    std::cout << "ROS is Best what do you think? (anthing/nothing)";
    std::cin >> srv.request.inputString;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  if (client.call(srv)) {
    if (srv.response.outputString == "Warnings") {
        ROS_WARN_STREAM("Did not recieve any message");
      } else {
        ROS_INFO_STREAM(srv.response.outputString);
      }
    } else {
      ROS_ERROR_STREAM("Failed to call service UpdateString");
    }

    /**
     * If the user responds n to the question, the program gives off a fatal error 
     */
  if (srv.request.inputString == "nothing") {
      ROS_FATAL_STREAM("The user doesn't use ROS");
    }

  ros::spin();
}

  return 0;
}
