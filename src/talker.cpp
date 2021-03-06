/**
 * @file talker.cpp
 * @author Karan Sutradhar
 * @brief The talker.cpp file for beginner_tutorials ROS package.
 * It contains code for publishing a simple message.
 * @Copyright "Copyright 2020" <Karan Sutradhar>
 */

#include <sstream>
#include <tf/transform_broadcaster.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include "beginner_tutorials/UpdateString.h"

std::string actualMsgs = "I am the ROS talker talking";

bool UpdateString(
    beginner_tutorials::UpdateString::Request& request,
    beginner_tutorials::UpdateString::Response& response) {
  ROS_INFO_STREAM("request: " << request.inputString);
  if (request.inputString == "anything") {
    response.outputString = "Yes";
  } else if (request.inputString == "nothing") {
    response.outputString = "Wrong";
  } else {
    request.inputString = "Warnings";
  }
  ROS_INFO_STREAM("Responding back the messages " << response.outputString);
  return true;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
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
  ros::init(argc, argv, "talker");

  int frequency = 10;

  if (argc >= 2)
    int frequency = atoi(argv[1]);

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  tf::TransformBroadcaster b;
  tf::Transform transform;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok()){
    /**
     * ServiceServer creates a server of the service UpdateString
     */
    ros::ServiceServer service = n.advertiseService("conversation", UpdateString);

    std_msgs::String msg;

    std::stringstream ss;
    ss << "ROS is the best" << count;
    msg.data = ss.str();

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::ServiceServer server = n.advertiseService("UpdateString", UpdateString);
  ros::Rate loop_rate(frequency);

  ROS_WARN_STREAM("The input frequency by the user is: " <<frequency);
  if (frequency < 0){
    ROS_FATAL_STREAM("Invalid input freuency");
    frequency = 10;
  }
  else if(frequency == 0){
    ROS_ERROR_STREAM("0 cannot be the input frequency ");
    frequency = 10;
  }
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << actualMsgs << count;
    msg.data = ss.str();

    //Display ROS info data
    ROS_INFO_STREAM("Message : "<< msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ROS_INFO_STREAM("Wait for the Response");
    ros::spinOnce();
    
    loop_rate.sleep();
    ++count;

    transform.setOrigin(tf::Vector3(0.0, 2.0, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    b.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "RosWorld", "talker"));
  }
}


  return 0;
}
