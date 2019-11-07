/**
 * Distributed under the BSD License (license terms found in LICENSE or at https://opensource.org/licenses/BSD-2-Clause)
 *
 * @file talker.hpp
 *
 * @brief Publishing node talker.
 *
 * @author Andre Ferreira
 *
 * @copyright  Andre Ferreira
 *
 */
#include <tf/transform_broadcaster.h>
#include <string>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/changeBaseOtput.h"


/// Define string Variable to be published at the topic
extern std::string baseString = "Base String without change";
/**

 * @brief changeString service function that changes the publishing node string in the topic 
 *
 * Function get request value and set it to response variable and 
 * the baseString, which is the variable published in the
 * topic.
 *
 * @param beginner_tutorials::changeBaseOtput::Request &req beginner_tutorials::changeBaseOtput::Response &res , that are the request and response from srv file


 * @return TRUE , Returns True to let know that the service has finished

 */
bool changeString(beginner_tutorials::changeBaseOtput::Request &req,
                  beginner_tutorials::changeBaseOtput::Response &res) {
/// Set the input to the output and to the base string
  res.stringOutput = req.stringInput;
  baseString = res.stringOutput;
  return true;
}


/// This tutorial demonstrates simple sending of messages over the ROS system.

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

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

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

  ros::Publisher chatter_pub = n.advertise < std_msgs::String
      > ("chatter", 1000);
  /// Create and advertise service over ROS
  ros::ServiceServer service = n.advertiseService("changeString", changeString);
  /// Create Variable that takes care of loop frequency and initialize to default value
  int frequency = 1;
  /// If node stated by the launch file,recieves argument and pass to variable
  if (argc > 1) {
    frequency = std::stoi(argv[1]);
  }
  /// If frequency different then default create the logging messages
  if (frequency != 1) {
    ROS_DEBUG_STREAM("Loop frequency changed to " << frequency);
    ROS_WARN_STREAM("Loop frequency different from Default");
  }
  /// if frequency smaller or equal to 0 print shutdown system
  if (frequency <= 0) {
    ROS_FATAL_STREAM("Invalid Loop frequency value ,it cannot be smaller then 1 , SYSTEM WILL BE SHUTDOWN");
    ros::shutdown();
  }
  /// If frequency > 300 use default frequency and print logging messages
  if (frequency > 300) {
    ROS_ERROR_STREAM(
        "Maximun Loop Frequency allowed in this node is 300, Frequency will be set to Default ");
    frequency = 1;
    ROS_INFO_STREAM(" LOOP frequency set to " << frequency);
  }
/// Set loop rate according to desired frequency
  ros::Rate loop_rate(frequency);
/// Create a TransformBroadcaster object that is use to send transformartions
  static tf::TransformBroadcaster br;
/// Create a transform object
  tf::Transform transform;
/// Set translation vector
  transform.setOrigin(tf::Vector3(1.0, 1.0, 1.0) );
/// Orientation /vector
  tf::Quaternion q;

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
    ss << baseString << count;
    msg.data = ss.str();
    ROS_INFO_STREAM(msg.data.c_str());
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);
/// Set rotation
    q.setRPY(count, (count+1), (count+2));
    transform.setRotation(q);
/// Brodcasting the Transform
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "talkWithFrame"));

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}

