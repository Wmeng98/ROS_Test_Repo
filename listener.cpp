#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"
#include <string>
   
   /**
  32  * This tutorial demonstrates simple receipt of messages over the ROS system.
  33  */
/*
This is the callback function that will get called when a new message has arrived on the chatter topic. The message is passed in a boost shared_ptr, which means you can store it off if you want, without worrying about it getting deleted underneath you, and without copying the underlying data.

*/

  void chatterCallback(const geometry_msgs::Point32::ConstPtr& pnt)
  {
    ROS_INFO("I heard: [%f]", pnt->x);
  }
   
  int main(int argc, char **argv)
  {
  /**
  42    * The ros::init() function needs to see argc and argv so that it can perform
  43    * any ROS arguments and name remapping that were provided at the command line.
  44    * For programmatic remappings you can use a different version of init() which takes
  45    * remappings directly, but for most command-line programs, passing argc and argv is
  46    * the easiest way to do it.  The third argument to init() is the name of the node.
  47    *
  48    * You must call one of the versions of ros::init() before using any other
  49    * part of the ROS system.
  50    */
    ros::init(argc, argv, "listener");
   
     /**
  54    * NodeHandle is the main access point to communications with the ROS system.
  55    * The first NodeHandle constructed will fully initialize this node, and the last
  56    * NodeHandle destructed will close down the node.
  57    */
    ros::NodeHandle n;
 
     /**
  61    * The subscribe() call is how you tell ROS that you want to receive messages
  62    * on a given topic.  This invokes a call to the ROS
  63    * master node, which keeps a registry of who is publishing and who
  64    * is subscribing.  Messages are passed to a callback function, here
  65    * called chatterCallback.  subscribe() returns a Subscriber object that you
  66    * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
  67    * object go out of scope, this callback will automatically be unsubscribed from
  68    * this topic.
  69    *
  70    * The second parameter to the subscribe() function is the size of the message
  71    * queue.  If messages are arriving faster than they are being processed, this
  72    * is the number of messages that will be buffered up before beginning to throw
  73    * away the oldest ones.
  74    */
/*

Subscribe to the chatter topic with the master. ROS will call the chatterCallback() function whenever a new message arrives. The 2nd argument is the queue size, in case we are not able to process messages fast enough. In this case, if the queue reaches 1000 messages, we will start throwing away old messages as new ones arrive.

NodeHandle::subscribe() returns a ros::Subscriber object, that you must hold on to until you want to unsubscribe. When the Subscriber object is destructed, it will automatically unsubscribe from the chatter topic.

There are versions of the NodeHandle::subscribe() function which allow you to specify a class member function, or even anything callable by a Boost.Function object. The roscpp overview contains more information.


*/

    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
   
     /**
  78    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
  79    * callbacks will be called from within this thread (the main one).  ros::spin()
  80    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
  81    */

/*

ros::spin() enters a loop, calling message callbacks as fast as possible. Don't worry though, if there's nothing for it to do it won't use much CPU. ros::spin() will exit once ros::ok() returns false, which means ros::shutdown() has been called, either by the default Ctrl-C handler, the master telling us to shutdown, or it being called manually.

There are other ways of pumping callbacks, but we won't worry about those here. The roscpp_tutorials package has some demo applications which demonstrate this. The roscpp overview also contains more information.

Again, here's a condensed version of what's going on:

Initialize the ROS system
Subscribe to the chatter topic
Spin, waiting for messages to arrive
When a message arrives, the chatterCallback() function is called


*/

    ros::spin();
   
    return 0;
  }
