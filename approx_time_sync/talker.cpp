#include "ros/ros.h"
//ros/ros.h is a convenience include that includes all the headers necessary to use the most common public pieces of the ROS system.
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"
#include <sstream>
#include <cstdlib> 
#include <string>
#include <beginner_tutorials/Num.h>
#include <beginner_tutorials/point_cloud.h>


using namespace beginner_tutorials;

/**
* This tutorial demonstrates simple sending of messages over the ROS system.
*/
int main(int argc, char **argv)
{
/**
  38    * The ros::init() function needs to see argc and argv so that it can perform
  44    * You must call one of the versions of ros::init() before using any other
*/
	ros::init(argc, argv, "talker");
 
/**
  50    * NodeHandle is the main access point to communications with the ROS system.
  51    * The first NodeHandle constructed will fully initialize this node, and the last
  52    * NodeHandle destructed will close down the node.
*/
/*
Create a handle to this process' node. The first NodeHandle created will actually do the initialization of the node, and the last one destructed will cleanup any resources the node was using.
*/
  ros::NodeHandle n;
   
     /**
  57    * The advertise() function is how you tell ROS that you want to
  58    * publish on a given topic name. This invokes a call to the ROS
  59    * master node, which keeps a registry of who is publishing and who
  60    * is subscribing. After this advertise() call is made, the master
  61    * node will notify anyone who is trying to subscribe to this topic name,
  62    * and they will in turn negotiate a peer-to-peer connection with this
  63    * node.  advertise() returns a Publisher object which allows you to
  64    * publish messages on that topic through a call to publish().  Once
  65    * all copies of the returned Publisher object are destroyed, the topic
  66    * will be automatically unadvertised.
  67    *
  68    * The second parameter to advertise() is the size of the message queue
  69    * used for publishing messages.  If messages are published more quickly
  70    * than we can send them, the number here specifies how many messages to
  71    * buffer up before throwing some away.

			Tell the master that we are going to be publishing a message of type std_msgs/String on the topic chatter. This lets the master tell any nodes listening on chatter that we are going to publish data on that topic. The second argument is the size of our publishing queue. In this case if we are publishing too quickly it will buffer up a maximum of 1000 messages before beginning to throw away old ones.
NodeHandle::advertise() returns a ros::Publisher object, which serves two purposes: 1) it contains a publish() method that lets you publish messages onto the topic it was created with, and 2) when it goes out of scope, it will automatically unadvertise.


  72    */
  
	//ros::Publisher chatter_pub = n.advertise<beginner_tutorials::point_cloud>("chatter", 1000);
	ros::Publisher chatter_pub = n.advertise<beginner_tutorials::Num>("chatter", 1000);

	/*
A ros::Rate object allows you to specify a frequency that you would like to loop at. It will keep track of how long it has been since the last call to Rate::sleep(), and sleep for the correct amount of time.

In this case we tell it we want to run at 10Hz.
*/   
  ros::Rate loop_rate(10);
   
     /**
  78    * A count of how many messages we have sent. This is used to create
  79    * a unique string for each message.
  80    */
/*
By default roscpp will install a SIGINT handler which provides Ctrl-C handling which will cause ros::ok() to return false if that happens.

ros::ok() will return false if:

a SIGINT is received (Ctrl-C)
we have been kicked off the network by another node with the same name
ros::shutdown() has been called by another part of the application.
all ros::NodeHandles have been destroyed
Once ros::ok() returns false, all ROS calls will fail.

*/
  int count = 0;
  while (ros::ok())
  {
       /**
  85      * This is a message object. You stuff it with data, and then publish it.
  86      */
	
	beginner_tutorials::point_cloud pnt_cld;
	
	// convert count to string
	std::ostringstream os;
	os << count;

	pnt_cld.header.stamp = ros::Time::now();
	pnt_cld.meta = "point cloud data # " + os.str();
	pnt_cld.x = (std::rand() % 100);
	pnt_cld.y = (std::rand() % 100);
	pnt_cld.z = (std::rand() % 100);

	Num msg;

	msg.num = (std::rand() % 100);
	msg.header.stamp = ros::Time::now(); 
	
	//geometry_msgs::Point32 pnt;
	//pnt.x = (std::rand() % 10);
	//pnt.y = (std::rand() % 10);
	//pnt.z = 0;
   
    std::stringstream ss;
    ss << "hello world " << count;

/*
ROS_INFO and friends are our replacement for printf/cout. See the rosconsole documentation for more information
*/

    ROS_INFO_STREAM("Publish: " << msg.num);
   
       /**
  96      * The publish() function is how you send messages. The parameter
  97      * is the message object. The type of this object must agree with the type
  98      * given as a template parameter to the advertise<>() call, as was done
  99      * in the constructor above.
 100      */
   chatter_pub.publish(msg);
  
   ros::spinOnce();

   loop_rate.sleep();
   ++count;
/*

Now we use the ros::Rate object to sleep for the time remaining to let us hit our 10Hz publish rate.

Here's the condensed version of what's going on:

Initialize the ROS system
Advertise that we are going to be publishing std_msgs/String messages on the chatter topic to the master
Loop while publishing messages to chatter 10 times a second
Now we need to write a node to receive the messsages.


*/
  } 
  return 0;
}
