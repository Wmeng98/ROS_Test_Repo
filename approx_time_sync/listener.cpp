#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Point32.h"
#include <string> 
#include <beginner_tutorials/Num.h>
#include <beginner_tutorials/point_cloud.h> 

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>



// The TimeSynchronizer filter synchronizes incoming channels by the timestamps 
// contained in their headers, and outputs them in the form of a single callback that takes 
// the same number of channels. The C++ implementation can synchronize up to 9 channels.


	//void chatter2Callback(const std_msgs::String::ConstPtr& msg) {
	//	ROS_INFO_STREAM("Subscriber also heard: " << msg->data);
	
using namespace beginner_tutorials;

	void callback(const beginner_tutorials::Num::ConstPtr& msg1, const beginner_tutorials::Num::ConstPtr& msg2) {
		// compute data from two topics here...
		ROS_INFO_STREAM("x: " << msg1->num << ", " << "y: " << msg2->num);
	}
   
  int main(int argc, char **argv)
  {
		
    ros::init(argc, argv, "listener");
		
    ros::NodeHandle n;
		
		message_filters::Subscriber<beginner_tutorials::Num> num_sub(n, "chatter", 1000);
		message_filters::Subscriber<beginner_tutorials::Num> num_sub2(n, "chatter2", 1000);	
		
		//message_filters::Subscriber<beginner_tutorials::point_cloud> cld_sub(n, "chatter", 1000); // Need a header for custom data type, necxessary for synchronization...
		
    //ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
		//ros::Subscriber sub2 = n.subscribe("chatter2", 1000, chatter2Callback);
	
		typedef message_filters::sync_policies::ApproximateTime<beginner_tutorials::Num, beginner_tutorials::Num> MySyncPolicy;
		// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
		message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), num_sub, num_sub2);
		sync.registerCallback(boost::bind(&callback, _1, _2));


		//message_filters::TimeSynchronizer<beginner_tutorials::Num, beginner_tutorials::Num> sync(num_sub, num_sub2, 10);
		//sync.registerCallback(boost::bind(&callback, _1, _2));
		// bind callback(x, y) arguments
		//You can register multiple callbacks with the registerCallbacks() method. They will get called in the order they are registered.
		
    ros::spin();
   
    return 0;
  }
