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

int main(int argc, char **argv) {
	ros::init(argc, argv, "whisper");
	ros::NodeHandle n;

	ros::Publisher chatter2_pub = n.advertise<beginner_tutorials::Num>("chatter2", 1000);
	ros::Rate loop_rate(10);

	int count = 0;
	while(ros::ok()) {

		std::stringstream ss;
		ss << "whisper...";

		Num msg;

		msg.num = (std::rand() % 10);
		msg.header.stamp = ros::Time::now(); 


		ROS_INFO_STREAM("Publish: " << msg.num);
		chatter2_pub.publish(msg);

		ros::spinOnce();
		//ROS pushes your subscriber callback onto a queue. 
		//It does not call it immediately. ROS only processes your callbacks 
		//	when you tell it to with ros::spinOnce().
		loop_rate.sleep();
		++count;
	}
	return 0;
}
