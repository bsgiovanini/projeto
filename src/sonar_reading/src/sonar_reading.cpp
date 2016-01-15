// This is a ROS version of the standard "hello , world" // program.
// This header defines the standard ROS classes.
#include <ros/ros.h>
#include <std_msgs/Float64.h>
int main(int argc , char **argv) {
	// Initialize the ROS system.
	ros::init(argc, argv, "hello_ros");
	// Establish this program as a ROS node.
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<std_msgs::Float64> ("sensor/dist", 1000);
	ros::Rate rate(2);
	while(ros::ok()) {
		// Create and f i l l in the message . The other four
		// fields , which are ignored by turtlesim , default to 0. 
		std_msgs::Float64 msg;
		msg.data = 100.0f;
		pub.publish(msg);
		// Send a message to rosout with the details.
		ROS_INFO_STREAM("Sending "<< msg.data);
		rate.sleep();
	}
}
