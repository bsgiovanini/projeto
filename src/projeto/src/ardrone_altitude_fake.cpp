// This is a ROS version of the standard "hello , world" // program.
// This header defines the standard ROS classes.
#include <ros/ros.h>
#include <ardrone_autonomy/Navdata.h>
#include <ardrone_autonomy/navdata_altitude.h>
#include <string>
#include <stdio.h>

ros::Publisher  pub_pose;

void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{


    ardrone_autonomy::navdata_altitude msg;
    msg.header.frame_id = "odom";
    msg.header.stamp = ros::Time::now();
    msg.altitude_vz = -msg_in.vz;

	//cout << "Sending status...." << endl;

    pub_pose.publish(msg);
    //cout << "time took: "<< stop.tv_usec - start.tv_usec << endl;

}



int main(int argc, char **argv)
{


    ros::init(argc, argv, "alt");

    ros::NodeHandle n;
// %Tag(SUBSCRIBER)%
    pub_pose                = n.advertise<ardrone_autonomy::navdata_altitude>("/ardrone/navdata_altitude", 1);
    ros::Subscriber sub_nav = n.subscribe("/ardrone/navdata", 1, nav_callback);
    ros::spin();

    return 0;
}

