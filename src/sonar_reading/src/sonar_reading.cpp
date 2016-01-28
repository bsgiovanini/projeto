// This is a ROS version of the standard "hello , world" // program.
// This header defines the standard ROS classes.
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <string>
#include <stdio.h>

#define MAX_READING 150 // max reading in cm

int getSensorAddress(int sensorNumber) {
	switch(sensorNumber) {
		case 1: return 0x3a;
		case 2: return 0x70;
		case 3: return 0x3c;
		case 4: return 0x37;
		default: return -1;
	}

}

int fd;
int main(int argc , char **argv) {

	// Initialize the ROS system.
	ros::init(argc, argv, "dist");
	// Establish this program as a ROS node.
	ros::NodeHandle nh("~");

	int sensorNumber = -1;

	nh.getParam("sensor", sensorNumber);
	int dID = getSensorAddress(sensorNumber);
	if (dID < 0) {
		printf("error getting sensor address\n");
                return -1;
	}
	if((fd=wiringPiI2CSetup(dID))<0) {
                printf("error opening i2c channel\n");
                return -1;
        }

	sensor_msgs::Range msg;

	msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  	msg.header.frame_id =  "ultrasound_ranger";
  	msg.field_of_view = 1;
  	msg.min_range = 0.20;
  	msg.max_range = 1.50;
	ros::Publisher pub = nh.advertise<sensor_msgs::Range> ("dist", 1000);
	ros::Rate rate(10);
	while(ros::ok()) {
		int e = wiringPiI2CWrite(fd, 0x51);
		rate.sleep();
		int r = wiringPiI2CReadReg16(fd, 0xe1);
		int val = (r >> 8) & 0xff | (r << 8) & 0x1;
		if (val <= MAX_READING) {
            msg.range = val/100.0;
	     	msg.header.stamp = ros::Time();
            pub.publish(msg);
            ROS_INFO_STREAM("Sending "<< msg.range);
		} else {
            ROS_INFO_STREAM("Not sending "<< val/100.0);
		}
		// Send a message to rosout with the details.
	}
}
