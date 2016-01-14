/*
* ardrone_joystick:
*
* This software provides a connection between a PS3-Joystick and the ardrone_brown - drone-driver
*
* It receives the joystick state from the joy-node and publishes the corresponding commands to the driver
*
* Author: Nikolas Engelhard
*
*
*/


#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"
#include <fstream>

const int PUBLISH_FREQ = 50;



using namespace std;

ofstream result_command_txt;

struct TeleopArDrone
{
    ros::Subscriber joy_sub, project_collision_mode, project_distance;
    ros::Publisher pub_takeoff, pub_land, pub_toggle_state, pub_vel;

    bool got_first_joy_msg;

    bool is_flying;
    bool toggle_pressed_in_last_msg;
    bool cam_toggle_pressed_in_last_msg;

    bool autopilot_;
    bool autopilot_toggle_pressed_in_last_msg;

    std_srvs::Empty srv_empty;

    ros::NodeHandle nh_;
    geometry_msgs::Twist twist;
    ros::ServiceClient srv_cl_cam;

    void onCollisionMode(const std_msgs::Bool collision_msg) {
        autopilot_ = collision_msg.data;
    }

    void onDistance(const std_msgs::Float32 distance) {


        if (distance.data < 3.00) {

            char text[50];
            float val_cmd = sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y);
            sprintf (text, "%f;%f\n", distance.data, val_cmd);
            result_command_txt << text;
        }

    }

	void joyCb(const sensor_msgs::JoyConstPtr joy_msg){


        cout << "recebendo..." << joy_msg->axes[1] <<" "<< joy_msg->axes[0] << " " << joy_msg->axes[3] << " " << joy_msg->axes[2] <<  endl;
         if (!got_first_joy_msg){
        ROS_INFO("Found joystick with %zu buttons and %zu axes", joy_msg->buttons.size(), joy_msg->axes.size());

          autopilot_ = false;
          got_first_joy_msg = true;
         }

         // mapping from joystick to velocity
         float scale = 1;

         twist.linear.x = scale*joy_msg->axes[1]; // forward, backward
         twist.linear.y = scale*joy_msg->axes[0]; // left right
         twist.linear.z = scale*joy_msg->axes[3]; // up down
         twist.angular.z = scale*joy_msg->axes[2]; // yaw

         // button 10 (L1): dead man switch
         bool dead_man_pressed = joy_msg->buttons.at(10);

         // button 11 (R1): switch emergeny state
         bool emergency_toggle_pressed = joy_msg->buttons.at(11);

         // button 0 (select): switch camera mode
         bool cam_toggle_pressed = joy_msg->buttons.at(0);

         if(joy_msg->buttons.at(14))
            twist.angular.x = twist.angular.y = 1;
         else
            twist.angular.x = twist.angular.y = 0;

         if (!is_flying && dead_man_pressed){
          ROS_INFO("L1 was pressed, Taking off!");
          pub_takeoff.publish(std_msgs::Empty());
          is_flying = true;
         }

         if (is_flying && !dead_man_pressed){
          ROS_INFO("L1 was released, landing");
          pub_land.publish(std_msgs::Empty());
          is_flying = false;
         }

         // toggle only once!
         if (!toggle_pressed_in_last_msg && emergency_toggle_pressed){
          ROS_INFO("Changing emergency status");
          pub_toggle_state.publish(std_msgs::Empty());
         }
         toggle_pressed_in_last_msg = emergency_toggle_pressed;


         if (!cam_toggle_pressed_in_last_msg && cam_toggle_pressed){
          ROS_INFO("Changing Camera");
          if (!srv_cl_cam.call(srv_empty))  ROS_INFO("Failed to toggle Camera");
         }
         cam_toggle_pressed_in_last_msg = cam_toggle_pressed;



	}


	TeleopArDrone(){

	 twist.linear.x = twist.linear.y = twist.linear.z = 0;
	 twist.angular.x = twist.angular.y = twist.angular.z = 0;

	 is_flying = false;
	 got_first_joy_msg = false;

	 joy_sub = nh_.subscribe("/joy", 1,&TeleopArDrone::joyCb, this);

	 project_collision_mode = nh_.subscribe("/project/collision_mode", 1,&TeleopArDrone::onCollisionMode, this);
	 project_distance = nh_.subscribe("/project/mydist", 1,&TeleopArDrone::onDistance, this);

	 toggle_pressed_in_last_msg = cam_toggle_pressed_in_last_msg = false;

	 pub_takeoff       = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
	 pub_land          = nh_.advertise<std_msgs::Empty>("/ardrone/land",1);
	 pub_toggle_state  = nh_.advertise<std_msgs::Empty>("/ardrone/reset",1);
	 //pub_enable_autopilot = nh_.advertise<std_msgs::Bool>("/ardrone/enable_controller",1);
	 pub_vel           = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
	 srv_cl_cam        = nh_.serviceClient<std_srvs::Empty>("/ardrone/togglecam");
	}


    template<typename T>
    bool isApprox(const T& v, const T& threshold, const T& tolerance)
    {
      return std::abs(v - threshold) < tolerance;
    }

	void send_cmd_vel() {
	  if(!autopilot_)
        pub_vel.publish(twist);
    }


};


int main(int argc, char **argv)
{

    result_command_txt.open("commando_por_distancia.csv");

    result_command_txt << "Time ;Distance to obstacle; Command\n";

  ros::init(argc, argv, "ardrone_teleop");

  ROS_INFO("Started ArDrone joystick-Teleop");
  ROS_INFO("Press and hold L1 for takeoff");
  ROS_INFO("Press R1 to toggle emergency-state");
  ROS_INFO("Press 'select' to choose camera");
  ROS_INFO("Press 'start' to enable/disable the controller");

  TeleopArDrone teleop;
  ros::Rate pub_rate(PUBLISH_FREQ);

  while (teleop.nh_.ok())
  {
    ros::spinOnce();
    teleop.send_cmd_vel();
    pub_rate.sleep();
  }

  result_command_txt.close();

  return 0;
}