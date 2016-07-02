#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <fstream>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sensor_msgs/PointCloud.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Joy.h"

#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

ofstream result_txt;
float pose_x, pose_y, pose_z;
vector<Vector3d> trajectory;
vector<float> trajectory_tm;
Vector3d vel;
Vector3d velr;
float short_dist, control_mode, ttc;
float command_x, command_y, command_z, command_yaw;


void pose_callback(const geometry_msgs::PoseStamped& msg_in)
{
    pose_x = msg_in.pose.position.x;
    pose_y = msg_in.pose.position.y;
    pose_z = msg_in.pose.position.z;
}

void vel_callback(const geometry_msgs::Point& msg_in)
{
    vel = Vector3d(msg_in.x, msg_in.y, msg_in.z);
}

void velr_callback(const geometry_msgs::Point& msg_in)
{
    velr = Vector3d(msg_in.x, msg_in.y, msg_in.z);
}

void shortDist_callback(const geometry_msgs::Point& msg_in)
{
    short_dist = msg_in.x;
    control_mode = msg_in.y;
    ttc = msg_in.z;
}


void trajectory_callback(const sensor_msgs::PointCloud& msg_in)
{

    trajectory.clear();
    trajectory_tm.clear();
    int i = 0;
    for (i = 0; i < msg_in.points.size(); i++) {
        trajectory.push_back(Vector3d(msg_in.points[i].x, msg_in.points[i].y, msg_in.points[i].z));
        trajectory_tm.push_back(msg_in.channels[0].values[i]);
    }

	cout << "Length of array = " << msg_in.points.size() << endl;

}

void joy_callback(const sensor_msgs::JoyConstPtr& joy_msg){

    float scale = 1;

    command_x = scale*joy_msg->axes[1];
    command_y = scale*joy_msg->axes[0];
    command_z = scale*joy_msg->axes[3];
    command_yaw = scale*joy_msg->axes[2];
}


int main(int argc, char** argv){

  result_txt.open("mybag.csv");

  result_txt << "time, tf_x, tf_y, tf_z, loc_x, loc_y, loc_z, vel_x, vel_y, vel_z, velr_x, velr_y, velr_z, shortDist, control_mode, ttc, joy_x, joy_y, joy_z, joy_yaw, traj_futura\n";

  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  ros::Subscriber sub_pose = node.subscribe("/project/pose", 1, pose_callback);

  ros::Subscriber sub_vel = node.subscribe("/project/vel", 1, vel_callback);

  ros::Subscriber sub_velr = node.subscribe("/project/velr", 1, velr_callback);

  ros::Subscriber sub_shortDist = node.subscribe("/project/shortDist", 1, shortDist_callback);

  ros::Subscriber sub_trajectory = node.subscribe("/project/trajectory", 1, trajectory_callback);

  ros::Subscriber joy_sub = node.subscribe("/joy", 1, joy_callback);

  tf::TransformListener listener;

  ros::Rate rate(200.0);
  while (node.ok()){

    ros::spinOnce();
    tf::StampedTransform mtransform;
    try{
      listener.lookupTransform("/nav", "/base_link",
                               ros::Time(0), mtransform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    string traj_text ("[");


    for (unsigned j=0; j<trajectory.size(); j++) {
        Vector3d pos = trajectory.at(j);
        float tm = trajectory_tm.at(j);
        traj_text += "{" + boost::to_string(tm) + ":"  + "[" + boost::to_string(pos(0)) +";"+ boost::to_string(pos(1)) + ";"+ boost::to_string(pos(2)) +  "]" + "}+";
    }

    traj_text += "]";

    result_txt
        << ros::Time::now().toSec() << ","
        << mtransform.getOrigin().x() << "," << mtransform.getOrigin().y() << "," << mtransform.getOrigin().z() << ","
        << pose_x << "," << pose_y << "," << pose_z << ","
        << vel(0) << "," << vel(1) << "," << vel(2) << ","
        << velr(0) << "," << velr(1) << "," << velr(2) << ","
        << short_dist << "," << control_mode << "," << ttc << ","
        << command_x << "," << command_y << "," << command_z <<  "," << command_yaw << ","
        << traj_text << ","
        << endl;

    rate.sleep();
  }

  result_txt.close();
  return 0;
};
