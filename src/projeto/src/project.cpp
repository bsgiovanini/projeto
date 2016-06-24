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

#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

ofstream result_txt;
float pose_x, pose_y, pose_z;
vector<Vector3d> trajectory;
vector<float> trajectory_tm;


void pose_callback(const geometry_msgs::PoseStamped& msg_in)
{
    pose_x = msg_in.pose.position.x;
    pose_y = msg_in.pose.position.y;
    pose_z = msg_in.pose.position.z;
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

int main(int argc, char** argv){

  result_txt.open("mybag.csv");

  result_txt << "time, tf_x, tf_y, tf_z, pose_x, pose_y, pose_z\n";

  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  ros::Subscriber sub_pose = node.subscribe("/project/pose", 1, pose_callback);

  ros::Subscriber sub_trajectory = node.subscribe("/project/trajectory", 1, trajectory_callback);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){

    ros::spinOnce();
    tf::StampedTransform mtransform;
    try{
      listener.lookupTransform("/nav", "/base_footprint",
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
        traj_text += "[" + boost::to_string(tm) + ":"  + "[" + boost::to_string(pos(0)) +";"+ boost::to_string(pos(1)) + ";"+ boost::to_string(pos(2)) +  "]" + "],";
    }

    traj_text += "]";

    result_txt
        << ros::Time::now().toSec() << ","
        << mtransform.getOrigin().x() << "," << mtransform.getOrigin().y() << "," << mtransform.getOrigin().z() << ","
        << pose_x << "," << pose_y << "," << pose_z << ","
        << traj_text << ","
        << endl;



    rate.sleep();
  }

  result_txt.close();
  return 0;
};
