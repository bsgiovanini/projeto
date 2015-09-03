
#define M_PI 3.14159265358979323846
#define MAX_RANGE 2.99
#define MAX_DIST 1000
#define V_MAX 1.5 // max velocity considered in m/s
#define TIME_AHEAD 1.25 // amount of time will be looked to predict the trajectory
#define DELTA_VOL V_MAX*TIME_AHEAD
#define TTC_LIMIT 5.0
#define OCTREE_RESOLUTION 0.15


// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"

#include <ardrone_autonomy/Navdata.h>
#include <sensor_msgs/Range.h>
#include "sensor_msgs/Joy.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include <sys/time.h>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <limits>
#include <unistd.h>
#include <vector>

using namespace Eigen;
using namespace std;


ros::Publisher pub_enable_collision_mode, pub_vel;
geometry_msgs::Twist twist;


Vector3d theta(0,0,0);
Vector3d previous_theta(0,0,0);
Vector3d previous_omega(0,0,0);
Vector3d previous_vel(0,0,0);


Vector3d x(0,0,0);// global pose quadrotor

Vector3d s_front_rel_pose;

float previous_tm = 0.0;


Quaternion<double> rotation(Vector3d theta) {

    AngleAxisd rollAngle(theta(0), Vector3d::UnitX());
    AngleAxisd pitchAngle(theta(1), Vector3d::UnitY());
    AngleAxisd yawAngle(theta(2), Vector3d::UnitZ());

    Quaternion<double> q = rollAngle * pitchAngle * yawAngle;

    return q;
}

Matrix3d angularTransformationMatrix(Vector3d angles) {
    float phi = angles(0);
    float theta = angles(1);
    //float psi = angles(2);
    Matrix3d W;
    W <<
        1, 0, -sin(theta),
        0, cos(phi), cos(theta)*sin(phi),
        0, -sin(phi), cos(theta)*cos(phi)
    ;
    return W;

}

Vector3d thetadot2omega(Vector3d thetadot, Vector3d angles) {

    Matrix3d T = angularTransformationMatrix(angles);
    return (T * thetadot);
}

Vector3d omega2thetadot(Vector3d omega, Vector3d angles) {

    Matrix3d T = angularTransformationMatrix(angles).inverse();
    return (T * omega);
}


double degree_to_rad(int degrees) {
    return M_PI / 180 * degrees;

}

void load_sonar_rel_transform_m() {

    Vector3d sonar_f_rel_linear_pos(0.1, 0.0, 0.12);
    Vector3d sonar_f_rel_rot_pos(0, 0, 0);
    s_front_rel_pose = rotation(sonar_f_rel_rot_pos).matrix() * sonar_f_rel_linear_pos;

}

void sonar_front_callback(const sensor_msgs::Range& msg_in)
{
	//ROS_INFO("Range: [%f]", msg_in.range);

    //Vector3d global_s_front_pose = x + rotation(theta).matrix() * s_front_rel_pose;
    //ROS_INFO("I heard sx: [%f]  sy: [%f] sz: [%f]", global_s_front_pose(0), global_s_front_pose(1), global_s_front_pose(2));

    //Vector3d global_end_ray = global_s_front_pose + rotation(theta).matrix() * Vector3d(msg_in.range, 0, 0);

    //ROS_INFO("I heard rayx: [%f]  rayy: [%f] rayz: [%f]", global_end_ray(0), global_end_ray(1), global_end_ray(2));

}

//void predict_state(float dt, Vector3d x_global, Vector3d v_local, )

void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{

    struct timeval stop, start;
    gettimeofday(&start, NULL);
    //do stuff

    float timestamp = msg_in.tm/1000000;
    //timestamp in microsecs
    float dt = timestamp - previous_tm; //geting dt in secs

	double vx_= msg_in.vx*0.001;
	double vy_= msg_in.vy*0.001;
	double vz_= msg_in.vz*0.001;

	theta(0) = degree_to_rad(msg_in.rotX);
	theta(1) = degree_to_rad(msg_in.rotY);
	theta(2) = degree_to_rad(msg_in.rotZ);

	Matrix3d R = rotation(theta).matrix();

	Vector3d velV (vx_, vy_, vz_);

	Vector3d vel = R * velV;

    x = x + vel*dt;

    ROS_INFO("odometria x_x: [%f]  x_y: [%f] x_z: [%f]", x(0), x(1), x(2));

    Vector3d thetadot = (theta - previous_theta)/dt;

    Vector3d omega = thetadot2omega(thetadot, theta);

    //Vector3d omegadot = (omega - previous_omega)/dt;

    //ROS_INFO("I heard ax: [%f]  ay: [%f] az: [%f]", omegadot(0), omegadot(1), omegadot(2));

    //Vector3d acc_linear = (vel - previous_vel)/dt;

    previous_tm = timestamp;

    previous_vel = vel;

    previous_theta = theta;

    previous_omega = omega;

    gettimeofday(&stop, NULL);

    //cout << "time took: "<< stop.tv_usec - start.tv_usec << endl;

}
// %EndTag(CALLBACK)%


int main(int argc, char **argv)
{
    load_sonar_rel_transform_m();

    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

// %Tag(SUBSCRIBER)%
    ros::Subscriber sub_nav = n.subscribe("/ardrone/navdata", 1, nav_callback);
// %EndTag(SUBSCRIBER)%
    ros::Subscriber sub_sensor = n.subscribe("/sonar_front", 1, sonar_front_callback);

    //ros::Subscriber joy_sub = n.subscribe("/joy", 1, joy_callback);

    //pub_enable_collision_mode = n.advertise<std_msgs::Bool>("/project/collision_mode",1);
    //pub_vel                   = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);


// %Tag(SPIN)%
    ros::spin();
// %EndTag(SPIN)%

//    struct sigaction sigIntHandler;

//    sigIntHandler.sa_handler = my_handler;
//    sigemptyset(&sigIntHandler.sa_mask);
//   sigIntHandler.sa_flags = 0;

//    sigaction(SIGINT, &sigIntHandler, NULL);

//    pause();

    return 0;
}
// %EndTag(FULLTEXT)%
