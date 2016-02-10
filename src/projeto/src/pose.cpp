
#define M_PI 3.14159265358979323846
#define ALTD_MIN 0.0


// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include <ardrone_autonomy/Navdata.h>
#include <ardrone_autonomy/navdata_altitude.h>
#include "projeto/QuadStatus.h"
#include "geometry_msgs/PoseStamped.h"
#include <math.h>
#include <eigen3/Eigen/Dense>

#include <sys/time.h>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <limits>
#include <unistd.h>
#include <vector>
#include <fstream>



using namespace Eigen;
using namespace std;


ros::Publisher  pub_pose, pub_pose_rviz;

Vector3d theta(0,0,0);
Vector3d previous_theta(0,0,0);
Vector3d previous_omega(0,0,0);
Vector3d previous_vel(0,0,0);

ofstream txt;

double vz = 0.0;

Vector3d x(0,0,0);// global pose quadrotor

int freq_pub_pose = 0;
float previous_tm = 0.0;
float last_pose_tm = 0.0;

void f_vector_print(string name, Vector3d vectors) {

    cout << name << ": x: " << vectors(0) << ", y: " << vectors(1) << ", z: " << vectors(2) << endl;
}


Quaternion<double> rotation(Vector3d theta) {

    AngleAxisd rollAngle(theta(0), Vector3d::UnitX());
    AngleAxisd pitchAngle(theta(1), Vector3d::UnitY());
    AngleAxisd yawAngle(theta(2), Vector3d::UnitZ());

    Quaternion<double> q = yawAngle * pitchAngle * rollAngle;

    return q;
}


double degree_to_rad(int degrees) {
    return M_PI / 180 * degrees;

}

void alt_callback(const ardrone_autonomy::navdata_altitude& msg_in) {

    vz = msg_in.altitude_vz*0.001;
}


void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{

    struct timeval stop, start;
    gettimeofday(&start, NULL);
    //do stuff

   float timestamp = msg_in.tm*0.000001;

    //timestamp in microsecs

    double vx_= msg_in.vx*0.001;
    double vy_= msg_in.vy*0.001;
    double vz_ = vz;


    theta(0) = degree_to_rad(msg_in.rotX);
    theta(1) = degree_to_rad(msg_in.rotY);
    theta(2) = degree_to_rad(msg_in.rotZ);

    //ROS_INFO("I heard ax: [%f]  ay: [%f] az: [%f]", vx_, vy_, vz_);

    Vector3d velV (vx_, vy_, vz_);

    Quaternion<double> rotQ = rotation(theta);

    Matrix3d R = rotQ.matrix();

    Vector3d vel = R * velV;

    //pthread_mutex_lock(&mutex_1);

    double altitude = msg_in.altd*0.001;

    if (msg_in.state == 3 || msg_in.state == 4 || msg_in.state == 7) { //verify if it is flying, hovering and over a minimal altitud

	float dt = timestamp - previous_tm; //geting dt in secs

        if (altitude > ALTD_MIN) {

                Vector3d x_new = x + vel*dt;

                x_new(2) = msg_in.altd*0.001;

                char prefix_x [1000];

                sprintf (prefix_x, "%f;%f;%f;%f;%f;%f;%f;%f;%f\n",vx_, vy_, vz_,x_new(0), x_new(1), x_new(2), theta(0), theta(1), theta(2));

                txt << prefix_x;

            x = x_new;
        }
    }

    if (!freq_pub_pose || (timestamp - last_pose_tm) >= (1/(freq_pub_pose*1.0))) {

        projeto::QuadStatus status;
        status.header.frame_id = "odom";
        status.header.stamp = ros::Time::now();

        status.position.x = x(0);
        status.position.y = x(1);
        status.position.z = x(2);

        status.vel.x = vel(0);
        status.vel.y = vel(1);
        status.vel.z = vel(2);

        status.theta.x = theta(0);
        status.theta.y = theta(1);
        status.theta.z = theta(2);

	//cout << "Sending status...." << endl;

        pub_pose.publish(status);

        last_pose_tm = timestamp;

        if(pub_pose_rviz.getNumSubscribers()) {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "odom";
            pose.header.stamp = ros::Time::now();
            pose.pose.orientation.x = rotQ.x();
            pose.pose.orientation.y = rotQ.y();
            pose.pose.orientation.z = rotQ.z();
            pose.pose.orientation.w = rotQ.w();
            pose.pose.position.x = x(0);
            pose.pose.position.y = x(1);
            pose.pose.position.z = x(2);

            pub_pose_rviz.publish(pose);
        }

    }

    previous_tm = timestamp;

    gettimeofday(&stop, NULL);

    //cout << "time took: "<< stop.tv_usec - start.tv_usec << endl;

}



int main(int argc, char **argv)
{

    txt.open("velocidade.txt");

    ros::init(argc, argv, "pose");

    ros::NodeHandle n("~");

    n.getParam("freq_pub_pose", freq_pub_pose);

    if (freq_pub_pose > 0) {
        cout << "Pose frequency in " << freq_pub_pose << "Hz" << endl;
    } else {
        cout << "Pose frequency not setted " << endl;
    }

// %Tag(SUBSCRIBER)%
    pub_pose                = n.advertise<projeto::QuadStatus>("/project/status", 1);
    pub_pose_rviz           = n.advertise<geometry_msgs::PoseStamped>("/project/pose", 1);
    ros::Subscriber sub_nav = n.subscribe("/ardrone/navdata", 1, nav_callback);
    ros::Subscriber sub_alt = n.subscribe("/ardrone/navdata_altitude", 1, alt_callback);


    ros::spin();

    return 0;
}

