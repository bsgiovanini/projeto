/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#define M_PI 3.1415926535897931
#define MAX_RANGE 3.00
#define MAP_MAX_RANGE MAX_RANGE-0.01
#define MAX_DIST 1000
#define V_MAX 1.5 // max velocity considered in m/s
#define TIME_AHEAD 1.25 // amount of time will be looked to predict the trajectory
#define DELTA_VOL V_MAX*TIME_AHEAD
#define TTC_LIMIT 5.0
#define OCTREE_RESOLUTION 0.05


// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sensor_msgs/PointCloud.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"

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
#include <cstdlib>
#include <cmath>
#include <limits>
#include <fstream>


using namespace Eigen;
using namespace octomap;

using namespace std;

pthread_mutex_t mutex_1     = PTHREAD_MUTEX_INITIALIZER;


ros::Publisher pub_enable_collision_mode, pub_vel, pub_grid_cell, pub_pose, pub_pc, pub_pc2, pub_pc3;
geometry_msgs::Twist twist;


Vector3d theta(0,0,0);
Vector3d previous_theta(0,0,0);
Vector3d previous_omega(0,0,0);
Vector3d previous_vel(0,0,0);

float previous_tm = 0.0;

float quadrotor_sphere_radius = 0.8;


class PID {
    public:
     double kp, kd, ki;
     double _last_time, _last_error, _error_sum;

     void configure(double kp_l, double kd_l, double ki_l) {
        kp = kp_l;
        kd = kd_l;
        ki = ki_l;
     }

     void reset() {
        _last_time = 0;
        _last_error = std::numeric_limits<double>::infinity();
        _error_sum = 0;
     }

    double getCommand(double e, double timestamp) {
    // Compute dt in seconds
        double dt = timestamp - _last_time;

        double de = 0;
        if (_last_time != 0) {
            // Compute de (error derivation)
            if (_last_error < std::numeric_limits<double>::infinity()) {

                de = (e - _last_error) / dt;
            }

            // Integrate error
            _error_sum += e * dt;
        }

        // Update our trackers
        _last_time = timestamp;
        _last_error = e;

        // Compute commands
        double command = kp * e + ki * _error_sum + kd * de;

        return command;
    }

     PID(double kp_l, double kd_l, double ki_l) {
        configure(kp_l, kd_l, ki_l);
        reset();
     }

     PID() {
     }

};


class Command
{
   public:
      double x_l;
      double y_l;
      double z_l;
      double z_a;

      Command(double x, double y, double z, double z_2) {
            x_l = x;
            y_l = y;
            z_l = z;
            z_a = z_2;
      }

};


Command current_command(0.0, 0.0, 0.0, 0.0);

float kp_x = 0.1;
float kd_x = 0.0;
float ki_x = 0.1;

float kp_y = 0.1;
float kd_y = 0.0;
float ki_y = 0.1;

float kp_z = 0.1;
float kd_z = 0.0;
float ki_z = 0.1;

float kp_yaw = 0.1;
float kd_yaw = 0.0;
float ki_yaw = 0.1;


PID pid_x(kp_x, kd_x, ki_x);
PID pid_y(kp_y, kd_y, ki_y);
PID pid_z(kp_z, kd_z, ki_z);
PID pid_yaw(kp_yaw, kd_yaw, ki_yaw);

Vector3d pos_obj;
double yaw_obj;
int control_mode = 0;
float contador;

float last_tm_ins = 0.0;

ofstream pid_x_txt, pid_y_txt, pid_z_txt, pid_yaw_txt;


Vector3d x(0,0,0);// global pose quadrotor

Vector3d acc_max(1.5, 1.5, 1.0); //global max acc quadrotor meter per sec

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

double generateGaussianNoise(double mu, double sigma)
{
	const double epsilon = std::numeric_limits<double>::min();
	const double two_pi = 2.0*3.14159265358979323846;

	static double z0, z1;
	static bool generate;
	generate = !generate;

	if (!generate)
	   return z1 * sigma + mu;

	double u1, u2;
	do
	 {
	   u1 = rand() * (1.0 / RAND_MAX);
	   u2 = rand() * (1.0 / RAND_MAX);
	 }
	while ( u1 <= epsilon );

	z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
	z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
	return z0 * sigma + mu;
}

void send_collision_mode_msg(bool value) {

    std_msgs::Bool msg;
    msg.data = value;
    pub_enable_collision_mode.publish(msg);
}

void send_velocity_command(Command cmd) {

    cout << "enviando " << endl;
    twist.linear.x = cmd.x_l; // forward, backward
    twist.linear.y = cmd.y_l; // left right
    twist.linear.z = cmd.z_l; // up down
    twist.angular.z = cmd.z_a; // yaw
    pub_vel.publish(twist);


}

void joy_callback(const sensor_msgs::JoyConstPtr joy_msg){

    float scale = 1;

    current_command.x_l = scale*joy_msg->axes[1];
    current_command.y_l = scale*joy_msg->axes[0];
    current_command.z_l = scale*joy_msg->axes[3];
    current_command.z_a = scale*joy_msg->axes[2];
}

double within(double v,double vmin,double vmax) {

    if (abs(v) > 0.01) {
        if (v < vmin) {
            return vmin;
        } else if (v > vmax) {
            return vmax;
        } else {
            return v;
        }
    } else {
        return 0.0;
    }
}

float first_timestamp = -1;

void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{

    struct timeval stop, start;
    gettimeofday(&start, NULL);
    //do stuff

    float timestamp = msg_in.tm/1000000;
    if (first_timestamp < 0) {
        first_timestamp = timestamp;
    }
    //timestamp in microsecs
    float dt = timestamp - previous_tm; //geting dt in secs

	double vx_= msg_in.vx*0.001;
	double vy_= msg_in.vy*0.001;
	double vz_= msg_in.vz*0.001;

	theta(0) = degree_to_rad(msg_in.rotX);
	theta(1) = degree_to_rad(msg_in.rotY);
	theta(2) = degree_to_rad(msg_in.rotZ);



	//ROS_INFO("I heard ax: [%f]  ay: [%f] az: [%f]", vx_, vy_, vz_);

	Vector3d velV (vx_, vy_, vz_);

	Quaternion<double> rotQ = rotation(theta);

	Matrix3d R = rotQ.matrix();

	Vector3d vel = R * velV;

	Vector3d x_new = x + vel*dt;

    //pthread_mutex_lock(&mutex_1);
    x = x_new;

    //cout << "time took: "<< stop.tv_usec - start.tv_usec << endl;

    if (!control_mode) {
        send_collision_mode_msg(true);
        control_mode = 1;
    } else {

    //Vector3d u(pid_x.getCommand(pos_obj(0) - x(0), timestamp), pid_y.getCommand(pos_obj(1) - x(1), timestamp), pid_z.getCommand(pos_obj(2) - x(2), timestamp));
        double u_x = pid_x.getCommand(pos_obj(0) - x(0), timestamp);
        double u_y = pid_y.getCommand(pos_obj(1) - x(1), timestamp);
        double u_z = pid_z.getCommand(pos_obj(2) - x(2), timestamp);
        double u_yaw = pid_yaw.getCommand(yaw_obj - theta(2), timestamp);



        double cx   = within(cos(theta(2)) * u_x + sin(theta(2)) * u_y, -1, 1);
        double cy   = within(-sin(theta(2)) * u_x + cos(theta(2)) * u_y, -1, 1);
        double cz   = within(u_z, -1, 1);
        double cyaw = within(u_yaw, -1, 1);

        //cout << " c_x: "<< cx << " c_y " << cy << " c_z " <<  cz  << " c_yaw " << cyaw << endl;
        //cout << " ----- fim ------ " << endl;

        Command cmd(cx, cy, cz, cyaw);
        send_velocity_command(cmd);

        f_vector_print("vel", velV);

        if ((velV.norm() > 0.001) && (timestamp - last_tm_ins >= 0.2)) {

            char prefix_x [50], prefix_y[50], prefix_z[50], prefix_yaw[50];

            sprintf (prefix_x, "%f;%f;%f\n",timestamp - first_timestamp, pos_obj(0), x(0));
            sprintf (prefix_y, "%f;%f;%f\n",timestamp - first_timestamp, pos_obj(1), x(1));
            sprintf (prefix_z, "%f;%f;%f\n",timestamp - first_timestamp, pos_obj(2), x(2));
            sprintf (prefix_yaw, "%f;%f;%f\n",timestamp - first_timestamp, yaw_obj, theta(2));
            pid_x_txt << prefix_x;
            pid_y_txt << prefix_y;
            pid_z_txt << prefix_z;
            pid_yaw_txt << prefix_yaw;

            last_tm_ins = timestamp;

        }





    }

    previous_tm = timestamp;

}
// %EndTag(CALLBACK)%

void my_handler(int s){

    printf("Escrevendo no arquivo %d\n",s);

    pid_x_txt.close();
    pid_y_txt.close();
    pid_z_txt.close();
    pid_yaw_txt.close();
    exit(1);

}

int main(int argc, char **argv)
{
    //load_sonar_rel_transform_m();
    //tree.setProbHit(0.5);

    pos_obj = Vector3d(0.5,0.5,0.5);
    yaw_obj = 1;

    char prefix_x [50], prefix_y[50], prefix_z[50], prefix_yaw[50];
    sprintf (prefix_x, "x_kp_%.2f_kd_%.2f_ki_%.2f.csv", kp_x, kd_x, ki_x);
    sprintf (prefix_y, "y_kp_%.2f_kd_%.2f_ki_%.2f.csv", kp_y, kd_y, ki_y);
    sprintf (prefix_z, "z_kp_%.2f_kd_%.2f_ki_%.2f.csv", kp_z, kd_z, ki_z);
    sprintf (prefix_yaw, "yaw_kp_%.2f_kd_%.2f_ki_%.2f.csv", kp_yaw, kd_yaw, ki_yaw);

    pid_x_txt.open(prefix_x);
    pid_y_txt.open(prefix_y);
    pid_z_txt.open(prefix_z);
    pid_yaw_txt.open(prefix_yaw);

    pid_x_txt << "Time;Objective;Position\n";
    pid_y_txt << "Time;Objective;Position\n";
    pid_z_txt << "Time;Objective;Position\n";
    pid_yaw_txt << "Time;Objective;Position\n";
    //fprintf (pid_y_txt*, "Time \t Objective \t Position\n");
    //fprintf (pid_z_txt*, "Time \t Objective \t Position\n");
    //fprintf (pid_yaw_txt*, "Time \t Objective \t Position\n");


  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */

   ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who ising and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
  ros::Subscriber sub_nav = n.subscribe("/ardrone/navdata", 1, nav_callback);
// %EndTag(SUBSCRIBER)%
  //ros::Subscriber sub_sensor_f = n.subscribe("/sonar_front", 1, sonar_front_callback);

  //ros::Subscriber sub_sensor_l = n.subscribe("/sonar_left", 1, sonar_left_callback);

  //ros::Subscriber sub_sensor_r = n.subscribe("/sonar_right", 1, sonar_right_callback);

  ros::Subscriber joy_sub = n.subscribe("/joy", 1, joy_callback);

  pub_enable_collision_mode = n.advertise<std_msgs::Bool>("/project/collision_mode",1);
  pub_vel                   = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  pub_grid_cell             = n.advertise<nav_msgs::GridCells>("/project/grid_cells",1);
  pub_pose                  = n.advertise<geometry_msgs::PoseStamped>("/project/pose",1);
  pub_pc                    = n.advertise<sensor_msgs::PointCloud>("/project/ray",  1);
  pub_pc2                   = n.advertise<sensor_msgs::PointCloud>("/project/trajectory",  1);
  pub_pc3                   = n.advertise<sensor_msgs::PointCloud>("/project/cast",  1);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */

// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  struct sigaction sigIntHandler;

   sigIntHandler.sa_handler = my_handler;
   sigemptyset(&sigIntHandler.sa_mask);
   sigIntHandler.sa_flags = 0;

   sigaction(SIGINT, &sigIntHandler, NULL);

   pause();

  return 0;
}
// %EndTag(FULLTEXT)%
