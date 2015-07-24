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

#define M_PI 3.14159265358979323846
#define MAX_RANGE 2.99

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ardrone_autonomy/Navdata.h>
#include <sensor_msgs/Range.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

using namespace Eigen;
using namespace std;
using namespace octomap;

double vx_=0.0;
double vy_=0.0;
double vz_=0.0;

Vector3d theta(0,0,0);
Vector3d previous_theta(0,0,0);
Vector3d previous_omega(0,0,0);
Vector3d previous_vel(0,0,0);


float previous_tm = 0.0;

float quadrotor_sphere_radius = 0.2;

float octree_resolution = 0.1;
OcTree tree (octree_resolution);  // create empty tree with resolution 0.1

Vector3d x(0,0,0);// global pose quadrotor
Vector3d s_front_rel_pose;

Vector3d acc_max(1.5, 1.5, 1.0); //global max acc quadrotor meter per sec

Quaternion<double> rotation(Vector3d theta) {

    AngleAxisd rollAngle(theta(0), Vector3d::UnitX());
    AngleAxisd pitchAngle(theta(1), Vector3d::UnitY());
    AngleAxisd yawAngle(theta(2), Vector3d::UnitZ());

    Quaternion<double> q = rollAngle * pitchAngle * yawAngle;

    return q;
}

Vector3d thetadot2omega(Vector3d thetadot, Vector3d angles) {
    float phi = angles(0);
    float theta = angles(1);
    float psi = angles(2);
    Matrix3d W;
    W <<
        1, 0, -sin(theta),
        0, cos(phi), cos(theta)*sin(phi),
        0, -sin(phi), cos(theta)*cos(phi)
    ;
    return (W * thetadot);
}

Vector3d omega2thetadot(Vector3d omega, Vector3d angles) {
    float phi = angles(0);
    float theta = angles(1);
    float psi = angles(2);
    Matrix3d W;
    W <<
        1, 0, -sin(theta),
        0, cos(phi), cos(theta)*sin(phi),
        0, -sin(phi), cos(theta)*cos(phi)
    ;

    return (W.inverse() * omega);
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
	ROS_INFO("Range: [%f]", msg_in.range);

    Vector3d global_s_front_pose = x + rotation(theta).matrix() * s_front_rel_pose;
    //ROS_INFO("I heard sx: [%f]  sy: [%f] sz: [%f]", global_s_front_pose(0), global_s_front_pose(1), global_s_front_pose(2));

    point3d startPoint ((float) global_s_front_pose(0), (float) global_s_front_pose(1), (float) global_s_front_pose(2));

    Vector3d global_end_ray = global_s_front_pose + rotation(theta).matrix() * Vector3d(msg_in.range, 0, 0);

    //ROS_INFO("I heard rayx: [%f]  rayy: [%f] rayz: [%f]", global_end_ray(0), global_end_ray(1), global_end_ray(2));

    point3d endPoint ((float) global_end_ray(0), (float) global_end_ray(1), (float) global_end_ray(2));

    //if (msg_in.range < MAX_RANGE) {
        tree.insertRay (startPoint, endPoint, MAX_RANGE);
    // }

    // insert some measurements of occupied cells

/*    for (int x=-20; x<20; x++) {
        for (int y=-20; y<20; y++) {
            for (int z=-20; z<20; z++) {
                point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
                tree.updateNode(endpoint, true); // integrate 'occupied' measurement
            }
        }
    }

    // insert some measurements of free cells

    for (int x=-30; x<30; x++) {
        for (int y=-30; y<30; y++) {
            for (int z=-30; z<30; z++) {
                point3d endpoint ((float) x*0.02f-1.0f, (float) y*0.02f-1.0f, (float) z*0.02f-1.0f);
                tree.updateNode(endpoint, false);  // integrate 'free' measurement
            }
        }
    }

    point3d query (0., 0., 0.);
    OcTreeNode* result = tree.search (query);


    query = point3d(-1.,-1.,-1.);
    result = tree.search (query);


    query = point3d(1.,1.,1.);
    result = tree.search (query);
    */


}

Vector3d predict_trajectory(Vector3d omega0, Vector3d omegadot, Vector3d theta0, Vector3d a, Vector3d xdot0, Vector3d x0, float tstart, float tend, float dt_p) {

    Vector3d omega_p = omega0;
    Vector3d theta_p = theta0;
    Vector3d xdot_p = xdot0;
    Vector3d x_p = x0;

    int nIntervals = (tend - tstart)/dt_p;

    for (int nTurn = 1; nTurn <= nIntervals; nTurn++) {

        omega_p = omega_p + (dt_p * omegadot);

        Vector3d thetadot_p = omega2thetadot(omega_p, theta_p); //taxa de variacao angular

        //cout << thetadot << endl;

        theta_p = theta_p + (dt_p * thetadot_p); //orientacao

        //cout << theta << endl;

        xdot_p = xdot_p + (dt_p * a); //velocidade linear

        //cout << xdot << endl;
        x_p = x_p + (dt_p * xdot_p); //posicao linear

    }

    return x_p;

    //ROS_INFO("I heard ax: [%f]  ay: [%f] az: [%f]", x_p(0), x_p(1), x_p(2));

}

int there_will_be_collision(Vector3d pos, Vector3d obs_center) {

    float c_factor = (octree_resolution/2) + quadrotor_sphere_radius;

    //front - left - bottom
    float flb_x_c_obstacle = obs_center(0) - c_factor;
    float flb_y_c_obstacle = obs_center(1) - c_factor;
    float flb_z_c_obstacle = obs_center(2) - c_factor;

    //back - left - bottom
    float blb_x_c_obstacle = obs_center(0) + c_factor;
    float blb_y_c_obstacle = obs_center(1) - c_factor;
    float blb_z_c_obstacle = obs_center(2) - c_factor;

    //back - right - bottom
    float brb_x_c_obstacle = obs_center(0) + c_factor;
    float brb_y_c_obstacle = obs_center(1) + c_factor;
    float brb_z_c_obstacle = obs_center(2) - c_factor;

    //front - right - bottom
    float frb_x_c_obstacle = obs_center(0) - c_factor;
    float frb_y_c_obstacle = obs_center(1) + c_factor;
    float frb_z_c_obstacle = obs_center(2) - c_factor;

    //front - left - top
    float flt_x_c_obstacle = obs_center(0) - c_factor;
    float flt_y_c_obstacle = obs_center(1) - c_factor;
    float flt_z_c_obstacle = obs_center(2) + c_factor;

    //back - left - top
    float blt_x_c_obstacle = obs_center(0) + c_factor;
    float blt_y_c_obstacle = obs_center(1) - c_factor;
    float blt_z_c_obstacle = obs_center(2) + c_factor;

    //font - right - top
    float frt_x_c_obstacle = obs_center(0) - c_factor;
    float frt_y_c_obstacle = obs_center(1) + c_factor;
    float frt_z_c_obstacle = obs_center(2) + c_factor;

    //back - right - top
    float brt_x_c_obstacle = obs_center(0) + c_factor;
    float brt_y_c_obstacle = obs_center(1) + c_factor;
    float brt_z_c_obstacle = obs_center(2) + c_factor;

    int verify_x = ((x(0) >= flb_x_c_obstacle) && (x(0) >= frb_x_c_obstacle) && (x(0) >= flt_x_c_obstacle) && (x(0) >= frt_x_c_obstacle) &&
    (x(0) <= blb_x_c_obstacle) && (x(0) <= brb_x_c_obstacle) && (x(0) <= blt_x_c_obstacle) && (x(0) <= brt_x_c_obstacle));

    int verify_y = ((x(1) >= flb_y_c_obstacle) && (x(1) >= blb_y_c_obstacle) && (x(1) >= flt_y_c_obstacle) && (x(1) >= blt_y_c_obstacle) &&
    (x(1) <=  frb_y_c_obstacle ) && (x(1) <= brb_y_c_obstacle) && (x(1) <= frt_y_c_obstacle ) && (x(1) <= brt_y_c_obstacle));

    int verify_z = ((x(2) >= flb_z_c_obstacle) && (x(2) >= frb_z_c_obstacle) && (x(2) >= blb_z_c_obstacle) && (x(2) >= brb_z_c_obstacle) &&
    (x(2) <= flt_z_c_obstacle  ) && (x(2) <= frt_z_c_obstacle ) && (x(2) <= blt_z_c_obstacle) && (x(2) <= brt_z_c_obstacle));

    return verify_x && verify_y && verify_z;

}


void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{

    float timestamp = msg_in.tm/1000000;
    //timestamp in microsecs
    float dt = timestamp - previous_tm; //geting dt in secs

	vx_=msg_in.vx*0.001;
	vy_=msg_in.vy*0.001;
	vz_=msg_in.vz*0.001;

	theta(0) = degree_to_rad(msg_in.rotX);
	theta(1) = degree_to_rad(msg_in.rotY);
	theta(2) = degree_to_rad(msg_in.rotZ);

	Matrix3d R = rotation(theta).matrix();

	Vector3d velV (vx_, vy_, vz_);

	Vector3d vel = R * velV;

    x = x + vel*dt;

    Vector3d thetadot = (theta - previous_theta)/dt;

    Vector3d omega = thetadot2omega(thetadot, theta);

    Vector3d omegadot = (omega - previous_omega)/dt;

    //ROS_INFO("I heard ax: [%f]  ay: [%f] az: [%f]", omegadot(0), omegadot(1), omegadot(2));

    Vector3d acc_linear = (vel - previous_vel)/dt;

    Vector3d future_position = predict_trajectory(omega, omegadot, theta, acc_linear, vel, x, timestamp, timestamp + 0.5, dt);

    OcTreeKey bbxMinKey, bbxMaxKey;

    //tree.getMetricMin(temp_x,temp_y,temp_z);
    //point3d min_vol(temp_x,temp_y,temp_z);

    //tree.getMetricMax(temp_x,temp_y,temp_z);
    //point3d max_vol(temp_x,temp_y,temp_z);

    point3d min_vol = point3d(x(0)-1, x(1)-1, x(2)-1);
    point3d max_vol = point3d(x(0)+1, x(1)+1, x(2)+1);

    tree.coordToKeyChecked(min_vol, bbxMinKey);
    tree.coordToKeyChecked(max_vol, bbxMaxKey);

    //OcTree::leaf_bbx_iterator end_bbx = tree.end_leafs_bbx();

    //cout << "points min "<< min_vol <<" max " << max_vol << endl;

    for(OcTree::leaf_bbx_iterator it = tree.begin_leafs_bbx(bbxMinKey, bbxMaxKey), end_bbx = tree.end_leafs_bbx(); it!= end_bbx; ++it)
    {
        if (it->getValue() > 0.0) {
            point3d coords = it.getCoordinate();
            if (there_will_be_collision(future_position, Vector3d(coords(0), coords(1), coords(2)))) {
                cout << "Opa!! vai colidir!!!";
            }
        }
      //manipulate node, e.g.:
      //cout << "Node center: " << it.getCoordinate() << endl;
      //cout << "Node size: " << it.getSize() << endl;
      //cout << "Node value: " << it->getValue() << endl;
    }


	//ROS_INFO("getting sensor reading");
	//
	//ROS_INFO("I heard ax: [%f]  ay: [%f] az: [%f]", acc_linear(0), acc_linear(1), acc_linear(2));

    //ROS_INFO("Time: [%f]", dt);
    previous_tm = timestamp;

    previous_vel = vel;

    previous_theta = theta;

    previous_omega = omega;
}
// %EndTag(CALLBACK)%

void my_handler(int s){

    printf("Escrevendo no arquivo %d\n",s);

    //point3d min_vol ((float)(x(0)+0.12), (float)(x(1)+0.12), (float)(x(2)+0.06));

    //point3d max_vol ((float)(x(0)+0.24), (float)(x(1)+0.24), (float)(x(2)+0.20));



    tree.writeBinary("simple_tree.bt");

    exit(1);

}

int main(int argc, char **argv)
{
    load_sonar_rel_transform_m();
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
   * master node, which keeps a registry of who is publishing and who
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
  ros::Subscriber sub_sensor = n.subscribe("/sonar_front", 1, sonar_front_callback);
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
