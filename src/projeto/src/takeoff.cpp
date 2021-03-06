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
#define MAX_RANGE 1.00 // sonar max range in meters
#define MAP_MAX_RANGE MAX_RANGE-0.01
#define MAX_DIST 1000 // a big enough initialization distance (meters)
#define V_MAX 1.5 // max velocity considered in m/s
#define TIME_AHEAD 1.5 // amount of time will be watch out to predict the trajectory
#define DELTA_VOL V_MAX*TIME_AHEAD
#define TTC_LIMIT 2.0 // time to colide limit
#define OCTREE_RESOLUTION 0.25 // tree grid resolution (meters)
#define CONTROL_LIMIT 1.0 // duration of the automatic control (in second)
#define OCCUPIED_PROB 0.0 // 0 is 50%
#define TRAJECTORY_DT 0.25 // time interval between future trajectory steps (seconds)


// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sensor_msgs/PointCloud.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/Point.h"
#include "projeto/QuadStatus.h"
#include "std_msgs/Float32.h"

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


ros::Publisher pub_enable_collision_mode, pub_vel, pub_grid_cell, pub_pc, pub_pc2, pub_pc3, pub_dist;
geometry_msgs::Twist twist;

ofstream result_command_txt;


Vector3d theta(0,0,0);
Vector3d previous_theta(0,0,0);
Vector3d previous_omega(0,0,0);
Vector3d previous_vel(0,0,0);

float previous_tm = 0.0;

float quadrotor_sphere_radius = 0.32;

bool production_mode;



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

PID pid_x(0.5, 0, 0.35);
PID pid_y(0.5, 0, 0.35);
PID pid_z(0.8, 0, 0.35);
PID pid_yaw(1.0, 0, 0.30);


OcTree tree (OCTREE_RESOLUTION);  // create empty tree with resolution

Vector3d pos_obj;
double yaw_obj;
int control_mode = 0;
float contador;


Vector3d x(0,0,0);// global pose quadrotor
Vector3d s_1_rel_pose;
Vector3d s_2_rel_pose;
Vector3d s_3_rel_pose;
Vector3d s_4_rel_pose;
Matrix3d s_1_rel_rot_pos;
Matrix3d s_2_rel_rot_pos;
Matrix3d s_3_rel_rot_pos;
Matrix3d s_4_rel_rot_pos;



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

    return M_PI / 180.0 * degrees;

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

void load_sonar_rel_transform_m() {

    Vector3d sonar_1_rel_linear_pos(0.1, -0.1, 0.12);
    s_1_rel_rot_pos = rotation(Vector3d(0, 0, 0)).matrix();
    s_1_rel_pose = sonar_1_rel_linear_pos;

    Vector3d sonar_2_rel_linear_pos(0.1, -0.2, 0.12);
    s_2_rel_rot_pos = rotation(Vector3d(0, 0, 0)).matrix();
    //s_2_rel_rot_pos = rotation(Vector3d(0, 0, -degree_to_rad(30))).matrix();
    s_2_rel_pose = sonar_2_rel_linear_pos;

    Vector3d sonar_3_rel_linear_pos(0.1, 0.1, 0.12);
    s_3_rel_rot_pos = rotation(Vector3d(0, 0, 0)).matrix();
    //s_3_rel_rot_pos = rotation(Vector3d(0, 0, degree_to_rad(30))).matrix();
    s_3_rel_pose = sonar_3_rel_linear_pos;

    Vector3d sonar_4_rel_linear_pos(0.1, 0.2, 0.12);
    s_4_rel_rot_pos = rotation(Vector3d(0, 0, 0)).matrix();
    //s_3_rel_rot_pos = rotation(Vector3d(0, 0, degree_to_rad(30))).matrix();
    s_4_rel_pose = sonar_4_rel_linear_pos;

}


void sonar_callback(const sensor_msgs::Range& msg_in, Vector3d s_rel_pose, Matrix3d s_rel_rot_pose, Vector3d v_pose, Vector3d attitude, int debug) {


     Matrix3d R = rotation(attitude).matrix();

     Vector3d global_s_pose = v_pose + R * s_rel_pose;

    // cout << msg_in.range << endl;

     Vector3d interm = s_rel_pose +  s_rel_rot_pose * Vector3d(msg_in.range, 0, 0);

     Vector3d global_end_ray = v_pose + R * (interm);

     point3d startPoint ((float) global_s_pose(0), (float) global_s_pose(1), (float) global_s_pose(2));

     point3d endPoint ((float) global_end_ray(0), (float) global_end_ray(1), (float) global_end_ray(2));



     if (!production_mode && debug) {
        sensor_msgs::PointCloud pc;
        pc.header.frame_id = "/nav";
        pc.header.stamp = ros::Time();
        pc.channels.resize(1);
        pc.channels[0].name="ray";
        pc.channels[0].values.resize(1);
        pc.points.resize(1);
        pc.channels[0].values[0] = 0;
        pc.points[0].x = global_end_ray(0);
        pc.points[0].y = global_end_ray(1);
        pc.points[0].z = global_end_ray(2);
        pub_pc.publish(pc);

    }

     if (global_end_ray(2) > 0.1) {  //evict the ground

        tree.insertRay (startPoint, endPoint, MAP_MAX_RANGE);

        if (!production_mode) {
            point3d endcast;

            point3d direction ((float) (global_end_ray(0) - global_s_pose(0)), (float) (global_end_ray(1) - global_s_pose(1)), (float) (global_end_ray(2) - global_s_pose(2)));

            int found = tree.castRay(startPoint, direction, endcast, true, MAX_RANGE);



            if (found) {


                float variance = (OCTREE_RESOLUTION * sqrt(2))/2 - OCTREE_RESOLUTION/2;

                double noise = generateGaussianNoise(0.0, variance);



                float endcast_a = endcast(0); //> 0 ? endcast(0)- variance : endcast(0) + variance ;
                float endcast_b = endcast(1);//> 0 ? endcast(1)- variance : endcast(1) + variance ;
                float endcast_c = endcast(2);//> 0 ? endcast(2)- variance : endcast(2) + variance ;

                Vector3d endcast_w(endcast_a, endcast_b, endcast_c);

                float distance = (endcast_w - global_s_pose).norm();

                std_msgs::Float32 pb_d;
                pb_d.data = distance;
                pub_dist.publish(pb_d);

                if (distance < MAX_RANGE) {

                    char text[50];
                    float val_cmd = TIME_AHEAD;
                    sprintf (text, "%f,%f\n", distance, val_cmd);
                    result_command_txt << text;
                }


                Vector3d interm2 = s_rel_pose +  s_rel_rot_pose * Vector3d(distance - OCTREE_RESOLUTION/2 + noise, 0, 0);

                Vector3d x_temp = endcast_w - R*(interm2);

                Vector3d x_new = x_temp;//x*0.8 + x_temp*0.2;

        //pthread_mutex_lock(&mutex_1);
        //x = x_temp;
        //pthread_mutex_unlock(&mutex_1);

        //x = x_new;
                if (debug) {
                //f_vector_print("predict", x);
                //f_vector_print("endcast", x_new);

                //cout << noise << endl;
                    sensor_msgs::PointCloud pc3;
                    pc3.header.frame_id = "/nav";
                    pc3.header.stamp = ros::Time();
                    pc3.channels.resize(1);
                    pc3.channels[0].name="ray";
                    pc3.channels[0].values.resize(1);
                    pc3.points.resize(1);
                    pc3.channels[0].values[0] = 0;
                    pc3.points[0].x = x_new(0);
                    pc3.points[0].y = x_new(1);
                    pc3.points[0].z = x_new(2);

                    pub_pc3.publish(pc3);
                }
            }
        }

     }



}


void sonar_1_callback(const sensor_msgs::Range& msg_in)
{	//ROS_INFO("Range: [%f]", msg_in.range);
    sonar_callback(msg_in, s_1_rel_pose, s_1_rel_rot_pos, x, theta, 1);
}

void sonar_2_callback(const sensor_msgs::Range& msg_in)
{	//ROS_INFO("Range: [%f]", msg_in.range);
    sonar_callback(msg_in, s_2_rel_pose, s_2_rel_rot_pos, x, theta, 0);
}

void sonar_3_callback(const sensor_msgs::Range& msg_in)
{	//ROS_INFO("Range: [%f]", msg_in.range);
    sonar_callback(msg_in, s_3_rel_pose, s_3_rel_rot_pos, x, theta, 0);
}

void sonar_4_callback(const sensor_msgs::Range& msg_in)
{	//ROS_INFO("Range: [%f]", msg_in.range);
    sonar_callback(msg_in, s_4_rel_pose, s_4_rel_rot_pos, x, theta, 0);
}



vector<Vector3d> predict_trajectory2(Vector3d xdot0, Vector3d x0, float tstart, float tend, float dt_p, Vector3d future_position) {


    Vector3d x_p = x0;
    vector<Vector3d> trajectory;

    int nIntervals = (tend - tstart)/dt_p;

    for (int nTurn = 1; nTurn <= nIntervals; nTurn++) {


        //cout << xdot << endl;
        x_p = x_p + (dt_p * xdot0); //posicao linear

        trajectory.push_back(x_p);

    }

    future_position = x_p;

    return trajectory;

    //ROS_INFO("I heard ax: [%f]  ay: [%f] az: [%f]", x_p(0), x_p(1), x_p(2));

}


int in_perimeter(Vector3d pos, Vector3d obs_center, float c_factor) {

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

    int verify_x = ((pos(0) >= flb_x_c_obstacle) && (pos(0) >= frb_x_c_obstacle) && (pos(0) >= flt_x_c_obstacle) && (pos(0) >= frt_x_c_obstacle) &&
    (pos(0) <= blb_x_c_obstacle) && (pos(0) <= brb_x_c_obstacle) && (pos(0) <= blt_x_c_obstacle) && (pos(0) <= brt_x_c_obstacle));

    int verify_y = ((pos(1) >= flb_y_c_obstacle) && (pos(1) >= blb_y_c_obstacle) && (pos(1) >= flt_y_c_obstacle) && (pos(1) >= blt_y_c_obstacle) &&
    (pos(1) <=  frb_y_c_obstacle ) && (pos(1) <= brb_y_c_obstacle) && (pos(1) <= frt_y_c_obstacle ) && (pos(1) <= brt_y_c_obstacle));

    int verify_z = ((pos(2) >= flb_z_c_obstacle) && (pos(2) >= frb_z_c_obstacle) && (pos(2) >= blb_z_c_obstacle) && (pos(2) >= brb_z_c_obstacle) &&
    (pos(2) <= flt_z_c_obstacle  ) && (pos(2) <= frt_z_c_obstacle ) && (pos(2) <= blt_z_c_obstacle) && (pos(2) <= brt_z_c_obstacle));

    return verify_x && verify_y && verify_z;


}



int there_will_be_collision(Vector3d pos, Vector3d obs_center) {

    float c_factor = (OCTREE_RESOLUTION * sqrt(2)/2) + quadrotor_sphere_radius;

    return in_perimeter(pos, obs_center, c_factor);

}



void send_collision_mode_msg(bool value) {

    std_msgs::Bool msg;
    msg.data = value;
    pub_enable_collision_mode.publish(msg);
}

void send_velocity_command(Command cmd) {

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

void nav_callback(const projeto::QuadStatus status)
{

    struct timeval stop, start;
    gettimeofday(&start, NULL);
    //do stuff

    double timestamp = status.header.stamp.toSec();

    theta = Vector3d(status.theta.x, status.theta.y, status.theta.z);

    Vector3d x_new(status.position.x, status.position.y, status.position.z);

    Vector3d vel(status.vel.x, status.vel.y, status.vel.z);

	//theta(0) = degree_to_rad(msg_in.rotX);
	//theta(1) = degree_to_rad(msg_in.rotY);
	//theta(2) = degree_to_rad(msg_in.rotZ);

	//ROS_INFO("I heard ax: [%f]  ay: [%f] az: [%f]", vx_, vy_, vz_);

	//Vector3d velV (vx_, vy_, vz_);

	//Quaternion<double> rotQ = rotation(theta);

	//Matrix3d R = rotQ.matrix();

	//Vector3d vel = R * velV;

    //pthread_mutex_lock(&mutex_1);

    //float dt = timestamp - previous_tm; //geting dt in secs



    if (x_new(0) < -10 || x_new(0) > 10 || x_new(1) < -10 || x_new(1) > 10) {

        f_vector_print("theta", theta);
		f_vector_print("x", x);
		f_vector_print("x_new", x_new);
		return;
    }

    x = x_new;

    previous_tm = timestamp;

   // pthread_mutex_unlock(&mutex_1);

    Vector3d future_position;

    vector<Vector3d> trajectory = predict_trajectory2(vel, x, timestamp, timestamp + TIME_AHEAD, TRAJECTORY_DT, future_position);


    float short_dist = MAX_DIST;

    nav_msgs::GridCells gcells;
    vector<geometry_msgs::Point> obstaclerepo;

    if (!production_mode) {
        sensor_msgs::PointCloud pc;
        pc.header.frame_id = "/nav";
        pc.header.stamp = ros::Time();
        pc.channels.resize(1);
        pc.channels[0].name="trajectory";
        pc.channels[0].values.resize(trajectory.size());
        pc.points.resize(trajectory.size());

        int i = 0;
        for (vector<Vector3d>::iterator it=trajectory.begin(); it!=trajectory.end(); ++it) {

            Vector3d pos = *it;

            pc.channels[0].values[i] = 0;
            pc.points[i].x = pos(0);
            pc.points[i].y = pos(1);
            pc.points[i].z = pos(2);
            i++;
        }
        pub_pc2.publish(pc);

        gcells.header.frame_id = "/nav";
        gcells.header.stamp = ros::Time();
        gcells.cell_width = OCTREE_RESOLUTION;
        gcells.cell_height = OCTREE_RESOLUTION;


    }

    if (control_mode) {

//        cout << "TIMESTAMP " << timestamp << endl;
//        cout << "CONTADOR " << contador << endl;
//        cout << "LIMIT " << CONTROL_LIMIT << endl;
        if (timestamp < contador + CONTROL_LIMIT) {

            double u_x = pid_x.getCommand(pos_obj(0) - x(0), timestamp);
            double u_y = pid_y.getCommand(pos_obj(1) - x(1), timestamp);
            double u_z = pid_z.getCommand(pos_obj(2) - x(2), timestamp);
            double u_yaw = pid_yaw.getCommand(yaw_obj - theta(2), timestamp);



            double cx   = within(cos(theta(2)) * u_x + sin(theta(2)) * u_y, -1, 1);
            double cy   = within(-sin(theta(2)) * u_x + cos(theta(2)) * u_y, -1, 1);
            double cz   = within(u_z, -1, 1);
            double cyaw = within(u_yaw, -1, 1);

           // f_vector_print("objetivo", pos_obj);

           // f_vector_print("posicao", x);

	   cout << "SENDING AUTOMATIC CONTROLS" << endl;

            Command cmd(cx, cy, cz, cyaw);
            send_velocity_command(cmd);

        } else {

            control_mode = 0;
	    cout << "CONTROL MODE OFF" << endl;
            send_collision_mode_msg(false);
            pid_x.reset();
            pid_y.reset();
            pid_z.reset();
            pid_yaw.reset();
        }


    } else {

        OcTreeKey bbxMinKey, bbxMaxKey;

        //Vector3d lim1 = x + R * Vector3d(0, -DELTA_VOL/2, -1.0);
        //Vector3d lim2 = x + R * Vector3d(DELTA_VOL, DELTA_VOL/2, 1.0);

        point3d min_vol = point3d(x(0)-DELTA_VOL/2, x(1) -DELTA_VOL/2, x(2)-1.0);
        point3d max_vol = point3d(x(0)+DELTA_VOL/2, x(1) +DELTA_VOL/2, x(2)+1.0);


        tree.coordToKeyChecked(min_vol, bbxMinKey);
        tree.coordToKeyChecked(max_vol, bbxMaxKey);

        for(OcTree::leaf_bbx_iterator it = tree.begin_leafs_bbx(bbxMinKey, bbxMaxKey), end_bbx = tree.end_leafs_bbx(); it!= end_bbx; ++it){

            //cout << "passei" << endl;

            point3d coords = it.getCoordinate();

            Vector3d wrapped_coords = Vector3d(coords(0), coords(1), coords(2));


            //if (!in_perimeter(x, wrapped_coords, 0.5)) {
            if (!production_mode) {
                geometry_msgs::Point cell;
                cell.x = coords(0);
                cell.y = coords(1);
                cell.z = coords(2);
                obstaclerepo.push_back(cell);
            }


            if (it->getValue() > OCCUPIED_PROB) {


                for (vector<Vector3d>::iterator it=trajectory.begin(); it!=trajectory.end(); ++it) {

                    Vector3d pos = *it;

                    if (there_will_be_collision(pos, wrapped_coords)) {

                        float dist = (wrapped_coords - x).norm();

			cout << "DIST " << dist << endl;

                        if (dist < short_dist) {
                            short_dist = dist;
                        }
                        break;
                    }

                }

            }
              //manipulate node, e.g.:
              //cout << "Node center: " << it.getCoordinate() << endl;
              //cout << "Node size: " << it.getSize() << endl;
              //cout << "Node value: " << it->getValue() << endl;

            if (short_dist < MAX_DIST) {

                float ttc = short_dist/vel.norm();

                if (ttc < TTC_LIMIT) {

                    pos_obj = x;
                    yaw_obj = atan2(sin(theta(2)),cos(theta(2)));
                    send_collision_mode_msg(true);
                    control_mode = 1;
                    contador = timestamp;
	            cout << "CONTROL MODE ON" << endl;
                    pid_x.reset();
                    pid_y.reset();
                    pid_z.reset();
                    pid_yaw.reset();

                }

            }
        }
        //}

    }

    if (!production_mode) {
        int count_cells = 0;
        gcells.cells.resize(obstaclerepo.size());
        while (!obstaclerepo.empty()) {
            gcells.cells[count_cells++] = obstaclerepo.back();
            obstaclerepo.pop_back();
        }
        //pub_grid_cell.publish(gcells);
    }


    gettimeofday(&stop, NULL);



    previous_vel = vel;

    previous_theta = theta;

    //previous_omega = omega;

    gettimeofday(&stop, NULL);

    //cout << "time took: "<< stop.tv_sec - start.tv_sec << "." << (stop.tv_usec - start.tv_usec)/1000000 << "s" << endl;

}
// %EndTag(CALLBACK)%

void my_handler(int s){

    printf("Escrevendo no arquivo %d\n",s);

    //point3d min_vol ((float)(x(0)+0.12), (float)(x(1)+0.12), (float)(x(2)+0.06));

    //point3d max_vol ((float)(x(0)+0.24), (float)(x(1)+0.24), (float)(x(2)+0.20));

    result_command_txt.close();

    tree.writeBinary("simple_tree.bt");

    exit(1);

}



int main(int argc, char **argv)
{

    if (argc > 1 && strcmp(argv[1], "production") == 0) {
        production_mode = true;
    } else {
        production_mode = false;
    }

    cout << "production_mode = " << production_mode << endl;


    load_sonar_rel_transform_m();


    if (!production_mode) {

        char text[50];
        float val_cmd = TIME_AHEAD;
        sprintf (text, "tempo_%f_por_distancia.csv\n", val_cmd);

        result_command_txt.open(text);

        result_command_txt << "Time,Distance to obstacle\n";

    }

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
  ros::Subscriber sub_nav = n.subscribe("/project/status", 1, nav_callback);
// %EndTag(SUBSCRIBER)%
  ros::Subscriber sub_sensor_1 = n.subscribe("/sensor1/dist", 1, sonar_1_callback);

  ros::Subscriber sub_sensor_2 = n.subscribe("/sensor2/dist", 1, sonar_2_callback);

  ros::Subscriber sub_sensor_3 = n.subscribe("/sensor3/dist", 1, sonar_3_callback);

  ros::Subscriber sub_sensor_4 = n.subscribe("/sensor4/dist", 1, sonar_4_callback);



  ros::Subscriber joy_sub = n.subscribe("/joy", 1, joy_callback);

  pub_enable_collision_mode = n.advertise<std_msgs::Bool>("/project/collision_mode",1);
  pub_vel                   = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);

  if(!production_mode) {
    pub_grid_cell             = n.advertise<nav_msgs::GridCells>("/project/grid_cells",1);
    pub_pc                    = n.advertise<sensor_msgs::PointCloud>("/project/ray",  1);
    pub_pc2                   = n.advertise<sensor_msgs::PointCloud>("/project/trajectory",  1);
    pub_pc3                   = n.advertise<sensor_msgs::PointCloud>("/project/cast",  1);
    pub_dist                  = n.advertise<std_msgs::Float32>("/project/mydist",  1);
 }
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */

// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  if(!production_mode) {
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    pause();
 }

  return 0;
}
// %EndTag(FULLTEXT)%
