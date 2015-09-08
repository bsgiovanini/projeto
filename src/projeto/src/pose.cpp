
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

class EKF {
    public:
        Vector3d _pose;

        double _last_yaw;

        Matrix3d _sigma, _q, _r;


        /*void predict_measure(Vector3d measure_pose_rel) {

            double state_yaw = _pose(2);

            double mx = measure_pose_rel(0);
            double my = measure_pose_rel(1);



        }*/

        double rand_normal(double mean, double stddev) {
            static double n2 = 0.0;
            static int n2_cached = 0;
            if (!n2_cached) {
                double x, y, r;
            do {
                x = 2.0*rand()/RAND_MAX - 1;
                y = 2.0*rand()/RAND_MAX - 1;

                r = x*x + y*y;
            } while (r == 0.0 || r > 1.0);
                 {
                 double d = sqrt(-2.0*log(r)/r);
            double n1 = x*d;
            n2 = y*d;
                 double result = n1*stddev + mean;
                 n2_cached = 1;
                 return result;
                 }
             } else {
                 n2_cached = 0;
                 return n2*stddev + mean;
             }
         }

        void predict_state(float dt, Vector3d v_local, Vector3d theta) {

            //double roll = theta(0);
            //double pitch = theta(1);
            double yaw = theta(2);
            double vx = v_local(0);
            double vy = v_local(1);

            if (_last_yaw == -9999999) {
                _last_yaw = yaw;
                return;
            }

            //cout << _last_yaw << " " << yaw << endl;

            double o_dx = vx * dt;
            double o_dy = vy * dt;
            double o_dyaw = yaw - _last_yaw;

//            cout << " yaw: " << _last_yaw << " " << yaw << endl;
//            cout << " o: " << o_dx << " " << o_dy << " "  << o_dyaw <<  endl;

            _last_yaw = yaw;

            _pose(2) = yaw + o_dyaw;
            _pose(2) = atan2(sin(_pose(2)), cos(_pose(2))); //normalize yaw

            _pose(0) = _pose(0) + o_dx * cos(_pose(2)) - o_dy* sin(_pose(2));
            _pose(1) = _pose(1) + o_dx * sin(_pose(2)) + o_dy* cos(_pose(2));

            Matrix3d F;
            F <<    1, 0, -sin(_pose(2)) * o_dx - cos(_pose(2)) * o_dy,
                    0, 1,  cos(_pose(2)) * o_dx - sin(_pose(2)) * o_dy,
                    0, 0, 1;

            _sigma = F * (_sigma * (F.transpose())) + _q;


        }

        Vector3d h(Vector3d x, Vector3d m) {

            Vector3d toReturn;
            toReturn << x(0) + cos(x(2)) * m(0) - sin(x(2)) * m(1),
                        x(1) + sin(x(2)) * m(0) + cos(x(2)) * m(1),
                        x(2);
            return toReturn;

        }

        void correct_state(Vector3d measure_pose_rel) {

            double state_yaw = _pose(2);

            double mx = measure_pose_rel(0);
            double my = measure_pose_rel(1);

            double random_number = rand_normal(0, 0.02);

            Vector3d z = h(_pose, measure_pose_rel) + Vector3d(random_number, random_number, random_number);

            Matrix3d H;
            H <<    1, 0, -sin(state_yaw) * mx - cos(state_yaw) * my,
                    0, 1,  cos(state_yaw) * mx - sin(state_yaw) * my,
                    0, 0, 1;

            Matrix3d Ht = H.transpose();

            //calc Gain

            Matrix3d G = _sigma * Ht * ((H * _sigma * Ht + _r).inverse());

            //cout << z << " " << h(_pose, measure_pose_rel) << endl;

            _pose = _pose + G*(z - h(_pose, measure_pose_rel));

            _sigma = (Matrix3d::Identity() - G * H) * _sigma;


        }

        void reset() {

            _pose = Vector3d::Zero();
            _sigma = Matrix3d::Identity();

            _q =  Matrix3d(Vector3d(0.0003, 0.0003, 0.0001).asDiagonal());
            _r =  Matrix3d(Vector3d(0.0004, 0.0004, 0.0004).asDiagonal());
            _last_yaw = -9999999;
        }

        EKF() {
            reset();
        }

        Vector3d state() {
            return _pose;
        }


};


EKF ekf;

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

    Vector3d global_s_front_pose = x + rotation(theta).matrix() * s_front_rel_pose;
    //ROS_INFO("I heard sx: [%f]  sy: [%f] sz: [%f]", global_s_front_pose(0), global_s_front_pose(1), global_s_front_pose(2));

    Vector3d global_end_ray = global_s_front_pose + rotation(theta).matrix() * Vector3d(msg_in.range, 0, 0);

    //ROS_INFO("I heard rayx: [%f]  rayy: [%f] rayz: [%f]", global_end_ray(0), global_end_ray(1), global_end_ray(2));

    Vector3d rel_ray_pose = s_front_rel_pose + Vector3d(msg_in.range, 0, 0);

    ekf.correct_state(rel_ray_pose);

    cout << "odometria x_x: " <<  x(0) << " x_y: " << x(1) << endl;

    Vector3d k_x = ekf.state();

    cout << "kalman f  x_x: " <<  k_x(0) << " x_y: " << k_x(1) << endl;
    //ROS_INFO("kalman f  x_x: [%f]  x_y: [%f]", k_x(0), k_x(1));
}


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

    ekf.predict_state(dt, velV, theta);



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
