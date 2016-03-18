
#define M_PI 3.14159265358979323846
#define ALTD_MIN 0.01

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

#include <deque>



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
Vector3d xl(0,0,0);

int freq_pub_pose = 0;
float previous_tm = 0.0;
float last_pose_tm = 0.0;

void f_vector_print(string name, Vector3d vectors) {

    cout << name << ": x: " << vectors(0) << ", y: " << vectors(1) << ", z: " << vectors(2) << endl;
}

struct sample {

    Vector3d data;
    unsigned long long time_us;
};

deque<sample> buffer_acc;

#define ACC_BUFFER_SIZE 10
#define ACC_DIFF_1_AND_2 1.5
#define ACC_MAX 1.5
#define ACC_DELTA_T 100000 //in microssec. deve ser <= ao intervalo de amostras da IMU
#define TIME(a,b) ((a*1000000ull) + b)


Quaternion<double> rotation(Vector3d theta) {

    AngleAxisd rollAngle(theta(0), Vector3d::UnitX());
    AngleAxisd pitchAngle(theta(1), Vector3d::UnitY());
    AngleAxisd yawAngle(theta(2), Vector3d::UnitZ());

    Quaternion<double> q = yawAngle * pitchAngle * rollAngle;

    return q;
}


double degree_to_rad(int degrees) {
    return M_PI / 180.0 * degrees;

}

void alt_callback(const ardrone_autonomy::navdata_altitude& msg_in) {

    vz = -msg_in.altitude_vz*0.001;
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

void getRawAcc(double ax, double ay, double az, Vector3d &acc1, Vector3d &acc2, unsigned long long &time_us) {

    struct timeval time_sample;
    acc1 = Vector3d(ax, ay, az) + Vector3d(generateGaussianNoise(0.0, 0.3), generateGaussianNoise(0.0, 0.3), generateGaussianNoise(0.0, 0.3));
    acc2 = Vector3d(ax, ay, az) + Vector3d(generateGaussianNoise(0.0, 0.3), generateGaussianNoise(0.0, 0.3), generateGaussianNoise(0.0, 0.3));

    gettimeofday(&time_sample, NULL);
    time_us = TIME(time_sample.tv_sec,time_sample.tv_usec);
}

bool acceptDiffAccs(Vector3d acc1, Vector3d acc2) {

    Vector3d diff(abs(acc1(0) - acc2(0)), abs(acc1(1) - acc2(1)), abs(acc1(2) - acc2(2)));

    return (diff(0) <= ACC_DIFF_1_AND_2 && diff(1) <= ACC_DIFF_1_AND_2 && diff(2) <= ACC_DIFF_1_AND_2);


}

bool acceptMaxAcc(Vector3d acc) {

    return (abs(acc(0)) <= ACC_MAX && abs(acc(1)) <= ACC_MAX && abs(acc(2)) <= ACC_MAX);
}

Vector3d doInterpolation(sample lastS, sample newS) {

    f_vector_print("last: ", lastS.data);

    Vector3d x =  (((newS.data - lastS.data) * ACC_DELTA_T )/ (newS.time_us - lastS.time_us)) + lastS.data;

    return x;

}


void defineAcceleration(double ax, double ay, double az) {

    //calibrate accelerometers

    Vector3d acc1;
    Vector3d acc2;
    unsigned long long time;
    getRawAcc(ax, ay, az, acc1, acc2, time);

    if (!acceptDiffAccs(acc1, acc2)) { //eliminate if the difference between accs is big

        cout << "nao aceitou diff" << endl;
        return;
    }

    Vector3d acc = (acc1 + acc2) * 0.5;

    if (!acceptMaxAcc(acc)) {
        cout << "nao aceitou acc max" << endl;
        return;
    }


    if (buffer_acc.empty()) { //first measurement
        sample sp;
        sp.data = acc;
        sp.time_us = time;
        buffer_acc.push_back(sp);
        return;
    }

    sample newSample;
    newSample.data = acc;
    newSample.time_us = time;

    for(deque<sample>::iterator it = buffer_acc.begin(); it != buffer_acc.end(); it++){
        sample s = *it;
        newSample.data += s.data;
    }
    newSample.data = newSample.data/(buffer_acc.size() + 1);

    f_vector_print(" acc ", acc);
    cout << " o time " << time << endl;

    unsigned long long timeSample = buffer_acc.back().time_us + ACC_DELTA_T;

    if (timeSample <= time) {

        while (timeSample <= time) {
            sample sp;
            sp.data = doInterpolation(buffer_acc.back(), newSample);
            sp.time_us = timeSample;
            buffer_acc.push_back(sp);
            timeSample += ACC_DELTA_T;
        }
    }

    if (buffer_acc.size() == ACC_BUFFER_SIZE) {
        buffer_acc.pop_front();
    }


    sample last = buffer_acc.back();

    txt << last.data(0) <<";" << last.data(1)<<";" << last.data(2)<<";" << acc1(0)<<";" << acc1(1)<<";" << acc1(2)<<";" << acc2(0)<<";" << acc2(1)<<";" << acc2(2)<<";" << time <<"\n";

}

void definePosition(double ax, double ay, double az) {

    defineAcceleration(ax, ay, az);
}

void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{

    struct timeval stop, start;
    gettimeofday(&start, NULL);
    //do stuff

   float timestamp = msg_in.tm*0.000001;

   definePosition(msg_in.ax, msg_in.ay, msg_in.az);



    //timestamp in microsecs

    double vx_= msg_in.vx*0.001;
    double vy_= msg_in.vy*0.001;
    double vz_= vz;


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

//    if (msg_in.state == 3 || msg_in.state == 4 || msg_in.state == 7) { //verify if it is flying, hovering and over a minimal altitud


        if (altitude > ALTD_MIN) {
		if (previous_tm == 0.0) {
			previous_tm = timestamp;
		}

		float dt = timestamp - previous_tm; //geting dt in secs

                Vector3d x_new = x + vel*dt;

		Vector3d x_loc = xl + velV*dt;

                x_new(2) = msg_in.altd*0.001;

                char prefix_x [1000];

                sprintf (prefix_x, "%f;%f;%f;%f;%f;%f;%f;%f;%f\n",vel(0), vel(1), vel(2),x_new(0), x_new(1), x_new(2), theta(0), theta(1), theta(2));
	    	if (!freq_pub_pose || (timestamp - last_pose_tm) >= (1/(freq_pub_pose*1.0))) {
			f_vector_print("vel", vel);
			f_vector_print("vll", velV);


			cout << "tm: " <<  timestamp << " dt: " << dt << endl;

			f_vector_print("posg", x_new);
			f_vector_print("posl", x_loc);

		}

                txt << prefix_x;

            x = x_new;

	    xl = x_loc;

	    previous_tm = timestamp;
        }
//    }

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

    //previous_tm = timestamp;

    gettimeofday(&stop, NULL);

    //cout << "time took: "<< stop.tv_usec - start.tv_usec << endl;

}



int main(int argc, char **argv)
{

    txt.open("comp.txt");


    defineAcceleration(1.2, 1.0, 0.9); usleep(100000);
    defineAcceleration(1.2, 1.0, 0.9); usleep(100000);
    defineAcceleration(1.2, 1.0, 0.9); usleep(100000);
    defineAcceleration(1.2, 1.0, 0.9); usleep(100000);
    defineAcceleration(1.2, 1.0, 0.9); usleep(100000);
    defineAcceleration(1.2, 1.0, 0.9); usleep(100000);
    defineAcceleration(1.2, 1.0, 0.9); usleep(100000);
    defineAcceleration(1.2, 1.0, 0.9); usleep(100000);
    defineAcceleration(1.2, 1.0, 0.9); usleep(100000);
    defineAcceleration(1.2, 1.0, 0.9); usleep(100000);
    defineAcceleration(1.2, 1.0, 0.9); usleep(100000);
    defineAcceleration(1.2, 1.0, 0.9); usleep(100000);
    defineAcceleration(1.2, 1.0, 0.9); usleep(100000);
    defineAcceleration(1.2, 1.0, 0.9); usleep(100000);
    defineAcceleration(1.2, 1.0, 0.9); usleep(100000);
    defineAcceleration(1.2, 1.0, 0.9); usleep(100000);


    for(deque<sample>::iterator it = buffer_acc.begin(); it != buffer_acc.end(); it++){
        sample sp = *it;
        f_vector_print("acc sample", sp.data);
        cout << " time: " << sp.time_us << endl;
    }

    txt.close();

    /*ros::init(argc, argv, "pose");

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


    ros::spin();*/

    return 0;
}

