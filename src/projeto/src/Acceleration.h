
#define M_PI 3.14159265358979323846
#define ACC_BUFFER_SIZE 1
#define ACC_DIFF_1_AND_2 1.0
#define ACC_MAX 4.0
#define ACC_DELTA_T 100000 //in microssec. deve ser <= ao intervalo de amostras da IMU
#define TIME(a,b) ((a*1000000ull) + b)


#include <math.h>
#include <eigen3/Eigen/Dense>

#include <sys/time.h>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <limits>
#include <unistd.h>
#include <vector>


#include <deque>

using namespace std;



namespace Project {

    struct sample {
        Eigen::Vector3d data;
        unsigned long long time_us;
    };

    class Acceleration {

        public:



            deque<sample> buffer_acc;

            Eigen::Quaternion<double> rotation(Eigen::Vector3d theta) {

                Eigen::AngleAxisd rollAngle(theta(0), Eigen::Vector3d::UnitX());
                Eigen::AngleAxisd pitchAngle(theta(1), Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd yawAngle(theta(2), Eigen::Vector3d::UnitZ());

                Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;

                return q;
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

            void getRawAcc(double ax, double ay, double az, Eigen::Vector3d &acc1, Eigen::Vector3d &acc2, unsigned long long &time_us);

            bool acceptDiffAccs(Eigen::Vector3d acc1, Eigen::Vector3d acc2);

            bool acceptMaxAcc(Eigen::Vector3d acc);

            Eigen::Vector3d doInterpolation(Project::sample lastS, Project::sample newS);

            void updateAcceleration(double ax, double ay, double az);

            Eigen::Vector3d getAcceleration(unsigned long long timestamp);

            Eigen::Vector3d getPrevAcceleration(unsigned long long timestamp);


    };

    class Velocity {

        public:

    }


}


