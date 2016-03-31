#include "Acceleration.h"
#include <fstream>
#include <iostream>

ofstream txt;

using namespace std;


void f_vector_print(string name, Eigen::Vector3d vectors) {
    cout << name << ": x: " << vectors(0) << ", y: " << vectors(1) << ", z: " << vectors(2) << endl;
}

void Project::Acceleration::getRawAcc(double ax, double ay, double az, Eigen::Vector3d &acc1, Eigen::Vector3d &acc2, unsigned long long &time_us) {

    struct timeval time_sample;
    acc1 = Eigen::Vector3d(ax, ay, az) + Eigen::Vector3d(generateGaussianNoise(0.0, 0.3), generateGaussianNoise(0.0, 0.3), generateGaussianNoise(0.0, 0.3));
    acc2 = Eigen::Vector3d(ax, ay, az) + Eigen::Vector3d(generateGaussianNoise(0.0, 0.3), generateGaussianNoise(0.0, 0.3), generateGaussianNoise(0.0, 0.3));

    gettimeofday(&time_sample, NULL);
    time_us = TIME(time_sample.tv_sec,time_sample.tv_usec);
}

bool Project::Acceleration::acceptDiffAccs(Eigen::Vector3d acc1, Eigen::Vector3d acc2) {

    Eigen::Vector3d diff(abs(acc1(0) - acc2(0)), abs(acc1(1) - acc2(1)), abs(acc1(2) - acc2(2)));

    return (diff(0) <= ACC_DIFF_1_AND_2 && diff(1) <= ACC_DIFF_1_AND_2 && diff(2) <= ACC_DIFF_1_AND_2);


}

bool Project::Acceleration::acceptMaxAcc(Eigen::Vector3d acc) {

    return (abs(acc(0)) <= ACC_MAX && abs(acc(1)) <= ACC_MAX && abs(acc(2)) <= ACC_MAX);
}

Eigen::Vector3d Project::Acceleration::doInterpolation(sample lastS, sample newS) {

    f_vector_print("last: ", lastS.data);

    Eigen::Vector3d x =  (((newS.data - lastS.data) * ACC_DELTA_T )/ (newS.time_us - lastS.time_us)) + lastS.data;

    return x;

}


void Project::Acceleration::updateAcceleration(double ax, double ay, double az) {

    //calibrate accelerometers

    Eigen::Vector3d acc1;
    Eigen::Vector3d acc2;
    unsigned long long time;
    getRawAcc(ax, ay, az, acc1, acc2, time);

    if (!acceptDiffAccs(acc1, acc2)) { //eliminate if the difference between accs is big

        cout << "nao aceitou diff" << endl;
        return;
    }

    Eigen::Vector3d acc = (acc1 + acc2) * 0.5;

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

    cout << " buffer_acc " << buffer_acc.size() << endl;

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

    while (buffer_acc.size() > ACC_BUFFER_SIZE) {
        buffer_acc.pop_front();
    }


    sample last = buffer_acc.back();

    txt << last.data(0) <<";" << last.data(1)<<";" << last.data(2)<<";" << acc1(0)<<";" << acc1(1)<<";" << acc1(2)<<";" << acc2(0)<<";" << acc2(1)<<";" << acc2(2)<<";" << time <<"\n";

}

 Eigen::Vector3d Project::Acceleration::getAcceleration(unsigned long long timestamp) {
    return buffer_acc.back().data;
 }

 Eigen::Vector3d Project::Acceleration::getPrevAcceleration(unsigned long long timestamp) {
    if (buffer_acc.size() > 1)
        return buffer_acc.at(buffer_acc.size()-1).data;
    else
        return Eigen::Vector3d(0,0,0);
 }


int main(int argc, char **argv)
{

    Project::Acceleration acc;


    txt.open("comp.txt");


    float x = 0.3;
    float y = 0.3;
    float z = 0.3;

    acc.updateAcceleration(x+=0.1, y+=0.1, z+=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z-=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z+=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z-=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z+=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z-=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z+=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z-=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z+=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z-=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z+=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z-=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z+=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z-=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z+=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z-=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z+=1.0); usleep(100000);
    acc.updateAcceleration(x+=1.0, y+=1.0, z-=1.0); usleep(100000);
    acc.updateAcceleration(x-=0.1, y-=0.1, z+=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y-=0.9, z-=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z+=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z-=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z+=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z-=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z+=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z-=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z+=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z-=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z+=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z-=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z+=1.0); usleep(100000);
    acc.updateAcceleration(x+=0.1, y+=0.1, z-=1.0); usleep(100000);




    for(deque<Project::Acceleration::sample>::iterator it = acc.buffer_acc.begin(); it != acc.buffer_acc.end(); it++){
        Project::Acceleration::sample sp = *it;
        f_vector_print("acc sample", sp.data);
        cout << " time: " << sp.time_us << endl;
    }

    txt.close();

    return 0;
}

