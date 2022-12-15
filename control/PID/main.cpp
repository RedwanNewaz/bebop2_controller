//
// Created by redwan on 12/15/22.
//
#include <iostream>
#include <vector>
#include <iterator>
#include <math.h>
#include "PID.h"
using namespace std;

const int NUM_CONTROLLER = 4;
const int NUM_GAINS = 3;
const double GOAL_THRES = 0.002;
const double sample_time = 0.03;

void set_gains(const vector<double>& gains, bebop2::PID* controller)
{


    const double MAX_OUT = 1;
    const double MIN_OUT = -1;

    for (int i = 0; i < NUM_CONTROLLER; ++i) {
        int j = i * NUM_GAINS;
        controller[i].init(sample_time, MAX_OUT, MIN_OUT, gains[j], gains[j + 1], gains[j + 2]);
    }
}

double goal_distance(const vector<double>& X, const vector<double>& setPoints)
{
    double error = 0;
    for (int i = 0; i < X.size(); ++i) {
        double e = X[i] - setPoints[i];
        error += e * e;
    }
    return sqrt(error);
}

bool compute_control(const vector<double>& X, const vector<double>& setPoints, bebop2::PID* controller, vector<double>& control)
{
    if(goal_distance(X, setPoints) < GOAL_THRES)
        return false;
    for (int i = 0; i < NUM_CONTROLLER; ++i) {
        control[i] = controller[i].calculate(setPoints[i], X[i]);
    }
    return true;
}

int main(int argc, char* argv[])
{
    cout << "PID Controller Initialized" << endl;
    bebop2::PID quadController[NUM_CONTROLLER];

    vector<double> gains{
        0.1, 0, 0,
        -0.1, 0, 0,
        0.1, 0, 0,
        0, 0, 0
    };

    set_gains(gains, quadController);

    vector<double>X{0,0,0,0}, Xg{1,1,1,0.0}, U(NUM_CONTROLLER);
    while (compute_control(X, Xg, quadController, U))
    {

        /*
         *  linear.x  (+)      Translate forward
                      (-)      Translate backward
            linear.y  (+)      Translate to left
                      (-)      Translate to right
            linear.z  (+)      Ascend
                      (-)      Descend
            angular.z (+)      Rotate counter clockwise
                      (-)      Rotate clockwise
         */

        for (int i = 0; i < NUM_CONTROLLER; ++i) {
            if(i == 1)
                X[i] -= U[i] * sample_time;
            else
                X[i] += U[i] * sample_time;
        }

        copy(X.begin(), X.end(), ostream_iterator<double>(cout, " "));
        cout << endl;
    }



    return 0;
}
