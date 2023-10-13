#include <iostream>
#include <Eigen/Dense>
#include <algorithm>
#include "LQRController.h"
using namespace Eigen;
using namespace std;


int main() {
    // Define system matrices (A and B), state vector (x), control input vector (u),
    // cost matrices (Q and R), and the LQR controller gain (K).

    MatrixXd x(8, 1);
    MatrixXd u(4, 1);
    MatrixXd xg(8, 1);
    MatrixXd u0(4, 1);
    x.setZero();

    std::vector<double>gain{
            0.075, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
            0.0, 0.075, 0.0, 0.0, 0.0, 0.00075, 0.0, 0.0,
            0.0, 0.0, 0.085, 0.0, 0.0, 0.0, 0.001, 0.0,
            0.0, 0.0, 0.0, 0.08, 0.0, 0.0, 0.0, 0.002,
            0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.00075, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.002
    };

    std::vector<double> obsNoise{
            0.0003, 0.0, 0.0, 0.0,
            0.0, 0.0001, 0.0, 0.0,
            0.0, 0.0, 0.0001, 0.0,
            0.0, 0.0, 0.0, 0.0001
    };


    // Define simulation parameters
    double dt = 0.03;  // Time step
    double simulationTime = 10.0;
    int numSteps = static_cast<int>(simulationTime / dt);


    LQRController lqr[4];
    for (int i = 0; i < 4; ++i) {
        lqr[i].set(dt, 0.1, gain, obsNoise);
    }
    lqr[0].updateGoal(1.0, 0);
    lqr[1].updateGoal(-1.0, 1);
    lqr[2].updateGoal(1.2, 2);
    // Simulation loop
    int count = 0;

    for (int i = 0; i < numSteps; ++i) {

        printf("[step] = %02d \n", ++count);
        std::vector<bool>terminated(4, false);
        for (int j = 0; j < 4; ++j) {
            // compute control for current control axis
            u(j, 0) = lqr[j].getControl()(j, 0);

            // TODO replace this with state estimator feedback
            MatrixXd state = lqr[j].predict(u);
            // update position
            lqr[j].updateState(state(j, 0), j);
            //update velocity
            lqr[j].updateState(state(4 + j, 0), 4 + j);
            // check termination
            terminated[j] = lqr[j].isTerminated();
        }
//        cout << x.transpose() << endl;
        int check = std::count(terminated.begin(), terminated.end(), false);
        if(check == 0)
            break;

//        cout <<x.transpose() << endl;



    }

    return 0;
}
