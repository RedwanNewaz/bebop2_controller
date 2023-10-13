#include <iostream>
#include <Eigen/Dense>
#include <algorithm>
#include "LQR.h"
using namespace Eigen;
using namespace std;

int main() {
    // Define system matrices (A and B), state vector (x), control input vector (u),
    // cost matrices (Q and R), and the LQR controller gain (K).

    MatrixXd x(8, 1);
    MatrixXd xg(8, 1);
    MatrixXd u0(4, 1);


    // Define simulation parameters
    double dt = 0.03;  // Time step
    double simulationTime = 10.0;
    int numSteps = static_cast<int>(simulationTime / dt);

    u0 << 0, 0, 0, 0;

    // Initialize state (position, velocity, orientation, angular velocity) and control input
    x << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    xg << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    LQRController lqr(dt);

    // Simulation loop
    int count = 0;
    for (int i = 0; i < numSteps; ++i) {
        // Calculate control input using LQR
        MatrixXd u_hat = lqr.computeControlInput(x, xg);
//        cout << u_hat << endl;

        MatrixXd u = u0 + u_hat;
        // Update the state (example dynamics)
        x = lqr.predict(x, u);
        u0 = u;

        MatrixXd xPos = x.block<3, 1>(0, 0);
        MatrixXd gPos = xg.block<3, 1>(0, 0);
        double dist = (gPos - xPos).norm();
        printf("[%02d]: error (%.2lf) | x = ", ++count, dist);
        cout <<x.transpose() << endl;
        if(dist < 0.1)
            break;


    }

    return 0;
}