#include <iostream>
#include <Eigen/Dense>
#include <algorithm>

using namespace Eigen;
using namespace std;

MatrixXd solveContinuousARE(const MatrixXd& A, const MatrixXd& B, const MatrixXd& Q, const MatrixXd& R) {
    // Solve the continuous-time Algebraic Riccati Equation (CARE)
    MatrixXd P = MatrixXd::Zero(A.rows(), A.cols());  // Initialize P
    double tolerance = 1e-6;
    int max_iterations = 100;

    for (int i = 0; i < max_iterations; ++i) {
        MatrixXd P_next = A.transpose() * P * A - A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A + Q;
        if ((P - P_next).norm() < tolerance) {
            P = P_next;
            break;
        }
        P = P_next;
    }

    return P;
}

MatrixXd calculateLQRControllerGain(const MatrixXd& A, const MatrixXd& B, const MatrixXd& Q, const MatrixXd& R, const MatrixXd& P) {
    // Calculate the LQR controller gain
    MatrixXd K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;

    return K;
}

int main() {
    // Define system matrices (A and B), state vector (x), control input vector (u),
    // cost matrices (Q and R), and the LQR controller gain (K).
    MatrixXd A(8, 8);
    MatrixXd B(8, 4);
    MatrixXd x(8, 1);
    MatrixXd xg(8, 1);
    MatrixXd u(4, 1);
    MatrixXd Q(8, 8);
    MatrixXd R(4, 4);
    MatrixXd K(4, 8);

    // Define simulation parameters
    double dt = 0.03;  // Time step
    double simulationTime = 10.0;
    int numSteps = static_cast<int>(simulationTime / dt);

    // Initialize state (position, velocity, orientation, angular velocity) and control input
    x << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    xg << -1.0, 1.0, 1.0, M_PI_2, 0.0, 0.0, 0.0, 0.0;


    // Define system dynamics (example values)
    A <<    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    B <<    dt, 0.0, 0.0, 0.0,
            0.0, dt, 0.0, 0.0,
            0.0, 0.0, dt, 0.0,
            0.0, 0.0, 0.0, dt,
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

    // Define cost matrices (example values)
    MatrixXd Gain(8, 8);
    Gain << 0.075, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
            0.0, 0.075, 0.0, 0.0, 0.0, 0.00075, 0.0, 0.0,
            0.0, 0.0, 0.085, 0.0, 0.0, 0.0, 0.001, 0.0,
            0.0, 0.0, 0.0, 0.08, 0.0, 0.0, 0.0, 0.002,
            0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.00075, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.002;

    Q << Gain.completeOrthogonalDecomposition().pseudoInverse();

    R <<    0.0003, 0.0, 0.0, 0.0,
            0.0, 0.0001, 0.0, 0.0,
            0.0, 0.0, 0.0001, 0.0,
            0.0, 0.0, 0.0, 0.0001;


    // Solve the continuous-time Algebraic Riccati Equation (CARE)
    MatrixXd P = solveContinuousARE(A, B, Q, R);

    // Calculate the LQR controller gain
    K = calculateLQRControllerGain(A, B, Q, R, P);

    cout << K << endl;


    // Simulation loop
    for (int i = 0; i < numSteps; ++i) {
        // Calculate control input using LQR
        VectorXd error = x - xg;

        u = -K * error;
        // Update the state (example dynamics)
        x += A * x + B * u;

        MatrixXd xPos = x.block<3, 1>(0, 0);
        MatrixXd gPos = xg.block<3, 1>(0, 0);
        double dist = (gPos - xPos).norm();
        cout <<x.transpose() << " | " << dist << endl;
        if(dist < 0.25)
            break;


    }

    return 0;
}
