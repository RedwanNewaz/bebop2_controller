//
// Created by redwan on 12/15/22.
//

#ifndef BEBOP2_CONTROLLER_PID_H
#define BEBOP2_CONTROLLER_PID_H
#define NUM_CONTROLLER  4
#define NUM_GAINS 3

namespace bebop2 {
    /// @brief Calculates an error value as the difference between desired output and the current output and applies a correction based on proportional, integral and derivative terms(denoted by P, I, D respectively).
    class PID {

    public:
        /// @brief A default Constuctor.
        PID();
        /** @brief Init method initiliazes all the controller parameters like proportional constant, Integral constant, Derivative constant, 
        *   rate of change, maximum and minimum output values. 
        */
        void init(double dt, double max, double min, double Kp, double Kd, double Ki);
        /** @brief At first it calculates the error by subtracting the setpoint and current position of the drone, 
        *   Then the Proportional, Integral, Derivate terms are calculated using that error and their repective constans.
        *   Following that, we find out the total ouptput by the sum of three terms and tehn restrict the output withtin the boundaries of minimum, maximum valeus. 
        *   @param setpoint It is the desired position of the drone.
        *   @param pv It is the current position of the drone.
        *   @return Returns the error between the setpoint and the current postion of the drone.
        */
        /** @note PID Gains identifiers.
        *          - Kp = Proportionality Constant
        *          - Kd = Derivative Constant
        *          - Ki = Intergration Constant
        */  
        double calculate( double setpoint, double pv );



    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;

    };

} // bebop2

#endif //BEBOP2_CONTROLLER_PID_H
