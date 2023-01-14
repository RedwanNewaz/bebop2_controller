//
// Created by redwan on 12/15/22.
//

#ifndef BEBOP2_CONTROLLER_PID_H
#define BEBOP2_CONTROLLER_PID_H
#define NUM_CONTROLLER  4
#define NUM_GAINS 3

namespace bebop2 {

    class PID {

    public:
        PID();
        void init(double dt, double max, double min, double Kp, double Kd, double Ki);
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
