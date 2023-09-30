//
// Created by redwan on 3/5/23.
//
//
// Created by roboticslab on 12/13/22.
//
#include "ros/ros.h"
#include <iostream>

#include "airlib/localization/sensors.h"
#include "airlib/localization/filters.h"
#include "airlib/control/controller.h"


const double DT = 0.1; // Define your time step value here

void calcInput(std::vector<double> &x, std::vector<double> &y) {
    // Define a parameter t
    int numPoints = 200;
    double xScale = 3.5;
    double yScale = 2.5;

    for (int i = 0; i < numPoints; ++i) {
        double t = 2 * M_PI * i / numPoints;
        double xx = xScale + xScale * cos(t) * sin(t); // You can adjust the scaling factor (2) for size
        double yy = yScale + yScale * sin(t); // You can adjust the vertical offset (+1) for position
        x.push_back(xx);
        y.push_back(yy);
    }
}


void state_publisher(bebop2::StateObserverPtr stateObserver, ros::Publisher& pub)
{
    try {
        // code that may throw
        while(ros::ok())
        {
            std::promise<std::vector<double>> promise;
            stateObserver->getState(std::ref(promise));
            auto xEst = promise.get_future();
            auto state = xEst.get();
//                ROS_INFO("[State] (%lf, %lf, %lf, %lf)", state[0], state[1], state[2], state[3]);

            nav_msgs::Odometry odom;
            odom.header.stamp = ros::Time::now();
            odom.header.frame_id = "map";

            odom.pose.pose.position.x = state[0];
            odom.pose.pose.position.y = state[1];
            odom.pose.pose.position.z = state[2];

            tf::Quaternion q;
            q.setRPY(0, 0, state[3]);
            odom.pose.pose.orientation.x = q.x();
            odom.pose.pose.orientation.y = q.y();
            odom.pose.pose.orientation.z = q.z();
            odom.pose.pose.orientation.w = q.w();

            pub.publish(odom);

        }
    } catch(...) {

    }
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "airlib_bebop2");
    ROS_INFO("Bebop2Sim INITIALIZED!");
    ros::NodeHandle nh;

    std::vector<double> gains;
    double alpha, dt;
    ros::param::get("/alpha", alpha);
    ros::param::get("/pid_gains", gains);
    ros::param::get("/dt", dt);

    ROS_INFO_STREAM("[Bebop2Sim] initialized");
    auto pub = nh.advertise<nav_msgs::Odometry>("apriltag/state", 10);

    std::vector<double>noise{0.01, 0.01, 0.01, 0.001};
    auto stateSensor = std::make_shared<bebop2::DummyState>(noise);
    auto stateFilter = std::make_shared<ComplementaryFilter>(alpha);
    auto stateObserver = std::make_shared<bebop2::StateObserver>(stateFilter, stateSensor);


    auto controller = std::make_shared<controller::quad_pids>(gains, dt);
    bebop2::ControllerInterface interface(nh, controller);

    //start the simulator
    stateSensor->takeoff_callback();
    auto sub = nh.subscribe("cmd_vel", 10, &bebop2::DummyState::cmd_vel_callback, stateSensor.get());


    ros::AsyncSpinner spinner(4);
    spinner.start();
    std::vector<double> wp_x, wp_y;
    calcInput(wp_x, wp_y);
    std::vector<double>x0{wp_x[0], wp_y[1], 1.0, 0.0};
    stateSensor->set(x0);

    std::thread stateThread(state_publisher, std::ref(stateObserver), std::ref(pub));


    for (int i = 0; i < wp_x.size(); ++i) {
        ROS_INFO("[WP %d] (%lf, %lf)", i, wp_x[i], wp_y[i]);
        std::vector<double>x{wp_x[i], wp_y[i], 1.0, M_PI_2};
        interface.set_goal_state(x);
        std::this_thread::sleep_for(200ms);
    }

    ros::waitForShutdown();

    return 0;
}
