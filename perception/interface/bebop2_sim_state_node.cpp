// Created by roboticslab on 12/13/22.
//
#include "ros/ros.h"
#include <iostream>
#include <chrono>

#include <nav_msgs/Odometry.h>
#include "../perception/sensor/dummy/DummyState.h"
#include "../perception/filter/LPF/ComplementaryFilter.h"
#include "../perception/interface/StateObserver.h"


const double DT = 0.1; // Define your time step value here


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
    double alpha, dt, max_vel, max_acc;
    ros::param::get("/alpha", alpha);
    ros::param::get("/pid_gains", gains);
    ros::param::get("/dt", dt);
    ros::param::get("/max_vel", max_vel);
    ros::param::get("/max_acc", max_acc);

    ROS_INFO_STREAM("[Bebop2Sim] initialized");
    auto pub = nh.advertise<nav_msgs::Odometry>("apriltag/state", 10);

    std::vector<double>noise{0.01, 0.01, 0.01, 0.001};
    auto stateSensor = std::make_shared<bebop2::DummyState>(noise);
    auto stateFilter = std::make_shared<ComplementaryFilter>(alpha);
    auto stateObserver = std::make_shared<bebop2::StateObserver>(stateFilter, stateSensor);

    //start the simulator
    stateSensor->takeoff_callback();
    auto sub = nh.subscribe("cmd_vel", 10, &bebop2::DummyState::cmd_vel_callback, stateSensor.get());


    ros::AsyncSpinner spinner(4);
    spinner.start();
    std::vector<double> wp_x, wp_y;



    std::vector<double>x0{0.0, 0.0, 0.0, 0.0};
    stateSensor->set(x0);

    state_publisher(stateObserver, pub);


    ros::waitForShutdown();

    return 0;
}
