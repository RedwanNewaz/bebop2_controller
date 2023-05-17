//
// Created by airlab on 5/16/23.


#include "StateViz.h"

namespace bebop2 {
    StateViz::StateViz() {
        pub_marker_ = nh_.advertise<visualization_msgs::Marker>("/bebop2/goal", 10);
        timer_ = nh_.createTimer(ros::Duration(1.0), &StateViz::timerCallback, this);
    }



    void StateViz::set_marker_from_pose(const tf::Transform &pose, visualization_msgs::Marker &msg) {
        msg.header.frame_id = "map";
        msg.ns = "target";
        msg.header.stamp = ros::Time::now();
        msg.action = visualization_msgs::Marker::ADD;


        msg.scale.x = 0.35;
        msg.scale.y = 0.35;
        msg.scale.z = 0.35;

        // update position
        double x, y, z, yaw;
        msg.pose.position.x = x = pose.getOrigin().x();
        msg.pose.position.y = y = pose.getOrigin().y();
        msg.pose.position.z = z = pose.getOrigin().z();

        // update orientation
        msg.pose.orientation.x = pose.getRotation().x();
        msg.pose.orientation.y = pose.getRotation().y();
        msg.pose.orientation.z = pose.getRotation().z();
        msg.pose.orientation.w = pose.getRotation().w();



    }

    void StateViz::set_marker_color(const MARKER_COLOR &color, visualization_msgs::Marker &msg) {
        switch (color) {
            case RED:
            {
                msg.color.a = 0.5;
                msg.color.r = 1;
                break;
            }
            case GREEN:
            {
                msg.color.a = 0.5;
                msg.color.g = 1;
                break;
            }
            case BLUE:
            {
                msg.color.a = 0.5;
                msg.color.b = 1;
                break;
            }
            case YELLOW:
            {
                msg.color.a = 0.5;
                msg.color.r = 1;
                msg.color.g = 1;
                break;
            }
            case CYAN:
            {
                msg.color.a = 0.5;
                msg.color.b = 1;
                msg.color.g = 1;
                break;
            }
            case GRAY:
            {
                msg.color.a = 0.5;
                msg.color.r = 0.67;
                msg.color.b = 0.67;
                msg.color.g = 0.67;
                break;
            }
        }

    }

    void StateViz::update(const tf::Transform &pose, const MARKER_COLOR& color) {
        visualization_msgs::Marker msg;
        set_marker_from_pose(pose, msg);
        set_marker_color(color, msg);
        msg.type = visualization_msgs::Marker::SPHERE;

        pub_marker_.publish(msg);

    }

    void StateViz::setDrone(const tf::Transform &pose) {

        visualization_msgs::Marker msg;
        set_marker_from_pose(pose, msg);
        msg.ns = "drone";
        msg.type = visualization_msgs::Marker::MESH_RESOURCE;
        msg.mesh_resource = "package://bebop2_controller_nodelet/config/bebop.dae";
//        msg.mesh_use_embedded_materials = true;
        msg.scale.x = msg.scale.y = msg.scale.z = 0.001;

        set_marker_color(GRAY, msg);
        msg.color.a = 1.0;
        msg.id = 101;

        pub_marker_.publish(msg);

        // populate trajectory
        auto point = msg.pose.position;
        traj_.push_back(point);

    }

    void StateViz::timerCallback(const ros::TimerEvent &event) {
        if(traj_.empty())
            return;

        visualization_msgs::Marker msg;
        msg.header.frame_id = "map";
        msg.ns = "trajectory";
        msg.id = 302;
        msg.header.stamp = ros::Time::now();
        msg.action = visualization_msgs::Marker::ADD;
        msg.type = visualization_msgs::Marker::POINTS;

        set_marker_color(RED, msg);

        msg.scale.x = 0.02;
        msg.scale.y = 0.02;
        msg.scale.z = 0.02;

        // copy traj to msg
        std::copy(traj_.begin(), traj_.end(), std::back_inserter(msg.points));

        pub_marker_.publish(msg);


    }
} // bebop2