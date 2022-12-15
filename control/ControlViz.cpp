//
// Created by redwan on 12/14/22.
//

#include "ControlViz.h"

namespace bebop2 {
    ControlViz::ControlViz( ros::NodeHandle &nh) : nh_(nh) {
        pub_marker_ = nh_.advertise<visualization_msgs::Marker>("/bebop2/goal", 10);

    }



    void ControlViz::set_marker_from_pose(const tf::Transform &pose, visualization_msgs::Marker &msg) {
        msg.header.frame_id = "map";
        msg.ns = "robotTarget";
        msg.header.stamp = ros::Time::now();
        msg.action = visualization_msgs::Marker::ADD;
        msg.type = visualization_msgs::Marker::SPHERE;

        msg.scale.x = 0.35;
        msg.scale.y = 0.35;
        msg.scale.z = 0.35;

        // update position
        msg.pose.position.x = pose.getOrigin().x();
        msg.pose.position.y = pose.getOrigin().y();
        msg.pose.position.z = pose.getOrigin().z();

        // update orientation
        msg.pose.orientation.x = pose.getRotation().x();
        msg.pose.orientation.y = pose.getRotation().y();
        msg.pose.orientation.z = pose.getRotation().z();
        msg.pose.orientation.w = pose.getRotation().w();

    }

    void ControlViz::set_marker_color(const MARKER_COLOR &color, visualization_msgs::Marker &msg) {
        switch (color) {
            case RED:
            {
                msg.color.a = 0.75;
                msg.color.r = 1;
                break;
            }
            case GREEN:
            {
                msg.color.a = 0.75;
                msg.color.g = 1;
                break;
            }
            case BLUE:
            {
                msg.color.a = 0.75;
                msg.color.b = 1;
                break;
            }
            case YELLOW:
            {
                msg.color.a = 0.75;
                msg.color.r = 1;
                msg.color.g = 1;
                break;
            }
            case CYAN:
            {
                msg.color.a = 0.75;
                msg.color.b = 1;
                msg.color.g = 1;
                break;
            }
        }

    }

    void ControlViz::update(const tf::Transform &pose, const MARKER_COLOR& color) {
        visualization_msgs::Marker msg;
        set_marker_from_pose(pose, msg);
        set_marker_color(color, msg);

        pub_marker_.publish(msg);

    }

    void ControlViz::setDrone(const tf::Transform &pose) {

        visualization_msgs::Marker msg;
        set_marker_from_pose(pose, msg);
        msg.mesh_resource = "package://bebop2_controller/config/bebop.dae";
        msg.mesh_use_embedded_materials = true;

//        set_marker_color(color, msg);

        pub_marker_.publish(msg);

    }
} // bebop2