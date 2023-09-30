//
// Created by redwan on 12/14/22.
//

#include "airlib/control/ControlViz.h"

namespace bebop2 {
    ControlViz::ControlViz( ros::NodeHandle &nh) : nh_(nh) {
        pub_marker_ = nh_.advertise<visualization_msgs::Marker>("/bebop2/goal", 10);
        std::vector<std::string>header{"x", "y", "z", "yaw"};
        logger_ = std::make_unique<LoggerCSV>(header, LOGGER_FQ);

        std::vector<int>tagIds;
        ros::param::get("/apriltags", tagIds);

        for(auto tagId : tagIds)
        {
            std::string tag_name = "tag" + std::to_string(tagId);
            std::vector<double> value;
            ros::param::get("/" + tag_name, value);
            geometry_msgs::Point p;
            p.x = value[0];
            p.y = value[1];
            p.z = value[2];
            landmarks_.emplace_back(p);
        }

    }



    void ControlViz::set_marker_from_pose(const tf::Transform &pose, visualization_msgs::Marker &msg) {
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

        yaw = tf::getYaw(pose.getRotation());
        //TODO add logger enable flag
        logger_->addRow(std::vector<double>{x, y, z, yaw});

    }

    void ControlViz::set_marker_color(const MARKER_COLOR &color, visualization_msgs::Marker &msg) {
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

    void ControlViz::update(const tf::Transform &pose, const MARKER_COLOR& color) {
        visualization_msgs::Marker msg, landmarkMsg;
        set_marker_from_pose(pose, msg);
        set_marker_color(color, msg);
        msg.type = visualization_msgs::Marker::SPHERE;

        pub_marker_.publish(msg);

        set_marker_color(CYAN, landmarkMsg);
        landmarkMsg.type = visualization_msgs::Marker::CUBE_LIST;
        landmarkMsg.pose.position.x = landmarkMsg.pose.position.y = landmarkMsg.pose.position.z = 0;
        landmarkMsg.pose.orientation.x = landmarkMsg.pose.orientation.y = landmarkMsg.pose.orientation.z = 0;
        landmarkMsg.pose.orientation.w = 1;
        landmarkMsg.scale.x = landmarkMsg.scale.y = landmarkMsg.scale.z = 0.2;
        landmarkMsg.header.frame_id = "map";
        landmarkMsg.header.stamp = ros::Time::now();
        landmarkMsg.id = 254;
        std::copy(landmarks_.begin(), landmarks_.end(), std::back_inserter(landmarkMsg.points));
        pub_marker_.publish(landmarkMsg);

    }

    void ControlViz::setDrone(const tf::Transform &pose) {

        visualization_msgs::Marker msg, traj_msg;
        set_marker_from_pose(pose, msg);
        msg.ns = "drone";
        msg.type = visualization_msgs::Marker::MESH_RESOURCE;
        msg.mesh_resource = "package://bebop2_controller/config/bebop.dae";
//        msg.mesh_use_embedded_materials = true;
        msg.scale.x = msg.scale.y = msg.scale.z = 0.001;

        set_marker_color(GRAY, msg);
        msg.color.a = 1.0;
        msg.id = 101;

        pub_marker_.publish(msg);

        traj_.emplace_back(msg.pose.position);
        std::copy(traj_.begin(), traj_.end(), std::back_inserter(traj_msg.points));

        traj_msg.header.frame_id = "map";
        traj_msg.ns = "traj";
        traj_msg.header.stamp = ros::Time::now();
        traj_msg.color.a = traj_msg.color.g = traj_msg.color.b = 1.0;
        traj_msg.type = visualization_msgs::Marker::POINTS;
        traj_msg.scale.x = traj_msg.scale.y = traj_msg.scale.z = 0.05;
        traj_msg.id = 102;
        pub_marker_.publish(traj_msg);


    }

    void ControlViz::plot_covariance_ellipse(const Eigen::MatrixXd &xEst, const Eigen::MatrixXd &PEst) {

        Eigen::MatrixXd Pxy = PEst.block<2, 2>(0, 0);
        Eigen::EigenSolver<Eigen::MatrixXd> eig_solver(Pxy);

        Eigen::VectorXd eig_val = eig_solver.eigenvalues().real();
        Eigen::MatrixXd eig_vec = eig_solver.eigenvectors().real();

        int big_ind, small_ind;
        if (eig_val(0) >= eig_val(1)) {
            big_ind = 0;
            small_ind = 1;
        } else {
            big_ind = 1;
            small_ind = 0;
        }

        double step = 0.1;
        std::vector<double> t;
        for (double i = 0; i <= 2 * M_PI + 0.1; i += step) {
            t.push_back(i);
        }

        double a, b;
        try {
            a = std::sqrt(eig_val(big_ind));
        } catch (const std::exception& e) {
            a = 0.0;
        }

        try {
            b = std::sqrt(eig_val(small_ind));
        } catch (const std::exception& e) {
            b = 0.0;
        }

        std::vector<double> x;
        std::vector<double> y;
        for (double it : t) {
            x.push_back(a * std::cos(it));
            y.push_back(b * std::sin(it));
        }

        double angle = std::atan2(eig_vec(1, big_ind), eig_vec(0, big_ind));
        Eigen::MatrixXd fx(t.size(), 2);

        for (size_t i = 0; i < t.size(); ++i) {
            fx(i, 0) = x[i];
            fx(i, 1) = y[i];
        }

        fx = fx * rot_mat_2d(angle);



        // plot ellipse
        visualization_msgs::Marker ellipse;
        set_marker_color(GREEN, ellipse);
        ellipse.type = visualization_msgs::Marker::LINE_STRIP;
        ellipse.pose.position.x = ellipse.pose.position.y = ellipse.pose.position.z = 0;
        ellipse.pose.orientation.x = ellipse.pose.orientation.y = ellipse.pose.orientation.z = 0;
        ellipse.pose.orientation.w = 1;
        ellipse.scale.x = ellipse.scale.y = ellipse.scale.z = 0.05;
        ellipse.header.frame_id = "map";
        ellipse.header.stamp = ros::Time::now();
        ellipse.id = 255;


        for (int i = 0; i < fx.rows(); ++i) {
            geometry_msgs::Point p;
            p.x = fx(i, 0) + xEst(0, 0);
            p.y = fx(i, 1) + xEst(1, 0);
            p.z = xEst(2, 0);
            ellipse.points.emplace_back(p);
        }

        pub_marker_.publish(ellipse);
    }

    Eigen::Matrix2d ControlViz::rot_mat_2d(double angle) {
        Eigen::Matrix2d rotation_matrix;
        rotation_matrix << std::cos(angle), -std::sin(angle),
                std::sin(angle),  std::cos(angle);
        return rotation_matrix;
    }
} // bebop2