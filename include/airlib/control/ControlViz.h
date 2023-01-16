//
// Created by redwan on 12/14/22.
//

#ifndef BEBOP2_CONTROLLER_CONTROLVIZ_H
#define BEBOP2_CONTROLLER_CONTROLVIZ_H
#include <ros/ros.h>
#include <memory>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include "airlib/utility/LoggerCSV.h"

namespace bebop2 {
    enum MARKER_COLOR
    {
        RED,
        GREEN,
        BLUE,
        CYAN,
        YELLOW,
        GRAY
    };

    class ControlViz {
    public:
        explicit ControlViz( ros::NodeHandle &nh);
        void setDrone(const tf::Transform& pose);
        void update(const tf::Transform& pose, const MARKER_COLOR& color);

    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_marker_;
        std::unique_ptr<LoggerCSV> logger_;
        const int LOGGER_FQ = 10; // Hz
    protected:
        void set_marker_from_pose(const tf::Transform& pose, visualization_msgs::Marker& msg);
        void set_marker_color(const MARKER_COLOR& color, visualization_msgs::Marker& msg);


    };

} // bebop2

#endif //BEBOP2_CONTROLLER_CONTROLVIZ_H
