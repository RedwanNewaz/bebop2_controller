//
// Created by redwan on 12/14/22.
//

#ifndef BEBOP2_CONTROLLER_CONTROLVIZ_H
#define BEBOP2_CONTROLLER_CONTROLVIZ_H
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

namespace bebop2 {
    enum MARKER_COLOR
    {
        RED,
        GREEN,
        BLUE,
        CYAN,
        YELLOW
    };

    class ControlViz {
    public:
        explicit ControlViz( ros::NodeHandle &nh);
        void update(const tf::Transform& pose, const MARKER_COLOR& color);

    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_marker_;

    protected:
        void set_marker_from_pose(const tf::Transform& pose, visualization_msgs::Marker& msg);
        void set_marker_color(const MARKER_COLOR& color, visualization_msgs::Marker& msg);


    };

} // bebop2

#endif //BEBOP2_CONTROLLER_CONTROLVIZ_H
