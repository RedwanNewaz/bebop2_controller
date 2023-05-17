//
// Created by airlab on 5/16/23.
//

#ifndef BEBOP2_CONTROLLER_NODELET_STATEVIZ_H
#define BEBOP2_CONTROLLER_NODELET_STATEVIZ_H

#include <ros/ros.h>
#include <memory>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

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
    /// @brief This class helps in the 3D visualization of data being rendered. It receives the updated position and orientation of the drone and sends points and lines to a graphical interface.
    /// Hence, visualizes the movement, position, orientation and behaviors of the drone.
    class StateViz {
    public:
        /**
        * @brief Publishes messages through /bebop2/goal topic. 
        * @param nh ROS Node Handle which deals with messages.
        */

        explicit StateViz();
        /** 
        * @brief This function starts and slightly lifts the drone. 
        * As a result sets the Marker from the pose parameter and also sets the color by invoking the set_marker_from_pose() and set_marker_color() functions.
        * @param pose Drone's positon.
        * @note Sets the path of the color to gray. 
        */
        void setDrone(const tf::Transform& pose);
        /** 
        * @brief This function updates the path and the color of the path of the robot.
        * It also publishes the messages from the visualization_msgs::Marker.
        * Lastly, it sets the Marker object type to be a sphere.
        * @param pose current position of the robot.  
        * @param color color of the path in which the robot is in.
        * @note Calls the set_marker_from_pose() and set_marker_color() functions to update the path and color.
        */
        void update(const tf::Transform& pose, const MARKER_COLOR& color);

        /**
         * @brief This timer function display the robot traversed trajectory in the RVIZ.
         * It publishes the messages from the visualization_msgs::Marker.
         * Lastly, it sets the Marker object type to be a point list.
         */
         void timerCallback(const ros::TimerEvent& event);

    private:
        /// ROS NodeHandle for dealing with various messages.
        ros::NodeHandle nh_;
        /// ROS Publisher to advertise various visualization messages to /bebop2/goal.
        ros::Publisher pub_marker_;
        /// ROS Timer to periodically publish the traversed trajectory
        ros::Timer timer_;
        /// data structure to store robot previous positions
        std::vector<geometry_msgs::Point> traj_;
    protected:
        /**
        * @brief Receives the current position of the drone and uses that position to populate the point position and orientation of the marker messages.
        * The marker messages will be used by a graphical interface to visualize the movement of the robot.
        * @param pose Robot's pose having a /map frame.
        * @param msg is a Marker object from visualization msgs that has attributes like position, orientation etc.
        */
        void set_marker_from_pose(const tf::Transform& pose, visualization_msgs::Marker& msg);
        /**
        * @brief sets different colors for different flights and paths of the drone for better understanding.
        * @param color color of the path
        * @param msg is Marker object same as the parameter msg in set_marker_from_pose() and has attributes to set color as well.
        */
        void set_marker_color(const MARKER_COLOR& color, visualization_msgs::Marker& msg);


    };

} // bebop2

#endif //BEBOP2_CONTROLLER_NODELET_STATEVIZ_H
