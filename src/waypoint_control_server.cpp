#include <ros/ros.h>
#include <filesystem>

#include <actionlib/server/simple_action_server.h>
#include <bebop2_controller/WaypointsAction.h>

#include "rapidcsv.h"
#include "airlib/control/controller.h"
#include "waypoint_trajectory.h"

class WaypointController
{
protected:
    ros::NodeHandle nh_;
    std::string action_name_;
    actionlib::SimpleActionServer<bebop2_controller::WaypointsAction> as_;
    bebop2_controller::WaypointsFeedback feedback_;
    bebop2_controller::WaypointsResult result_;

    std::shared_ptr<bebop2::ControllerInterface> interface_;
    double max_vel, max_acc;

    struct point {
        double x;
        double y;
        double z;

        double operator -(const point&other) const
        {
            double dx = this->x - other.x;
            double dy = this->y - other.y;

            return std::sqrt(dx * dx + dy * dy);
        }
    };



public:
    WaypointController(std::string name) :
            as_(nh_, name, boost::bind(&WaypointController::executeCB, this, _1), false), // Use the provided name for the action server
            action_name_(name)
    {
        as_.start();
        ros::param::get("/max_vel", max_vel);
        ros::param::get("/max_acc", max_acc);

        std::vector<double> gains;
        double dt;
        ros::param::get("/pid_gains", gains);
        ros::param::get("/dt", dt);
        auto controller = std::make_shared<controller::quad_pids>(gains, dt);
        interface_ = std::make_shared<bebop2::ControllerInterface>(nh_, controller);

        ROS_INFO("[ros] param = (%lf, %lf, %lf)", max_vel, max_acc, dt);
    }

    ~WaypointController(void)
    {
    }

    void executeCB(const bebop2_controller::WaypointsGoalConstPtr &goal)
    {

        const std::string path = goal->csv_path;
        if(!std::filesystem::exists(path))
        {
            ROS_ERROR("csv file not found!");
            return;
        }

        feedback_.time_sequence.clear();
        feedback_.time_sequence.push_back(0);


        // read csv file values
        rapidcsv::Document doc(path, rapidcsv::LabelParams(-1, -1));
        std::vector<float> xValues = doc.GetColumn<float>(0);
        std::vector<float> yValues = doc.GetColumn<float>(1);
        std::vector<float> zValues = doc.GetColumn<float>(2);
        int N = xValues.size();
        feedback_.time_sequence.push_back(1);
        //convert it to a path: vector of waypoints
        std::vector<point> path_points(N);
        WAYPOINTS demo;
        for (int i = 0; i < N; ++i) {
            path_points[i] = point{xValues[i], yValues[i], zValues[i]};

            demo.push_back({xValues[i], yValues[i], zValues[i]});
        }
        feedback_.time_sequence.push_back(3);

//        // interpolate the path based on a fixed distance
//        double m_resolution = 0.1;
//        auto final_path = interpolateWaypoints(path_points, m_resolution);
//
////        for(const auto& point:final_path)
////        {
////            ROS_INFO("[Waypoints] next point = (%lf, %lf, %lf)", point[0], point[1], point[2]);
////        }
        feedback_.time_sequence.push_back(4);

        // generate trajectory
        auto messageQueue = std::make_shared<MessageQueue>();
        waypoint_trajectory communicator(max_vel, max_acc, messageQueue);
        communicator.start(demo);
        feedback_.time_sequence.push_back(5);
//
//        ROS_INFO("[path length] %zu", final_path.size());




        // execute trajectory
        bool terminated = false;
        while(!terminated)
        {
            std::vector<double> received_message;
            if(messageQueue->pop(received_message))
            {
                ROS_INFO("[TrajController] next point = (%lf, %lf, %lf)", received_message[0], received_message[1], received_message[2]);
                interface_->set_goal_state(received_message);

            }

            std::this_thread::sleep_for(2ms);
            terminated = messageQueue->isTerminated();
        }
        feedback_.time_sequence.push_back(6);

        result_.tracking_sequence = feedback_.time_sequence;


    }

    // interpolation
    WAYPOINTS interpolateWaypoints(const std::vector<point>& waypoints, double resolution) {
        WAYPOINTS interpolatedWaypoints;

        // Iterate through each pair of waypoints
        for (size_t i = 0; i < waypoints.size() - 1; ++i) {
            const point& start = waypoints[i];
            const point& end = waypoints[i + 1];

            // Calculate the distance between start and end waypoints
            double dx = end.x - start.x;
            double dy = end.y - start.y;
            double dz = end.z - start.z;
            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            // Calculate the number of interpolated points between start and end waypoints
            int numInterpolatedPoints = std::ceil(distance / resolution);

            // Calculate the step size for interpolation
            double stepSize = 1.0 / numInterpolatedPoints;

            // Interpolate and add the waypoints
            for (int j = 0; j <= numInterpolatedPoints; ++j) {
                double t = stepSize * j;
                double interpolatedX = start.x + t * dx;
                double interpolatedY = start.y + t * dy;
                double interpolatedZ = start.z + t * dz;
                interpolatedWaypoints.push_back({interpolatedX, interpolatedY, interpolatedZ});
            }
        }

        return interpolatedWaypoints;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_controller");
    WaypointController controller("waypoint_action"); // Use a meaningful name for your action server

    // Your ROS node logic here

    ros::spin();
    return 0;
}
