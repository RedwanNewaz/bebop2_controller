#include <ros/ros.h>
#include <filesystem>

#include <actionlib/server/simple_action_server.h>
#include <bebop2_controller/WaypointsAction.h>

#include "rapidcsv.h"
#include "airlib/control/controller.h"
#include "traj_constant_velocity.h"

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

        WAYPOINTS demo;
        for (int i = 0; i < N; ++i)
            demo.push_back({xValues[i], yValues[i], zValues[i]});

        feedback_.time_sequence.push_back(3);

        feedback_.time_sequence.push_back(4);

        // generate trajectory
        auto messageQueue = std::make_shared<MessageQueue>();
        traj_constant_velocity communicator(max_vel, max_acc, messageQueue);
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



};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_controller");
    WaypointController controller("waypoint_action"); // Use a meaningful name for your action server

    // Your ROS node logic here

    ros::spin();
    return 0;
}
