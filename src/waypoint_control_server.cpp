#include <ros/ros.h>
#include <filesystem>
#include <chrono>
#include <actionlib/server/simple_action_server.h>
#include <bebop2_controller/WaypointsAction.h>

#include "rapidcsv.h"
#include "airlib/control/controller.h"
#include "trajectory_planner.h"

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
    bool force_terminate_, isAlive_;


public:
    WaypointController(std::string name) :
            as_(nh_, name,
//                boost::bind(&WaypointController::executeCB, this, _1),
                false), // Use the provided name for the action server
            action_name_(name)
    {


        ros::param::get("/max_vel", max_vel);
        ros::param::get("/max_acc", max_acc);

        as_.registerGoalCallback(boost::bind(&WaypointController::executeCB, this));
        as_.registerPreemptCallback(boost::bind(&WaypointController::preemptCB, this));



        std::vector<double> gains;
        double dt;
        ros::param::get("/pid_gains", gains);
        ros::param::get("/dt", dt);
        auto controller = std::make_shared<controller::quad_pids>(gains, dt);
        interface_ = std::make_shared<bebop2::ControllerInterface>(nh_, controller);

        ROS_INFO("[ros] param = (%lf, %lf, %lf)", max_vel, max_acc, dt);
        as_.start();
        force_terminate_ = isAlive_ = false;
    }

    ~WaypointController(void)
    {
        ROS_INFO("%s: terminated", action_name_.c_str());
    }

    void preemptCB()
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // Set the action state to preempted

        as_.setPreempted();

    }

    void executeCB()
    {

        // this needs to return quickly to avoid blocking the executor, so spin up a new thread

//        if (as_.isActive())
//        {
//            ROS_WARN("%s: Received a new goal while the previous goal is still active. Preempting the previous goal.", action_name_.c_str());
//             preemptCB();
//            force_terminate_ = true;
//
//        }

        if(isAlive_)
        {
            force_terminate_ = true;

            preemptCB();
            as_.setAborted(result_);
            std::this_thread::sleep_for(2s);

        }

        ROS_INFO_STREAM("accept new goal");
        auto goal = as_.acceptNewGoal();
//        execute(goal);
        std::thread{std::bind(&WaypointController::execute, this, std::placeholders::_1), goal}.detach();
        isAlive_ = true;
    }


    void execute(const bebop2_controller::WaypointsGoalConstPtr &goal)
    {

        const std::string path = goal->csv_path;
        if(!std::filesystem::exists(path))
        {
            ROS_ERROR("csv file not found!");
            return;
        }

        // read csv file values
        rapidcsv::Document doc(path, rapidcsv::LabelParams(-1, -1));
        std::vector<float> xValues = doc.GetColumn<float>(0);
        std::vector<float> yValues = doc.GetColumn<float>(1);
        std::vector<float> zValues = doc.GetColumn<float>(2);
        int N = xValues.size();

        WAYPOINTS demo;
        for (int i = 0; i < N; ++i)
            demo.push_back({xValues[i], yValues[i], zValues[i]});


        // generate trajectory
        std::vector<std::string> debug_msg;

        auto messageQueue = std::make_shared<MessageQueue>();
        waypoint_trajectory_interface *wp_inf;

        auto planner = static_cast<PlannerType>(goal->method);
        std::string selected_planner;
        switch (planner) {
            case CV:
                selected_planner = "[planner typer]: constant velocity";
                wp_inf = new traj_planner::constant_velocity(max_vel, max_acc, messageQueue);
                break;
            case SNAP:

                selected_planner = "[planner typer]: minimum snap";
                wp_inf = new traj_planner::minimum_snap(max_vel, max_acc, messageQueue);
                break;
            case JERK:
                selected_planner = "[planner typer]: minimum jerk";
                wp_inf = new traj_planner::minimum_jerk(max_vel, max_acc, messageQueue);
                break;
        }

        debug_msg.emplace_back(selected_planner);
        ROS_INFO_STREAM(selected_planner);
        wp_inf->start(demo);

        // execute trajectory
        bool terminated = false;
        int num_setpoints = 0;
        auto start_exec = std::chrono::high_resolution_clock::now();

        while(!terminated || !force_terminate_)
        {
            std::vector<double> received_message;
            if(messageQueue->pop(received_message))
            {
                ROS_INFO("%s next point = (%lf, %lf, %lf)", selected_planner.c_str(), received_message[0], received_message[1], received_message[2]);
                feedback_.setpoint.clear();
                feedback_.setpoint.push_back(received_message[0]);
                feedback_.setpoint.push_back(received_message[1]);
                feedback_.setpoint.push_back(received_message[2]);
                as_.publishFeedback(feedback_);
                interface_->set_goal_state(received_message);
                ++num_setpoints;
            }
            std::this_thread::sleep_for(2ms);

            terminated = messageQueue->isTerminated() || force_terminate_;
            messageQueue->setTerminate(force_terminate_);
        }

        messageQueue->setTerminate(true);
        delete wp_inf;


        auto end_exec = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = end_exec - start_exec;


        debug_msg.emplace_back("[Total setpoints]: " + std::to_string(num_setpoints));
        debug_msg.emplace_back("[Elapsed time]: " + std::to_string(elapsed_seconds.count()) + " sec");


        for(const auto& msg: debug_msg)
            result_.result += msg + " \n";


        as_.setSucceeded(result_);



        while (!messageQueue->isQuit())
        {
            std::cout <<"waiting to quit thread " << std::endl;
            std::this_thread::sleep_for(100ms);
        }

        isAlive_ = false;
        std::cout <<"TrajController terminated " << std::endl;
//        preemptCB();




    }
private:
    enum PlannerType
    {
        CV = 0,
        JERK,
        SNAP
    };



};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_controller");
    WaypointController controller("waypoint_action"); // Use a meaningful name for your action server

    // Your ROS node logic here
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
