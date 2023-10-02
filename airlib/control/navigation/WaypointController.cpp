//
// Created by redwan on 10/1/23.
//

#include "airlib/control/navigation/WaypointController.h"

// Define the constructor
WaypointController::WaypointController(std::string name)
        : as_(nh_, name, false), action_name_(name)
{
    // Constructor implementation
    ros::param::get("/max_vel", max_vel);
    ros::param::get("/max_acc", max_acc);

    as_.registerGoalCallback(boost::bind(&WaypointController::executeCB, this));
    as_.registerPreemptCallback(boost::bind(&WaypointController::preemptCB, this));
    force_terminate_ = isAlive_ = false;


    std::vector<double> gains;
    double dt;
    ros::param::get("/pid_gains", gains);
    ros::param::get("/dt", dt);
    auto controller = std::make_shared<controller::quad_pids>(gains, dt);
    interface_ = std::make_shared<bebop2::ControllerInterface>(nh_, controller);

    ROS_INFO("[ros] param = (%lf, %lf, %lf)", max_vel, max_acc, dt);
    as_.start();
}

// Define the destructor
WaypointController::~WaypointController()
{
    ROS_INFO("%s: terminated", action_name_.c_str());
}

// Define other member methods as needed
void WaypointController::preemptCB()
{
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // Set the action state to preempted
    as_.setPreempted();
}

void WaypointController::executeCB()
{
    if (isAlive_)
    {
        ROS_WARN("%s: Received a new goal while the previous goal is still active. Preempting the previous goal.", action_name_.c_str());
        if(as_.isActive())
            preemptCB();
        force_terminate_ = true;
    }

    std::unique_lock lk(mtx_);
    cv_.wait(lk, [&]{return !isAlive_ && !force_terminate_;});

    ROS_INFO_STREAM("accept new goal");
    auto goal = as_.acceptNewGoal();
    std::thread{std::bind(&WaypointController::execute, this, std::placeholders::_1), goal}.detach();
    isAlive_ = true;
    lk.unlock();
}

void WaypointController::execute(const bebop2_controller::WaypointsGoalConstPtr &goal)
{
    std::unique_lock lk(mtx_);

    const std::string csv_path = goal->csv_path;
    int method = goal->method;
    if(!std::filesystem::exists(csv_path))
    {
        ROS_ERROR("csv file not found!");
        lk.unlock();
        return;
    }
    // generate trajectory
    auto demo = getPath(csv_path);
    auto messageQueue = std::make_shared<MessageQueue>();
    std::string selected_planner;
    auto wp_inf = getPlanner(method, messageQueue, selected_planner);
    wp_inf->start(demo);
    control_loop(selected_planner, messageQueue);

    lk.unlock();
    force_terminate_ = isAlive_ = false;
    cv_.notify_all();

}

WAYPOINTS WaypointController::getPath(const std::string &path) {
    // read csv file values
    rapidcsv::Document doc(path, rapidcsv::LabelParams(-1, -1));
    std::vector<float> xValues = doc.GetColumn<float>(0);
    std::vector<float> yValues = doc.GetColumn<float>(1);
    std::vector<float> zValues = doc.GetColumn<float>(2);
    int N = xValues.size();

    WAYPOINTS demo;
    for (int i = 0; i < N; ++i)
        demo.push_back({xValues[i], yValues[i], zValues[i]});
    return demo;
}

WaypointController::WPI WaypointController::getPlanner(int method, WaypointController::MQ  messageQueue, std::string& selected_planner) {
    auto planner = static_cast<PlannerType>(method);

    switch (planner) {
        case CV:
            selected_planner = "[planner typer]: constant velocity";
            return std::make_unique<traj_planner::constant_velocity>(max_vel, max_acc, messageQueue);

        case SNAP:

            selected_planner = "[planner typer]: minimum snap";
            return std::make_unique<traj_planner::minimum_snap>(max_vel, max_acc, messageQueue);

        case JERK:
            selected_planner = "[planner typer]: minimum jerk";
            return std::make_unique<traj_planner::minimum_jerk>(max_vel, max_acc, messageQueue);

    }
}

void WaypointController::control_loop(const std::string &selected_planner, WaypointController::MQ messageQueue) {


    std::vector<std::string> debug_msg;
    debug_msg.emplace_back(selected_planner);
    ROS_INFO_STREAM(selected_planner);

    // execute trajectory
    bool terminated = false;
    int num_setpoints = 0;
    auto start_exec = std::chrono::high_resolution_clock::now();

    while(!terminated)
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

    auto end_exec = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_exec - start_exec;
    debug_msg.emplace_back("[Total setpoints]: " + std::to_string(num_setpoints));
    debug_msg.emplace_back("[Elapsed time]: " + std::to_string(elapsed_seconds.count()) + " sec");
    result_.result = "";
    for(const auto& msg: debug_msg)
        result_.result += msg + " \n";

    if(force_terminate_)
    {
        result_.result += "[!!! Thread !!!] status: Force terminated";
        messageQueue->setTerminate(true);
    }
    else
        as_.setSucceeded(result_);


    ROS_INFO_STREAM( "\n\n" << result_.result);

}
