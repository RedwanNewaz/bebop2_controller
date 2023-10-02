//
// Created by airlab on 10/2/23.
//

#include "airlib/control/navigation/P2PNav.h"

P2PNav::P2PNav(std::string name) :
        as_(nh_, name, false), action_name_(name)
{
    // Constructor implementation
    ros::param::get("/max_vel", max_vel);
    ros::param::get("/max_acc", max_acc);

    as_.registerGoalCallback(boost::bind(&P2PNav::executeCB, this));
    as_.registerPreemptCallback(boost::bind(&P2PNav::preemptCB, this));
    force_terminate_ = isAlive_ = isStateValid_ = false;
    current_state_.resize(3);


    std::vector<double> gains;
    double dt;
    ros::param::get("/pid_gains", gains);
    ros::param::get("/dt", dt);
    auto controller = std::make_shared<controller::quad_pids>(gains, dt);
    interface_ = std::make_shared<bebop2::ControllerInterface>(nh_, controller);

    // subscrobe state topic
    state_sub_ = nh_.subscribe("apriltag/state", 10, &P2PNav::state_callback, this);

    ROS_INFO("[ros] param = (%lf, %lf, %lf)", max_vel, max_acc, dt);
    as_.start();
    ROS_INFO("[P2PNav]: %s started ", name.c_str());
}

void P2PNav::preemptCB() {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // Set the action state to preempted
    as_.setPreempted();
}

void P2PNav::executeCB() {

    if(!isStateValid_)
    {
        ROS_WARN("%s: Avoiding the goal because the robot state is invalid.", action_name_.c_str());
        return;
    }

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
    std::thread{std::bind(&P2PNav::execute, this, std::placeholders::_1), goal}.detach();
    isAlive_ = true;
    lk.unlock();
}

void P2PNav::execute(const bebop2_controller::SetpointsGoalConstPtr &goal) {
    std::unique_lock lk(mtx_);

    WAYPOINTS demo;
    demo.emplace_back(current_state_);
    demo.push_back({goal->setpoint.x, goal->setpoint.y, goal->setpoint.z});

    auto messageQueue = std::make_shared<MessageQueue>();
    auto selected_planner = "[planner typer]: constant velocity";
    auto wp_inf =  std::make_unique<traj_planner::constant_velocity>(max_vel, max_acc, messageQueue);
    wp_inf->start(demo);
    control_loop(selected_planner, messageQueue);
    lk.unlock();
    force_terminate_ = isAlive_ = false;
    cv_.notify_all();
}


void P2PNav::control_loop(const std::string &selected_planner, P2PNav::MQ messageQueue) {
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
            feedback_.feedback.clear();
            feedback_.feedback.push_back(received_message[0]);
            feedback_.feedback.push_back(received_message[1]);
            feedback_.feedback.push_back(received_message[2]);
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
    result_.info = "";
    for(const auto& msg: debug_msg)
        result_.info += msg + " \n";

    if(force_terminate_)
    {
        result_.info += "[!!! Thread !!!] status: Force terminated";
        messageQueue->setTerminate(true);
    }
    else
        as_.setSucceeded(result_);


    ROS_INFO_STREAM( "\n\n" << result_.info);
}

void P2PNav::state_callback(const nav_msgs::Odometry_<std::allocator<void>>::ConstPtr &msg) {
    auto p = msg->pose.pose.position;
    current_state_[0] = p.x;
    current_state_[1] = p.y;
    current_state_[2] = p.z;
    isStateValid_ = true;
}

