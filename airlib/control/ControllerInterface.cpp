//
// Created by redwan on 12/15/22.
//

#include <numeric>
#include <utility>
#include "airlib/control/ControllerInterface.h"

namespace bebop2
{
    
    ControllerInterface::ControllerInterface(ros::NodeHandle& nh, ControllerPtr controller) :
    m_nh(nh), drone_controller_(controller){

        ros::param::get("/dt", dt_);
        ros::param::get("/goal_thres", m_goal_thres);

        joystick_sub_ = m_nh.subscribe("/joy", 10, &bebop2::ControllerInterface::joystick_callback, this);
        drone_takeoff_pub_ = m_nh.advertise<std_msgs::Empty>("takeoff", 1);
        drone_land_pub_ = m_nh.advertise<std_msgs::Empty>("land", 1);
        cmd_vel_pub_ = m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        joystick_timer_ = m_nh.createTimer(ros::Duration(0.05), &bebop2::ControllerInterface::joystick_timer_callback, this);

        state_sub_ = m_nh.subscribe("apriltag/state", 10, &ControllerInterface::state_callback, this);
        set_pose_sub_ = m_nh.subscribe("set_pose", 10, &ControllerInterface::set_goal_state, this);
        viz_ = std::make_unique<ControlViz>(nh);
        m_buttonState = ENGAGE;

        ROS_INFO("[ControllerInterface] Initialization complete ...");

    }
    ControllerInterface::~ControllerInterface()
    {

    }

    
    void ControllerInterface::joystick_callback(const sensor_msgs::Joy_<std::allocator<void>>::ConstPtr &msg) {

        std::lock_guard<std::mutex> lk(m_mu);
        auto it = std::find(msg->buttons.begin(), msg->buttons.end(), 1);
        if(it != msg->buttons.end())
        {
            int index = std::distance(msg->buttons.begin(), it);
            m_buttonState = static_cast<ButtonState>(index);

            std_msgs::Empty empty;

            if(m_buttonState == TAKEOFF)
            {
                drone_takeoff_pub_.publish(empty);
                ROS_INFO("[Button] pressed = %d -> TAKE OFF", m_buttonState);
                m_buttonState = ENGAGE;
            }

            if(m_buttonState == LAND)
            {
                drone_land_pub_.publish(empty);
                ROS_INFO("[Button] pressed = %d -> LAND", m_buttonState);
            }


            if(!setPose_.empty())
            {
                auto pose = getStateVecToTransform(setPose_);
                switch (m_buttonState) {
                    case TAKEOFF: viz_->update(pose, GREEN); break;
                    case IDLE:    viz_->update(pose, BLUE); break;
                    case ENGAGE:  viz_->update(pose, YELLOW);  break;
                    case CONTROL: viz_->update(pose, CYAN);   break;
                    case LAND:    viz_->update(pose, RED);    break;
                }
            }

        }
        axes_values_.clear();
        std::copy(msg->axes.begin(), msg->axes.end(), std::back_inserter(axes_values_));

    }

    
    void ControllerInterface::joystick_timer_callback(const ros::TimerEvent &event) {
        if(axes_values_.empty() || std::accumulate(axes_values_.begin(), axes_values_.end(), 0.0) == 0.0)
            return;

        auto filter = [&](float val){
            float sign = val >= 0 ? 1 : -1;
            return abs(val) > DEAD_ZONE ? sign * STEP_INCR : 0;
        };

        float dz = filter(axes_values_[Z_AXIS_INDEX]);
        float dx = -filter(axes_values_[X_AXIS_INDEX]);
        float dy = filter(axes_values_[Y_AXIS_INDEX]);

        //  update_setpoint with (dx, dy, dz);
        if(!setPose_.empty())
        { setPose_[0] += dx;
            setPose_[1] += dy;
            setPose_[2] += dz;
            setPose_[3] = M_PI_2;

            auto pose = getStateVecToTransform(setPose_);
            switch (m_buttonState) {
                case TAKEOFF: viz_->update(pose, GREEN); break;
                case IDLE:    viz_->update(pose, BLUE); break;
                case ENGAGE:  viz_->update(pose, YELLOW);  break;
                case CONTROL: viz_->update(pose, CYAN);   break;
                case LAND:    viz_->update(pose, RED);    break;
            }
        }

    }




    
    void ControllerInterface::control_loop(const std::vector<double>& state) {


//        ROS_INFO_STREAM("[ControllerInterface] state size" << state.size());

        if(m_buttonState == ENGAGE)
        {
            // set setpoint at current position
            setPose_.clear();
            std::copy(state.begin(), state.end(), std::back_inserter(setPose_));
            // visualize your state with sphere
            auto pose = getStateVecToTransform(setPose_);
            viz_->update(pose, YELLOW);

        }
        else if(m_buttonState == CONTROL)
        {
            // check setPose_ with in ellipse or not
            if(isPointInRotatedEllipse(setPose_[0], setPose_[1], uncertaintyEllipse_))
            {
                std::vector<double>U(4, 0);
                publish_cmd_vel(U);
            }
            else if(goal_distance(state, setPose_) > m_goal_thres)
            {
                // actively control position
                std::vector<double>U;
                drone_controller_->compute_control(state, setPose_, U);
//                ROS_INFO("[PositionController] vx = %lf, vy = %lf, vz = %lf, wz = %lf", U[0], U[1], U[2], U[3]);
                publish_cmd_vel(U);
            }
            else
            {
                std::vector<double>U(4, 0);
                publish_cmd_vel(U);
            }

        }

        // show drone
        auto transform = getStateVecToTransform(state);
        viz_->setDrone(transform);



    }


    
    void ControllerInterface::publish_cmd_vel(const std::vector<double> &U) {

        std::lock_guard<std::mutex> lk(m_mu);
        assert(U.size() == 4 && "4 commands must be sent");
        geometry_msgs::Twist msg;

        msg.linear.x  = U[0];
        msg.linear.y  = U[1];
        msg.linear.z  = U[2];
        msg.angular.z = U[3];
        cmd_vel_pub_.publish(msg);

    }

    
    double ControllerInterface::goal_distance(const std::vector<double> &X, const std::vector<double> &setPoints) {
        double error = 0;
        for (int i = 0; i < X.size(); ++i) {
            double e = X[i] - setPoints[i];
            error += e * e;
        }
        return sqrt(error);
    }


    
    tf::Transform ControllerInterface::getStateVecToTransform(const std::vector<double> &state) {
        tf::Transform pose;
        tf::Quaternion q;
        q.setRPY(0, 0, state[3]);
        pose.setOrigin(tf::Vector3(state[0], state[1], state[2]));
        pose.setRotation(q);
        return pose;
    }

    void ControllerInterface::state_callback(const nav_msgs::Odometry::ConstPtr &msg) {
        auto p = msg->pose.pose.position;
        auto q = msg->pose.pose.orientation;
        tf::Matrix3x3 m(tf::Quaternion(q.x, q.y, q.z, q.w));
        double roll, pitch, yaw;
        m.getRPY(roll,pitch, yaw);
        yaw = yaw + M_PI_2;
        yaw = fmod(yaw + M_PI, 2 * M_PI) - M_PI;

        std::vector<double>state{p.x, p.y, p.z, yaw};
        control_loop(state);

        Eigen::MatrixXd xEst(3, 1);
        xEst << p.x, p.y, p.z;

        Eigen::MatrixXd PEst(6, 6);
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                int oneDindex = (i * 6) + j; // Indexes
                PEst(i, j) = msg->pose.covariance[oneDindex];
            }
        }

        uncertaintyEllipse_ = viz_->plot_covariance_ellipse(xEst, PEst);

    }

    void ControllerInterface::set_goal_state(const geometry_msgs::PoseStamped::ConstPtr & msg) {
        if(m_buttonState != CONTROL)
            m_buttonState = CONTROL;

        setPose_.clear();
        auto qq = msg->pose.orientation;
        tf::Quaternion q(qq.x, qq.y, qq.z, qq.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        auto pp = msg->pose.position;
        setPose_.push_back(pp.x);
        setPose_.push_back(pp.y);
        setPose_.push_back(pp.z);
        setPose_.push_back(yaw);

        tf::Transform pose;
        pose.setOrigin(tf::Vector3(pp.x, pp.y, pp.z));
        pose.setRotation(q);
        viz_->update(pose, CYAN);
    }

    bool ControllerInterface::isPointInRotatedEllipse(double x, double y, const COV_ELLIPSE &ellipse) {
        // Translate point to the ellipse's local coordinate system
        double dx = x - ellipse.center_x;
        double dy = y - ellipse.center_y;

        // Rotate the point back to the unrotated coordinate system
        double angleRad = -ellipse.angle * M_PI / 180.0;
        double xRotated = dx * cos(angleRad) - dy * sin(angleRad);
        double yRotated = dx * sin(angleRad) + dy * cos(angleRad);

        // Check if the point satisfies the equation of the unrotated ellipse
        double normalizedX = xRotated / ellipse.major_axis;
        double normalizedY = yRotated / ellipse.minor_axis;

        // Equation of an unrotated ellipse: (x/a)^2 + (y/b)^2 <= 1, where a and b are semi-axes lengths
        return (normalizedX * normalizedX + normalizedY * normalizedY) <= 1.0;
    }


}