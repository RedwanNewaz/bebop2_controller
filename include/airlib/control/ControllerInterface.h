//
// Created by redwan on 12/15/22.
//
/**
 * Controller Base has
 *  a) joystick controller to setup goal location, takeoff and landing commands
 *  b) visualization interface to show the result in rviz
 * Controller Base should take different type of controller
 *  e.g., i) PID controller
 *        ii) LQR controller
 * Controller Base should take a StatePtr and obtain robot state from there
 * Variable convention
 *  private variables end with _
 *  protected variables start with m_
 */
#ifndef airlib_CONTROLLERBASE_H
#define airlib_CONTROLLERBASE_H
#include <ros/ros.h>
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"
#include <mutex>
#include <memory>
#include <geometry_msgs/Twist.h>
#include <cassert>
#include <functional>
#include <algorithm>
#include "airlib/control/ControlViz.h"
#include "airlib/control/ControllerBase.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>



namespace bebop2
{
    const float STEP_INCR = 0.02;
    const float DEAD_ZONE = 0.3;
    const int X_AXIS_INDEX = 2;
    const int Y_AXIS_INDEX = 3;
    const int Z_AXIS_INDEX = 5;
    const int NUM_CONTROLS = 4;



    /**
    * @brief Responsible to control the position, state and rotation of the drone and also provides the code to make the robot hover. It publishes the takeoff, landing commands along with the current position of the drone and the state of the axes and buttons of the joystick controller.
    * It also updates the state of the robot in RVIZ which is used for visaulation. 
    */
    class ControllerInterface{
    public:

        /** @brief Subscribes to sensor_msgs/joy which reports the state of a joystick's axes and buttons.
         * It also publishes the message of drone takeoff and landing along with the linear and angular position of the drone.
         * The constructor is also responsible to call the appropriate functions at specifc rates that plays a role in setting the set-point at a current location.
         * Lastly it contructs an object to send the updated pose to the RVIZ for visualization and also sets the state of the controller's button.
         * @param mGetState Obtains the current state of the robot 
         * @param nh Used to create publishers and subscribes. For eg: The ControllerInterface constructor publishes messages during takeoff and landing with the help of the **ros::NodeHandle** aka nh parameter.
        */ 
        explicit ControllerInterface(ros::NodeHandle& nh, ControllerPtr controller);

        virtual ~ControllerInterface();

        void set_goal_state(const geometry_msgs::PoseStamped::ConstPtr & msg);

    private:
        /// Array of four real numbers in which the first three numbers most commonly represented by x,y,z are vectors and the last number is w which represents the rotation of the robot about the vectors.
        std::vector<float>axes_values_;
        /// @brief Timer to call the joystick_timer_callback() function at a specific rate.
        /// @note The joystick_timer_callback() is invoked every 0.05 second.    
        ros::Timer joystick_timer_;
        /// @brief Timer to call the control_loop() function at a specific rate.
        ros::Timer controller_timer_;
        /// @brief Unique Pointer that points to the address of ControlViz class in the ControlViz.h header file.
        std::unique_ptr<ControlViz> viz_;
        /// Subscribes from sensors_msgs/joy and calls the joystick_callback() as a callback function.       
        ros::Subscriber joystick_sub_;
        /// Publishes the takeoff, landing and velocity information respectively.      
        ros::Publisher drone_takeoff_pub_,  drone_land_pub_,cmd_vel_pub_;
        /// @brief Co-ordinates of the points set by the controller.
        std::vector<double> setPose_;
        /// @brief state uncertainty
        COV_ELLIPSE uncertaintyEllipse_;
    protected:
        /// This ROS Handle attribute is used to create various publishers and subscribers which also deals with messages.
        ros::NodeHandle m_nh;
        /// Locks some functions for safe operations
        std::mutex m_mu;
        /// @brief Applications applied to control the state of the robot using Joystick controller.
        enum ButtonState{
            /// Setpoint can be freely moved with joystick. Sphere color blue
            IDLE = 0,
            /// Drone will takeoff from the ground
            TAKEOFF, 
            /// Drone will land
            LAND, 
            /// Set current location as a set point (for hover). Sphere color yellow
            ENGAGE, 
            /// Start PID controller for the current setpoint. Sphere color cyan
            CONTROL 
        }m_buttonState;


        // controller param
        /// Sample time
        double dt_; 
        /// Goal region threshold
        double m_goal_thres;

        ///State callback
        ros::Subscriber state_sub_, set_pose_sub_;
        void state_callback(const nav_msgs::Odometry::ConstPtr& msg);

        bool isPointInRotatedEllipse(double x, double y, const COV_ELLIPSE& ellipse);




    private:
        /// @brief This function finds out the engaged button of the joystick i.e, checks the state of the button and
        /// and publishes the resulted behavior of the drone (Take Off or Land).
        /// It also updates the positon of drone which is also passed to RVIZ to update the position and path for visalization.
        /// @param msg reports the state of a joysticks axes and buttons.
        void joystick_callback(const sensor_msgs::Joy::ConstPtr& msg);
        /**
        * @brief Updates the setpoint of the drone when the axes of the joystick controller is changed or moved. As a result, it also updates the color and path of the drone for visualization. 
        * @param event Creates a timer to call a function at a specific rate.
        */   
        void joystick_timer_callback(const ros::TimerEvent& event);
       /// @brief Responsible for making the drone hover at a certain position
        void control_loop(const std::vector<double>& state);
        /// @brief Sets the position and rotation of the drone through the setpoints .
        /// @param state is used as an parameter to pass the setpoints.
        /// @return Returns the tf::Transform pose which is the new position and rotated angle of the robot.
        tf::Transform getStateVecToTransform(const std::vector<double>& state);

        ControllerPtr drone_controller_;
    protected:
        /// @brief This function sets the linear positon of the drone to all 3 linear axes and to angular z-axis and publishes the message of the current position of the drone.
        /// @param U is array of commands of size 4 which sets the above mentioned position of the drone.
        void publish_cmd_vel(const std::vector<double>&U);
         /// @brief This function calculates the distance between the setpoint of the drone and the current state position.
        /// @param X is an array of vectors of data type double representing the present position of the drone.
        /// @param setPoints is an array of vectors representing the commanded position.
        /// @return Errors between the current and commanded position of the robot.
        double goal_distance(const std::vector<double>& X, const std::vector<double>& setPoints);

 };
}


#endif //airlib_CONTROLLERBASE_H
