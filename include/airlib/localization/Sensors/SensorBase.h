//
// Created by redwan on 1/14/23.
//

#ifndef BEBOP2_CONTROLLER_SENSORBASE_H
#define BEBOP2_CONTROLLER_SENSORBASE_H
#include <iostream>
#include <memory>
#include <vector>
#include <tf/transform_broadcaster.h>

class SensorBase;

typedef std::shared_ptr<SensorBase> SensorPtr;

/**
*   @brief This class provides a conection to acquire internal state vector 
    
*   so that other components required for the bebop to operate successfully can do estimation and calculation. 
*   It also publishes the drone's positon and transformation so that we can picture it in a graphical interface such as RVIZ.  
*/
class SensorBase: std::enable_shared_from_this<SensorBase>
{
public:
    /// @brief systematic way to share SensorPtr to control class 
    /// @return  SensorPtr 
    SensorPtr getPtr()
    {
        return shared_from_this();
    }
    /**
     * @brief This is an interface to access to the internal state vector so that other modules can do their computation 
     * @note this interface depends on bool empty()  
     * @param result copy internal state vector to result vector 
     */
    virtual void operator()(std::vector<double>& result) = 0;
    /**
     * @brief This is an interface to check whether internal state vector is empty or not so that we can safely read internal state vector.
     * @return true if internal state vector has not been populated yet
     * @return false if internal state vector has been initialized
     * @note previous void operator()(std::vector<double>& result) depends on it
     */
    virtual bool empty() = 0;

    /**
     * @brief Publish robot transformation so that we can visualize it in RVIZ.  
     * tf::TransformBroadcaster m_br; variable does not depends of NodeHandle.
     * Therefore we can implement publish_tf in this abstract class.
     * @param state vector must have at least 4 parameters [x, y, z, yaw]
     * @return broadcast map to robot tf
     * @note This function depends on private vairable tf::TransformBroadcaster m_br;
     */
    void publish_tf(const std::vector<double> &state) {
        tf::Transform globalCoord;
        tf::Quaternion q;
        q.setRPY(0, 0, state[3]);
        globalCoord.setOrigin(tf::Vector3(state[0], state[1], state[2]));
        globalCoord.setRotation(q);
        m_br.sendTransform(tf::StampedTransform(globalCoord, ros::Time::now(), "map", "robot"));
//        ROS_INFO_STREAM(robot_position_);
    }

private:
    /// @brief broadcast map to robot transformation. 
    tf::TransformBroadcaster m_br;


};

#endif //BEBOP2_CONTROLLER_SENSORBASE_H
