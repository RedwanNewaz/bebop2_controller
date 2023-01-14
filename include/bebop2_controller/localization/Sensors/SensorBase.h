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

class SensorBase: std::enable_shared_from_this<SensorBase>
{
public:
    SensorPtr getPtr()
    {
        return shared_from_this();
    }
    virtual void operator()(std::vector<double>& result) = 0;
    virtual bool empty() = 0;

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
    tf::TransformBroadcaster m_br;


};

#endif //BEBOP2_CONTROLLER_SENSORBASE_H
