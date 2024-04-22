#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "trajectory/trajectory_planner.h"



class PyTrajGen{
public:
    PyTrajGen(double max_vel, double max_acc, const std::string& plannerType):
    max_acc_(max_acc), max_vel_(max_vel)
    {
        messageQueue_ = std::make_shared<MessageQueue>();
        if (plannerType == "CV")
            planner_ =  std::make_unique<traj_planner::constant_velocity>(max_vel_, max_acc_, messageQueue_);
        else if (plannerType == "MinSnap")
            planner_ =  std::make_unique<traj_planner::minimum_snap>(max_vel_, max_acc_, messageQueue_);
        else // default min jerk
            planner_ =  std::make_unique<traj_planner::minimum_jerk>(max_vel_, max_acc_, messageQueue_);

    }

    void clearWaypoints()
    {
        wps_.clear();
    }

    void addWaypoint(double x, double y, double z)
    {
        wps_.emplace_back(std::vector<double>{x, y, z});
    }


    std::vector<std::vector<double>> getTraj()
    {
        // compute local trajectory using dynamic window
        planner_->convert_waypoints(wps_);
        std::vector<std::vector<double>> result;
        for(auto& s: planner_->getTrajectory())
        {
            std::vector<double> elem;
            for (int i = 0; i < s.size(); ++i) {
                elem.push_back(s[i]);
            }
            result.emplace_back(elem);
        }
        return result;
    }
private:
    double max_vel_, max_acc_;
    WAYPOINTS wps_;
    std::shared_ptr<MessageQueue> messageQueue_;
    std::unique_ptr<waypoint_trajectory_interface> planner_;
};

namespace py = pybind11;

PYBIND11_MODULE(py_traj_gen, handle) {
    handle.doc() = "py_traj_gen plugin"; // optional module docstring
    py::class_<PyTrajGen>(handle, "PyTrajGen")
            .def(py::init<double, double, const std::string&>())
            .def("addWaypoint", &PyTrajGen::addWaypoint)
            .def("clearWaypoints", &PyTrajGen::clearWaypoints)
            .def("getTraj", [](PyTrajGen & self){
                py::array out = py::cast(self.getTraj());
                return out;
            });
}