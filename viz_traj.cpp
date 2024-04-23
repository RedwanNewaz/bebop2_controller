#include "viz_traj.h"


viz_traj::viz_traj(double max_vel, double max_acc, QString plannerType, QObject *parent)
    : QObject{parent}
{
    if (plannerType == "CV")
        planner_ = std::make_unique<traj_planner::constant_velocity>(max_vel, max_acc);
    else if (plannerType == "MinJerk")
        planner_ = std::make_unique<traj_planner::minimum_jerk>(max_vel, max_acc);
    else if (plannerType == "MinSnap")
        planner_ = std::make_unique<traj_planner::minimum_snap>(max_vel, max_acc);

}

void viz_traj::setWaypoints(const QVector<double> &X, const QVector<double> &Y)
{
    WAYPOINTS wps;

    for(int j = 0; j < X.size(); ++j)
        wps.push_back({X[j], Y[j], 1.0});
    planner_->convert_waypoints(wps);
    traj_ = planner_->getTrajectory();
    currentIndex_ = 0;
    qDebug() << "traj points " << traj_.size();
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &viz_traj::run);
    timer_->start(1);
}


void viz_traj::run()
{

    timer_->stop();
    if (currentIndex_ < traj_.size())
    {
        double dt = 0.0 + 1e-9;
        auto point = traj_[currentIndex_++];
        QVector<double> elem(point.begin()+1, point.end());
        emit setpoint(elem);
        if (currentIndex_ < traj_.size())
            dt = traj_[currentIndex_][0] - point[0];
        int nap_time = 1000 * dt;
        timer_->setInterval(nap_time);
        timer_->start();
        qDebug() << "traj index " << currentIndex_ << " / " << traj_.size();
    }



}
