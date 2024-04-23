#ifndef VIZ_TRAJ_H
#define VIZ_TRAJ_H
#include <memory>
#include <QObject>
#include <QDebug>
#include <QThread>
#include <chrono>
#include <thread>
#include <QTimer>
#include "traj_gen/trajectory_planner.h"

class viz_traj : public QObject
{
    Q_OBJECT
public:
    explicit viz_traj(double max_vel, double max_acc, QString plannerType, int robotIndex, QObject *parent = nullptr);
    void setWaypoints(const QVector<double>& X, const QVector<double>& Y);
//    void setup(QThread& cThread);
    QTimer *timer_;
signals:
    void setpoint(QVector<double>);

private:
    std::unique_ptr<waypoint_trajectory_interface> planner_;
    WAYPOINTS traj_;
    int currentIndex_;
    int robotIndex_;


public slots:
    void run();

};

#endif // VIZ_TRAJ_H
