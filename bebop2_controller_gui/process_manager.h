#ifndef PROCESSMANAGER_H
#define PROCESSMANAGER_H

#include <QObject>
#include <QHash>
#include <QSettings>
#include <QFileDialog>
#include <QWidget>
#include <QVector>
#include <QProcess>
#include "dfa.h"

/// @brief It creates and manages processes for various ROS nodes (such as BEBOP, APRILTAG) based on the process IDs given to it.
///It also provides functionality to start and stop processes and handles events such as opening a bag file or an RVIZ file.
class ProcessManager : public QWidget
{
    Q_OBJECT
public:
    /// @brief Constructor for the Process Manager which initializes commands launching and running various ROS Nodes using keys like BEBOP, APRILTAG etc.
    /// @param parent QWidget pointer
    /// @note Marked explicit to prevent implicit conversations from QWidget to ProcessManager 
    explicit ProcessManager(QWidget *parent = nullptr);
   /// @brief Destructor of the Process Manager Class. This destructor deletes the dynamically allocated QSettings and checks if there are processes currently running and calls the stop() member function to stop them. 
    virtual ~ProcessManager();
   /// @brief Starts new processes based on the given set of process ids. 
   /// @param pids Reference to a constant std::set<int> which contains a set of integer process IDs that are to be started by the ProcessManager. It is used to determine which processes should be started, based on the values of the process IDs in the set.
    void start(const std::set<int>& pids);
   /// @brief Stops all currently running processes.
    void stop();

signals:

public slots:
    /// @brief Event handler that is called when the user clicks on the "bag_file" menu option
    void on_bag_file_triggered();
   /// @brief Event handler that is called when the "Open RVIZ file" action is triggered in the menu bar.
    void on_rviz_file_triggered();

protected:
    /// @brief Checks whether a file exists at the specified path.
    /// @param path path of the file
    /// @return True if the the file exists else False
    bool file_exist(const QString& path);

private:
    QHash<int, QString> m_programs;
    QSettings *m_settings;
    QVector<QProcess*> m_processes;
};

#endif // PROCESSMANAGER_H
