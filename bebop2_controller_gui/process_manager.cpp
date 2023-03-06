#include "process_manager.h"
#include <QDebug>

ProcessManager::ProcessManager(QWidget *parent) : QWidget(parent)
{
    // roslaunch
    m_programs[BEBOP] = "roslaunch bebop2_controller bebop_autonomy.launch";
    m_programs[APRILTAG] = "roslaunch bebop2_controller bebop_apriltag.launch";
    m_programs[EKF] = "roslaunch bebop2_controller bebop_ekf.launch";

    // roslaunch with args
    m_programs[START] = "roslaunch bebop2_controller bebop_controller.launch ";

    // rosrun
    m_programs[JOY] = "rosrun joy joy_node";

    // rosrun with args
    m_programs[RVIZ] = "rosrun rviz rviz -d ";
    m_programs[BAG] = "rosrun rosbag play --clock ";

    // settings
    m_settings = new QSettings("gui.conf");




}

ProcessManager::~ProcessManager()
{
    delete m_settings;
    if(!m_processes.empty())
        stop();
}

void ProcessManager::start(const std::set<int> &pids)
{
    // stop processes
    if(!m_processes.empty())
        stop();

    for(const auto& pid: pids)
    {
        if(m_programs.find(pid) != m_programs.end())
        {
            QString program = m_programs[pid];

            if (pid == RVIZ && !file_exist(m_settings->value("rviz_file").toString()))
            {
                qDebug() << "[ProcessManager] RVIZ file not found!";
                continue;
            }
            else if(pid == BAG && !file_exist(m_settings->value("bag_file").toString()))
            {
                qDebug() << "[ProcessManager] BAG file not found!";
                continue;
            }
            // add them to process queue
            int filterType = LOWPASS;
            if(pids.count(EKF))
                filterType = EKF;
            // check which param needs arguments
            QString args;
            if(pid == START)
            {
                args = (filterType == EKF) ? "filter:=ekf" : "filter:=lowpass";
                if(pids.count(BAG))
                    args += " exp:=sim-bag";
                else if(pids.count(DUMMY))
                    args += " exp:=sim-dummy";
                else if(pids.count(EXP))
                    args += " exp:=bebop-apriltag";

            }
            else if (pid == RVIZ)
                args = m_settings->value("rviz_file").toString();
            else if (pid == BAG)
                args = m_settings->value("bag_file").toString();
            qDebug() << "[ProcessManager] " << program + args;

            QProcess *proc = new QProcess(this);
            QString full_program = program + args;
            QStringList proc_args = full_program.split(" ");
            QString proc_base = proc_args.front();
            proc_args.pop_front();
            proc->start(proc_base, proc_args);
            m_processes.push_back(proc);
        }



    }
}

void ProcessManager::stop()
{
    for(auto& proc: m_processes)
    {
        proc->terminate();
    }
    m_processes.clear();
}

void ProcessManager::on_bag_file_triggered()
{

    QString bagFile = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                    "/home",
                                                    tr("Rosbag (*.bag)"));
    if(!bagFile.isEmpty())
        m_settings->setValue("bag_file", bagFile);
}

void ProcessManager::on_rviz_file_triggered()
{
    QString rvizFile = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                    "/home",
                                                    tr("RVIZ (*.rviz)"));
    if(!rvizFile.isEmpty())
        m_settings->setValue("rviz_file", rvizFile);
}

bool ProcessManager::file_exist(const QString &path)
{
    QFileInfo check_file(path);
    // check if file exists and if yes: Is it really a file and no directory?
    if (check_file.exists() && check_file.isFile()) {
        return true;
    } else {
        return false;
    }
}
