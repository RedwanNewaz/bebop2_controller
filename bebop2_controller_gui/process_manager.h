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

class ProcessManager : public QWidget
{
    Q_OBJECT
public:
    explicit ProcessManager(QWidget *parent = nullptr);
    virtual ~ProcessManager();
    void start(const std::set<int>& pids);
    void stop();

signals:

public slots:
    void on_bag_file_triggered();
    void on_rviz_file_triggered();

protected:
    bool file_exist(const QString& path);

private:
    QHash<int, QString> m_programs;
    QSettings *m_settings;
    QVector<QProcess*> m_processes;
};

#endif // PROCESSMANAGER_H
