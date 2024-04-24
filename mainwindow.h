#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QHash>
#include <QDebug>
#include <QProcess>
#include <memory>
#include "eight.h"
#include "viz_traj.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_verticalSlider_sliderMoved(int position);

    void on_horizontalSlider_sliderMoved(int position);

    void on_saveButton_clicked();

    void on_pushButton_clicked();

    void on_processStandardOutput();

    void on_radioButtonSpiral_clicked();

    void on_radioButtonRect_clicked();

    void simulateTrajectory();

    void setpoint(QVector<double> point);



    void on_checkMultiRobot_stateChanged(int arg1);

private:
    Ui::MainWindow *ui;
    Path::Eight::Base *path_;
    double pathScaleX_, pathScaleY_;
    QHash<int, QString> methods_;
    int sendCounter_;
    QProcess *proc;
    QVector<viz_traj*> display_trajs;

};
#endif // MAINWINDOW_H
