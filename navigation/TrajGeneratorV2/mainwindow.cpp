#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <fmt/format.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("TrajectoryGenerator");
    ui->savePathText->setPlainText("/home/roboticslab/CppDev/TrajViewer/vicon_eight_traj_qt.csv");
    path_ = new GeomPath::Eight(150, ui->customPlot, this);

    methods_[0] = "CV";
    methods_[1] = "MinJerk";
    methods_[2] = "MinSnap";

    ui->methodBox->addItem(methods_[0]);
    ui->methodBox->addItem(methods_[1]);
    ui->methodBox->addItem(methods_[2]);
    ui->methodBox->setCurrentIndex(1);
    sendCounter_ = 0;
    ui->radioButtonSim->setChecked(true);

    on_radioButtonSpiral_clicked();

}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_verticalSlider_sliderMoved(int position)
{
    double value = (position - 50.0) / 90.0;
    if (ui->radioButtonSpiral->isChecked()){
        path_->generate_spiral(pathScaleX_, pathScaleY_ + value);
    } else{
        path_->generate_rect(pathScaleX_, pathScaleY_ + value);
    }
//    path_->generate(pathScaleX_, pathScaleY_ + value);
}

void MainWindow::on_horizontalSlider_sliderMoved(int position)
{
    double value = (position - 50.0) / 90.0;

    if (ui->radioButtonSpiral->isChecked()){
        path_->generate_spiral(pathScaleX_ + value, pathScaleY_);
    } else{
        path_->generate_rect(pathScaleX_ + value, pathScaleY_);
    }

}

void MainWindow::on_saveButton_clicked()
{

     path_->write(ui->savePathText->toPlainText().toStdString().c_str());
}

void MainWindow::on_processStandardOutput()
{
    QString output = QString::fromLocal8Bit(proc->readAllStandardOutput());
    QStringList lines = output.split("\\r\\n");
    lines.removeAll(QString(""));
    QStringListIterator it(lines);
    while(it.hasNext()){
        QString view = it.next();
        if (!view.isEmpty())
            qDebug() << qUtf8Printable(view)<< endl;
    }
}

void MainWindow::on_pushButton_clicked()
{
    on_saveButton_clicked();
    int method = ui->methodBox->currentIndex();
    sendCounter_ += 1;

    qDebug() << methods_[method] << " trajectory sent";

    QStringList cmds;

    bool isSim = ui->radioButtonSim->isChecked();
    QString topic = isSim ? "/waypoint_action/goal" : "/bebop/waypoint_action/goal";
    cmds <<  "pub" << "--once";
    cmds << topic;
    cmds << "bebop2_controller/WaypointsActionGoal";
    auto path = ui->savePathText->toPlainText();

    cmds <<   "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: map}, goal_id: {stamp: {secs: 0, nsecs: 0}, id: " + methods_[method] + QString::number(sendCounter_) + "}, goal: {csv_path: " + path + ", method: " + QString::number(method) + "}}";


    qDebug() << cmds;

    proc = new QProcess(this);
    this->connect(proc, SIGNAL(readyReadStandardOutput()), this, SLOT(on_processStandardOutput()));

    proc->start("rostopic", cmds);

}

void MainWindow::on_radioButtonSpiral_clicked()
{
    qDebug() << "Spiral Eight Selected";
    pathScaleX_ = 3.5;
    pathScaleY_ = 1.5;
    ui->methodBox->setCurrentIndex(1);
    double Xvalue = (ui->horizontalSlider->value() - 50.0) / 90.0;
    double Yvalue = (ui->verticalSlider->value() - 50.0) / 90.0;

//    auto path_ = paths_[0];
    path_->generate_spiral(pathScaleX_+Xvalue, pathScaleY_+Yvalue);
}

void MainWindow::on_radioButtonRect_clicked()
{
    qDebug() << "Rectangle Eight Selected";
    pathScaleX_ = 1.5;
    pathScaleY_ = 1.5;
    ui->methodBox->setCurrentIndex(0);

    double Xvalue = (ui->horizontalSlider->value() - 50.0) / 90.0;
    double Yvalue = (ui->verticalSlider->value() - 50.0) / 90.0;

//    auto path_ = paths_[1];
    path_->generate_rect(pathScaleX_+Xvalue, pathScaleY_+Yvalue);
}
