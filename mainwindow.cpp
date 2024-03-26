#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <fmt/format.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("TrajectoryGenerator");
    ui->savePathText->setPlainText("/var/tmp");


    methods_[0] = "CV";
    methods_[1] = "MinJerk";
    methods_[2] = "MinSnap";

    ui->methodBox->addItem(methods_[0]);
    ui->methodBox->addItem(methods_[1]);
    ui->methodBox->addItem(methods_[2]);
    ui->methodBox->setCurrentIndex(1);
    sendCounter_ = 0;

    on_radioButtonSpiral_clicked();

}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_verticalSlider_sliderMoved(int position)
{
    double value = (position - 50.0) / 90.0;
    path_->generate(pathScaleX_, pathScaleY_ + value);
}

void MainWindow::on_horizontalSlider_sliderMoved(int position)
{
    double value = (position - 50.0) / 90.0;

    path_->generate(pathScaleX_ + value, pathScaleY_);

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
    bool isMultiRobot = ui->checkMultiRobot->isChecked();
    int numRobots = (isMultiRobot) ? 2 : 1;

    qDebug() << methods_[method] << " trajectory sent";

    for(int i = 0; i < numRobots; ++i)
    {
        QStringList cmds;

        bool isSim = ui->radioButtonSim->isChecked();
        bool isBebop5 = ui->bebop5->isChecked();

        QString topic = "";
        if(isSim)
            topic = "/waypoint_action/goal";
        else if (isMultiRobot)
        {
            if (i ==0)
                 topic = "/bebop5/waypoint_action/goal";
            else
                 topic = "/bebop7/waypoint_action/goal";
        }
        else if(isBebop5)
            topic = "/bebop5/waypoint_action/goal";
        else
            topic = "/bebop7/waypoint_action/goal";
//        QString topic = isSim ? "/waypoint_action/goal" : "/bebop/waypoint_action/goal";


        cmds <<  "pub" << "--once";
        cmds << topic;
        cmds << "bebop2_controller/WaypointsActionGoal";
        auto path = ui->savePathText->toPlainText() + "/" + QString::number(i+1) + ".csv";

        cmds <<   "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: map}, goal_id: {stamp: {secs: 0, nsecs: 0}, id: " + methods_[method] + QString::number(i * sendCounter_) + "}, goal: {csv_path: " + path + ", method: " + QString::number(method) + "}}";


        qDebug() << cmds;

        proc = new QProcess(this);
        this->connect(proc, SIGNAL(readyReadStandardOutput()), this, SLOT(on_processStandardOutput()));

        proc->start("rostopic", cmds);

    }

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
    path_ = new Path::Eight::Spiral(1, ui->customPlot, 150, this);
    path_->generate(pathScaleX_+Xvalue, pathScaleY_+Yvalue);
}

void MainWindow::on_radioButtonRect_clicked()
{
    qDebug() << "Rectangle Eight Selected";
    pathScaleX_ = 1.5;
    pathScaleY_ = 1.5;
    ui->methodBox->setCurrentIndex(0);

    double Xvalue = (ui->horizontalSlider->value() - 50.0) / 90.0;
    double Yvalue = (ui->verticalSlider->value() - 50.0) / 90.0;

    int numRobots = ui->checkMultiRobot->isChecked()?2:1;


    path_ = new Path::Eight::Rectangle(numRobots, ui->customPlot, 0.3, this);
    path_->generate(pathScaleX_+Xvalue, pathScaleY_+Yvalue);
}
