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
            qDebug() << qUtf8Printable(view)<< Qt::endl;
    }
}

void MainWindow::on_pushButton_clicked()
{
    bool isSim = ui->radioButtonSim->isChecked();
    if(isSim)
    {
        simulateTrajectory();
        return;
    }

    on_saveButton_clicked();
    int method = ui->methodBox->currentIndex();
    sendCounter_ += 1;

    int numRobots = ui->checkMultiRobot->isChecked()?2:1;

    qDebug() << methods_[method] << " trajectory sent";
    QStringList topics;
    topics << "/bebop5/waypoint_action/goal" << "/bebop7/waypoint_action/goal";

    bool isBebop5 = ui->bebop5->isChecked();
    auto sendCmd = [&](int i)
    {
        QStringList cmds;
        cmds <<  "pub" << "--once";
        cmds << topics[i];
        cmds << "bebop2_controller/WaypointsActionGoal";
        auto path = ui->savePathText->toPlainText() + "/" + QString::number(i+1) + ".csv";
//        cmds <<   "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: map}, goal_id: {stamp: {secs: 0, nsecs: 0}, id: " + methods_[method] + QString::number(sendCounter_) + "}, goal: {csv_path: " + path + ", method: " + QString::number(method) + "}}";

        // Format the string
        std::string formatted = fmt::format(
              "{{"
              "header: {{seq: 0, stamp: {{secs: 0, nsecs: 0}}, frame_id: map}}, "
              "goal_id: {{stamp: {{secs: 0, nsecs: 0}}, id: {0}{1}}}, "
              "goal: {{csv_path: {2}, method: {3}}}"
              "}}",
              methods_[method].toStdString(), sendCounter_, path.toStdString(), method
        );
        // Print or use the formatted string
        std::cout << formatted << std::endl;
        cmds <<  QString::fromStdString(formatted);

        qDebug() << cmds;




        proc = new QProcess(this);
        this->connect(proc, SIGNAL(readyReadStandardOutput()), this, SLOT(on_processStandardOutput()));
        proc->start("rostopic", cmds);
    };

    if (numRobots > 1)
        for(int i = 0; i < numRobots; ++i)
            sendCmd(i);
    else if (isBebop5)
        sendCmd(0);
    else
        sendCmd(1);

}

void MainWindow::on_radioButtonSpiral_clicked()
{
    qDebug() << "Spiral Eight Selected";
    pathScaleX_ = 3.5;
    pathScaleY_ = 1.5;
    ui->methodBox->setCurrentIndex(2);
    double Xvalue = (ui->horizontalSlider->value() - 50.0) / 90.0;
    double Yvalue = (ui->verticalSlider->value() - 50.0) / 90.0;

    int numRobots = ui->checkMultiRobot->isChecked()?2:1;
    path_ = new Path::Eight::Spiral(numRobots, ui->customPlot, 150, this);
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

void MainWindow::simulateTrajectory()
{
    int method = ui->methodBox->currentIndex();
    qDebug() << methods_[method] << " trajectory simulating";
    int numRobots = ui->checkMultiRobot->isChecked()?2:1;


    double max_vel = ui->maxVel->text().toDouble();
    double max_acc = ui->maxAcc->text().toDouble();
    display_trajs.clear();
    for (int i=0; i<numRobots; ++i)
    {
        auto X = path_->getPointsAxis(i, 0);
        auto Y = path_->getPointsAxis(i, 1);
        auto viz = new viz_traj (max_vel, max_acc, methods_[method], i);
        connect(viz, SIGNAL(setpoint(QVector<double>)), this, SLOT(setpoint(QVector<double>)));
        viz->setWaypoints(X, Y);
        display_trajs.push_back(viz);
    }
}

void MainWindow::setpoint(QVector<double> point)
{
//    qDebug() << point.size() << " received";
    path_->movePoint(point);
}

