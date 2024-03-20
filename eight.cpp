#include "eight.h"
#include <QDebug>
namespace Path
{
    Eight::Base::Base(int numRobots, QCustomPlot *customPlot, QObject *parent) :
        numRobots_(numRobots), customPlot_(customPlot), QObject(parent)
    {
        // create graph and assign data to it:
        customPlot_->addGraph();
        customPlot_->graph(0)->setLineStyle(QCPGraph::lsNone);
        customPlot_->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 10));
        // create another graph and assign data to it:
        customPlot_->addGraph();
        customPlot->graph(1)->setPen(QPen(Qt::red));
        customPlot_->graph(1)->setLineStyle(QCPGraph::lsNone);
        customPlot_->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 10));
        // give the axes some labels:
        customPlot_->xAxis->setLabel("x");
        customPlot_->yAxis->setLabel("y");


    }

    Eight::Spiral::Spiral(int numRobots, QCustomPlot *customPlot, int numPoints, QObject *parent)
        :Base(numRobots, customPlot, parent), numPoints_(numPoints)
    {

    }


    void Eight::Spiral::generate(double xScale, double yScale)
    {
      return (numRobots_ == 1) ? singleRobotPath(xScale, yScale) : multiRobotPath(xScale, yScale);
    }

    void Eight::Spiral::singleRobotPath(double xScale, double yScale)
    {
        waypoints_.clear();

        double xmin, xmax, ymin, ymax;
        xmin = ymin = std::numeric_limits<double>::max();
        xmax = ymax = -std::numeric_limits<double>::max();

        for (int i = 0; i <= numPoints_; ++i)
        {
                double t = 2 * M_PI * i / (double) numPoints_;
                double x = xScale * cos(t) * sin(t); // You can adjust the scaling factor (2) for size
                double y = yScale * sin(t);

                waypoints_[std::make_pair(0, 0)].push_back(x);
                waypoints_[std::make_pair(0, 1)].push_back(y);

                // keep track of axis
                xmin = std::min(xmin, x);
                xmax = std::max(xmax, x);

                ymin = std::min(ymin, y);
                ymax = std::max(ymax, y);

        }

        customPlot_->graph(0)->setData(waypoints_[std::make_pair(0, 0)], waypoints_[std::make_pair(0, 1)]);
        customPlot_->graph(1)->setData({}, {});


        // set axes ranges, so we see all data:
        customPlot_->xAxis->setRange(xmin - 0.5, xmax + 0.5);
        customPlot_->yAxis->setRange(ymin - 0.5, ymax + 0.5);

        customPlot_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
        customPlot_->replot();
    }

    void Eight::Spiral::multiRobotPath(double xScale, double yScale)
    {
        //TODO implement it
    }

    Eight::Rectangle::Rectangle(int numRobots, QCustomPlot *customPlot, double resolution, QObject *parent)
    :Base(numRobots, customPlot, parent), resolution_(resolution)
    {

    }

    void Eight::Rectangle::generate(double xScale, double yScale)
    {
      return (numRobots_ == 1) ? singleRobotPath(xScale, yScale) : multiRobotPath(xScale, yScale);
    }

    void Eight::Rectangle::singleRobotPath(double xScale, double yScale)
    {
        double xmin, xmax, ymin, ymax;
        xmin = -xScale;
        ymin = -yScale;
        xmax = xScale;
        ymax = yScale;

        QVector<double> X_, Y_;
        X_.resize(10);
        Y_.resize(10);

        X_[0] = Y_[0] = X_[9] = Y_[9] = Y_[8] = Y_[4] = Y_[1] = Y_[5] = 0;
        X_[1] = X_[2] = X_[5] = X_[6] = xmax;
        X_[3] = X_[4] = X_[7] = X_[8] = xmin;
        Y_[6] = Y_[7] = ymin;
        Y_[2] = Y_[3] = ymax;
        interpolateWaypoints(X_, Y_, 0);

        customPlot_->graph(0)->setData(waypoints_[std::make_pair(0, 0)], waypoints_[std::make_pair(0, 1)]);
        customPlot_->graph(1)->setData({}, {});



        // set axes ranges, so we see all data:
        customPlot_->xAxis->setRange(xmin - 0.5, xmax + 0.5);
        customPlot_->yAxis->setRange(ymin - 0.5, ymax + 0.5);

        customPlot_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
        customPlot_->replot();
    }

    void Eight::Rectangle::multiRobotPath(double xScale, double yScale)
    {
        waypoints_.clear();
        //implement it
        QVector<double> X1(6), Y1(6), X2(5), Y2(5);
        double xmin, xmax, ymin, ymax;
        xmin = -xScale;
        ymin = -yScale;
        xmax = xScale;
        ymax = yScale;

        X1[3] = X1[4] = xmin;
        X1[1] = X1[2] = xmax;
        X1[5] = Y1[5] = Y1[4] = Y1[1] = 0;
        Y1[3] = Y1[2] = ymax;
        interpolateWaypoints(X1, Y1, 0);




        X2[0] = X2[4] = X2[3] = xmin;
        X2[1] = X2[2] = xmax;
        Y2[4] = Y2[1] = 0;
        Y2[2] = Y2[3] = ymin;
        interpolateWaypoints(X2, Y2, 1);

         customPlot_->graph(0)->setData(waypoints_[std::make_pair(0, 0)], waypoints_[std::make_pair(0, 1)]);

        customPlot_->graph(1)->setData(waypoints_[std::make_pair(1, 0)], waypoints_[std::make_pair(1, 1)]);

        // set axes ranges, so we see all data:
        customPlot_->xAxis->setRange(xmin - 0.5, xmax + 0.5);
        customPlot_->yAxis->setRange(ymin - 0.5, ymax + 0.5);

        customPlot_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
        customPlot_->replot();

        qDebug() << waypoints_.size();


    }

    void Eight::Rectangle::interpolateWaypoints(QVector<double> tempX, QVector<double> tempY, int index)
    {

        // Iterate through each pair of waypoints
        for (size_t i = 0; i < tempX.size() - 1; ++i) {
            auto startX = tempX[i];
            auto startY = tempY[i];
            auto endX = tempX[i + 1];
            auto endY = tempY[i+1];

            // Calculate the distance between start and end waypoints
            double dx = endX - startX;
            double dy = endY - startY;

            double distance = std::sqrt(dx * dx + dy * dy);

            // Calculate the number of interpolated points between start and end waypoints
            int numInterpolatedPoints = std::ceil(distance / resolution_);

            // Calculate the step size for interpolation
            double stepSize = 1.0 / numInterpolatedPoints;

            // Interpolate and add the waypoints
            for (int j = 0; j <= numInterpolatedPoints; ++j) {
                double t = stepSize * j;
                double interpolatedX = startX + t * dx;
                double interpolatedY = startY + t * dy;
                waypoints_[std::make_pair(index, 0)].push_back(interpolatedX);
                waypoints_[std::make_pair(index, 1)].push_back(interpolatedY);
            }
        }
    }

    void Eight::Base::write(const char *filename)
    {


        double z = 1.0;// You can adjust the vertical offset (+1) for position

        for (int robotId = 0; robotId < numRobots_; ++robotId) {
            std::stringstream ss;
            for (int i = 0; i < waypoints_[std::make_pair(robotId,0)].size(); ++i)
            {
               ss << waypoints_[std::make_pair(robotId,0)][i] << "," << waypoints_[std::make_pair(robotId,1)][i] << "," << z << "\n";
            }

            std::ofstream myfile;
            std::string outputFile(filename);
            outputFile = outputFile + "/" + std::to_string(robotId + 1) + ".csv";
            qDebug() << "[@saving] " << outputFile.c_str();
            myfile.open(outputFile.c_str());
            myfile << ss.str();
            myfile.close();
        }


    }


}
