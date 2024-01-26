#include "eight.h"
#include <QDebug>
namespace GeomPath
{
    Eight::Eight(int numPoints, QCustomPlot *customPlot, QObject *parent) :
        numPoints_(numPoints), customPlot_(customPlot), QObject(parent)
    {
        // create graph and assign data to it:
        customPlot_->addGraph();
        customPlot_->graph(0)->setLineStyle(QCPGraph::lsNone);
        customPlot_->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 10));
        // give the axes some labels:
        customPlot_->xAxis->setLabel("x");
        customPlot_->yAxis->setLabel("y");
    }

    void Eight::generate_spiral(double xScale, double yScale)
    {
        X_.clear();
        Y_.clear();
        double xmin, xmax, ymin, ymax;
        xmin = ymin = std::numeric_limits<double>::max();
        xmax = ymax = -std::numeric_limits<double>::max();

        for (int i = 0; i <= numPoints_; ++i)
        {
                double t = 2 * M_PI * i / (double) numPoints_;
                double x = xScale * cos(t) * sin(t); // You can adjust the scaling factor (2) for size
                double y = yScale * sin(t);
                X_.push_back(x);
                Y_.push_back(y);

                // keep track of axis
                xmin = std::min(xmin, x);
                xmax = std::max(xmax, x);

                ymin = std::min(ymin, y);
                ymax = std::max(ymax, y);

        }

        customPlot_->graph(0)->setData(X_, Y_);

        // set axes ranges, so we see all data:
        customPlot_->xAxis->setRange(xmin - 0.5, xmax + 0.5);
        customPlot_->yAxis->setRange(ymin - 0.5, ymax + 0.5);

        customPlot_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
        customPlot_->replot();
    }

    void Eight::generate_rect(double xScale, double yScale)
    {
        X_.clear();
        Y_.clear();
        double xmin, xmax, ymin, ymax;
        xmin = -xScale;
        ymin = -yScale;
        xmax = xScale;
        ymax = yScale;

        X_.resize(10);
        Y_.resize(10);

        X_[0] = Y_[0] = X_[9] = Y_[9] = Y_[8] = Y_[4] = Y_[1] = Y_[5] = 0;
        X_[1] = X_[2] = X_[5] = X_[6] = xmax;
        X_[3] = X_[4] = X_[7] = X_[8] = xmin;
        Y_[6] = Y_[7] = ymin;
        Y_[2] = Y_[3] = ymax;
        interpolateWaypoints(0.2);

        customPlot_->graph(0)->setData(X_, Y_);

        // set axes ranges, so we see all data:
        customPlot_->xAxis->setRange(xmin - 0.5, xmax + 0.5);
        customPlot_->yAxis->setRange(ymin - 0.5, ymax + 0.5);

        customPlot_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
        customPlot_->replot();
    }

    void Eight::interpolateWaypoints(double resolution)
    {
        // Iterate through each pair of waypoints
        QVector<double> tempX, tempY;
        std::copy(X_.begin(), X_.end(), std::back_inserter(tempX));
        std::copy(Y_.begin(), Y_.end(), std::back_inserter(tempY));

        X_.clear();
        Y_.clear();

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
            int numInterpolatedPoints = std::ceil(distance / resolution);

            // Calculate the step size for interpolation
            double stepSize = 1.0 / numInterpolatedPoints;

            // Interpolate and add the waypoints
            for (int j = 0; j <= numInterpolatedPoints; ++j) {
                double t = stepSize * j;
                double interpolatedX = startX + t * dx;
                double interpolatedY = startY + t * dy;
                X_.push_back(interpolatedX);
                Y_.push_back(interpolatedY);
            }
        }
    }

    void Eight::write(const char *filename) const
    {
//        if(X_.size() < numPoints_ || Y_.size() < numPoints_)
//            return;
        qDebug() << X_.size();
        qDebug() << Y_.size();

        double z = 1.0;// You can adjust the vertical offset (+1) for position
        std::stringstream ss;
        for (int i = 0; i < X_.size(); ++i)
        {
           ss << X_[i] << "," << Y_[i] << "," << z << "\n";
        }

        std::ofstream myfile;
        myfile.open (filename);
        myfile << ss.str();
        myfile.close();
    }





}
