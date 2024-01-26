#ifndef EIGHT_H
#define EIGHT_H

#include <QObject>
#include <cmath>
#include <QVector>
#include <numeric>
#include "qcustomplot.h"
#include <sstream>
#include <fstream>

namespace GeomPath
{
    class Eight : public QObject
    {
        Q_OBJECT
    public:
        explicit Eight(int numPoints, QCustomPlot *customPlot, QObject *parent = nullptr);

        void generate_spiral(double xScale, double yScale);
        void generate_rect(double xScale, double yScale);
        void interpolateWaypoints(double resolution);
        void write(const char *filename) const;

    signals:

    protected:
        int numPoints_;
        QCustomPlot *customPlot_;
        QVector<double> X_, Y_;


    };
}

#endif // EIGHT_H
