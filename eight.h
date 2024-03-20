#ifndef EIGHT_H
#define EIGHT_H

#include <QObject>
#include <cmath>
#include <QVector>
#include <numeric>
#include "qcustomplot.h"
#include <sstream>
#include <fstream>
#include <unordered_map>

// Only for pairs of std::hash-able types for simplicity.
// You can of course template this struct to allow other hash functions
struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1,T2> &p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);

        // Mainly for demonstration purposes, i.e. works but is overly simple
        // In the real world, use sth. like boost.hash_combine
        return 10 * h1 + h2;
    }
};



namespace Path
{

    namespace Eight {
        class Base : public QObject
        {
            Q_OBJECT
        public:
            explicit Base(int numRobots, QCustomPlot *customPlot, QObject *parent);
            virtual void generate(double xScale, double yScale) = 0;
            void write(const char *filename);

        protected:
            virtual void singleRobotPath(double xScale, double yScale) = 0;
            virtual void multiRobotPath(double xScale, double yScale) = 0;

        signals:

        protected:
            int numRobots_;
            QCustomPlot *customPlot_;
            std::unordered_map<std::pair<int, int>, QVector<double>, pair_hash> waypoints_;
        };

        class Spiral: public Base{
        public:
            explicit Spiral(int numRobots, QCustomPlot *customPlot, int numPoints, QObject *parent = nullptr);
            void generate(double xScale, double yScale) override;
            void singleRobotPath(double xScale, double yScale) override;
            void multiRobotPath(double xScale, double yScale) override;
        private:
            int numPoints_;
        };

        class Rectangle: public Base{
        public:
            explicit Rectangle(int numRobots, QCustomPlot *customPlot, double resolution, QObject *parent = nullptr);
            void generate(double xScale, double yScale) override;
            void singleRobotPath(double xScale, double yScale) override;
            void multiRobotPath(double xScale, double yScale) override;
        private:
            double resolution_;
            void interpolateWaypoints(QVector<double> tempX, QVector<double> tempY, int index);
        };

    }

}

#endif // EIGHT_H
