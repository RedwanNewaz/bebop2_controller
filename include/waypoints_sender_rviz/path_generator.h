#pragma once 
#include <vector>
#include <array>
#include <cmath>

namespace Eight{

    class PathGenerator{

        public:
            PathGenerator() = default;
            ~PathGenerator() = default;
            std::vector<std::array<float, 3>> generatePath(float t0)
            {
                std::vector<std::array<float, 3>> path;
                for(int i = 0; i < numPoints_; i++)
                {
                    std::array<float, 3> point;
                    float t = 2 * M_PI * (i + t0) / (float) numPoints_;
                    float x = pathSecaleX_ * cos(t) * sin(t); // You can adjust the scaling factor (2) for size
                    float y = pathSecaleY_ * sin(t);
                    point[0] = x;
                    point[1] = y;
                    point[2] = 1.0;
                    path.push_back(point);
                }
                return path;
            }
            
            void setPathScale(float scaleX, float scaleY)
            {
                pathSecaleX_ = scaleX;
                pathSecaleY_ = scaleY;
            }

            void setNumPoints(int numPoints)
            {
                numPoints_ = numPoints;
            }
        private:
            float pathSecaleX_{3.5};
            float pathSecaleY_{1.5};
            int numPoints_{150};
    };
}