#pragma once 
#include <visualization_msgs/Marker.h>

namespace Drone{

    class PathViewer{
        public:
            PathViewer() = default;
            ~PathViewer() = default;
            void setPath(const std::vector<std::array<float, 3>>& path)
            {
                path_ = path;
            }

            void setMarker(visualization_msgs::Marker& marker)
            {
                marker_ = marker;
            }

            void setMarkerScale(float scale)
            {
                markerScale_ = scale;
            }

            void setMarkerColor(float r, float g, float b, float a)
            {
                markerColor_[0] = r;
                markerColor_[1] = g;
                markerColor_[2] = b;
                markerColor_[3] = a;
            }

            void setMarkerType(int type)
            {
                markerType_ = type;
            }

            void setMarkerLifetime(float lifetime)
            {
                markerLifetime_ = lifetime;
            }

            void setMarkerFrameId(const std::string& frameId)
            {
                markerFrameId_ = frameId;
            }

            visualization_msgs::Marker generateMarker()
            {
                visualization_msgs::Marker marker = marker_;
                marker.header.frame_id = markerFrameId_;
                marker.type = markerType_;
                marker.scale.x = markerScale_;
                marker.scale.y = markerScale_;
                marker.scale.z = markerScale_;
                marker.color.r = markerColor_[0];
                marker.color.g = markerColor_[1];
                marker.color.b = markerColor_[2];
                marker.color.a = markerColor_[3];
                marker.id = 0;
                marker.lifetime = ros::Duration(markerLifetime_);
                // marker.pose.position.x = 0.0;
                // marker.pose.position.y = 0.0;
                // marker.pose.position.z = 0.0;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;

                marker.ns = "path";
                marker.points.clear();
                for(const auto& point : path_)
                {
                    geometry_msgs::Point p;
                    p.x = point[0];
                    p.y = point[1];
                    p.z = point[2];
                    marker.points.push_back(p);
                }
                return marker;
            }

        private:
            std::vector<std::array<float, 3>> path_;
            visualization_msgs::Marker marker_;
            float markerScale_{0.1};
            float markerColor_[4]{1.0, 0.0, 0.0, 1.0};
            int markerType_{visualization_msgs::Marker::LINE_STRIP};
            float markerLifetime_{0.0};
            std::string markerFrameId_{"map"};
    };
}