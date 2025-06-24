#pragma once

#include <ros/ros.h>
#include <math.h>
#include <quad_gap/utils/Gap.h>
#include <quad_gap/utils/Trajectory.h>
#include <quad_gap/config/QuadGapConfig.h>
#include <vector>
#include <map>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

namespace quad_gap
{
    class Visualizer 
    {
        public: 
            Visualizer() {};
            ~Visualizer() {};

            Visualizer(ros::NodeHandle& nh, const QuadGapConfig& cfg);
            
            Visualizer& operator=(Visualizer other)
            {
                cfg_ = other.cfg_;
                
                return *this;
            };
            
            Visualizer(const Visualizer &t) {cfg_ = t.cfg_;};

        protected:

            void clearMarkerPublisher(const ros::Publisher & publisher)
            {
                visualization_msgs::Marker clearMarker;
                clearMarker.id = 0;
                clearMarker.ns =  "clear";
                clearMarker.action = visualization_msgs::Marker::DELETEALL;
                publisher.publish(clearMarker);
            }

            void clearMarkerArrayPublisher(const ros::Publisher & publisher)
            {
                visualization_msgs::MarkerArray clearMarkerArray;
                visualization_msgs::Marker clearMarker;
                clearMarker.id = 0;
                clearMarker.ns =  "clear";
                clearMarker.action = visualization_msgs::Marker::DELETEALL;
                clearMarkerArray.markers.push_back(clearMarker);
                publisher.publish(clearMarkerArray);
            }

            const QuadGapConfig* cfg_;
    };
}