#pragma once

#include <ros/ros.h>
#include <math.h>
#include <quad_gap/utils/Gap.h>
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

            Visualizer(ros::NodeHandle& nh, const quad_gap::QuadGapConfig& cfg);
            Visualizer& operator=(Visualizer other) {cfg_ = other.cfg_;};
            Visualizer(const Visualizer &t) {cfg_ = t.cfg_;};

        protected:
            const QuadGapConfig* cfg_;
    };
}