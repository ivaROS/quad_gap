#pragma once

#include <quad_gap/visualization/Visualizer.h>

namespace quad_gap
{
    class GoalVisualizer : public Visualizer
    {
        public: 
            using Visualizer::Visualizer;
            GoalVisualizer(ros::NodeHandle& nh, const quad_gap::QuadGapConfig& cfg);
            void localGoal(geometry_msgs::PoseStamped);
            void drawGapGoal(visualization_msgs::MarkerArray&, quad_gap::Gap);
            void drawGapGoals(std::vector<quad_gap::Gap>);
        private: 
            ros::Publisher goal_pub;
            ros::Publisher gapwp_pub;
            std_msgs::ColorRGBA gapwp_color;
            std_msgs::ColorRGBA localGoal_color;
    };
}