#pragma once

#include <quad_gap/visualization/Visualizer.h>

namespace quad_gap
{
    class GoalVisualizer : public Visualizer
    {
        public: 
            using Visualizer::Visualizer;
            GoalVisualizer(ros::NodeHandle& nh, const QuadGapConfig& cfg);
            void localGoal(geometry_msgs::PoseStamped);
            void drawGapGoal(visualization_msgs::MarkerArray&, Gap);
            void drawGapGoals(std::vector<Gap>);

            /**
            * \brief Visualize global goal
            * \param globalGoalOdomFrame global goal in odom frame
            */
            void drawGlobalGoal(const geometry_msgs::PoseStamped & globalGoalOdomFrame);

            void drawGlobalPathLocalWaypoint(const geometry_msgs::PoseStamped & globalPathLocalWaypoint);

        private: 
            ros::Publisher globalPathLocalWaypointPublisher; /**< Publisher for global path local waypoint */
            ros::Publisher globalGoalPublisher; /**< Publisher for global goal */
            ros::Publisher gapwp_pub;

            std_msgs::ColorRGBA gapwp_color;
            std_msgs::ColorRGBA globalPathLocalWaypointColor; /**< Color to visualize global path local waypoint with */
            std_msgs::ColorRGBA globalGoalColor; /**< Color to visualize global goal with */
    };
}