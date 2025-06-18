#pragma once

#include <quad_gap/visualization/Visualizer.h>

namespace quad_gap
{
    class GoalVisualizer : public Visualizer
    {
        public: 
            using Visualizer::Visualizer;
            GoalVisualizer(ros::NodeHandle& nh, const QuadGapConfig& cfg);

            // void localGoal(const geometry_msgs::PoseStamped);

            void drawGapGoals(const std::vector<Gap> & gaps);

            /**
            * \brief Visualize global goal
            * \param globalGoalOdomFrame global goal in odom frame
            */
            void drawGlobalGoal(const geometry_msgs::PoseStamped & globalGoalOdomFrame);

            void drawGlobalPathLocalWaypoint(const geometry_msgs::PoseStamped & globalPathLocalWaypoint);

        private: 
    
            void drawGapGoal(visualization_msgs::Marker & marker, const Gap & gap);  

            ros::Publisher globalPathLocalWaypointPublisher; /**< Publisher for global path local waypoint */
            ros::Publisher globalGoalPublisher; /**< Publisher for global goal */
            ros::Publisher gapGoalPublisher;

            std_msgs::ColorRGBA gapGoalColor;
            std_msgs::ColorRGBA globalPathLocalWaypointColor; /**< Color to visualize global path local waypoint with */
            std_msgs::ColorRGBA globalGoalColor; /**< Color to visualize global goal with */
    };
}