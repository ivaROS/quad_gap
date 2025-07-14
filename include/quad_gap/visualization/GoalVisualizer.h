#pragma once

#include <quad_gap/visualization/Visualizer.h>

namespace quad_gap
{
    class GoalVisualizer : public Visualizer
    {
        public: 
            using Visualizer::Visualizer;
            GoalVisualizer(const rclcpp::Node::SharedPtr & node, const QuadGapConfig& cfg);

            // void localGoal(const geometry_msgs::msg::PoseStamped);

            void drawGapGoals(const std::vector<Gap *> & gaps);

            /**
            * \brief Visualize global goal
            * \param globalGoalOdomFrame global goal in odom frame
            */
            void drawGlobalGoal(const geometry_msgs::msg::PoseStamped & globalGoalOdomFrame);

            void drawGlobalPathLocalWaypoint(const geometry_msgs::msg::PoseStamped & globalPathLocalWaypoint);

        private: 
    
            void drawGapGoal(visualization_msgs::msg::Marker & marker, Gap * gap);  

            rclcpp::Node::SharedPtr node_; /**< Node handle for ROS communication */

            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr globalPathLocalWaypointPublisher; /**< Publisher for global path local waypoint */
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr globalGoalPublisher; /**< Publisher for global goal */
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr gapGoalPublisher;

            std_msgs::msg::ColorRGBA gapGoalsColor;
            std_msgs::msg::ColorRGBA globalPathLocalWaypointColor; /**< Color to visualize global path local waypoint with */
            std_msgs::msg::ColorRGBA globalGoalColor; /**< Color to visualize global goal with */
    };
}