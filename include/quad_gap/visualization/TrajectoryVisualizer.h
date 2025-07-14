#pragma once

#include <quad_gap/visualization/Visualizer.h>

namespace quad_gap
{
    class TrajectoryVisualizer : public Visualizer
    {
            using Visualizer::Visualizer;
        public: 
            TrajectoryVisualizer(const rclcpp::Node::SharedPtr & node, const QuadGapConfig& cfg);
            void drawGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped> & globalPlan);

            // void pubAllTraj(const std::vector<geometry_msgs::msg::PoseArray> & prr);
            void drawCurrentTrajectory(const Trajectory & traj);

            /**
            * \brief Visualize counter for planning loop
            * \param planningLoopIdx counter for planning loop
            */
            void drawPlanningLoopIdx(const int & planningLoopIdx);

            /**
            * \brief Visualize occurrence of a trajectory switch for planner
            * \param trajSwitchIndex trajectory switch count
            * \param chosenTraj new trajectory that planner is switching to
            */
            void drawTrajectorySwitchCount(const int & trajSwitchIndex,  const Trajectory & traj);

            void drawGapTrajectories(const std::vector<Trajectory> & trajs);

            /**
            * \brief Visualize snippet of global plan that is within current robot view
            * \param globalPlanSnippet visible snippet of global plan
            */
            void drawRelevantGlobalPlanSnippet(const std::vector<geometry_msgs::msg::PoseStamped> & globalPlanSnippet);

        private: 

            rclcpp::Node::SharedPtr node_; /**< Node handle for ROS communication */

            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajSwitchIdxPublisher; /**< Publisher for planner trajectory switch count */
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr planLoopIdxPublisher; /**< Publisher for planning loop idx */
            
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr currentTrajectoryPublisher_; /**< ROS publisher for currently tracked trajectory */
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr globalPlanPublisher;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gapTrajectoriesPublisher;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr globalPlanSnippetPublisher; /**< Publisher for visible snippet of global plan */

    };
}