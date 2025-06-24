#pragma once

#include <quad_gap/visualization/Visualizer.h>

namespace quad_gap
{
    class TrajectoryVisualizer : public Visualizer
    {
            using Visualizer::Visualizer;
        public: 
            TrajectoryVisualizer(ros::NodeHandle& nh, const QuadGapConfig& cfg);
            void drawGlobalPlan(const std::vector<geometry_msgs::PoseStamped> & globalPlan);

            // void pubAllTraj(const std::vector<geometry_msgs::PoseArray> & prr);
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
            void drawTrajectorySwitchCount(const int & trajSwitchIndex, const geometry_msgs::PoseArray & path);

            void drawGapTrajectories(const std::vector<Trajectory> & trajs);

            /**
            * \brief Visualize snippet of global plan that is within current robot view
            * \param globalPlanSnippet visible snippet of global plan
            */
            void drawRelevantGlobalPlanSnippet(const std::vector<geometry_msgs::PoseStamped> & globalPlanSnippet);

        private: 

            ros::Publisher trajSwitchIdxPublisher; /**< Publisher for planner trajectory switch count */
            ros::Publisher planLoopIdxPublisher; /**< Publisher for planning loop idx */
            ros::Publisher currentTrajectoryPublisher_; /**< ROS publisher for currently tracked trajectory */

            ros::Publisher globalPlanPublisher;
            ros::Publisher gapTrajectoriesPublisher;
            ros::Publisher globalPlanSnippetPublisher; /**< Publisher for visible snippet of global plan */
            
    };
}