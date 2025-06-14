#pragma once

#include <quad_gap/visualization/Visualizer.h>

namespace quad_gap
{
    class TrajectoryVisualizer : public Visualizer
    {
            using Visualizer::Visualizer;
        public: 
            TrajectoryVisualizer(ros::NodeHandle& nh, const QuadGapConfig& cfg);
            void rawGlobalPlan(const std::vector<geometry_msgs::PoseStamped> & );
            void trajScore(geometry_msgs::PoseArray, std::vector<double>);
            void pubAllTraj(std::vector<geometry_msgs::PoseArray> prr);
            void pubAllScore(std::vector<geometry_msgs::PoseArray>, std::vector<std::vector<double>>);


            /**
            * \brief Visualize snippet of global plan that is within current robot view
            * \param globalPlanSnippet visible snippet of global plan
            */
            void drawRelevantGlobalPlanSnippet(const std::vector<geometry_msgs::PoseStamped> & globalPlanSnippet);

        private: 
            ros::Publisher goal_selector_traj_vis;
            ros::Publisher trajectory_score;
            ros::Publisher all_traj_viz;
            ros::Publisher globalPlanSnippetPublisher; /**< Publisher for visible snippet of global plan */
            
    };
}