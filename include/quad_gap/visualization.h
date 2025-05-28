#pragma once

#include <ros/ros.h>
#include <math.h>
#include <quad_gap/gap.h>
#include <quad_gap/potentialgap_config.h>
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
    class Visualizer {
        public: 
            Visualizer() {};
            ~Visualizer() {};

            Visualizer(ros::NodeHandle& nh, const quad_gap::PotentialGapConfig& cfg);
            Visualizer& operator=(Visualizer other) {cfg_ = other.cfg_;};
            Visualizer(const Visualizer &t) {cfg_ = t.cfg_;};

        protected:
            const PotentialGapConfig* cfg_;
    };

    class GapVisualizer : public Visualizer{
            using Visualizer::Visualizer;
        public: 

            GapVisualizer(ros::NodeHandle& nh, const quad_gap::PotentialGapConfig& cfg);
            void initialize(ros::NodeHandle& nh, const quad_gap::PotentialGapConfig& cfg);
            void drawGap(visualization_msgs::MarkerArray &, quad_gap::Gap g, std::string ns, std::string color = "Default");
            void drawGaps(std::vector<quad_gap::Gap> g, std::string ns, std::string color = "Default");
            void drawManipGap(visualization_msgs::MarkerArray &, quad_gap::Gap g, bool &);
            void drawManipGaps(std::vector<quad_gap::Gap> vec);

        private:
            std::map<std::string, std::vector<std_msgs::ColorRGBA>> colormap;
            ros::Publisher gaparc_publisher;
            ros::Publisher gapside_publisher;
            ros::Publisher gaqgoal_publisher;
    };

    class TrajectoryVisualizer : public Visualizer
    {
            using Visualizer::Visualizer;
        public: 
            TrajectoryVisualizer(ros::NodeHandle& nh, const quad_gap::PotentialGapConfig& cfg);
            void rawGlobalPlan(const std::vector<geometry_msgs::PoseStamped> & );
            void trajScore(geometry_msgs::PoseArray, std::vector<double>);
            void pubAllTraj(std::vector<geometry_msgs::PoseArray> prr);
            void pubAllScore(std::vector<geometry_msgs::PoseArray>, std::vector<std::vector<double>>);
        private: 
            ros::Publisher goal_selector_traj_vis;
            ros::Publisher trajectory_score;
            ros::Publisher all_traj_viz;
    };

    class GoalVisualizer : public Visualizer
    {
        public: 
            using Visualizer::Visualizer;
            GoalVisualizer(ros::NodeHandle& nh, const quad_gap::PotentialGapConfig& cfg);
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