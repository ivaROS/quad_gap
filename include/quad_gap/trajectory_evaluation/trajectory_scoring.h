#pragma once

#include <ros/ros.h>
#include <math.h>
#include <quad_gap/gap.h>
#include <quad_gap/config/QuadGapConfig.h>
#include <vector>
#include <map>
#include <numeric>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>
#include <omp.h>
#include <boost/thread/mutex.hpp>
#include "tf/transform_datatypes.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <quad_gap/robot_geo_parser.h>

namespace quad_gap{
    class TrajectoryEvaluator{
        public:
        TrajectoryEvaluator(){};
        ~TrajectoryEvaluator(){};

        TrajectoryEvaluator(ros::NodeHandle& nh, const quad_gap::QuadGapConfig& cfg, RobotGeoProc& robot_geo_proc);
        // TrajectoryEvaluator(ros::NodeHandle& nh, const quad_gap::QuadGapConfig& cfg, RobotGeoStorage& robot_geo_storage);
        TrajectoryEvaluator& operator=(TrajectoryEvaluator other) 
        {
            cfg_ = other.cfg_;
            robot_geo_proc_ = other.robot_geo_proc_;
        }
        TrajectoryEvaluator(const TrajectoryEvaluator &t) 
        {
            cfg_ = t.cfg_;
            robot_geo_proc_ = t.robot_geo_proc_;
        }
        
        void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const>);
        void updateGapContainer(const std::vector<quad_gap::Gap>);
        void updateLocalGoal(geometry_msgs::PoseStamped, geometry_msgs::TransformStamped);

        std::vector<double> scoreGaps();
        quad_gap::Gap returnAndScoreGaps();
        
        // Full Scoring
        std::vector<double> scoreTrajectories(std::vector<geometry_msgs::PoseArray>);
        geometry_msgs::PoseStamped getLocalGoal() {return local_goal; }; // in robot frame
        std::vector<double> scoreTrajectory(geometry_msgs::PoseArray traj);
        
        private:
            const QuadGapConfig* cfg_;
            boost::shared_ptr<sensor_msgs::LaserScan const> msg;
            std::vector<quad_gap::Gap> gaps;
            geometry_msgs::PoseStamped local_goal;
            boost::mutex gap_mutex, gplan_mutex, egocircle_mutex;

            double scorePose(geometry_msgs::Pose pose);
            // int searchIdx(geometry_msgs::Pose pose);
            double dist2Pose(float theta, float dist, geometry_msgs::Pose pose);
            double chapterScore(double d, double rmax_offset_val);
            double terminalGoalCost(geometry_msgs::Pose pose);

            int search_idx = -1;

            double r_inscr, rmax, cobs, w, terminal_weight;
            RobotGeoProc robot_geo_proc_;
    };
}