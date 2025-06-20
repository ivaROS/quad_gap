#pragma once

#include <ros/ros.h>
#include <quad_gap/config/QuadGapConfig.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <quad_gap/utils/Gap.h>
#include "quad_gap/TrajPlan.h"
#include <quad_gap/trajectory_generation/GapTrajectoryGenerator.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

namespace quad_gap 
{
    class TrajectoryController 
    {
        public:

            TrajectoryController(ros::NodeHandle& nh, const QuadGapConfig& cfg);

            geometry_msgs::Twist controlLaw(const geometry_msgs::Pose & current, 
                                            const nav_msgs::Odometry & desired,
                                            const sensor_msgs::LaserScan & inflated_egocircle, 
                                            const geometry_msgs::PoseStamped & init_pose);
            
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan);

            int targetPoseIdx(const geometry_msgs::Pose & curr_pose, const TrajPlan & ref_pose);

            TrajPlan trajGen(const geometry_msgs::PoseArray & orig_traj);

        private:
            Eigen::Matrix2cf getComplexMatrix(const float & x, const float & y, const float & quat_w, const float & quat_z);
            Eigen::Matrix2cf getComplexMatrix(const float & x, const float & y, const float & theta);
            float dist2Pose(const float & theta, const float & dist, const geometry_msgs::Pose & pose);

            std::vector<geometry_msgs::Point> findLocalLine(const int & idx);
            float polDist(const float & l1, const float & t1, const float & l2, const float & t2);

            bool geqThres(const float dist);

            Eigen::Vector2f car2pol(const Eigen::Vector2f & a);
            Eigen::Vector2f pol2car(const Eigen::Vector2f & a);
            Eigen::Vector3f projection_method(const float & min_diff_x, const float & min_diff_y);

            float thres;
            const QuadGapConfig* cfg_;
            boost::shared_ptr<sensor_msgs::LaserScan const> scan_;
            boost::mutex egocircle_l;
            ros::Publisher projection_viz;
            ros::Time last_time;
    };
}