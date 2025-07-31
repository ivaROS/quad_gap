#pragma once

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

#include <math.h>
#include <quad_gap/utils/Gap.h>
#include <quad_gap/utils/Trajectory.h>
#include <quad_gap/config/QuadGapConfig.h>
#include <vector>
#include <map>
#include <numeric>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <boost/shared_ptr.hpp>
#include <omp.h>
#include <boost/thread/mutex.hpp>
// #include "tf/transform_datatypes.h"
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <quad_gap/utils/RobotGeometryStorage.h>
#include <quad_gap/utils/RobotGeometryProcessor.h>

namespace quad_gap 
{
    
    class TrajectoryEvaluator
    {
        public:
            TrajectoryEvaluator(const QuadGapConfig& cfg, RobotGeometryProcessor& robot_geo_proc);
            // TrajectoryEvaluator(ros::NodeHandle& nh, const QuadGapConfig& cfg, RobotGeometryStorage& robot_geo_storage);
            
            // TrajectoryEvaluator& operator=(TrajectoryEvaluator other) 
            // {
            //     cfg_ = other.cfg_;
            //     robotGeoProc_ = other.robotGeoProc_;
            
            //     return *this;
            // }
            
            // TrajectoryEvaluator(const TrajectoryEvaluator &t) 
            // {
            //     cfg_ = t.cfg_;
            //     robotGeoProc_ = t.robotGeoProc_;
            // }

            /**
            * \brief receive new laser scan and update member variable accordingly
            * \param scan new laser scan
            */            
            void updateEgoCircle(std::shared_ptr<sensor_msgs::msg::LaserScan const> msg);
            
            
            /**
            * \brief Helper function for transforming global path local waypoint into robot frame
            * \param globalPathLocalWaypointOdomFrame Current local waypoint along global plan in robot frame
            * \param odom2rbt transformation from odom frame to robot frame
            */            
            void transformGlobalPathLocalWaypointToRbtFrame(const geometry_msgs::msg::PoseStamped & globalPathLocalWaypointOdomFrame, 
                                                            const geometry_msgs::msg::TransformStamped & odom2rbt);
            
            // std::vector<float> scoreGaps();
            // Gap * returnAndScoreGaps();
            
            // Full Scoring
            // std::vector<float> scoreTrajectories(const std::vector<geometry_msgs::msg::PoseArray> & sample_traj);
            // geometry_msgs::msg::PoseStamped getLocalGoal() {return globalPathLocalWaypointRobotFrame_; }; // in robot frame
            
            // std::vector<float> & posewiseCosts,
            // float & terminalPoseCost    
            
            /**
            * \brief Function for evaluating pose-wise scores along candidate trajectory
            * \param traj candidate trajectory to score
            */            
            void evaluateTrajectory(Trajectory & traj);
        
        private:
            
            /**
            * \brief function for evaluating intermediate cost of pose for candidate trajectory (in static environment)
            * \param pose pose within candidate trajectory to evaluate
            * \param scan current laser scan
            * \return intermediate cost of pose
            */        
            float evaluatePose(const geometry_msgs::msg::Pose & pose, const sensor_msgs::msg::LaserScan & scan);

            // int searchIdx(geometry_msgs::msg::Pose pose);
            
            
            // float dist2Pose(const float & theta, const float & dist, const geometry_msgs::msg::Pose & pose);

            /**
            * \brief function for calculating intermediate trajectory cost (in static environment)
            * \param rbtToScanDist minimum distance from robot pose to current scan
            * \return intermediate cost of pose
            */            
            float chapterCost(const float & d);

            /**
            * \brief function for evaluating terminal waypoint cost for candidate trajectory
            * \param pose final pose in candidate trajectory to check against terminal waypoint
            * \return terminal waypoint cost for candidate trajectory
            */            
            float terminalGoalCost(const geometry_msgs::msg::Pose & pose);

            const QuadGapConfig* cfg_;
            std::shared_ptr<sensor_msgs::msg::LaserScan const> scan_;
            // std::vector<Gap *> gaps;
            geometry_msgs::msg::PoseStamped globalPathLocalWaypointRobotFrame_;

            boost::mutex globalPlanMutex_; /**< mutex locking thread for updating current global plan */
            boost::mutex scanMutex_; /**< mutex locking thread for updating current scan */
            boost::mutex gap_mutex;

            RobotGeometryProcessor * robotGeoProc_ = NULL; /**< Robot geometry processor */

            rclcpp::Logger logger_ {rclcpp::get_logger("TrajectoryEvaluator")};
    };
}