#pragma once

#include <ros/ros.h>
#include <math.h>
#include <quad_gap/utils/Gap.h>
#include <quad_gap/utils/Trajectory.h>
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

#include <quad_gap/utils/RobotGeometryStorage.h>
#include <quad_gap/utils/RobotGeometryProcessor.h>

namespace quad_gap {
    
    class TrajectoryEvaluator
    {
        public:
            TrajectoryEvaluator(const QuadGapConfig& cfg, RobotGeometryProcessor& robot_geo_proc);
            // TrajectoryEvaluator(ros::NodeHandle& nh, const QuadGapConfig& cfg, RobotGeometryStorage& robot_geo_storage);
            
            TrajectoryEvaluator& operator=(TrajectoryEvaluator other) 
            {
                cfg_ = other.cfg_;
                robot_geo_proc_ = other.robot_geo_proc_;
            
                return *this;
            }
            
            TrajectoryEvaluator(const TrajectoryEvaluator &t) 
            {
                cfg_ = t.cfg_;
                robot_geo_proc_ = t.robot_geo_proc_;
            }
            
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg);
            void updateGapContainer(const std::vector<Gap *> & observed_gaps);

            void transformGlobalPathLocalWaypointToRbtFrame(const geometry_msgs::PoseStamped & globalPathLocalWaypointOdomFrame, 
                                                            const geometry_msgs::TransformStamped & odom2rbt);
            
            std::vector<float> scoreGaps();
            Gap * returnAndScoreGaps();
            
            // Full Scoring
            // std::vector<float> scoreTrajectories(const std::vector<geometry_msgs::PoseArray> & sample_traj);
            geometry_msgs::PoseStamped getLocalGoal() {return globalPathLocalWaypointRobotFrame_; }; // in robot frame
            
            // std::vector<float> & posewiseCosts,
            // float & terminalPoseCost            
            void scoreTrajectory(Trajectory & traj);
        
        private:
            
            float costFn(Gap * g, int goal_idx);

            float scorePose(const geometry_msgs::Pose & pose, const sensor_msgs::LaserScan & scan);
            // int searchIdx(geometry_msgs::Pose pose);
            float dist2Pose(const float & theta, const float & dist, const geometry_msgs::Pose & pose);
            float chapterScore(const float & d, const float & rmax_offset_val);
            float terminalGoalCost(const geometry_msgs::Pose & pose);

            const QuadGapConfig* cfg_;
            boost::shared_ptr<sensor_msgs::LaserScan const> scan_;
            std::vector<Gap *> gaps;
            geometry_msgs::PoseStamped globalPathLocalWaypointRobotFrame_;

            boost::mutex globalPlanMutex_; /**< mutex locking thread for updating current global plan */
            boost::mutex scanMutex_; /**< mutex locking thread for updating current scan */
            boost::mutex gap_mutex;

            RobotGeometryProcessor * robot_geo_proc_ = NULL; /**< Robot geometry processor */
    };
}