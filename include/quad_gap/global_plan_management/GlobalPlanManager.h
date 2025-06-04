#pragma once

#include <ros/ros.h>
#include <math.h>
#include <quad_gap/utils/Gap.h>
#include <quad_gap/config/QuadGapConfig.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "tf/transform_datatypes.h"
#include <tf/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

#include <quad_gap/utils/RobotGeometryProcessor.h>

namespace quad_gap
{
    class GlobalPlanManager
    {
        public: 
            GlobalPlanManager() {};
            ~GlobalPlanManager() {};

            GlobalPlanManager(ros::NodeHandle& nh, const quad_gap::QuadGapConfig& cfg, RobotGeometryProcessor& robot_geo_proc);
            GlobalPlanManager& operator=(GlobalPlanManager other) 
            {
                cfg_ = other.cfg_;
                robot_geo_proc_ = other.robot_geo_proc_;
            };
            GlobalPlanManager(const GlobalPlanManager &t) 
            {
                cfg_ = t.cfg_;
                robot_geo_proc_ = t.robot_geo_proc_;
            };

            // Map Frame
            bool setGoal(const std::vector<geometry_msgs::PoseStamped> &);
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const>);
            void updateLocalGoal(geometry_msgs::TransformStamped map2rbt);
            geometry_msgs::PoseStamped getCurrentLocalGoal(geometry_msgs::TransformStamped rbt2odom);
            geometry_msgs::PoseStamped rbtFrameLocalGoal() {return local_goal;};
            std::vector<geometry_msgs::PoseStamped> getRawGlobalPlan();
            std::vector<geometry_msgs::PoseStamped> getRelevantGlobalPlan(geometry_msgs::TransformStamped);


        private:
            const QuadGapConfig* cfg_;
            RobotGeometryProcessor robot_geo_proc_;
            boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser;
            std::vector<geometry_msgs::PoseStamped> global_plan;
            std::vector<geometry_msgs::PoseStamped> mod_plan;
            geometry_msgs::PoseStamped local_goal; // Robot Frame
            boost::mutex goal_select_mutex;
            boost::mutex lscan_mutex;
            boost::mutex gplan_mutex;


            double threshold = 3;

            // If distance to robot is within
            bool isNotWithin(const double dist);
            // Pose to robot, when all in rbt frames
            double dist2rbt(geometry_msgs::PoseStamped);
            int PoseIndexInSensorMsg(geometry_msgs::PoseStamped pose);
            double getPoseOrientation(geometry_msgs::PoseStamped);
            bool VisibleOrPossiblyObstructed(geometry_msgs::PoseStamped pose);
            bool NoTVisibleOrPossiblyObstructed(geometry_msgs::PoseStamped pose);

    };
}