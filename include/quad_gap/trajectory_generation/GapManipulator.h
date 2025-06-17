#pragma once

#include <ros/ros.h>
#include <math.h>
#include <quad_gap/utils/Gap.h>
#include <quad_gap/config/QuadGapConfig.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

#include <quad_gap/utils/Utils.h>
#include <quad_gap/utils/RobotGeometryProcessor.h>

namespace quad_gap 
{
    class GapManipulator 
    {
        public: 
            GapManipulator(ros::NodeHandle& nh, const QuadGapConfig& cfg, RobotGeometryProcessor& robot_geo_proc) 
            {
                cfg_ = &cfg;
                robot_geo_proc_ = robot_geo_proc;
            };

            GapManipulator& operator=(GapManipulator & other) 
            {
                cfg_ = other.cfg_;
                robot_geo_proc_ = other.robot_geo_proc_;
            
                return *this;
            };
            
            GapManipulator(const GapManipulator &t) 
            {
                cfg_ = t.cfg_;
                robot_geo_proc_ = t.robot_geo_proc_;
            };

            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg);

            void setGapWaypoint(Gap & gap, const geometry_msgs::PoseStamped & localgoal);
            void reduceGap(Gap & gap, const geometry_msgs::PoseStamped & localgoal);
            void convertAxialGap(Gap & gap);
            void radialExtendGap(Gap & gap);
        
        private:
            boost::shared_ptr<sensor_msgs::LaserScan const> scan_;
            const QuadGapConfig* cfg_;
            int num_of_scan;
            boost::mutex egolock;

            Eigen::Vector2f car2pol(const Eigen::Vector2f & a);
            Eigen::Vector2f pol2car(const Eigen::Vector2f & a);
            Eigen::Vector2f pTheta(const float & th, const float & phiB, 
                                    const Eigen::Vector2f & pRp, const Eigen::Vector2f & pLp);
            bool checkGoalVisibility(const geometry_msgs::PoseStamped & localgoal);

            RobotGeometryProcessor robot_geo_proc_;


    };
}