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
            GapManipulator(){};
            ~GapManipulator(){};

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

            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const>);

            void setGapWaypoint(Gap&, geometry_msgs::PoseStamped);
            void reduceGap(Gap&, geometry_msgs::PoseStamped);
            void convertAxialGap(Gap&);
            void radialExtendGap(Gap&);
        
            private:
                boost::shared_ptr<sensor_msgs::LaserScan const> msg;
                const QuadGapConfig* cfg_;
                int num_of_scan;
                boost::mutex egolock;

                Eigen::Vector2f car2pol(Eigen::Vector2f);
                Eigen::Vector2f pol2car(Eigen::Vector2f);
                Eigen::Vector2f pTheta(float, float, Eigen::Vector2f, Eigen::Vector2f);
                bool checkGoalVisibility(geometry_msgs::PoseStamped);

                RobotGeometryProcessor robot_geo_proc_;


    };
}