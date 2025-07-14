#pragma once

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

#include <math.h>
#include <quad_gap/utils/Gap.h>
#include <quad_gap/config/QuadGapConfig.h>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <boost/shared_ptr.hpp>

#include <quad_gap/utils/Utils.h>
#include <quad_gap/utils/RobotGeometryProcessor.h>

namespace quad_gap 
{
    class GapManipulator 
    {
        public: 
            GapManipulator(const QuadGapConfig& cfg, RobotGeometryProcessor& robot_geo_proc) 
            {
                cfg_ = &cfg;
                robotGeoProc_ = &robot_geo_proc;
            };

            // GapManipulator& operator=(GapManipulator & other) 
            // {
            //     cfg_ = other.cfg_;
            //     robotGeoProc_ = other.robotGeoProc_;
            
            //     return *this;
            // };
            
            // GapManipulator(const GapManipulator &t) 
            // {
            //     cfg_ = t.cfg_;
            //     robotGeoProc_ = t.robotGeoProc_;
            // };

            /**
            * \brief update current scan
            * \param scan incoming scan
            */            
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::msg::LaserScan const> msg);

            /**
            * \brief function for reducing gap's angle to ensure that gap is convex (angle < 180 degrees)
            * \param gap queried gap
            * \param globalPathLocalWaypoint local waypoint along global path in robot frame
            */            
            void reduceGap(Gap * gap, const geometry_msgs::msg::PoseStamped & globalPathLocalWaypoint);

            /**
            * \brief function for convering radial gaps into swept gaps to allow maneuvering around corners
            * \param gap queried gap
            */               
            void convertRadialGap(Gap * gap);
            
            /**
            * \brief function for extending gap behind robot to ensure that robot starts its trajectory within gap
            * \param gap queried gap
            */            
            void radialExtendGap(Gap * gap);

            /**
            * \brief function for inflating gap radially and angularly to account for robot size
            * \param gap queried gap
            */            
            void inflateGapSides(Gap * gap);            
        
        private:
            boost::shared_ptr<sensor_msgs::msg::LaserScan const> scan_;
            const QuadGapConfig* cfg_;
            // int num_of_scan;
            boost::mutex egolock;

            // Eigen::Vector2f car2pol(const Eigen::Vector2f & a);
            // Eigen::Vector2f pol2car(const Eigen::Vector2f & a);
            // Eigen::Vector2f pTheta(const float & th, const float & phiB, 
            //                         const Eigen::Vector2f & pRp, const Eigen::Vector2f & pLp);

            RobotGeometryProcessor * robotGeoProc_ = NULL; /**< Robot geometry processor */


    };
}