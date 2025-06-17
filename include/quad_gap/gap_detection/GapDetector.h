#pragma once

// NON-ROS
#include <vector>
#include <boost/shared_ptr.hpp>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

// QUADGAP
#include <quad_gap/utils/Gap.h>
#include <quad_gap/config/QuadGapConfig.h>

#include <quad_gap/utils/RobotGeometryProcessor.h>

namespace quad_gap 
{
    class GapDetector 
    {
        public: 
            GapDetector(const QuadGapConfig& cfg, RobotGeometryProcessor& robot_geo_proc);

            GapDetector& operator=(GapDetector other) 
            {
                cfg_ = other.cfg_;
                robot_geo_proc_ = other.robot_geo_proc_;

                return *this;
            };

            GapDetector(const GapDetector &t) 
            {
                cfg_ = t.cfg_;
                robot_geo_proc_ = t.robot_geo_proc_;
            };

            /**
            * \brief Preprocess incoming laser scan to remove NaN/Inf values
            *
            * \param scan pointer to incoming laser scan
            */
            void preprocessScan(boost::shared_ptr<sensor_msgs::LaserScan> scan);

            std::vector<Gap> gapDetection(boost::shared_ptr<sensor_msgs::LaserScan const> scanPtr);

            std::vector<Gap> gapSimplification(const std::vector<Gap> & rawGaps);

        private:

            sensor_msgs::LaserScan scan_; /**< Current laser scan */
            const QuadGapConfig * cfg_ = NULL; /**< Planner hyperparameter config list */
            RobotGeometryProcessor robot_geo_proc_;

        };
}