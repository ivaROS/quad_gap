#pragma once

// NON-ROS
#include <vector>
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

// QUADGAP
#include <quad_gap/gap.h>
#include <quad_gap/config/QuadGapConfig.h>
#include <quad_gap/robot_geo_parser.h>

namespace quad_gap 
{
    class GapDetector 
    {
        public: 
            GapDetector();

            ~GapDetector();

            GapDetector(const QuadGapConfig& cfg, RobotGeoProc& robot_geo_proc);

            GapDetector& operator=(GapDetector other) 
            {
                cfg_ = other.cfg_;
                robot_geo_proc_ = other.robot_geo_proc_;
            };

            GapDetector(const GapDetector &t) 
            {
                cfg_ = t.cfg_;
                robot_geo_proc_ = t.robot_geo_proc_;
            };

            void hybridScanGap(boost::shared_ptr<sensor_msgs::LaserScan const>, std::vector<quad_gap::Gap>&);

            void mergeGapsOneGo(boost::shared_ptr<sensor_msgs::LaserScan const>, std::vector<quad_gap::Gap>&);

            RobotGeoProc robot_geo_proc_;

        private:
            const QuadGapConfig* cfg_;
    };


}