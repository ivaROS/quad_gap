#pragma once

#include <ros/ros.h>
#include <math.h>
#include <quad_gap/utils/Gap.h>
#include <quad_gap/config/QuadGapConfig.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>

#include <quad_gap/utils/Utils.h>
#include <quad_gap/utils/RobotGeometryProcessor.h>

namespace quad_gap
{
    class GapGoalPlacer
    {
        public:
            GapGoalPlacer(const QuadGapConfig& cfg, RobotGeometryProcessor& robot_geo_proc) 
            {
                cfg_ = &cfg;
                robot_geo_proc_ = &robot_geo_proc;
            };

            GapGoalPlacer& operator=(GapGoalPlacer & other) 
            {
                cfg_ = other.cfg_;
                robot_geo_proc_ = other.robot_geo_proc_;
                return *this;
            };

            GapGoalPlacer(const GapGoalPlacer &t) 
            {
                cfg_ = t.cfg_;
                robot_geo_proc_ = t.robot_geo_proc_;
            };

            /**
            * \brief update current scan
            * \param scan incoming scan
            */
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan);

            /**
            * \brief Place goal in the gap
            * \param gap the gap to place goal in
            * \param localgoal the local goal to place in the gap
            */
            void setGapWaypoint(Gap * gap, const geometry_msgs::PoseStamped & localgoal);

        private:
            /**
            * \brief checking if global path local waypoint lies within gap
            * \param leftPt left gap point
            * \param rightPt right gap point
            * \param globalPathLocalWaypoint local waypoint along global path in robot frame
            * \return boolean for if global path local waypoint lies within gap
            */              
            bool checkWaypointVisibility(const Eigen::Vector2f & leftPt, 
                                            const Eigen::Vector2f & rightPt,
                                            const Eigen::Vector2f & globalPathLocalWaypoint);
                                            
            boost::shared_ptr<sensor_msgs::LaserScan const> scan_;
            const QuadGapConfig* cfg_;        
            int num_of_scan;
            boost::mutex egolock;

            Eigen::Vector2f car2pol(const Eigen::Vector2f & a);

            RobotGeometryProcessor * robot_geo_proc_ = NULL; /**< Robot geometry processor, used to get robot radius and inscribed radius */

    };
}