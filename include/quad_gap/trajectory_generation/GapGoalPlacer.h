#pragma once

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

#include <math.h>
#include <quad_gap/utils/Gap.h>
#include <quad_gap/config/QuadGapConfig.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

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
                robotGeoProc_ = &robot_geo_proc;
            };

            // GapGoalPlacer& operator=(GapGoalPlacer & other) 
            // {
            //     cfg_ = other.cfg_;
            //     robotGeoProc_ = other.robotGeoProc_;
            //     return *this;
            // };

            // GapGoalPlacer(const GapGoalPlacer &t) 
            // {
            //     cfg_ = t.cfg_;
            //     robotGeoProc_ = t.robotGeoProc_;
            // };

            /**
            * \brief update current scan
            * \param scan incoming scan
            */
            void updateEgoCircle(std::shared_ptr<sensor_msgs::msg::LaserScan const> scan);

            /**
            * \brief Place goal in the gap
            * \param gap the gap to place goal in
            * \param localgoal the local goal to place in the gap
            */
            void setGapWaypoint(Gap * gap, const geometry_msgs::msg::PoseStamped & globalPathLocalWaypoint);

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
                                      
            float setBiasedGapGoalTheta(const float & leftTheta, const float & rightTheta, const float & globalGoalTheta,
                                        const float & leftToRightAngle, const float & leftToWaypointAngle,  const float & rightToWaypointAngle);

            std::shared_ptr<sensor_msgs::msg::LaserScan const> scan_;
            const QuadGapConfig* cfg_;        
            // int num_of_scan;
            boost::mutex scanMutex_;

            Eigen::Vector2f car2pol(const Eigen::Vector2f & a);

            RobotGeometryProcessor * robotGeoProc_ = NULL; /**< Robot geometry processor, used to get robot radius and inscribed radius */
            rclcpp::Logger logger_ {rclcpp::get_logger("GapGoalPlacer")};

    };
}