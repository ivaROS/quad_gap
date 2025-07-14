#pragma once

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

#include <quad_gap/config/QuadGapConfig.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
// #include <tf/tf.h>
#include <quad_gap/utils/Gap.h>
// #include "quad_gap/TrajPlan.h"
#include <quad_gap/trajectory_generation/GapTrajectoryGenerator.h>
#include <visualization_msgs/msg/marker.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2/LinearMath/Quaternion.h>

namespace quad_gap 
{
    class TrajectoryController 
    {
        public:

            TrajectoryController(const rclcpp::Node::SharedPtr & node, const QuadGapConfig& cfg);

            /**
            * \brief receive new laser scan and update member variable accordingly
            * \param scan new laser scan
            */
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::msg::LaserScan const> scan);

            // geometry_msgs::msg::Twist controlLaw(const geometry_msgs::msg::Pose & current, 
            //                                 const geometry_msgs::msg::Pose & desired,
            //                                 const sensor_msgs::msg::LaserScan & inflated_egocircle, 
            //                                 const geometry_msgs::msg::PoseStamped & init_pose);
            

            /**
            * \brief Control law for trajectory tracking
            * \param current current robot pose
            * \param desired desired robot pose
            * \return command velocity for robot
            */
            geometry_msgs::msg::Twist controlLawHolonomic(const geometry_msgs::msg::Pose & currentPoseOdomFrame, 
                                                        const geometry_msgs::msg::Pose & desiredPoseOdomFrame);

            /**
            * \brief Control law for trajectory tracking
            * \param current current robot pose
            * \param desired desired robot pose
            * \return command velocity for robot
            */
            geometry_msgs::msg::Twist controlLawNonholonomic(const geometry_msgs::msg::Pose & current, 
                                                        const geometry_msgs::msg::Pose & desired);                                                            

            /**
            * \brief Apply post-processing steps to command velocity including robot kinematic limits
            * along with last-resort safety modules such as projection operator or CBF
            * \param rawCmdVel raw command velocity
            * \param rbtPoseInSensorFrame robot pose in sensor frame
            * \param currRbtVel current robot velocity
            * \param currRbtAcc current robot acceleration
            * \return processed command velocity
            */
            geometry_msgs::msg::Twist processCmdVelHolonomic(const geometry_msgs::msg::Twist & rawCmdVel,
                                                        const geometry_msgs::msg::PoseStamped & rbtPoseInSensorFrame);
                                                         
            /**
            * \brief Apply post-processing steps to command velocity including robot kinematic limits
            * along with last-resort safety modules such as projection operator or CBF
            * \param rawCmdVel raw command velocity
            * \param rbtPoseInSensorFrame robot pose in sensor frame
            * \param currRbtVel current robot velocity
            * \param currRbtAcc current robot acceleration
            * \return processed command velocity
            */
            geometry_msgs::msg::Twist processCmdVelNonholonomic(const geometry_msgs::msg::Pose & currentPoseOdomFrame,
                                                            const geometry_msgs::msg::Pose & desiredPoseOdomFrame,
                                                            const geometry_msgs::msg::Twist & rawCmdVel,
                                                            const geometry_msgs::msg::PoseStamped & rbtPoseInSensorFrame);

            /**
            * \brief Control law for pure obstacle avoidance
            * \return command velocity for robot
            */
            geometry_msgs::msg::Twist obstacleAvoidanceControlLaw();

            /**
            * \brief Control law for pure obstacle avoidance
            * \return command velocity for robot
            */
            geometry_msgs::msg::Twist obstacleAvoidanceControlLawNonHolonomic();

            /**
            * \brief Extract pose within target trajectory that we should track
            * \param currPose current robot pose
            * \param localTrajectory selected local trajectory to track
            * \return index along local trajectory for which pose the robot should drive towards
            */
            int extractTargetPoseIdx(const geometry_msgs::msg::Pose & currPose, 
                                        const geometry_msgs::msg::PoseArray & localTrajectory);

        private:
            Eigen::Matrix2cf getComplexMatrix(const float & x, const float & y, const float & quat_w, const float & quat_z);
            Eigen::Matrix2cf getComplexMatrix(const float & x, const float & y, const float & theta);
            // float dist2Pose(const float & theta, const float & dist, const geometry_msgs::msg::Pose & pose);

            /**
            * \brief Helper function for clipping velocities to maximum allowed velocities
            * \param velLinXFeedback feedback linear command velocity in x direction
            * \param velLinYFeedback feedback linear command velocity in y direction
            * \param velAngFeedback feedback angular command velocity
            */
            void clipRobotVelocity(float & velLinXFeedback, 
                                   float & velLinYFeedback, 
                                   float & velAngFeedback);

            /**
            * \brief Function for running projection operator module on command velocity
            * \param rbtPoseInSensorFrame robot pose in sensor frame
            * \param cmdVelFeedback feedback command velocity
            * \param Psi projection operator function value
            * \param dPsiDx projection operator gradient values
            * \param velLinXSafe safe command velocity in x direction
            * \param velLinYSafe safe command velocity in y direction
            * \param minDistTheta orientation of minimum distance scan point
            * \param minDist range of minimum distance scan point
            */
            void runProjectionOperator(const geometry_msgs::msg::PoseStamped & rbtPoseInSensorFrame,
                                        Eigen::Vector2f & cmdVelFeedback,
                                        // float & Psi, 
                                        // Eigen::Vector2f & dPsiDx,
                                        float & velLinXSafe, 
                                        float & velLinYSafe,
                                        float & minDistTheta, 
                                        float & minDist);

            /**
            * \brief Function for calculating projection operator
            * \param closestScanPtToRobot minimum distance scan point
            * \return projection operator function and gradient values (Psi and dPsiDx)
            */
            void calculateProjectionOperator(const Eigen::Vector2f & closestScanPtToRobot,
                                                float & Psi, Eigen::Vector2f & dPsiDx);

            /**
            * \brief Function for visualizing projection operator output in RViz
            * \param weightedVelLinXSafe weighted safe command velocity in x direction
            * \param weightedVelLinYSafe weighted safe command velocity in y direction
            * \param minRangeTheta theta at which minimum range occurs
            * \param minRange minimum range in scan
            */
            void visualizeProjectionOperator(const float & weightedVelLinXSafe, 
                                                const float & weightedVelLinYSafe,
                                                const float & minRangeTheta, 
                                                const float & minRange);

            // std::vector<geometry_msgs::Point> findLocalLine(const int & idx);
            // float polDist(const float & l1, const float & t1, const float & l2, const float & t2);

            // bool geqThres(const float dist);

            // Eigen::Vector2f car2pol(const Eigen::Vector2f & a);
            // Eigen::Vector2f pol2car(const Eigen::Vector2f & a);
            // Eigen::Vector3f projection_method(const float & min_diff_x, const float & min_diff_y);

            float l_; /**< Lookahead distance for nonholonomic control */

            // float thres;
            const QuadGapConfig* cfg_;
            boost::shared_ptr<sensor_msgs::msg::LaserScan const> scan_;
            boost::mutex scanMutex_;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr projOpPublisher_;
            // rclcpp::Time last_time;
    };
}