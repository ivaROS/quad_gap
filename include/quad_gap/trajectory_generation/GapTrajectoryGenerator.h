#pragma once

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

#include <boost/numeric/odeint.hpp>

// #include <traj_generator.h>
// #include <turtlebot_trajectory_generator/near_identity.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <quad_gap/trajectory_generation/TrajectorySynthesisMethods.h>
#include <math.h>
#include <quad_gap/utils/Trajectory.h>
#include <quad_gap/utils/Gap.h>
#include <quad_gap/config/QuadGapConfig.h>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
// #include "tf/transform_datatypes.h"
// #include <tf/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <boost/shared_ptr.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quad_gap/trajectory_generation/Bezier.h>

#include <quad_gap/utils/RobotGeometryProcessor.h>

namespace quad_gap 
{
    class GapTrajGenerator
    {
        public:
            GapTrajGenerator(const QuadGapConfig& cfg, RobotGeometryProcessor& robot_geo_proc)
            { 
                cfg_ = &cfg;
                robotGeoProc_ = &robot_geo_proc;
            }

            GapTrajGenerator& operator=(GapTrajGenerator & other)
            {
                cfg_ = other.cfg_;
                robotGeoProc_ = other.robotGeoProc_;

                return *this;
            };

            GapTrajGenerator(const GapTrajGenerator &t)
            { 
                cfg_ = t.cfg_;
                robotGeoProc_ = t.robotGeoProc_;
            }

            // void updateTF(const geometry_msgs::msg::TransformStamped & tf) {planning2odom = tf;};

            Trajectory generateTrajectory(Gap * gap, const geometry_msgs::msg::PoseStamped & curr_pose);

            bool findBezierControlPts(Gap * selectedGap, 
                                        Bezier::Bezier<2>&, 
                                        const geometry_msgs::msg::TwistStamped & rbtVelRbtFrame);

            Trajectory generateBezierTrajectory(Gap * selectedGap, 
                                                const geometry_msgs::msg::TwistStamped & rbtVelRbtFrame);
            
            // std::vector<geometry_msgs::msg::PoseArray> generateTrajectory(std::vector<Gap>);

            Trajectory processTrajectory(const Trajectory & traj);

            void getOrientDecayedPath(Trajectory & traj);

            geometry_msgs::msg::PoseArray transformPath(const geometry_msgs::msg::PoseArray & poseArrayIn, 
                                                    const geometry_msgs::msg::TransformStamped & trans);


        private: 
            bool findBezierControlPtsNew(const Eigen::Vector2f pLeftSafe,
                                            const Eigen::Vector2f pRightSafe,    
                                            const Eigen::Vector2f & pGoal,
                                            const float & scaledMinDim,
                                            const float & q1IdealNorm,
                                            const float & q1MaxNorm,
                                            // const Eigen::Vector2f & rbt_orient_vec,
                                            Eigen::Vector2f & q1,
                                            Eigen::Vector2f & q2);

            // void findFrontFacingBezierControlPts(const bool & left,
            //                                         const Eigen::Vector2f pCloseSafe,
            //                                         const Eigen::Vector2f pFarSafe,
            //                                         const float & robot_geo_thresh_dist,
            //                                         const float & robot_geo_diagonal_thresh,
            //                                         const float & ideal_min_cp_length,
            //                                         const float & cp_max_length,
            //                                         const Eigen::Vector2f & rbt_orient_vec,
            //                                         const Eigen::Vector2f & pGoal,
            //                                         Eigen::Vector2f & midControlPt,
            //                                         Eigen::Vector2f & new_goal,
            //                                         bool & success);

            // void findBackFacingBezierControlPts(const bool & left,
            //                                     const Eigen::Vector2f pCloseSafe,
            //                                     const Eigen::Vector2f pFarSafe,
            //                                     const float & robot_geo_thresh_dist,
            //                                     const float & robot_geo_diagonal_thresh,
            //                                     const float & ideal_min_cp_length,
            //                                     const float & cp_max_length,
            //                                     const Eigen::Vector2f & rbt_orient_vec,
            //                                     const Eigen::Vector2f & pGoal,
            //                                     Eigen::Vector2f & midControlPt,
            //                                     Eigen::Vector2f & new_goal,
            //                                     bool & success);

            // Eigen::Vector2f getRotatedVec(const Eigen::Vector2f & orig_vec, 
            //                                 const float & chord_length, 
            //                                 const bool & ccw = true);

            // bool isLeftofLine(const Eigen::Vector2f & l1, 
            //                     const Eigen::Vector2f & l2, 
            //                     const Eigen::Vector2f & p)
            // {
            //     return ((l2[0] - l1[0])*(p[1] - l1[1]) - (l2[1] - l1[1])*(p[0] - l1[0])) >= 0;
            // }

            // bool isLargerAngle(const Eigen::Vector2f & v1, const Eigen::Vector2f & v2)
            // {
            //     // v1 angle is larger than and equal to v2 angle ccw
            //     float ang_1 = atan2(v1[1], v1[0]);
            //     float ang_2 = atan2(v2[1], v2[0]);

            //     return ang_1 >= ang_2;
            // }

            float getBezierDist(Bezier::Bezier<2>& quadBezier, const float & tStart, const float & tEnd, const int & numPts)
            {
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "[getBezierDist()]");

                float bezier_arclength = 0;
                float t_diff = (tEnd - tStart) / (numPts - 1);
                float t_k = 0.0;
                float t_kplus1 = 0.0;
                for (int k = 0; k < numPts; k++)
                {
                    t_k = tStart + k * t_diff;
                    t_kplus1 = tStart + (k + 1) * t_diff;

                    // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "t_k: " << t_k << ", t_kplus1: " << t_kplus1);

                    float x = quadBezier.valueAt(t_k, 0);
                    float y = quadBezier.valueAt(t_k, 1);

                    float x_next = quadBezier.valueAt(t_kplus1, 0);
                    float y_next = quadBezier.valueAt(t_kplus1, 1);

                    float dist = sqrt(pow(x - x_next, 2) + pow(y - y_next, 2));

                    // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Segment " << k << " distance: " << dist);

                    bezier_arclength += dist;
                }

                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Total bezier arc length: " << bezier_arclength);
                return bezier_arclength;
            }

            const QuadGapConfig* cfg_ = NULL;
            RobotGeometryProcessor * robotGeoProc_ = NULL; 
            rclcpp::Logger logger_ {rclcpp::get_logger("GapTrajectoryGenerator")};
    };
}