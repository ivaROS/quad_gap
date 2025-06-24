#pragma once

#include <ros/ros.h>
#include <boost/numeric/odeint.hpp>

// #include <traj_generator.h>
// #include <turtlebot_trajectory_generator/near_identity.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <quad_gap/trajectory_generation/TrajectorySynthesisMethods.h>
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
#include <nav_msgs/Odometry.h>
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
                robot_geo_proc_ = robot_geo_proc;
            }

            GapTrajGenerator& operator=(GapTrajGenerator & other)
            {
                cfg_ = other.cfg_;
                robot_geo_proc_ = other.robot_geo_proc_;

                return *this;
            };

            GapTrajGenerator(const GapTrajGenerator &t)
            { 
                cfg_ = t.cfg_;
                robot_geo_proc_ = t.robot_geo_proc_;
            }

            void updateTF(const geometry_msgs::TransformStamped & tf) {planning2odom = tf;};

            geometry_msgs::PoseArray generateTrajectory(Gap * gap, const geometry_msgs::PoseStamped & curr_pose);

            bool findBezierControlPts(Gap * selectedGap, 
                                        Bezier::Bezier<2>&, 
                                        const geometry_msgs::TwistStamped & rbtVelRbtFrame, 
                                        const geometry_msgs::TransformStamped & odom2rbt);

            geometry_msgs::PoseArray generateBezierTrajectory(Gap * selectedGap, 
                                                                const geometry_msgs::TwistStamped & rbtVelRbtFrame, 
                                                                const geometry_msgs::TransformStamped & odom2rbt);
            
            // std::vector<geometry_msgs::PoseArray> generateTrajectory(std::vector<Gap>);

            geometry_msgs::PoseArray transformPath(const geometry_msgs::PoseArray & posearr, 
                                                                const geometry_msgs::TransformStamped & trans);

            geometry_msgs::PoseArray processTrajectory(const geometry_msgs::PoseArray & pose_arr);

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

            void findFrontFacingBezierControlPts(const bool & left,
                                                    const Eigen::Vector2f pCloseSafe,
                                                    const Eigen::Vector2f pFarSafe,
                                                    const float & robot_geo_thresh_dist,
                                                    const float & robot_geo_diagonal_thresh,
                                                    const float & ideal_min_cp_length,
                                                    const float & cp_max_length,
                                                    const Eigen::Vector2f & rbt_orient_vec,
                                                    const Eigen::Vector2f & pGoal,
                                                    Eigen::Vector2f & midControlPt,
                                                    Eigen::Vector2f & new_goal,
                                                    bool & success);

            void findBackFacingBezierControlPts(const bool & left,
                                                const Eigen::Vector2f pCloseSafe,
                                                const Eigen::Vector2f pFarSafe,
                                                const float & robot_geo_thresh_dist,
                                                const float & robot_geo_diagonal_thresh,
                                                const float & ideal_min_cp_length,
                                                const float & cp_max_length,
                                                const Eigen::Vector2f & rbt_orient_vec,
                                                const Eigen::Vector2f & pGoal,
                                                Eigen::Vector2f & midControlPt,
                                                Eigen::Vector2f & new_goal,
                                                bool & success);
                                                        

            Eigen::Vector2f getRotatedVec(const Eigen::Vector2f & orig_vec, 
                                            const float & chord_length, 
                                            const bool & ccw = true);

            bool isLeftofLine(const Eigen::Vector2f & l1, 
                                const Eigen::Vector2f & l2, 
                                const Eigen::Vector2f & p)
            {
                return ((l2[0] - l1[0])*(p[1] - l1[1]) - (l2[1] - l1[1])*(p[0] - l1[0])) >= 0;
            }

            bool isLargerAngle(const Eigen::Vector2f & v1, const Eigen::Vector2f & v2)
            {
                // v1 angle is larger than and equal to v2 angle ccw
                float ang_1 = atan2(v1[1], v1[0]);
                float ang_2 = atan2(v2[1], v2[0]);

                return ang_1 >= ang_2;
            }

            float getBezierDist(Bezier::Bezier<2>& quadBezier, const float & t_start, const float & t_end, const int & steps)
            {
                float approx_dist = 0;
                float t_diff = (t_end - t_start) / (steps - 1);
                for (size_t k = 0; k < steps - 1; k++)
                {
                    float x = quadBezier.valueAt(t_start + k * t_diff, 0);
                    float y = quadBezier.valueAt(t_start + k * t_diff, 1);
                    float x_next = quadBezier.valueAt(t_start + (k + 1) * t_diff, 0);
                    float y_next = quadBezier.valueAt(t_start + (k + 1) * t_diff, 1);

                    float dist = sqrt(pow(x - x_next, 2) + pow(y - y_next, 2));
                    approx_dist += dist;
                }
                return approx_dist;
            }

            geometry_msgs::TransformStamped planning2odom;

            const QuadGapConfig* cfg_;
            RobotGeometryProcessor robot_geo_proc_;
    };
}