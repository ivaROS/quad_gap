#pragma once

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <vector>
#include "geometry_msgs/msg/pose_array.h"
#include <geometry_msgs/msg/pose_stamped.h>
#include "geometry_msgs/msg/twist.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <quad_gap/utils/Utils.h>

namespace quad_gap 
{
    typedef boost::array<float, 2> state_type;

    struct polar_gap_field
    {

        float xLeft_; /**< x-position of left gap point */
        float xRight_; /**< x-position of right gap point */
        float yLeft_; /**< y-position of left gap point */
        float yRight_; /**< y-position of right gap point */  
        float xGoal_; /**< x-position of gap goal point */
        float yGoal_; /**< y-position of gap goal point */
        float sigma_; /**< standard deviation-ish parameter used in expontential term of potential field */
        bool radial_; /**< boolean for if gap is radial*/

        float vRbtLinMax_ = 1.0; /**< maximum linear velocity of robot */

        Eigen::Matrix2f Rpi2_; /**< rotation matrix for pi/2 */
        Eigen::Matrix2f Rnegpi2_; /**< rotation matrix for -pi/2 */

        polar_gap_field(const float & xLeft, const float & xRight, 
                        const float & yLeft, const float & yRight,  
                        const float & xGoal, const float & yGoal, 
                        const bool & radial, const float & sigma)
            : xLeft_(xLeft), xRight_(xRight), yLeft_(yLeft), yRight_(yRight), 
                xGoal_(xGoal), yGoal_(yGoal), radial_(radial), sigma_(sigma) 
        {
            float rotAngle = M_PI_OVER_TWO;
            Rpi2_ << std::cos(rotAngle), -std::sin(rotAngle), 
                     std::sin(rotAngle), std::cos(rotAngle);
            Rnegpi2_ << std::cos(-rotAngle), -std::sin(-rotAngle), 
                         std::sin(-rotAngle), std::cos(-rotAngle);            
        }

        /**
        * \brief Helper function for clipping velocities to maximum allowed velocities
        * \param rbtVel current robot velocity
        */
        void clipVelocities(Eigen::Vector2f & rbtVel) 
        {
            // std::cout << "in clipVelocities with " << vX << ", " << vY << std::endl;
            // Eigen::Vector2f origVel(vX, vY);
            float speedX = std::abs(rbtVel[0]);
            float speedY = std::abs(rbtVel[1]);
            if (speedX <= vRbtLinMax_ && speedY <= vRbtLinMax_) 
            {
                // std::cout << "not clipping" << std::endl;
                return;
            } else 
            {
                // std::cout << "max: " << vx_absmax << ", norm: " << origVel.norm() << std::endl;
                // Eigen::Vector2f clipVel = vRbtLinMax_ * origVel / std::max(speedX, speedY);
                rbtVel = epsilonDivide(vRbtLinMax_ * rbtVel,  std::max(speedX, speedY));
                // return clipVel;
            }
        }

        void operator()(const state_type &x, state_type &dxdt, const float t)
        {
            // if (atan2(y_right, x_right) > atan2(y_left, x_left)) 
            // {
            //     std::swap(y_right, y_left);
            //     std::swap(x_right, x_left);
            // }
            
            Eigen::Vector2f pLeft(xLeft_, yLeft_);
            Eigen::Vector2f pRight(xRight_, yRight_);

            if (getSweptLeftToRightAngle(pLeft, pRight) < M_PI)
            {
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "don't need to switch");
            } else
            {
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "now need to switch");
                std::swap(yRight_, yLeft_);
                std::swap(xRight_, xLeft_);
                pRight << xRight_, yRight_;
                pLeft << xLeft_, yLeft_;
            }

            /*
            if (atan2(yRight_, xRight_) > atan2(yLeft_, xLeft_)) 
            {
                std::swap(yRight_, yLeft_);
                std::swap(xRight_, xLeft_);
            }
            */

            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   t: " << t);
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   rbt_0: (" << xRbtInit_ << ", " << yRbtInit_ << ")");

            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   rbt: (" << x[0] << ", " << x[1] << ")");

            Eigen::Vector2f rbtPosn(x[0], x[1]);

            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   pRight: " << pRight[0] << ", " << pRight[1]);
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   pLeft: " << pLeft[0] << ", " << pLeft[1]);
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   p_goal: " << xGoal_ << ", " << yGoal_);

            Eigen::Vector2f rbtToLeft = pLeft - rbtPosn;
            Eigen::Vector2f rbtToRight = pRight - rbtPosn;

            Eigen::Vector2f gapGoal(xGoal_, yGoal_);
            Eigen::Vector2f rbtToGoal = gapGoal - rbtPosn;

            float rbtToGoalDistance = rbtToGoal.norm();
            float thetaLeft = atan2(yLeft_, xLeft_);
            float thetaRight = atan2(yRight_, xRight_);
            float thetaRbt = atan2(x[1], x[0]);
            float thetaRbtToGoal = atan2(rbtToGoal(1), rbtToGoal(0));

            float newThetaRbtToGoal = std::min(std::max(thetaRbtToGoal, thetaRight), thetaLeft); // this can break for behind gaps

            float rbtToLeftAngle = std::abs(thetaLeft - thetaRbt);
            float rbtToRightAngle = std::abs(thetaRbt - thetaRight);
            // 0.05: jittery

            Eigen::Vector2f leftTerm = Rnegpi2_ * rbtToLeft.normalized() * exp(- rbtToLeftAngle / sigma_); // on robot's left
            Eigen::Vector2f rightTerm = Rpi2_ * rbtToRight.normalized() * exp(- rbtToRightAngle / sigma_); // on robot's right

            Eigen::Vector2f newRbtToGoal(rbtToGoalDistance * cos(newThetaRbtToGoal), rbtToGoalDistance * sin(newThetaRbtToGoal));

            Eigen::Vector2f circulationTerm = leftTerm + rightTerm;
            Eigen::Vector2f attractionField = newRbtToGoal.normalized();

            Eigen::Vector2f rbtVel = circulationTerm + attractionField;

            clipVelocities(rbtVel);

            dxdt[0] = rbtVel(0);
            dxdt[1] = rbtVel(1);
            return;
        }
    };

    struct g2g 
    {
        float goal_x, goal_y;
        g2g(float goal_x, float goal_y)
        : goal_x(goal_x), goal_y(goal_y) {}

        void operator() ( const state_type &x , state_type &dxdt , const float  t)
        {
            float goal_norm = sqrt(pow(goal_x - x[0], 2) + pow(goal_y - x[1], 2));
            if (goal_norm < 0.1) 
            {
                dxdt[0] = 0;
                dxdt[1] = 0;
            } else {
                dxdt[0] = (goal_x - x[0]);
                dxdt[1] = (goal_y - x[1]);
            }
        }
    };

    struct write_trajectory
    {
        geometry_msgs::PoseArray& _posearr;
        std::string _frame_id;
        float _coefs;

        write_trajectory(geometry_msgs::PoseArray& posearr, std::string frame_id)
        : _posearr(posearr), _frame_id(frame_id) { }

        void operator() (const state_type &x , float t)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = _frame_id;
            pose.pose.position.x = x[0];
            pose.pose.position.y = x[1];
            pose.pose.position.z = 0;

            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.pose.orientation.w = 1;
            _posearr.poses.push_back(pose.pose);
        }
    };

}