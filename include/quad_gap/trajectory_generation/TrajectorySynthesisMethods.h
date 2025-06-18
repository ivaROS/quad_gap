#pragma once

#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <vector>
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Twist.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace quad_gap 
{
    typedef boost::array<double, 2> state_type;

    struct polar_gap_field
    {

        double x_right, x_left, y_right, y_left, goal_x, goal_y;
        double _sigma;
        bool _radial;

        polar_gap_field(double x_right, double x_left, double y_right, double y_left, double goal_x, double goal_y, bool radial, double sigma)
            : x_right(x_right), x_left(x_left), y_right(y_right), y_left(y_left), goal_x(goal_x), goal_y(goal_y), _radial(radial), _sigma(sigma) {}

        void operator()(const state_type &x, state_type &dxdt, const double t)
        {
            if (atan2(y_right, x_right) > atan2(y_left, x_left)) 
            {
                std::swap(y_right, y_left);
                std::swap(x_right, x_left);
            }
            
            Eigen::Vector2d rbt(x[0], x[1]);
            Eigen::Vector2d p_right(x_right, y_right);
            Eigen::Vector2d p_left(x_left, y_left);

            Eigen::Vector2d vec_right = p_right - rbt;
            Eigen::Vector2d vec_left = p_left - rbt;

            Eigen::Matrix2d r_pi2;
            double rot_angle = M_PI / 2;
            r_pi2 << std::cos(rot_angle), -std::sin(rot_angle), std::sin(rot_angle), std::cos(rot_angle);
            Eigen::Matrix2d neg_r_pi2;
            neg_r_pi2 << std::cos(-rot_angle), -std::sin(-rot_angle), std::sin(-rot_angle), std::cos(-rot_angle);

            Eigen::Vector2d goal_pt(goal_x, goal_y);
            Eigen::Vector2d goal_vec = goal_pt - rbt;

            double r1 = sqrt(pow(x_right, 2) + pow(y_right, 2));
            double r2 = sqrt(pow(x_left, 2) + pow(y_left, 2));
            double rx = sqrt(pow(x[0], 2) + pow(x[1], 2));
            double rg = goal_vec.norm();
            double theta_right = atan2(y_right, x_right);
            double theta_left = atan2(y_left, x_left);
            double thetax = atan2(x[1], x[0]);
            double thetag = atan2(goal_vec(1), goal_vec(0));

            double new_theta = std::min(std::max(thetag, theta_right), theta_left);
            double theta_test = std::min(std::max(thetax, theta_right), theta_left);

            Eigen::Vector2d c1 = r_pi2     * (vec_right / vec_right.norm()) * exp(-std::abs(thetax - theta_right) / _sigma);
            Eigen::Vector2d c2 = neg_r_pi2 * (vec_left / vec_left.norm()) * exp(-std::abs(theta_left - thetax) / _sigma);

            // Since local goal will definitely be within the range of the gap, this limit poses no difference
            Eigen::Vector2d sub_goal_vec(rg * cos(new_theta), rg * sin(new_theta));

            bool left = r2 > r1;

            bool pass_gap;
            if (_radial) 
            {
                pass_gap = (rbt.norm() > std::min(p_right.norm(), p_left.norm()) + 0.18) && rbt.norm() > goal_pt.norm();
            } else 
            {
                pass_gap = (rbt.norm() > std::max(p_right.norm(), p_left.norm()) + 0.18) && rbt.norm() > goal_pt.norm();
            }


            Eigen::Vector2d v1 = p_right - p_left;
            Eigen::Vector2d v2 = p_right - rbt;

            Eigen::Vector2d polar_vec = rbt.norm() < 1e-3 || pass_gap ? Eigen::Vector2d(0, 0) : rbt / (rbt.norm());

            Eigen::Vector2d result(0, 0); 
            double coeffs = pass_gap ? 0.0 : 1.0;

            Eigen::Vector2d final_goal_vec(0,0);

            if (pass_gap)
            {
                result = Eigen::Vector2d(0, 0);
            } else {
                result = (c1 + c2) * coeffs;
                result += sub_goal_vec / sub_goal_vec.norm();
            }

            dxdt[0] = result(0);
            dxdt[1] = result(1);
            return;
        }
    };

    struct g2g 
    {
        double goal_x, goal_y;
        g2g(double goal_x, double goal_y)
        : goal_x(goal_x), goal_y(goal_y) {}

        void operator() ( const state_type &x , state_type &dxdt , const double  t)
        {
            double goal_norm = sqrt(pow(goal_x - x[0], 2) + pow(goal_y - x[1], 2));
            if (goal_norm < 0.1) {
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
        double _coefs;

        write_trajectory(geometry_msgs::PoseArray& posearr, std::string frame_id, double coefs)
        : _posearr(posearr), _frame_id(frame_id), _coefs(coefs) { }

        void operator()( const state_type &x , double t )
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = _frame_id;
            pose.pose.position.x = x[0] / _coefs;
            pose.pose.position.y = x[1] / _coefs;
            pose.pose.position.z = 0;

            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.pose.orientation.w = 1;
            _posearr.poses.push_back(pose.pose);
        }
    };

}