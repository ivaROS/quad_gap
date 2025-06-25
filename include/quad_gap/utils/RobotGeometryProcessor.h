#pragma once

#include <string>
#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <quad_gap/utils/Robot.h>

namespace quad_gap
{
    class RobotGeometryProcessor 
    {
        public:
            RobotGeometryProcessor() {};
            ~RobotGeometryProcessor() 
            {
                initialized_ = false;
            }

            RobotGeometryProcessor(const Robot & robot, const float & decay_factor = 0)
            {
                robot_ = robot;
                if(robot_.shape == RobotShape::circle)
                    decay_factor_ = 0;
                else
                    decay_factor_ = decay_factor;
                
                initialized_ = true;

                float o_new_ang = atan2(robotOrientationVector[1], robotOrientationVector[0]);
                Eigen::Vector2f o_normal(-robotOrientationVector[1], robotOrientationVector[0]);
                Eigen::Vector2f robot_f_vec = robot_.length / 2 * robotOrientationVector / robotOrientationVector.norm();
                Eigen::Vector2f robot_ccw_n_vec = robot_.width / 2 * o_normal / o_normal.norm();
                
                // 8 points ccw from -pi angle
                Eigen::Vector2f p1 = -robot_f_vec;
                Eigen::Vector2f p2 = -robot_f_vec - robot_ccw_n_vec;
                Eigen::Vector2f p3 = -robot_ccw_n_vec;
                Eigen::Vector2f p4 = robot_f_vec - robot_ccw_n_vec;
                Eigen::Vector2f p5 = robot_f_vec;
                Eigen::Vector2f p6 = robot_f_vec + robot_ccw_n_vec;
                Eigen::Vector2f p7 = robot_ccw_n_vec;
                Eigen::Vector2f p8 = -robot_f_vec + robot_ccw_n_vec;
                pt_list = {p1, p2, p3, p4, p5, p6, p7, p8};

                distsForNearestDist_.resize(pt_list.size() + sample_size);
                distsForEquivalentRL_.resize(pt_list.size() + sample_size);
            }

            bool initialized() { return initialized_; }

            float getEquivalentR(Eigen::Vector2f& orientation_vec, Eigen::Vector2f& pt_direct)
            {
                if(robot_.shape == RobotShape::circle)
                    return robot_.radius;
                
                Eigen::Vector2f o_vec = orientation_vec;
                Eigen::Vector2f p_vec = pt_direct;

                if (o_vec.norm() != 1)
                    o_vec = o_vec / o_vec.norm();
                
                if (p_vec.norm() != 1)
                    p_vec = p_vec / p_vec.norm();

                float er;
                float vec_dot_product = o_vec.dot(p_vec);

                if (vec_dot_product > (robot_.length / robot_.diagonal_length) && vec_dot_product <= 1)
                {
                    er = robot_.length / 2 / vec_dot_product;
                }
                else if (vec_dot_product <= (robot_.length / robot_.diagonal_length) && vec_dot_product > -(robot_.length / robot_.diagonal_length))
                {
                    er = robot_.width / 2 / (sqrt(1 - vec_dot_product * vec_dot_product));
                }
                else
                {
                    er = -robot_.length / 2 / vec_dot_product;
                }
                return er;
            }

            float getEquivalentPL(Eigen::Vector2f& orientation_vec, Eigen::Vector2f& motion_vec)
            {
                if (robot_.shape == RobotShape::circle)
                    return 2 * robot_.radius;

                Eigen::Vector2f o_vec = orientation_vec;
                Eigen::Vector2f m_vec = motion_vec;

                if (o_vec.norm() != 1)
                    o_vec = o_vec / o_vec.norm();
                
                if (m_vec.norm() != 1)
                    m_vec = m_vec / m_vec.norm();

                float m_ang = atan2(m_vec[1], m_vec[0]);
                float o_ang = atan2(o_vec[1], o_vec[0]);
                // float o_new_ang = o_ang - m_ang;
                // Eigen::Vector2f o_new_vec(cos(o_new_ang), sin(o_new_ang));
                float rot_ang = 0 - m_ang;
                Eigen::Matrix2f rot;
                rot << cos(rot_ang), -sin(rot_ang), sin(rot_ang), cos(rot_ang);
                Eigen::Vector2f o_new_vec = rot * o_vec;
                float o_new_ang = atan2(o_new_vec[1], o_new_vec[0]);
                // Eigen::Vector2f m_new_vec(1, 0);
                
                Eigen::Vector2f length_vec = robot_.length / 2 * o_new_vec;
                Eigen::Vector2f left_pt_vec;
                if (o_new_ang <= -M_PI_OVER_TWO || (o_new_ang > 0 && o_new_ang <= M_PI_OVER_TWO)) // TODO: float check
                {
                    left_pt_vec[0] = -length_vec[1];
                    left_pt_vec[1] = length_vec[0];
                }
                else if ((o_new_ang > -M_PI_OVER_TWO && o_new_ang <= 0) || o_new_ang > M_PI_OVER_TWO)
                {
                    left_pt_vec[0] = length_vec[1];
                    left_pt_vec[1] = -length_vec[0];
                }
                
                left_pt_vec = left_pt_vec / left_pt_vec.norm();
                left_pt_vec = robot_.width / 2 * left_pt_vec;
                Eigen::Vector2f corner_pt = length_vec + left_pt_vec;

                return 2 * abs(corner_pt[1]);
            }

            // float getEquivalentRL(Eigen::Vector2f& orientation_vec, Eigen::Vector2f& motion_vec)
            // {
            //     // Motion vec cannot be normalized

            //     if(robot_.shape == RobotShape::circle)
            //         return 2 * robot_.radius;

            //     Eigen::Vector2f o_vec = orientation_vec;
            //     Eigen::Vector2f m_vec = motion_vec;
            //     if(o_vec.norm() != 1)
            //         o_vec = o_vec / o_vec.norm();
                
            //     if(m_vec.norm() != 1)
            //         m_vec = m_vec / m_vec.norm();

            //     float m_ang = atan2(m_vec[1], m_vec[0]);
            //     float o_ang = atan2(o_vec[1], o_vec[0]);
            //     // float o_new_ang = o_ang - m_ang;
            //     // Eigen::Vector2f o_new_vec(cos(o_new_ang), sin(o_new_ang));
            //     float rot_ang = 0 - m_ang;
            //     Eigen::Matrix2f rot;
            //     rot << cos(rot_ang), -sin(rot_ang), sin(rot_ang), cos(rot_ang);
            //     Eigen::Vector2f o_new_vec = rot * o_vec;
            //     float o_new_ang = atan2(o_new_vec[1], o_new_vec[0]);
            //     Eigen::Vector2f m_new_vec(1, 0);
                
            //     Eigen::Vector2f length_vec = robot_.length / 2 * o_new_vec;
            //     Eigen::Vector2f pt_vec;
            //     if(o_new_ang <= -M_PI_OVER_TWO || (o_new_ang > 0 && o_new_ang <= M_PI_OVER_TWO)) // TODO: float check
            //     {
            //         pt_vec[0] = length_vec[1];
            //         pt_vec[1] = -length_vec[0];
            //     }
            //     else if((o_new_ang > -M_PI_OVER_TWO && o_new_ang <= 0) || o_new_ang > M_PI_OVER_TWO)
            //     {
            //         pt_vec[0] = -length_vec[1];
            //         pt_vec[1] = length_vec[0];
            //     }
                
            //     pt_vec = pt_vec / pt_vec.norm();
            //     pt_vec = robot_.width / 2 * pt_vec;
            //     Eigen::Vector2f corner_pt = motion_vec + length_vec + pt_vec;
            //     Eigen::Vector2f opposite_corner_pt = motion_vec - length_vec - pt_vec;

            //     return abs(corner_pt.norm() - opposite_corner_pt.norm());
            // }

            float getEquivalentRL(Eigen::Vector2f& orientation_vec, Eigen::Vector2f& motion_vec)
            {
                // Motion vec cannot be normalized

                // distsForEquivalentRL_.clear();

                if (robot_.shape == RobotShape::circle)
                    return 2 * robot_.radius;

                Eigen::Vector2f o_vec = orientation_vec;
                Eigen::Vector2f m_vec = motion_vec;

                o_vec.normalize();
                // if(o_vec.norm() != 1)
                //     o_vec = o_vec / o_vec.norm();
                
                m_vec.normalize();
                // if(m_vec.norm() != 1)
                //     m_vec = m_vec / m_vec.norm();

                float m_ang = atan2(m_vec[1], m_vec[0]);
                float o_ang = atan2(o_vec[1], o_vec[0]);
                // float o_new_ang = o_ang - m_ang;
                // Eigen::Vector2f o_new_vec(cos(o_new_ang), sin(o_new_ang));
                float rot_ang = 0 - o_ang;
                Eigen::Matrix2f rot;
                rot << cos(rot_ang), -sin(rot_ang), 
                        sin(rot_ang), cos(rot_ang);

                Eigen::Vector2f m_new_vec = motion_vec.norm() * rot * m_vec;
                // Eigen::Vector2f robotOrientationVector(1, 0);

                // std::vector<Eigen::Vector2f> pt_list{p1, p2, p3, p4, p5, p6, p7, p8};
                // std::vector<float> dists;

                float ang = 0.0;
                Eigen::Vector2f i_vec;
                float dist = 0.0;
                Eigen::Vector2f i_bound;                
                for (size_t i = 0; i < sample_size; i++)
                {
                    ang = idx2theta(ang); // i * res - M_PI;
                    // ang = (ang <= M_PI) ? ang : M_PI;
                    // ang = (ang >= -M_PI) ? ang : -M_PI;

                    i_vec << cos(ang), sin(ang);
                    dist = getEquivalentR(robotOrientationVector, i_vec);

                    i_bound = dist * i_vec;

                    distsForEquivalentRL_.at(i) = (m_new_vec + i_bound).norm();
                }

                for (size_t i = 0; i < pt_list.size(); i++)
                {
                    Eigen::Vector2f pt_vec_global = m_new_vec + pt_list.at(i);
                    distsForEquivalentRL_.at(i + sample_size) = pt_vec_global.norm();
                }
                // for (const Eigen::Vector2f & pt : pt_list)
                // {
                //     Eigen::Vector2f pt_vec_global = m_new_vec + pt;
                //     dist.push_back(pt_vec_global.norm());
                // }


                float max_dist = *std::max_element(distsForEquivalentRL_.begin(), distsForEquivalentRL_.end());
                float min_dist = *std::min_element(distsForEquivalentRL_.begin(), distsForEquivalentRL_.end());

                if (std::abs(m_new_vec[0]) < robot_.length / 2 && 
                    std::abs(m_new_vec[1]) < robot_.width / 2)
                {
                    return max_dist;
                } else
                {
                    return abs(max_dist - min_dist);
                }
            }

            float getDecayEquivalentPL(Eigen::Vector2f& orientation_vec, Eigen::Vector2f& motion_vec, const float & dist)
            {
                float max_epl = getEquivalentPL(orientation_vec, motion_vec);
                float min_epl = robot_.width;
                return (max_epl - min_epl) * exp(-decay_factor_ * dist) + min_epl;
            }

            float getLinearDecayEquivalentPL(const Eigen::Vector2f & orientation_vec, 
                                                Eigen::Vector2f & motion_vec, 
                                                const float & dist)
            {
                float t = dist / robot_.avg_lin_speed;
                float ang = t * robot_.avg_rot_speed;
                float ang_diff = orientation_vec.dot(motion_vec) / (orientation_vec.norm() * motion_vec.norm());
                float cur_ang = acos(ang_diff) - ang;
                cur_ang = cur_ang >= 0 ? cur_ang : 0;
                if(ang_diff < 0)
                    cur_ang = -cur_ang;

                float m_ang = atan2(motion_vec[1], motion_vec[0]);
                cur_ang = m_ang + cur_ang;
                Eigen::Vector2f cur_vec(cos(cur_ang), sin(cur_ang));

                float epl = getEquivalentPL(cur_vec, motion_vec);
                return epl;
            }

            float getLinearDecayEquivalentRL(const Eigen::Vector2f & orientation_vec, 
                                                Eigen::Vector2f& motion_vec, 
                                                const float & dist)
            {
                float t = dist / robot_.avg_lin_speed;
                float ang = t * robot_.avg_rot_speed;
                float ang_diff = orientation_vec.dot(motion_vec) / (orientation_vec.norm() * motion_vec.norm());
                float cur_ang = acos(ang_diff) - ang;
                cur_ang = cur_ang >= 0 ? cur_ang : 0;
                if(ang_diff < 0)
                    cur_ang = -cur_ang;

                float m_ang = atan2(motion_vec[1], motion_vec[0]);
                cur_ang = m_ang + cur_ang;
                Eigen::Vector2f cur_vec(cos(cur_ang), sin(cur_ang));

                Eigen::Vector2f m_new_vec = dist * motion_vec / motion_vec.norm();
                float erl = getEquivalentRL(cur_vec, m_new_vec);
                return erl;
            }

            /*
            * @brief Get the nearest distance from the robot to a scan point. Accounts for the robot shape.
            * @param orientation_vec The orientation vector of the robot.
            * @param poseToScan The relative vector from the robot origin to the scan point.
            * @return The nearest distance from the robot to the scan point.
            *         If the point is inside the robot bounding box, return -1.
            *         If the point is outside the robot bounding box, return the distance.
            *         If the robot is a circle, return the distance from the point to the robot radius.
            *         If the robot is a box, return the distance from the point to the robot bounding box.
            */
            float getNearestDistance(const Eigen::Vector2f & orientation_vec, const Eigen::Vector2f & poseToScan)
            {
                // ROS_INFO_STREAM_NAMED("RobotGeometryProcessor", "[getNearestDistance()]");

                // ROS_INFO_STREAM_NAMED("RobotGeometryProcessor", "  Robot shape: " << robot_.shape);
                // ROS_INFO_STREAM_NAMED("RobotGeometryProcessor", "  orientation_vec: " << orientation_vec.transpose());
                // ROS_INFO_STREAM_NAMED("RobotGeometryProcessor", "  poseToScan: " << poseToScan.transpose());

                if (robot_.shape == RobotShape::circle)
                {
                    // ROS_INFO_STREAM_NAMED("RobotGeometryProcessor", "  Robot is circle.");
                    return poseToScan.norm() - robot_.radius;
                }

                // poseToScan: relative vector from robot origin to scan point.

                // distsForNearestDist_.clear();

                float o_ang = atan2(orientation_vec[1], orientation_vec[0]);

                // ROS_INFO_STREAM_NAMED("RobotGeometryProcessor", "  o_ang: " << o_ang);
                float rot_ang = 0.0 - o_ang;

                // ROS_INFO_STREAM_NAMED("RobotGeometryProcessor", "  rot_ang: " << rot_ang);

                // ????
                Eigen::Matrix2f rot;
                rot << cos(rot_ang), -sin(rot_ang), 
                        sin(rot_ang), cos(rot_ang);
                
                Eigen::Vector2f pt_new_vec = rot * poseToScan;

                // ROS_INFO_STREAM_NAMED("RobotGeometryProcessor", "  pt_new_vec: " << pt_new_vec.transpose());

                if (std::abs(pt_new_vec[0]) < robot_.length / 2 && 
                    std::abs(pt_new_vec[1]) < robot_.width / 2)
                {
                    // ROS_INFO_STREAM_NAMED("RobotGeometryProcessor", "  Point is inside the robot bounding box.");
                    // ROS_WARN_STREAM_NAMED("RobotGeometryProcessor", "  Point is inside the robot bounding box.");
                    // The point is outside the robot bounding box.
                    return -1;
                }

                // if  (pt_new_vec[0] > (-robot_.length / 2) && 
                //      pt_new_vec[0] < (robot_.length / 2) && 
                //      pt_new_vec[1] > (-robot_.width / 2) && 
                //      pt_new_vec[1] < (robot_.width / 2))
                // {
                //     return -1;
                // }
                
                // Eigen::Vector2f o_new_vec(1, 0);

                float ang = 0.0;
                Eigen::Vector2f i_vec;
                float dist = 0.0;
                Eigen::Vector2f i_bound;
                for (size_t i = 0; i < sample_size; i++)
                {
                    ang = idx2theta(ang); // i * res - M_PI;
                    // ang = (ang <= M_PI) ? ang : M_PI;
                    // ang = (ang >= -M_PI) ? ang : -M_PI;

                    i_vec << cos(ang), sin(ang);
                    dist = getEquivalentR(robotOrientationVector, i_vec);

                    i_bound = dist * i_vec;

                    distsForNearestDist_.at(i) = (pt_new_vec - i_bound).norm();
                }

                // 8 corner points
                // Eigen::Vector2f o_normal(-robotOrientationVector[1], robotOrientationVector[0]);
                // Eigen::Vector2f robot_f_vec = robot_.length / 2 * robotOrientationVector / robotOrientationVector.norm();
                // Eigen::Vector2f robot_ccw_n_vec = robot_.width / 2 * o_normal / o_normal.norm();
                
                // // 8 points ccw from -pi angle
                // Eigen::Vector2f p1 = -robot_f_vec;
                // Eigen::Vector2f p2 = -robot_f_vec + (-robot_ccw_n_vec);
                // Eigen::Vector2f p3 = -robot_ccw_n_vec;
                // Eigen::Vector2f p4 = robot_f_vec - robot_ccw_n_vec;
                // Eigen::Vector2f p5 = robot_f_vec;
                // Eigen::Vector2f p6 = robot_f_vec + robot_ccw_n_vec;
                // Eigen::Vector2f p7 = robot_ccw_n_vec;
                // Eigen::Vector2f p8 = -robot_f_vec + robot_ccw_n_vec;

                Eigen::Vector2f pt_vec_global;
                for (int i = 0; i < pt_list.size(); i++)
                {
                    pt_vec_global = pt_new_vec - pt_list[i];
                    distsForNearestDist_.at(i + sample_size) = pt_vec_global.norm();
                }
                // for (const Eigen::Vector2f & pt : pt_list)
                // {
                //     pt_vec_global = pt_new_vec - pt;
                //     dists.push_back(pt_vec_global.norm());
                // }

                return *std::min_element(distsForNearestDist_.begin(), distsForNearestDist_.end());
            }

            float getRobotMaxRadius()
            {
                if (robot_.shape == RobotShape::circle)
                {
                    return robot_.radius;
                }
                else if(robot_.shape == RobotShape::box)
                {
                    return robot_.diagonal_length / 2;
                } else
                {
                    throw std::runtime_error("[getRobotMaxRadius()]: robot shape not recognized!");
                }
            }

            float getRobotMinRadius()
            {
                if (robot_.shape == RobotShape::circle)
                {
                    return robot_.radius;
                } else if (robot_.shape == RobotShape::box)
                {
                    return robot_.width / 2;
                } else
                {
                    throw std::runtime_error("[getRobotMinRadius()]: robot shape not recognized!");
                }
            }

            float getRobotHalfLength()
            {
                if (robot_.shape == RobotShape::circle)
                {
                    return robot_.radius;
                } else if (robot_.shape == RobotShape::box)
                {
                    return robot_.length / 2;
                } else
                {
                    throw std::runtime_error("[getRobotHalfLength()]: robot shape not recognized!");
                }
            }

            float getRobotHalfWidth()
            {
                if (robot_.shape == RobotShape::circle)
                {
                    return robot_.radius;
                } else if (robot_.shape == RobotShape::box)
                {
                    return robot_.width / 2;
                } else
                {
                    throw std::runtime_error("[getRobotHalfWidth()]: robot shape not recognized!");
                }
            }

            float getRobotAvgLinSpeed()
            {
                return robot_.avg_lin_speed;
            }

        public:
            Robot robot_;
            float decay_factor_ = 0;
            bool initialized_ = false;
            // Eigen::Vector2f p1, p2, p3, p4, p5, p6, p7, p8;
            std::vector<Eigen::Vector2f> pt_list;

            int sample_size = 5; // 20; // TOO SLOW
            // float res = M_PI * 2 / sample_size;

            std::vector<float> distsForNearestDist_;
            std::vector<float> distsForEquivalentRL_;

    };
}