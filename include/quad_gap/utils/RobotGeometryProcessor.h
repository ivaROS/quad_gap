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
                Eigen::Vector2f robot_f_vec = robot_.half_length * robotOrientationVector.normalized();
                Eigen::Vector2f robot_ccw_n_vec = robot_.half_width * o_normal.normalized();

                // 8 points ccw from -pi angle
                p1 = -robot_f_vec;
                p2 = -robot_f_vec - robot_ccw_n_vec;
                p3 = -robot_ccw_n_vec;
                p4 = robot_f_vec - robot_ccw_n_vec;
                p5 = robot_f_vec;
                p6 = robot_f_vec + robot_ccw_n_vec;
                p7 = robot_ccw_n_vec;
                p8 = -robot_f_vec + robot_ccw_n_vec;
                pt_list = {p1, p2, p3, p4, p5, p6, p7, p8};

                // distsForNearestDist_.resize(pt_list.size()); //  + sample_size
                distsForEquivalentRL_.resize(pt_list.size()); // + sample_size
            }

            bool initialized() { return initialized_; }

            float getEquivalentR(const Eigen::Vector2f& pt_direct)
            {
                if(robot_.shape == RobotShape::circle)
                    return robot_.radius;
                
                Eigen::Vector2f o_vec = robotOrientationVector;
                Eigen::Vector2f p_vec = pt_direct;

                if (o_vec.norm() != 1)
                    o_vec.normalize(); // = o_vec / o_vec.norm();
                
                if (p_vec.norm() != 1)
                    p_vec.normalize(); // = p_vec / p_vec.norm();

                float er;
                float vec_dot_product = o_vec.dot(p_vec);
                float length_ratio = robot_.length / robot_.diagonal_length;

                if (vec_dot_product > length_ratio && vec_dot_product <= 1)
                {
                    er = robot_.half_length / vec_dot_product;
                }
                else if (vec_dot_product <= length_ratio && vec_dot_product > -length_ratio)
                {
                    er = robot_.half_width / (sqrt(1 - vec_dot_product * vec_dot_product));
                }
                else
                {
                    er = -robot_.half_length / vec_dot_product;
                }
                return er;
            }

            float getEquivalentPL(const Eigen::Vector2f& projHeadingInc, const Eigen::Vector2f& gapDirInc)
            {
                if (robot_.shape == RobotShape::circle)
                    return 2 * robot_.radius;

                Eigen::Vector2f projHeading = projHeadingInc;
                Eigen::Vector2f gapDir = gapDirInc;

                if (projHeading.norm() != 1)
                    projHeading.normalize(); // = projHeading / projHeading.norm();

                if (gapDir.norm() != 1)
                    gapDir.normalize(); // = gapDir / gapDir.norm();

                // assert(projHeading.norm() == 1 && gapDir.norm() == 1);

                float alpha = atan2(gapDir[1], gapDir[0]);
                float theta = atan2(projHeading[1], projHeading[0]);

                // 
                float rot_ang = 0 - alpha;
                Eigen::Matrix2f rot;
                rot << cos(rot_ang), -sin(rot_ang), 
                        sin(rot_ang), cos(rot_ang);

                // Rotate the projHeading vector to align with the gap direction
                Eigen::Vector2f projHeadingGapFrame = rot * projHeading;
                float projThetaGapFrame = atan2(projHeadingGapFrame[1], projHeadingGapFrame[0]);

                Eigen::Vector2f halfLengthVec = robot_.half_length * projHeadingGapFrame;
                Eigen::Vector2f halfLengthVecRot;
                if ((projThetaGapFrame > 0 && projThetaGapFrame <= M_PI_OVER_TWO) || 
                    projThetaGapFrame <= -M_PI_OVER_TWO)
                {
                    // rotate half length vec 90 degrees ccw (left front edge of robot leads)
                    halfLengthVecRot[0] = -halfLengthVec[1];
                    halfLengthVecRot[1] = halfLengthVec[0];
                }
                else if ((projThetaGapFrame > -M_PI_OVER_TWO && projThetaGapFrame <= 0) || 
                            projThetaGapFrame > M_PI_OVER_TWO)
                {
                    // rotate half length vec 90 degrees cw (right front edge of robot leads)
                    halfLengthVecRot[0] = halfLengthVec[1];
                    halfLengthVecRot[1] = -halfLengthVec[0];
                }

                Eigen::Vector2f halfWidthVec = robot_.half_width * halfLengthVecRot.normalized();
                Eigen::Vector2f cornerPt = halfLengthVec + halfWidthVec;

                return 2 * abs(cornerPt[1]);
            }

            float getEquivalentRL(const Eigen::Vector2f& projHeading, const Eigen::Vector2f& pDes)
            {
                // Motion vec cannot be normalized

                // distsForEquivalentRL_.clear();

                if (robot_.shape == RobotShape::circle)
                    return 2 * robot_.radius;

                // Eigen::Vector2f o_vec = projHeading;
                Eigen::Vector2f eDes = pDes.normalized();

                // o_vec.normalize();
                // if(o_vec.norm() != 1)
                //     o_vec = o_vec / o_vec.norm();
                
                // m_vec.normalize();
                // if(m_vec.norm() != 1)
                //     m_vec = m_vec / m_vec.norm();

                // float thetaDes = atan2(eDes[1], eDes[0]);
                float projTheta = atan2(projHeading[1], projHeading[0]);
                // float o_new_ang = projTheta - thetaDes;
                // Eigen::Vector2f o_new_vec(cos(o_new_ang), sin(o_new_ang));
                float negProjTheta = -projTheta;
                Eigen::Matrix2f rot;
                rot << cos(negProjTheta), -sin(negProjTheta), 
                        sin(negProjTheta), cos(negProjTheta);

                // Rotate the desired vector to align with the projHeading
                Eigen::Vector2f pDesHeadingFrame = rot * pDes; // * pDes.norm() 
                // Eigen::Vector2f robotOrientationVector(1, 0);

                // std::vector<Eigen::Vector2f> pt_list{p1, p2, p3, p4, p5, p6, p7, p8};
                // std::vector<float> dists;

                // float ang = 0.0;
                // Eigen::Vector2f i_vec;
                // float dist = 0.0;
                // Eigen::Vector2f i_bound;                
                // for (size_t i = 0; i < sample_size; i++)
                // {
                //     ang = idx2theta(i); // i * res - M_PI;
                //     // ang = (ang <= M_PI) ? ang : M_PI;
                //     // ang = (ang >= -M_PI) ? ang : -M_PI;

                //     i_vec << cos(ang), sin(ang);
                //     dist = getEquivalentR(i_vec);

                //     i_bound = dist * i_vec;

                //     distsForEquivalentRL_.at(i) = (pDesHeadingFrame + i_bound).norm();
                // }

                for (size_t i = 0; i < pt_list.size(); i++)
                {
                    Eigen::Vector2f pt_vec_global = pDesHeadingFrame + pt_list.at(i);
                    distsForEquivalentRL_.at(i) = pt_vec_global.norm(); //  + sample_size
                }

                // for (const Eigen::Vector2f & pt : pt_list)
                // {
                //     Eigen::Vector2f pt_vec_global = pDesHeadingFrame + pt;
                //     dist.push_back(pt_vec_global.norm());
                // }


                float max_dist = *std::max_element(distsForEquivalentRL_.begin(), distsForEquivalentRL_.end());
                float min_dist = *std::min_element(distsForEquivalentRL_.begin(), distsForEquivalentRL_.end());

                if (std::abs(pDesHeadingFrame[0]) < (robot_.half_length) && 
                    std::abs(pDesHeadingFrame[1]) < (robot_.half_width))
                {
                    return max_dist;
                } else
                {
                    return abs(max_dist - min_dist);
                }
            }

            float getLinearDecayEquivalentPL(const Eigen::Vector2f & gapMidpt)
            {
                float dist = gapMidpt.norm();
                float alpha = atan2(gapMidpt[1], gapMidpt[0]);

                // time taken to reach the gap midpoint
                float t = dist / robot_.avg_lin_speed;

                // angle turned during that time
                // Odd to me, assumes robot is always rotating at a constant speed
                // What if gapMidPt is directly in front of the robot?
                float beta = t * robot_.avg_rot_speed;


                // float ang_diff = robotOrientationVector.dot(gapMidpt) / (robotOrientationVector.norm() * gapMidpt.norm());
                // float gapMidptFwdRatio = gapMidpt[0] / gapMidpt.norm(); // cos(gapMidptFwdRatio);

                
                // Trying to capture fact that if we over-rotate to align with gap midpoint,
                // that will give a negative angle which we would just zero to say we can align
                // Otherwise, if positive, means we can not rotate enough
                // float predTheta = alpha - beta;
                // predTheta = std::max(predTheta, 0.0f); // Ensure the angle is non-negative
                float headingError = 0.0f;
                float predTheta = 0.0f;
                if (alpha >= 0)
                {
                    headingError = std::max(alpha - beta, 0.0f); // non-negative
                    predTheta = alpha - headingError;
                } else
                {
                    headingError = std::max(alpha + beta, 0.0f); // non-negative
                    predTheta = alpha + headingError;
                }

                Eigen::Vector2f projHeading(cos(predTheta), sin(predTheta));
                Eigen::Vector2f gapDir(cos(alpha), sin(alpha));

                float epl = getEquivalentPL(projHeading, gapDir);
                return epl;
            }

            float getLinearDecayEquivalentRL(const Eigen::Vector2f& eDes, 
                                                const float & dist)
            {
                float alpha = atan2(eDes[1], eDes[0]);
                float t = dist / robot_.avg_lin_speed;
                float beta = t * robot_.avg_rot_speed;

                // float ang_diff = robotOrientationVector.dot(eDes) / (robotOrientationVector.norm() * eDes.norm());
                // float cur_ang = acos(ang_diff) - ang;
                // cur_ang = cur_ang >= 0 ? cur_ang : 0;
                // if(ang_diff < 0)
                //     cur_ang = -cur_ang;

                // float thetaDes = atan2(eDes[1], eDes[0]);
                // cur_ang = thetaDes + cur_ang;
                // Eigen::Vector2f cur_vec(cos(cur_ang), sin(cur_ang));
                float headingError = 0.0f;
                float predTheta = 0.0f;
                if (alpha >= 0)
                {
                    headingError = std::max(alpha - beta, 0.0f); // non-negative
                    predTheta = alpha - headingError;
                } else
                {
                    headingError = std::max(alpha + beta, 0.0f); // non-negative
                    predTheta = alpha + headingError;
                }

                Eigen::Vector2f projHeading(cos(predTheta), sin(predTheta));                

                Eigen::Vector2f pDes = dist * eDes / eDes.norm();
                float erl = getEquivalentRL(projHeading, pDes);
                return erl;
            }

            /*
            * @brief Get the nearest distance from the robot to a scan point. Accounts for the robot shape.
            * @param eTheta The orientation vector of the robot.
            * @param poseToScanOrigRbtFrame The relative vector from the robot origin to the scan point.
            * @return The nearest distance from the robot to the scan point.
            *         If the point is inside the robot bounding box, return -1.
            *         If the point is outside the robot bounding box, return the distance.
            *         If the robot is a circle, return the distance from the point to the robot radius.
            *         If the robot is a box, return the distance from the point to the robot bounding box.
            */
            float getNearestDistance(const Eigen::Vector2f & eTheta, const Eigen::Vector2f & poseToScanOrigRbtFrame)
            {
                // ROS_INFO_STREAM_NAMED("RobotGeometryProcessor", "[getNearestDistance()]");

                // ROS_INFO_STREAM_NAMED("RobotGeometryProcessor", "  Robot shape: " << robot_.shape);
                // ROS_INFO_STREAM_NAMED("RobotGeometryProcessor", "  eTheta: " << eTheta.transpose());
                // ROS_INFO_STREAM_NAMED("RobotGeometryProcessor", "  poseToScanOrigRbtFrame: " << poseToScanOrigRbtFrame.transpose());

                if (robot_.shape == RobotShape::circle)
                {
                    // ROS_INFO_STREAM_NAMED("RobotGeometryProcessor", "  Robot is circle.");
                    return poseToScanOrigRbtFrame.norm() - robot_.radius;
                }

                // poseToScanOrigRbtFrame: relative vector from robot origin to scan point (wrt original robot frame at t=0).

                // distsForNearestDist_.clear();

                float theta = atan2(eTheta[1], eTheta[0]);

                // ROS_INFO_STREAM_NAMED("RobotGeometryProcessor", "  theta: " << theta);
                float negTheta = -theta;

                // ROS_INFO_STREAM_NAMED("RobotGeometryProcessor", "  negTheta: " << negTheta);

                // Transform the poseToScanOrigRbtFrame vector to the robot's coordinate system.
                Eigen::Matrix2f rot;
                rot << cos(negTheta), -sin(negTheta), 
                        sin(negTheta), cos(negTheta);
                
                Eigen::Vector2f poseToScanCurrRbtFrame = rot * poseToScanOrigRbtFrame;

                // ROS_INFO_STREAM_NAMED("RobotGeometryProcessor", "  poseToScanCurrRbtFrame: " << poseToScanCurrRbtFrame.transpose());

                if (std::abs(poseToScanCurrRbtFrame[0]) < robot_.half_length && 
                    std::abs(poseToScanCurrRbtFrame[1]) < robot_.half_width)
                {
                    // ROS_INFO_STREAM_NAMED("RobotGeometryProcessor", "  Point is inside the robot bounding box.");
                    // ROS_WARN_STREAM_NAMED("RobotGeometryProcessor", "  Point is inside the robot bounding box.");
                    // The point is outside the robot bounding box.
                    return -1;
                }

                // float ang = 0.0;
                // Eigen::Vector2f i_vec;
                // float dist = 0.0;
                // Eigen::Vector2f i_bound;
                // for (size_t i = 0; i < sample_size; i++)
                // {
                //     ang = idx2theta(i);
                //     i_vec << cos(ang), sin(ang);

                //     dist = getEquivalentR(robotOrientationVector, i_vec);

                //     i_bound = dist * i_vec;

                //     distsForNearestDist_.at(i) = (poseToScanCurrRbtFrame - i_bound).norm();
                // }

                // 8 corner points
                // Eigen::Vector2f o_normal(-robotOrientationVector[1], robotOrientationVector[0]);
                // Eigen::Vector2f robot_f_vec = robot_.half_length * robotOrientationVector / robotOrientationVector.norm();
                // Eigen::Vector2f robot_ccw_n_vec = robot_.half_width * o_normal / o_normal.norm();
                
                // // 8 points ccw from -pi angle
                // Eigen::Vector2f p1 = -robot_f_vec;
                // Eigen::Vector2f p2 = -robot_f_vec + (-robot_ccw_n_vec);
                // Eigen::Vector2f p3 = -robot_ccw_n_vec;
                // Eigen::Vector2f p4 = robot_f_vec - robot_ccw_n_vec;
                // Eigen::Vector2f p5 = robot_f_vec;
                // Eigen::Vector2f p6 = robot_f_vec + robot_ccw_n_vec;
                // Eigen::Vector2f p7 = robot_ccw_n_vec;
                // Eigen::Vector2f p8 = -robot_f_vec + robot_ccw_n_vec;

                // Eigen::Vector2f pt_vec_global;
                // for (int i = 0; i < pt_list.size(); i++)
                // {
                //     pt_vec_global = poseToScanCurrRbtFrame - pt_list[i];
                //     distsForNearestDist_.at(i + sample_size) = pt_vec_global.norm();
                // }

                float thetaCurr = std::atan2(poseToScanCurrRbtFrame[1], poseToScanCurrRbtFrame[0]);

                if (thetaCurr > -M_PI_OVER_FOUR && thetaCurr <= M_PI_OVER_FOUR)
                {
                    return pointToLineSegmentDistance(p4, p6, poseToScanCurrRbtFrame);
                } else if (thetaCurr > M_PI_OVER_FOUR && thetaCurr <= 3 * M_PI_OVER_FOUR)
                {
                    return pointToLineSegmentDistance(p6, p8, poseToScanCurrRbtFrame);
                } else if (thetaCurr > -THREE_M_PI_OVER_FOUR && thetaCurr <= -M_PI_OVER_FOUR)
                {
                    return pointToLineSegmentDistance(p2, p4, poseToScanCurrRbtFrame);
                } else
                {
                    return pointToLineSegmentDistance(p8, p2, poseToScanCurrRbtFrame);
                }

                // float min_dist = *std::min_element(distsForNearestDist_.begin(), distsForNearestDist_.end());

                // return min_dist;
            }

            float pointToLineSegmentDistance(const Eigen::Vector2f & line_start, 
                                                const Eigen::Vector2f & line_end,
                                                const Eigen::Vector2f & point)
            {
                // Calculate the distance from the point to the line segment defined by line_start and line_end
                Eigen::Vector2f line_vec = line_end - line_start;
                Eigen::Vector2f point_vec = point - line_start;

                float line_length_squared = line_vec.squaredNorm();
                // if (line_length_squared == 0.0) // Line segment is a point
                // {
                //     return (point - line_start).norm();
                // }

                float t = std::max(0.0f, std::min(1.0f, point_vec.dot(line_vec) / line_length_squared));
                Eigen::Vector2f projection = line_start + t * line_vec;

                return (point - projection).norm();
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
                    return robot_.half_width;
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
                    return robot_.half_length;
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
                    return robot_.half_width;
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
            Eigen::Vector2f p1, p2, p3, p4, p5, p6, p7, p8;
            std::vector<Eigen::Vector2f> pt_list;

            // int sample_size = 0; // 20; // TOO SLOW
            // float res = M_PI * 2 / sample_size;

            // std::vector<float> distsForNearestDist_;
            std::vector<float> distsForEquivalentRL_;

    };
}