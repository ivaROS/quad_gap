#include <quad_gap/trajectory_generation/GapTrajectoryGenerator.h>

namespace quad_gap
{
    geometry_msgs::PoseArray GapTrajGenerator::generateTrajectory(Gap * gap, 
                                                                    const geometry_msgs::PoseStamped & curr_pose) 
    {
        // return geometry_msgs::PoseArray();
        geometry_msgs::PoseArray posearr;
        posearr.header.stamp = ros::Time::now();
        
        write_trajectory corder(posearr, cfg_->robot_frame_id);
        posearr.header.frame_id = cfg_->robot_frame_id;

        // if (gap->goal.discard) 
        // {
        //     return posearr;
        // }

        state_type x = {curr_pose.pose.position.x + 1e-5, curr_pose.pose.position.y + 1e-6}; 

        if (gap->isGoalWithin()) // (gap->goal.goalwithin) 
        {
            // ROS_INFO_STREAM("Goal to Goal");
            g2g inte_g2g(gap->getGoalX(),
                         gap->getGoalY());
            boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
                                                    inte_g2g, 
                                                    x, 
                                                    0.0f,
                                                    cfg_->traj.integrate_maxt,
                                                    cfg_->traj.integrate_stept,
                                                    corder);
            return posearr;
        }

        // float xLeft, xRight, yLeft, yRight;
        // float theta_left = idx2theta(gap->convex.leftIdx_);
        // float theta_right = idx2theta(gap->convex.rightIdx_);
        // xRight = gap->convex.rightRange_ * cos(theta_right);
        // yRight = gap->convex.rightRange_ * sin(theta_right);
        // xLeft = gap->convex.leftRange_ * cos(theta_left);
        // yLeft = gap->convex.leftRange_ * sin(theta_left);
        Eigen::Vector2f pLeft = gap->getManipLCartesian(); // (xLeft, yLeft);
        Eigen::Vector2f pRight = gap->getManipRCartesian(); // (xRight, yRight);

        float xLeft, xRight, yLeft, yRight;
        xLeft = pLeft[0];     // (gap->convex.leftRange_) * cos(idx2theta(gap->convex.leftIdx_));
        yLeft = pLeft[1];         // (gap->convex.leftRange_) * sin(idx2theta(gap->convex.leftIdx_));
        xRight = pRight[0];            // (gap->convex.rightRange_) * cos(idx2theta(gap->convex.rightIdx_));
        yRight = pRight[1];            // (gap->convex.rightRange_) * sin(idx2theta(gap->convex.rightIdx_));
        
        float goal_x = gap->getGoalX();
        float goal_y = gap->getGoalY();

        Eigen::Vector2f qB = gap->getQB();
        if (gap->isExtended()) 
        {
            x = {-qB(0) - 1e-6, -qB(1) + 1e-6};
            xRight -= qB(0);
            xLeft -= qB(0);
            yRight -= qB(1);
            yLeft -= qB(1);
            goal_x -= qB(0);
            goal_y -= qB(1);
            // gap->goal.x -= gap->qB(0);
            // gap->goal.y -= gap->qB(1);

        }
        
        polar_gap_field inte(xRight, xLeft,
                            yRight, yLeft,
                            goal_x,
                            goal_y,
                            gap->isRadial(),
                            cfg_->gap_manip.sigma);
        boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
                                                inte, 
                                                x, 
                                                0.0f,
                                                cfg_->traj.integrate_maxt,
                                                cfg_->traj.integrate_stept, 
                                                corder);

        if (gap->isExtended()) 
        {
            for (geometry_msgs::Pose & p : posearr.poses) 
            {
                p.position.x += qB(0);
                p.position.y += qB(1);
            }
        }

        return posearr;
    }

    bool GapTrajGenerator::findBezierControlPts(Gap * gap, 
                                                Bezier::Bezier<2>& BezierCurve, 
                                                const geometry_msgs::TwistStamped & rbtVelRbtFrame, 
                                                const geometry_msgs::TransformStamped & odom2rbt)
    {
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "[findBezierControlPts()]");

        // Find the intersections of triangle and circle
        Eigen::Vector2f pLeft = gap->getManipLCartesian(); // (xLeft, yLeft);
        Eigen::Vector2f pRight = gap->getManipRCartesian(); // (xRight, yRight);

        float xLeft, xRight, yLeft, yRight;
        xLeft = pLeft[0];     // (gap->convex.leftRange_) * cos(idx2theta(gap->convex.leftIdx_));
        yLeft = pLeft[1];         // (gap->convex.leftRange_) * sin(idx2theta(gap->convex.leftIdx_));
        xRight = pRight[0];            // (gap->convex.rightRange_) * cos(idx2theta(gap->convex.rightIdx_));
        yRight = pRight[1];            // (gap->convex.rightRange_) * sin(idx2theta(gap->convex.rightIdx_));
        
        float goal_x = gap->getGoalX();
        float goal_y = gap->getGoalY();

        // ROS_INFO_STREAM(goal_x << " " << goal_y << " " << gap->goal.goalwithin);

        // Check if goal is in the middle
        float thetaLeft = std::atan2(yLeft, xLeft);
        float thetaRight = std::atan2(yRight, xRight);
        float thetaGoal = std::atan2(goal_y, goal_x);

        // assert(thetaGoal >= thetaRight && thetaGoal <= thetaLeft);

        // Eigen::Vector2f pLeft(xRight, yRight);
        // Eigen::Vector2f pRight(xLeft, yLeft);
        float minSafeDist = gap->getMinSafeDist();
        // assert(minSafeDist <= pLeft.norm() && minSafeDist <= pRight.norm());

        if (minSafeDist > pLeft.norm() || minSafeDist > pRight.norm())
        {
            ROS_WARN_STREAM("The circle radius is larger than the triangle side length.");
            return false;
        }

        float robot_geo_scale = cfg_->traj.robot_geo_scale;
        float robot_geo_thresh_dist = float(robot_geo_proc_.getRobotMinRadius()) * robot_geo_scale; // width / 2

        float cp_max_length = minSafeDist - robot_geo_thresh_dist;
        
        if (cp_max_length <= 0)
        {
            ROS_WARN_STREAM("The circle is smaller than robot thresh.");
            return false;
        }

        Eigen::Vector2f pLeftSafe = minSafeDist * pLeft / pLeft.norm();
        Eigen::Vector2f pRightSafe = minSafeDist * pRight / pRight.norm();
        Eigen::Vector2f pGoal(goal_x, goal_y);

        Eigen::Vector2f rbt_orient_vec(1, 0);
        float x_speed = rbtVelRbtFrame.twist.linear.x;
        float ideal_min_cp_length = x_speed / 2; // For quadratic bezier curve, the ideally length B'(0) = 2 (P_1 - P_0) 

        // Find the closest gap side to orientation. By default, l side is closer.
        bool l_side;
        Eigen::Vector2f pCloseSafe;
        Eigen::Vector2f pFarSafe;

        if (abs(thetaLeft) < abs(thetaRight))
        {
            l_side = true;
            pCloseSafe = pLeftSafe;
            pFarSafe = pRightSafe;
        } else
        {
            l_side = false;
            pCloseSafe = pRightSafe;
            pFarSafe = pLeftSafe;
        }

        Eigen::Vector2f midControlPt;
        Eigen::Vector2f new_goal;
        bool success = false;

        // If the goal is inside the circle, directly go to goal
        if (pGoal.norm() <= minSafeDist)
        {
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Goal is within circle.");

            if (ideal_min_cp_length > cp_max_length)
                midControlPt = cp_max_length * rbt_orient_vec;
            else
                midControlPt = ideal_min_cp_length * rbt_orient_vec;
            
            new_goal = pGoal;

            BezierCurve = Bezier::Bezier<2>({ {0, 0}, 
                                                {midControlPt[0], midControlPt[1]}, 
                                                {new_goal[0], new_goal[1]} });

            ROS_DEBUG_STREAM("Goal is within circle.");
            return true;
        }

        float robot_geo_diagonal_thresh = float(robot_geo_proc_.getRobotMaxRadius()) * robot_geo_scale;
        // float cp_max_length = minSafeDist - robot_geo_diagonal_thresh;

        // Conditions
        if (thetaLeft > 0 && thetaRight <= 0)
        {
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Front-facing gap");

            float chosen_ang = atan2(pCloseSafe[1], pCloseSafe[0]);

            if (abs(chosen_ang) <= M_PI / 2)
            {
                float dist_inter_orient = abs(pCloseSafe[1]);

                if (dist_inter_orient >= robot_geo_thresh_dist)
                {
                    if (ideal_min_cp_length > cp_max_length)
                        midControlPt = cp_max_length * rbt_orient_vec;
                    else
                        midControlPt = ideal_min_cp_length * rbt_orient_vec;

                    // Goal point region
                    success = true;

                    // Eigen::Vector2f cp_l_vec = pLeft - cp;
                    // Eigen::Vector2f cp_l_vec_rot = getRotatedVec(cp_l_vec, robot_geo_thresh_dist, true);
                    // Eigen::Vector2f cp_r_vec = pRight - cp;
                    // Eigen::Vector2f cp_r_vec_rot = getRotatedVec(cp_r_vec, robot_geo_thresh_dist, false);
                    // Eigen::Vector2f l_new_vec = cp + cp_l_vec_rot;
                    // Eigen::Vector2f r_new_vec = cp + cp_r_vec_rot;
                    

                    // ROS_INFO_STREAM("Within 1: " << cp[0] << " " << cp[1] << " " << pLeft[0] << " " << pLeft[1] << " " << l_new_vec[0] << " " << l_new_vec[1] << " " << pRight[0] << " " << pRight[1] << " " << r_new_vec[0] << " " << r_new_vec[1]);
                    Eigen::Vector2f l_used_vec = pCloseSafe;
                    Eigen::Vector2f r_used_vec = pFarSafe;
                    if (!l_side)
                    {
                        l_used_vec = pFarSafe;
                        r_used_vec = pCloseSafe;
                    }

                    Eigen::Vector2f l_normal_vec(-l_used_vec[1], l_used_vec[0]);
                    Eigen::Vector2f r_normal_vec(r_used_vec[1], -r_used_vec[0]);

                    Eigen::Vector2f l_new = l_used_vec + robot_geo_thresh_dist * l_normal_vec / l_normal_vec.norm();
                    Eigen::Vector2f r_new = r_used_vec + robot_geo_thresh_dist * r_normal_vec / r_normal_vec.norm();

                    ROS_DEBUG_STREAM("Within 1: " << midControlPt[0] << " " << midControlPt[1] << " " << l_used_vec[0] << " " << l_used_vec[1] << " " << l_new[0] << " " << l_new[1] << " " << r_used_vec[0] << " " << r_used_vec[1] << " " << r_new[0] << " " << r_new[1]);


                    // if(!isLeftofLine(cp, l_new_vec, r_new_vec))
                    // if(!isLargerAngle(cp_r_vec_rot, cp_l_vec_rot))
                    if (!isLargerAngle(r_new, l_new))
                    {
                        success = false;
                        ROS_WARN_STREAM("The union region does not exist. [Orientation is within gap 1]");
                    }
                    else
                    {

                        // bool left_side_of_l = isLeftofLine(cp, l_new_vec, pGoal);
                        // bool left_side_of_r = isLeftofLine(cp, r_new_vec, pGoal);
                        // bool larger_than_l = isLargerAngle(pGoal - cp, cp_l_vec_rot);
                        // bool larger_than_r = isLargerAngle(pGoal - cp, cp_r_vec_rot);
                        // ROS_INFO_STREAM(larger_than_l << " " << larger_than_r);
                        bool larger_than_l = isLargerAngle(pGoal, l_new);
                        bool larger_than_r = isLargerAngle(pGoal, r_new);

                        // if(!left_side_of_l)
                        if (!larger_than_l)
                        {
                            // Eigen::Vector2f goal_cp_vec = pGoal - cp;
                            // new_goal = goal_cp_vec.norm() * cp_l_vec_rot / cp_l_vec_rot.norm() + cp;
                            new_goal = pGoal.norm() * l_new / l_new.norm();
                        }
                        // else if(left_side_of_l && left_side_of_r)
                        else if (larger_than_l && larger_than_r)
                        {
                            // Eigen::Vector2f goal_cp_vec = pGoal - cp;
                            // new_goal = goal_cp_vec.norm() * cp_r_vec_rot / cp_r_vec_rot.norm() + cp;
                            new_goal = pGoal.norm() * r_new / r_new.norm();
                        }
                        // else if(left_side_of_l && !left_side_of_r)
                        else if (larger_than_l && !larger_than_r)
                        {
                            new_goal = pGoal;
                        }else
                        {
                            ROS_WARN_STREAM("Goal point is in the wrong region. [Orientation is within gap 1]");
                            success = false;
                        }
                    }
                }
                else
                {
                    float dist_pFarSafe_orient = abs(pFarSafe[1]);
                    if (dist_pFarSafe_orient >= robot_geo_thresh_dist)
                    {
                        // Find the intersect for max length / 2
                        float length_devi = sqrt(robot_geo_diagonal_thresh * robot_geo_diagonal_thresh - dist_pFarSafe_orient * dist_pFarSafe_orient);
                        Eigen::Vector2f max_cp(pCloseSafe[0] - length_devi, 0);
                        
                        bool cp_success = false;

                        if(max_cp[0] <= 0)
                        {
                            ROS_WARN_STREAM("The chosen intersection point is too close to robot. (smaller than diagonal / 2)");
                            success = false;
                        }
                        else
                        {
                            if(max_cp.norm() <= ideal_min_cp_length)
                                midControlPt = max_cp.norm() * rbt_orient_vec;
                            else
                                midControlPt = ideal_min_cp_length * rbt_orient_vec;

                            cp_success = true;
                        }

                        if(cp_success)
                        {
                            success = true;
                            Eigen::Vector2f origin(0, 0);
                            if(l_side)
                            {
                                // Eigen::Vector2f cp_r_vec = r_vec - cp;
                                // Eigen::Vector2f cp_r_vec_rot = getRotatedVec(cp_r_vec, robot_geo_thresh_dist, false);
                                // Eigen::Vector2f r_new_vec = cp + cp_r_vec_rot;

                                // ROS_INFO_STREAM("Within 2 l side: " << cp[0] << " " << cp[1] << " " << r_vec[0] << " " << r_vec[1] << " " << r_new_vec[0] << " " << r_new_vec[1]);

                                Eigen::Vector2f r_used_vec = pFarSafe;
                                Eigen::Vector2f r_normal_vec(r_used_vec[1], -r_used_vec[0]);

                                Eigen::Vector2f r_new = r_used_vec + robot_geo_thresh_dist * r_normal_vec / r_normal_vec.norm();

                                ROS_DEBUG_STREAM("Within 2 l side: " << midControlPt[0] << " " << midControlPt[1] << " " << r_used_vec[0] << " " << r_used_vec[1] << " " << r_new[0] << " " << r_new[1]);

                                // if(!isLeftofLine(origin, cp, r_new_vec))
                                // if(!isLargerAngle(cp_r_vec_rot, Eigen::Vector2f(1,0)))
                                if(!isLargerAngle(r_new, Eigen::Vector2f(1,0)))
                                {
                                    success = false;
                                    ROS_WARN_STREAM("The union region does not exist. [Orientation is within gap 2 l side]");
                                }
                                else
                                {
                                
                                    // bool left_side_of_l = isLeftofLine(origin, cp, pGoal);
                                    // bool left_side_of_r = isLeftofLine(cp, r_new_vec, pGoal);
                                    // bool larger_than_l = isLargerAngle(pGoal - cp, Eigen::Vector2f(1,0));
                                    // bool larger_than_r = isLargerAngle(pGoal - cp, cp_r_vec_rot);
                                    // ROS_INFO_STREAM(larger_than_l << " " << larger_than_r);

                                    bool larger_than_l = isLargerAngle(pGoal, Eigen::Vector2f(1,0));
                                    bool larger_than_r = isLargerAngle(pGoal, r_new);

                                    // if(!left_side_of_l)
                                    if(!larger_than_l)
                                    {
                                        // Eigen::Vector2f goal_cp_vec = pGoal - cp;
                                        // new_goal = goal_cp_vec.norm() * rbt_orient_vec + cp;
                                        new_goal = pGoal.norm() * rbt_orient_vec;
                                    }
                                    // else if(left_side_of_l && left_side_of_r)
                                    else if(larger_than_l && larger_than_r)
                                    {
                                        // Eigen::Vector2f goal_cp_vec = pGoal - cp;
                                        // new_goal = goal_cp_vec.norm() * cp_r_vec_rot / cp_r_vec_rot.norm() + cp;
                                        new_goal = pGoal.norm() * r_new / r_new.norm();
                                    }
                                    // else if(left_side_of_l && !left_side_of_r)
                                    else if(larger_than_l && !larger_than_r)
                                    {
                                        new_goal = pGoal;
                                    }
                                    else
                                    {
                                        ROS_WARN_STREAM("Goal point is in the wrong region. [Orientation is within gap 2 l side]");
                                        success = false;
                                    }
                                }
                            }
                            else
                            {
                                // Eigen::Vector2f cp_l_vec = l_vec - cp;
                                // Eigen::Vector2f cp_l_vec_rot = getRotatedVec(cp_l_vec, robot_geo_thresh_dist, true);
                                // Eigen::Vector2f l_new_vec = cp + cp_l_vec_rot;
                                
                                // ROS_INFO_STREAM("Within 2 r side: " << cp[0] << " " << cp[1] << " " << l_vec[0] << " " << l_vec[1] << " " << l_new_vec[0] << " " << l_new_vec[1]);
                                
                                Eigen::Vector2f l_used_vec = pFarSafe;
                                Eigen::Vector2f l_normal_vec(-l_used_vec[1], l_used_vec[0]);

                                Eigen::Vector2f l_new = l_used_vec + robot_geo_thresh_dist * l_normal_vec / l_normal_vec.norm();

                                ROS_DEBUG_STREAM("Within 2 r side: " << midControlPt[0] << " " << midControlPt[1] << " " << l_used_vec[0] << " " << l_used_vec[1] << " " << l_new[0] << " " << l_new[1]);

                                // if(isLeftofLine(origin, cp, l_new_vec))
                                // if(isLargerAngle(cp_l_vec_rot, Eigen::Vector2f(1,0)))
                                if(isLargerAngle(l_new, Eigen::Vector2f(1,0)))
                                {
                                    success = false;
                                    ROS_WARN_STREAM("The union region does not exist. [Orientation is within gap 2 r side]");
                                }
                                else
                                {

                                    // bool left_side_of_l = isLeftofLine(cp, l_new_vec, pGoal);
                                    // bool left_side_of_r = isLeftofLine(origin, cp, pGoal);
                                    // bool larger_than_l = isLargerAngle(pGoal - cp, cp_l_vec_rot);
                                    // bool larger_than_r = isLargerAngle(pGoal - cp, Eigen::Vector2f(1,0));
                                    // ROS_INFO_STREAM(larger_than_l << " " << larger_than_r);
                                    bool larger_than_l = isLargerAngle(pGoal, l_new);
                                    bool larger_than_r = isLargerAngle(pGoal, Eigen::Vector2f(1,0));

                                    // if(!left_side_of_l)
                                    if(!larger_than_l)
                                    {
                                        // Eigen::Vector2f goal_cp_vec = pGoal - cp;
                                        // new_goal = goal_cp_vec.norm() * cp_l_vec_rot / cp_l_vec_rot.norm() + cp;
                                        new_goal = pGoal.norm() * l_new / l_new.norm();
                                    }
                                    // else if(left_side_of_l && left_side_of_r)
                                    else if(larger_than_l && larger_than_r)
                                    {
                                        // Eigen::Vector2f goal_cp_vec = pGoal - cp;
                                        // new_goal = goal_cp_vec.norm() * rbt_orient_vec + cp;
                                        new_goal = pGoal.norm() * rbt_orient_vec;
                                    }
                                    // else if(left_side_of_l && !left_side_of_r)
                                    else if(larger_than_l && !larger_than_r)
                                    {
                                        new_goal = pGoal;
                                    }
                                    else
                                    {
                                        ROS_WARN_STREAM("Goal point is in the wrong region. [Orientation is within gap 2 r side]");
                                        success = false;
                                    }
                                }
                            }
                            
                        }
                    }
                    else // dist to other inter < thresh
                    {
                        ROS_WARN_STREAM("Distances to the both intersections are smaller than diagonal / 2.");
                        success = false;
                    }
                }
            }
            else // the closest inter point has angle larger than pi/2
            {
                midControlPt = ideal_min_cp_length * rbt_orient_vec;

                success = true;

                // Goal point region
                // Eigen::Vector2f cp_l_vec = l_vec - cp;
                // Eigen::Vector2f cp_l_vec_rot = getRotatedVec(cp_l_vec, robot_geo_thresh_dist, true);
                // Eigen::Vector2f cp_r_vec = r_vec - cp;
                // Eigen::Vector2f cp_r_vec_rot = getRotatedVec(cp_r_vec, robot_geo_thresh_dist, false);
                // Eigen::Vector2f l_new_vec = cp + cp_l_vec_rot;
                // Eigen::Vector2f r_new_vec = cp + cp_r_vec_rot;
                
                // ROS_INFO_STREAM("Within 1 larger: " << cp[0] << " " << cp[1] << " " << l_vec[0] << " " << l_vec[1] << " " << l_new_vec[0] << " " << l_new_vec[1] << " " << r_vec[0] << " " << r_vec[1] << " " << r_new_vec[0] << " " << r_new_vec[1]);

                Eigen::Vector2f l_used_vec = pCloseSafe;
                Eigen::Vector2f r_used_vec = pFarSafe;
                if(!l_side)
                {
                    l_used_vec = pFarSafe;
                    r_used_vec = pCloseSafe;
                }

                Eigen::Vector2f l_normal_vec(-l_used_vec[1], l_used_vec[0]);
                Eigen::Vector2f r_normal_vec(r_used_vec[1], -r_used_vec[0]);

                Eigen::Vector2f l_new = l_used_vec + robot_geo_thresh_dist * l_normal_vec / l_normal_vec.norm();
                Eigen::Vector2f r_new = r_used_vec + robot_geo_thresh_dist * r_normal_vec / r_normal_vec.norm();

                ROS_DEBUG_STREAM("Within 1 larger: " << midControlPt[0] << " " << midControlPt[1] << " " << l_used_vec[0] << " " << l_used_vec[1] << " " << l_new[0] << " " << l_new[1] << " " << r_used_vec[0] << " " << r_used_vec[1] << " " << r_new[0] << " " << r_new[1]);

                // if(!isLeftofLine(cp, l_new_vec, r_new_vec))
                // if(!isLargerAngle(cp_r_vec_rot, cp_l_vec_rot))
                if(!isLargerAngle(r_new, l_new))
                {
                    success = false;
                    ROS_WARN_STREAM("The union region does not exist. [Orientation is within gap larger pi/2]");
                }
                else
                {

                    // bool left_side_of_l = isLeftofLine(cp, l_new_vec, pGoal);
                    // bool left_side_of_r = isLeftofLine(cp, r_new_vec, pGoal);
                    // bool larger_than_l = isLargerAngle(pGoal - cp, cp_l_vec_rot);
                    // bool larger_than_r = isLargerAngle(pGoal - cp, cp_r_vec_rot);
                    bool larger_than_l = isLargerAngle(pGoal, l_new);
                    bool larger_than_r = isLargerAngle(pGoal, r_new);

                    // if(!left_side_of_l)
                    if(!larger_than_l)
                    {
                        // Eigen::Vector2f goal_cp_vec = pGoal - cp;
                        // new_goal = goal_cp_vec.norm() * cp_l_vec_rot / cp_l_vec_rot.norm() + cp;
                        new_goal = pGoal.norm() * l_new / l_new.norm();
                    }
                    // else if(left_side_of_l && left_side_of_r)
                    else if(larger_than_l && larger_than_r)
                    {
                        // Eigen::Vector2f goal_cp_vec = pGoal - cp;
                        // new_goal = goal_cp_vec.norm() * cp_r_vec_rot / cp_r_vec_rot.norm() + cp;
                        new_goal = pGoal.norm() * r_new / r_new.norm();
                    }
                    // else if(left_side_of_l && !left_side_of_r)
                    else if(larger_than_l && !larger_than_r)
                    {
                        new_goal = pGoal;
                    }
                    else
                    {
                        ROS_WARN_STREAM("Goal point is in the wrong region. [Orientation is within gap larger pi/2");
                        success = false;
                    }
                }
            }
        }
        else // orientation is not within gap triangle
        {
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Backward-facing gap");
            
            if(ideal_min_cp_length > cp_max_length)
                midControlPt = cp_max_length * rbt_orient_vec;
            else
                midControlPt = ideal_min_cp_length * rbt_orient_vec;

            // By default, we will calculate r side geo
            Eigen::Vector2f cp_inter_vec = pCloseSafe - midControlPt;
            success = true;

            if(!l_side)
            {
                Eigen::Vector2f cp_inter_normal_vec(cp_inter_vec[1], -cp_inter_vec[0]);
                Eigen::Vector2f cp_new_inter_vec = robot_geo_thresh_dist * cp_inter_normal_vec / cp_inter_normal_vec.norm() + cp_inter_vec;
                Eigen::Vector2f new_inter_vec = cp_new_inter_vec + midControlPt;

                // Eigen::Vector2f cp_l_vec = l_vec - cp;
                // Eigen::Vector2f cp_l_vec_rot = getRotatedVec(cp_l_vec, robot_geo_thresh_dist, true);
                // Eigen::Vector2f l_new_vec = cp + cp_l_vec_rot;
                Eigen::Vector2f l_used = pFarSafe;
                Eigen::Vector2f pLeftSafe_normal_vec(-l_used[1], l_used[0]);
                Eigen::Vector2f l_new_inter_vec = robot_geo_thresh_dist * pLeftSafe_normal_vec / pLeftSafe_normal_vec.norm() + l_used;

                ROS_DEBUG_STREAM("Not within r side: " << midControlPt[0] << " " << midControlPt[1] << " " << pCloseSafe[0] << " " << pCloseSafe[1] << " " << new_inter_vec[0] << " " << new_inter_vec[1] << " " << l_used[0] << " " << l_used[1] << " " << l_new_inter_vec[0] << " " << l_new_inter_vec[1]);
                
                // if(!isLeftofLine(cp, l_new_vec, cp + cp_new_inter_vec))

                // Get intersection point p1 = (0, 0), p2 = l_new_inter_vec, p3 = cp, p4 = new_inter_vec
                float denominator = (0. - l_new_inter_vec[0]) * (midControlPt[1] - new_inter_vec[1]) - (0. - l_new_inter_vec[1]) * (midControlPt[0] - new_inter_vec[0]);
                float nom_x = 0. - (0. - l_new_inter_vec[0]) * (midControlPt[0] * new_inter_vec[1] - midControlPt[1] * new_inter_vec[0]);
                float nom_y = 0. - (0. - l_new_inter_vec[1]) * (midControlPt[0] * new_inter_vec[1] - midControlPt[1] * new_inter_vec[0]);

                bool has_inter = true;
                float inter_x, inter_y;
                if(denominator == 0)
                    has_inter = false;
                else
                {
                    inter_x = nom_x / denominator;
                    inter_y = nom_y / denominator;
                }

                bool inter_left_to_cp = isLeftofLine(Eigen::Vector2f(0, 0), midControlPt, Eigen::Vector2f(inter_x, inter_y));
                bool left_to_cp_line = isLeftofLine(Eigen::Vector2f(0, 0), midControlPt, pGoal);
                bool left_to_inter_line = isLeftofLine(Eigen::Vector2f(inter_x, inter_y), Eigen::Vector2f(inter_x, inter_y) + midControlPt, pGoal);
                bool left_to_cp_inter = isLeftofLine(midControlPt, new_inter_vec, pGoal);
                bool left_to_new_inter = isLeftofLine(Eigen::Vector2f(0, 0), l_new_inter_vec, pGoal);

                if(!has_inter || (has_inter && inter_left_to_cp) || (has_inter && !inter_left_to_cp && left_to_inter_line))
                {
                    if(left_to_cp_line || (!left_to_cp_line && left_to_cp_inter))
                    {
                        Eigen::Vector2f goal_cp_vec = pGoal - midControlPt;
                        new_goal = goal_cp_vec.norm() * cp_new_inter_vec / cp_new_inter_vec.norm() + midControlPt;
                    }
                    else if(!left_to_cp_line && !left_to_new_inter)
                    {
                        new_goal = pGoal.norm() * l_new_inter_vec / l_new_inter_vec.norm();
                    }
                    else if(!left_to_cp_line && left_to_new_inter && !left_to_cp_inter)
                    {
                        new_goal = pGoal;
                    }
                    else
                    {
                        ROS_WARN_STREAM("Goal point is in the wrong region. [Orientation is not within gap r side, parallel]");
                        success = false;
                    }
                }
                else if((has_inter && !inter_left_to_cp && !left_to_inter_line))
                {
                    new_goal = Eigen::Vector2f(inter_x, inter_y);
                }
                else
                {
                    ROS_WARN_STREAM("Goal point is in the wrong region. [Orientation is not within gap r side, not desired]");
                    success = false;
                }
                
            }
            else
            {
                Eigen::Vector2f cp_inter_normal_vec(-cp_inter_vec[1], cp_inter_vec[0]);
                Eigen::Vector2f cp_new_inter_vec = robot_geo_thresh_dist * cp_inter_normal_vec / cp_inter_normal_vec.norm() + cp_inter_vec;
                Eigen::Vector2f new_inter_vec = cp_new_inter_vec + midControlPt;

                // Eigen::Vector2f cp_r_vec = r_vec - cp;
                // Eigen::Vector2f cp_r_vec_rot = getRotatedVec(cp_r_vec, robot_geo_thresh_dist, false);
                // Eigen::Vector2f r_new_vec = cp + cp_r_vec_rot;
                Eigen::Vector2f r_used = pFarSafe;
                Eigen::Vector2f r_inter_normal_vec(r_used[1], -r_used[0]);
                Eigen::Vector2f r_new_inter_vec = robot_geo_thresh_dist * r_inter_normal_vec / r_inter_normal_vec.norm() + r_used;

                ROS_DEBUG_STREAM("Not within l side: " << midControlPt[0] << " " << midControlPt[1] << " " << pCloseSafe[0] << " " << pCloseSafe[1] << " " << new_inter_vec[0] << " " << new_inter_vec[1] << " " << r_used[0] << " " << r_used[1] << " " << r_new_inter_vec[0] << " " << r_new_inter_vec[1]);
                // if(!isLeftofLine(cp, cp + cp_new_inter_vec, r_new_vec))
                // Get intersection point p1 = (0, 0), p2 = r_new_inter_vec, p3 = cp, p4 = new_inter_vec
                float denominator = (0. - r_new_inter_vec[0]) * (midControlPt[1] - new_inter_vec[1]) - (0. - r_new_inter_vec[1]) * (midControlPt[0] - new_inter_vec[0]);
                float nom_x = 0. - (0. - r_new_inter_vec[0]) * (midControlPt[0] * new_inter_vec[1] - midControlPt[1] * new_inter_vec[0]);
                float nom_y = 0. - (0. - r_new_inter_vec[1]) * (midControlPt[0] * new_inter_vec[1] - midControlPt[1] * new_inter_vec[0]);

                bool has_inter = true;
                float inter_x, inter_y;
                if(denominator == 0)
                    has_inter = false;
                else
                {
                    inter_x = nom_x / denominator;
                    inter_y = nom_y / denominator;
                }

                bool inter_left_to_cp = isLeftofLine(Eigen::Vector2f(0, 0), midControlPt, Eigen::Vector2f(inter_x, inter_y));
                bool left_to_cp_line = isLeftofLine(Eigen::Vector2f(0, 0), midControlPt, pGoal);
                bool left_to_inter_line = isLeftofLine(Eigen::Vector2f(inter_x, inter_y), Eigen::Vector2f(inter_x, inter_y) + midControlPt, pGoal);
                bool left_to_cp_inter = isLeftofLine(midControlPt, new_inter_vec, pGoal);
                bool left_to_new_inter = isLeftofLine(Eigen::Vector2f(0, 0), r_new_inter_vec, pGoal);

                if(!has_inter || (has_inter && !inter_left_to_cp) || (has_inter && inter_left_to_cp && !left_to_inter_line))
                {
                    if(!left_to_cp_line || (left_to_cp_line && !left_to_cp_inter))
                    {
                        Eigen::Vector2f goal_cp_vec = pGoal - midControlPt;
                        new_goal = goal_cp_vec.norm() * cp_new_inter_vec / cp_new_inter_vec.norm() + midControlPt;
                    }
                    else if(left_to_cp_line && left_to_new_inter)
                    {
                        new_goal = pGoal.norm() * r_new_inter_vec / r_new_inter_vec.norm();
                    }
                    else if(left_to_cp_line && !left_to_new_inter && left_to_cp_inter)
                    {
                        new_goal = pGoal;
                    }
                    else
                    {
                        ROS_WARN_STREAM("Goal point is in the wrong region. [Orientation is not within gap l side, parallel]");
                        success = false;
                    }
                }
                else if((has_inter && inter_left_to_cp && left_to_inter_line))
                {
                    new_goal = Eigen::Vector2f(inter_x, inter_y);
                }
                else
                {
                    ROS_WARN_STREAM("Goal point is in the wrong region. [Orientation is not within gap l side, not desired]");
                    success = false;
                }
            }
            
        }

        if (success)
        {
            BezierCurve = Bezier::Bezier<2>({ {0, 0}, 
                                                {midControlPt[0], midControlPt[1]}, 
                                                {new_goal[0], new_goal[1]} });
        }

        return success;
    }

    geometry_msgs::PoseArray GapTrajGenerator::generateBezierTrajectory(Gap * gap, 
                                                                        const geometry_msgs::TwistStamped & rbtVelRbtFrame, 
                                                                        const geometry_msgs::TransformStamped & odom2rbt)
    {
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "[generateBezierTrajectory()]");
        geometry_msgs::PoseArray posearr;
        posearr.header.stamp = ros::Time::now();
        
        posearr.header.frame_id = cfg_->robot_frame_id;

        // if (gap->goal.discard) 
        // {
        //     ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "This waypoint is discard.");
        //     return posearr;
        // }

        Bezier::Bezier<2> quadraBezier;
        bool success = findBezierControlPts(gap, quadraBezier, rbtVelRbtFrame, odom2rbt);
        
        if(!success)
        {
            ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "No path is generated.");
            return posearr;
        }
        else
        {
            if(!cfg_->traj.bezier_interp)
            {
                for (float t = 0; t <= 1; t+=0.02)
                {
                    geometry_msgs::Pose pose;
                    pose.position.x = quadraBezier.valueAt(t, 0);
                    pose.position.y = quadraBezier.valueAt(t, 1);
                    posearr.poses.push_back(pose);
                }
                return posearr;
            }
            else
            {
                float des_dist = robot_geo_proc_.getRobotAvgLinSpeed() * cfg_->traj.bezier_unit_time;
                float entire_dist = getBezierDist(quadraBezier, 0, 1, 30);
                int num_sampled_pts = int(round(entire_dist / des_dist));
                num_sampled_pts = num_sampled_pts >= 2 ? num_sampled_pts : 2;

                float dist_thresh = des_dist / 10;
                float t_step = 1. / (num_sampled_pts - 1);
                float t_min = 0;
                for (size_t i = 0; i < num_sampled_pts; i++)
                {
                    float cur_t = i * t_step;
                    float cur_dist = getBezierDist(quadraBezier, t_min, cur_t, 5);
                    if(abs(cur_dist - des_dist) < dist_thresh)
                    {
                        geometry_msgs::Pose pose;
                        pose.position.x = quadraBezier.valueAt(cur_t, 0);
                        pose.position.y = quadraBezier.valueAt(cur_t, 1);
                        posearr.poses.push_back(pose);
                        t_min = cur_t;
                    }
                    else if(cur_dist > des_dist)
                    {
                        float t_prev = (i - 1) * t_step;
                        float t_interp = (cur_t + t_prev) / 2;
                        float interp_dist = getBezierDist(quadraBezier, t_min, t_interp, 5);

                        float t_high = cur_t;
                        float t_low = t_prev;
                        while(abs(interp_dist - des_dist) > dist_thresh)
                        {
                            if(interp_dist < des_dist)
                            {
                                t_low = t_interp;
                                t_interp = (t_interp + t_high) / 2;
                            }
                            else
                            {
                                t_high = t_interp;
                                t_interp = (t_interp + t_low) / 2;
                            }
                            interp_dist = getBezierDist(quadraBezier, t_min, t_interp, 5);
                            if(abs(t_interp - t_low) <= 1e-3 && abs(t_interp - t_high) <= 1e-3)
                                break;
                            // ROS_INFO_STREAM(t_interp << " " << t_low << " " << t_high << " " << interp_dist << " " << abs(interp_dist - des_dist) << " " << dist_thresh);
                        }
                        // ROS_INFO_STREAM("exit");
                        geometry_msgs::Pose pose;
                        pose.position.x = quadraBezier.valueAt(t_interp, 0);
                        pose.position.y = quadraBezier.valueAt(t_interp, 1);
                        posearr.poses.push_back(pose);
                        t_min = t_interp;
                    }
                }

                if (posearr.poses.size() < num_sampled_pts)
                {
                    geometry_msgs::Pose pose;
                    pose.position.x = quadraBezier.valueAt(1, 0);
                    pose.position.y = quadraBezier.valueAt(1, 1);
                    posearr.poses.push_back(pose);
                }
                return posearr;
            }
        }
    }

    geometry_msgs::PoseArray GapTrajGenerator::transformPath(const geometry_msgs::PoseArray & poseArrayIn,
                                                                const geometry_msgs::TransformStamped & trans)
    {
        geometry_msgs::PoseArray poseArrayOut;
        geometry_msgs::PoseStamped outplaceholder;
        // outplaceholder.header.frame_id = cfg_->odom_frame_id;
        outplaceholder.header.frame_id = trans.header.frame_id;
        geometry_msgs::PoseStamped inplaceholder;
        // inplaceholder.header.frame_id = cfg_->robot_frame_id;
        inplaceholder.header.frame_id = trans.child_frame_id;
        for (const geometry_msgs::Pose & pose : poseArrayIn.poses)
        {
            inplaceholder.pose = pose;
            tf2::doTransform(inplaceholder, outplaceholder, trans);
            poseArrayOut.poses.push_back(outplaceholder.pose);
        }
        // poseArrayOut.header.frame_id = cfg_->odom_frame_id;
        poseArrayOut.header.frame_id = trans.header.frame_id;
        poseArrayOut.header.stamp = trans.header.stamp;
        return poseArrayOut;
    }

    geometry_msgs::PoseArray GapTrajGenerator::processTrajectory(const geometry_msgs::PoseArray & pose_arr)
    {
        geometry_msgs::PoseArray new_pose_arr;

        Eigen::Quaternionf q;
        geometry_msgs::Pose old_pose;
        old_pose.position.x = 0;
        old_pose.position.y = 0;
        old_pose.position.z = 0;
        old_pose.orientation.x = 0;
        old_pose.orientation.y = 0;
        old_pose.orientation.z = 0;
        old_pose.orientation.w = 1;
        geometry_msgs::Pose new_pose;
        float dx, dy, result;

        std::vector<geometry_msgs::Pose> shortened;
        shortened.push_back(old_pose);
        for (const geometry_msgs::Pose & pose : pose_arr.poses)
        {
            dx = pose.position.x - shortened.back().position.x;
            dy = pose.position.y - shortened.back().position.y;
            result = sqrt(pow(dx, 2) + pow(dy, 2));
            if (result > 0.05)
                shortened.push_back(pose);
        }

        new_pose_arr.header = pose_arr.header;
        new_pose_arr.poses = shortened;

        // Fix rotation
        for (int idx = 1; idx < new_pose_arr.poses.size(); idx++)
        {
            new_pose = new_pose_arr.poses[idx];
            old_pose = new_pose_arr.poses[idx - 1];
            dx = new_pose.position.x - old_pose.position.x;
            dy = new_pose.position.y - old_pose.position.y;
            result = std::atan2(dy, dx);
            q = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(result, Eigen::Vector3f::UnitZ());
            q.normalize();
            new_pose_arr.poses[idx - 1].orientation.x = q.x();
            new_pose_arr.poses[idx - 1].orientation.y = q.y();
            new_pose_arr.poses[idx - 1].orientation.z = q.z();
            new_pose_arr.poses[idx - 1].orientation.w = q.w();
        }
        new_pose_arr.poses.pop_back();

        return new_pose_arr;
    }

    Eigen::Vector2f GapTrajGenerator::getRotatedVec(const Eigen::Vector2f & orig_vec, const float & chord_length, const bool & ccw)
    {
        float r = orig_vec.norm();
        float rotate_angle = acos((2 * r * r - chord_length * chord_length) / (2 * r * r));

        Eigen::Matrix2f rotate_mat;
        if(ccw)
            rotate_mat << cos(rotate_angle), -sin(rotate_angle), sin(rotate_angle), cos(rotate_angle);
        else
            rotate_mat << cos(rotate_angle), sin(rotate_angle), -sin(rotate_angle), cos(rotate_angle);

        Eigen::Vector2f rotated_vec = rotate_mat * orig_vec;

        return rotated_vec;
    }

}