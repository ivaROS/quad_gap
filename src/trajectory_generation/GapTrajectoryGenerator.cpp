#include <quad_gap/trajectory_generation/GapTrajectoryGenerator.h>

namespace quad_gap
{
    Trajectory GapTrajGenerator::generateTrajectory(Gap * gap, 
                                                    const geometry_msgs::PoseStamped & curr_pose) 
    {
        // return geometry_msgs::PoseArray();
        geometry_msgs::PoseArray pathRbtFrame;
        pathRbtFrame.header.stamp = ros::Time::now();
        
        write_trajectory corder(pathRbtFrame, cfg_->robot_frame_id);
        pathRbtFrame.header.frame_id = cfg_->robot_frame_id;

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
            Trajectory traj(pathRbtFrame);
            return traj;
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
        
        polar_gap_field inte(xLeft, xRight, 
                            yLeft, yRight,
                            goal_x,
                            goal_y,
                            gap->isRadial(),
                            cfg_->traj.sigma);
        boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
                                                inte, 
                                                x, 
                                                0.0f,
                                                cfg_->traj.integrate_maxt,
                                                cfg_->traj.integrate_stept, 
                                                corder);

        if (gap->isExtended()) 
        {
            for (geometry_msgs::Pose & p : pathRbtFrame.poses) 
            {
                p.position.x += qB(0);
                p.position.y += qB(1);
            }
        }

        Trajectory traj(pathRbtFrame);
        return traj;
    }

    bool GapTrajGenerator::findBezierControlPts(Gap * gap, 
                                                Bezier::Bezier<2>& BezierCurve, 
                                                const geometry_msgs::TwistStamped & rbtVelRbtFrame)
    {
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "[findBezierControlPts()]");

        // Bezier control poitns
        // Q0 = (0, 0)
        // Q1 = (x1, y1)
        // Q2 = (x2, y2) = goal
        Eigen::Vector2f q0(0.0, 0.0); // Start point

        // Find the intersections of triangle and circle
        Eigen::Vector2f pLeft = gap->getManipLCartesian(); // (xLeft, yLeft);
        Eigen::Vector2f pRight = gap->getManipRCartesian(); // (xRight, yRight);

        // float xLeft, xRight, yLeft, yRight;
        // xLeft = pLeft[0];
        // yLeft = pLeft[1]; 
        // xRight = pRight[0];
        // yRight = pRight[1];

        // ROS_INFO_STREAM_NAMED("GapManipulator", "        pLeft: (" << pLeft.transpose() << ")");
        // ROS_INFO_STREAM_NAMED("GapManipulator", "        pRight: (" << pRight.transpose() << ")");        

        float goal_x = gap->getGoalX();
        float goal_y = gap->getGoalY();

        // Check if goal is in the middle
        // float thetaLeft = std::atan2(yLeft, xLeft);
        // float thetaRight = std::atan2(yRight, xRight);
        // float thetaGoal = std::atan2(goal_y, goal_x);

        // assert(thetaGoal >= thetaRight && thetaGoal <= thetaLeft);

        // Eigen::Vector2f pLeft(xRight, yRight);
        // Eigen::Vector2f pRight(xLeft, yLeft);
        float minSafeDist = gap->getMinSafeDist();
        // assert(minSafeDist <= pLeft.norm() && minSafeDist <= pRight.norm());

        if (minSafeDist > pLeft.norm() || minSafeDist > pRight.norm())
        {
            ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "The circle radius is larger than the triangle side length.");
            return false;
        }

        float boxGeomScale = cfg_->traj.robot_geo_scale;
        float scaledMinDim = robotGeoProc_->getRobotMinRadius() * boxGeomScale; // width / 2

        // max norm for second control point
        float q1MaxNorm = minSafeDist - scaledMinDim;
        
        if (q1MaxNorm <= 0)
        {
            ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "The circle is smaller than robot thresh.");
            return false;
        }

        Eigen::Vector2f pLeftSafe = minSafeDist * pLeft / pLeft.norm();
        Eigen::Vector2f pRightSafe = minSafeDist * pRight / pRight.norm();
        Eigen::Vector2f pGoal(goal_x, goal_y);

        // Eigen::Vector2f rbt_orient_vec(1, 0);
        float x_speed = rbtVelRbtFrame.twist.linear.x;
        float q1IdealNorm = x_speed / 2; // For quadratic bezier curve, the ideally length B'(0) = 2 (P_1 - P_0) 


        // Find the closest gap side to orientation. By default, l side is closer.
        // bool left = abs(thetaLeft) < abs(thetaRight);
        // Eigen::Vector2f pCloseSafe;
        // Eigen::Vector2f pFarSafe;

        // if (left)
        // {
        //     pCloseSafe = pLeftSafe;
        //     pFarSafe = pRightSafe;
        // } else
        // {
        //     pCloseSafe = pRightSafe;
        //     pFarSafe = pLeftSafe;
        // }

        Eigen::Vector2f q1;
        Eigen::Vector2f q2;
        // bool success = false;

        ///////////////////////////////////////////////////////////
        // If the goal is inside the circle, directly go to goal //
        ///////////////////////////////////////////////////////////
        if (pGoal.norm() <= minSafeDist)
        {
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Goal is within circle.");

            if (q1IdealNorm > q1MaxNorm)
                q1 = q1MaxNorm * robotOrientationVector;
            else
                q1 = q1IdealNorm * robotOrientationVector;
            
            q2 = pGoal;

            BezierCurve = Bezier::Bezier<2>({ {q0[0], q0[1]}, 
                                                {q1[0], q1[1]}, 
                                                {q2[0], q2[1]} });

            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Goal is within circle.");
            return true;
        }

        float robot_geo_diagonal_thresh = robotGeoProc_->getRobotMaxRadius() * boxGeomScale;
        // float q1MaxNorm = minSafeDist - robot_geo_diagonal_thresh;

        bool success = findBezierControlPtsNew(pLeftSafe, pRightSafe, pGoal,
                                                scaledMinDim, q1IdealNorm, q1MaxNorm,
                                                q1, q2);

        // Conditions
        // if (thetaLeft < thetaRight)
        // {
        //     findBackFacingBezierControlPts(left,
        //                                     pCloseSafe,
        //                                     pFarSafe,
        //                                     scaledMinDim,
        //                                     robot_geo_diagonal_thresh,
        //                                     q1IdealNorm,
        //                                     q1MaxNorm,
        //                                     rbt_orient_vec,
        //                                     pGoal,
        //                                     q1,
        //                                     q2,
        //                                     success);
        // } else
        // {
        //     findFrontFacingBezierControlPts(left,
        //                                     pCloseSafe,
        //                                     pFarSafe,
        //                                     scaledMinDim,
        //                                     robot_geo_diagonal_thresh,
        //                                     q1IdealNorm,
        //                                     q1MaxNorm,
        //                                     rbt_orient_vec,
        //                                     pGoal,
        //                                     q1,
        //                                     q2,
        //                                     success);
        // }
        // // if (thetaLeft > 0 && thetaRight <= 0)
        // {

        // } else // orientation is not within gap triangle
        // {

        // }

        if (success)
        {
            BezierCurve = Bezier::Bezier<2>({ {q0[0], q0[1]}, 
                                                {q1[0], q1[1]}, 
                                                {q2[0], q2[1]} });
        }

        return success;
    }

    bool GapTrajGenerator::findBezierControlPtsNew(const Eigen::Vector2f pLeftSafe,
                                                    const Eigen::Vector2f pRightSafe,    
                                                    const Eigen::Vector2f & pGoal,
                                                    const float & scaledMinDim,
                                                    const float & q1IdealNorm,
                                                    const float & q1MaxNorm,
                                                    Eigen::Vector2f & q1,
                                                    Eigen::Vector2f & q2)
    {
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "[findBezierControlPtsNew]");

        ///////////////
        // Max's way //
        ///////////////

        // 1. Calculate q1
        if (q1IdealNorm > q1MaxNorm)
        {
            q1 = q1MaxNorm * robotOrientationVector;
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "q1IdealNorm is larger than q1MaxNorm");
        } else
        {
            q1 = q1IdealNorm * robotOrientationVector;
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "q1IdealNorm is smaller than q1MaxNorm");
        }
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "q1: " << q1.transpose());

        // 2. Calculate qL_infl and qL_infl
        Eigen::Vector2f eLeft = pLeftSafe.normalized();
        Eigen::Vector2f leftAngularInflDir = Rnegpi2 * eLeft; 

        Eigen::Vector2f pLeftSafeInfl = pLeftSafe + scaledMinDim * leftAngularInflDir;
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "pLeftSafeInfl: " << pLeftSafeInfl.transpose());

        Eigen::Vector2f eRight = pRightSafe.normalized();
        Eigen::Vector2f rightAngularInflDir = Rpi2 * eRight; 

        Eigen::Vector2f pRightSafeInfl = pRightSafe + scaledMinDim * rightAngularInflDir;
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "pRightSafeInfl: " << pRightSafeInfl.transpose());

        // Check if inflation failed

        float origLeftToRightAngle = getSweptLeftToRightAngle(pLeftSafe, pRightSafe);
        float newLeftToRightAngle = getSweptLeftToRightAngle(pLeftSafeInfl, pRightSafeInfl);

        if (newLeftToRightAngle > origLeftToRightAngle)
        {
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Inflation failed. The new angle is larger than the original angle.");
            // ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "Inflation failed. The new angle is larger than the original angle.");
            return false;
        }

        // 3. Calculate q2

        // WHAT IF NEW LINES INTERSECT?

        // if goal in between, set q2 to goal
        float leftToGoalSignedAngle = getSignedLeftToRightAngle(pLeftSafeInfl, pGoal);
        float rightToGoalSignedAngle = getSignedLeftToRightAngle(pRightSafeInfl, pGoal);

        if (leftToGoalSignedAngle >= 0 && rightToGoalSignedAngle <= 0)
        {
            // Goal is in between the inflated lines
            q2 = pGoal;
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Goal is in between the inflated lines.");
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "q2 set to goal: " << q2.transpose());
            return true;
        }

        // if goal closer to left side, set q2 to left side
        if (std::abs(leftToGoalSignedAngle) < std::abs(rightToGoalSignedAngle))
        {
            // Goal is closer to the left side
            q2 = pLeftSafeInfl.normalized() * pGoal.norm();
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Goal is closer to the left side.");
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "q2 set to left side: " << q2.transpose());
            return true;
        }

        // if goal closer to right side, set q2 to right side
        q2 = pRightSafeInfl.normalized() * pGoal.norm();
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Goal is closer to the right side.");
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "q2 set to right side: " << q2.transpose());
        return true;
    }

    // void GapTrajGenerator::findFrontFacingBezierControlPts(const bool & left,
    //                                                         const Eigen::Vector2f pCloseSafe,
    //                                                         const Eigen::Vector2f pFarSafe,
    //                                                         const float & scaledMinDim,
    //                                                         const float & robot_geo_diagonal_thresh,
    //                                                         const float & q1IdealNorm,
    //                                                         const float & q1MaxNorm,
    //                                                         const Eigen::Vector2f & rbt_orient_vec,
    //                                                         const Eigen::Vector2f & pGoal,
    //                                                         Eigen::Vector2f & q1,
    //                                                         Eigen::Vector2f & q2,
    //                                                         bool & success)
    // {
    //     ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Front-facing gap");

    //     float thetaCloseSafe = atan2(pCloseSafe[1], pCloseSafe[0]);

    //     if (abs(thetaCloseSafe) <= M_PI_OVER_TWO)
    //     {
    //         ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   Convex gap");

    //         float dist_inter_orient = abs(pCloseSafe[1]);

    //         ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "       dist_inter_orient: " << dist_inter_orient);
    //         ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "       scaledMinDim: " << scaledMinDim);

    //         if (dist_inter_orient >= scaledMinDim)
    //         {
    //             ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "       Larger than robot geo thresh dist");

    //             if (q1IdealNorm > q1MaxNorm)
    //                 q1 = q1MaxNorm * rbt_orient_vec;
    //             else
    //                 q1 = q1IdealNorm * rbt_orient_vec;

    //             // Goal point region
    //             success = true;

    //             // ROS_INFO_STREAM("Within 1: " << cp[0] << " " << cp[1] << " " << pLeft[0] << " " << pLeft[1] << " " << l_new_vec[0] << " " << l_new_vec[1] << " " << pRight[0] << " " << pRight[1] << " " << r_new_vec[0] << " " << r_new_vec[1]);
                
    //             // INFLATING
    //             Eigen::Vector2f l_used_vec = pCloseSafe;
    //             Eigen::Vector2f r_used_vec = pFarSafe;
    //             if (left)
    //             {
    //                 l_used_vec = pCloseSafe;
    //                 r_used_vec = pFarSafe;
    //             } else
    //             {
    //                 l_used_vec = pFarSafe;
    //                 r_used_vec = pCloseSafe;
    //             }

    //             Eigen::Vector2f l_normal_vec(-l_used_vec[1], l_used_vec[0]);
    //             Eigen::Vector2f r_normal_vec(r_used_vec[1], -r_used_vec[0]);

    //             Eigen::Vector2f l_new = l_used_vec + scaledMinDim * l_normal_vec / l_normal_vec.norm();
    //             Eigen::Vector2f r_new = r_used_vec + scaledMinDim * r_normal_vec / r_normal_vec.norm();

    //             ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Within 1: " << q1[0] << " " << q1[1] << " " << l_used_vec[0] << " " << l_used_vec[1] << " " << l_new[0] << " " << l_new[1] << " " << r_used_vec[0] << " " << r_used_vec[1] << " " << r_new[0] << " " << r_new[1]);

    //             if (!isLargerAngle(r_new, l_new))
    //             {
    //                 // Inflation failed
    //                 ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "The union region does not exist. [Orientation is within gap 1]");
    //                 success = false;
    //             } else
    //             {
    //                 // Setting q2
    //                 bool larger_than_l = isLargerAngle(pGoal, l_new);
    //                 bool larger_than_r = isLargerAngle(pGoal, r_new);

    //                 if (!larger_than_l)
    //                 {
    //                     q2 = pGoal.norm() * l_new / l_new.norm();
    //                 } else if (larger_than_l && larger_than_r)
    //                 {
    //                     q2 = pGoal.norm() * r_new / r_new.norm();
    //                 } else if (larger_than_l && !larger_than_r)
    //                 {
    //                     q2 = pGoal;
    //                 } else
    //                 {
    //                     ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "Goal point is in the wrong region. [Orientation is within gap 1]");
    //                     success = false;
    //                 }
    //             }
    //         } else
    //         {
    //             ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "       Smaller than robot geo thresh dist");

    //             float distPFarSafeOrient = abs(pFarSafe[1]);
    //             if (distPFarSafeOrient >= scaledMinDim)
    //             {
    //                 // Find the intersect for max length / 2
    //                 float length_devi = sqrt(robot_geo_diagonal_thresh * robot_geo_diagonal_thresh - distPFarSafeOrient * distPFarSafeOrient);
    //                 Eigen::Vector2f max_cp(pCloseSafe[0] - length_devi, 0);
                    
    //                 bool cp_success = false;

    //                 if (max_cp[0] <= 0)
    //                 {
    //                     ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "The chosen intersection point is too close to robot. (smaller than diagonal / 2)");
    //                     success = false;
    //                 } else
    //                 {
    //                     if (max_cp.norm() <= q1IdealNorm)
    //                         q1 = max_cp.norm() * rbt_orient_vec;
    //                     else
    //                         q1 = q1IdealNorm * rbt_orient_vec;

    //                     cp_success = true;
    //                 }

    //                 if (cp_success)
    //                 {
    //                     success = true;
    //                     if (left)
    //                     {
    //                         // ROS_INFO_STREAM("Within 2 l side: " << cp[0] << " " << cp[1] << " " << r_vec[0] << " " << r_vec[1] << " " << r_new_vec[0] << " " << r_new_vec[1]);

    //                         // Eigen::Vector2f r_normal_vec(pFarSafe[1], -pFarSafe[0]);
    //                         Eigen::Vector2f eFar = pFarSafe.normalized();
    //                         Eigen::Vector2f rightAngularInflDir = Rpi2 * eFar; 

    //                         Eigen::Vector2f pFarSafeInfl = pFarSafe + scaledMinDim * rightAngularInflDir;

    //                         ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Within 2 l side: ");
    //                         ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "q1: " << q1.transpose());
    //                         ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "pFarSafe: " << pFarSafe.transpose());
    //                         ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "pFarSafeInfl: " << pFarSafeInfl.transpose());

    //                         if (!isLargerAngle(pFarSafeInfl, rbt_orient_vec))
    //                         {
    //                             success = false;
    //                             ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "The union region does not exist. [Orientation is within gap 2 l side]");
    //                         } else
    //                         {
    //                             bool larger_than_l = isLargerAngle(pGoal, rbt_orient_vec);
    //                             bool larger_than_r = isLargerAngle(pGoal, pFarSafeInfl);

    //                             if (!larger_than_l)
    //                             {
    //                                 q2 = pGoal.norm() * rbt_orient_vec;
    //                             } else if (larger_than_l && larger_than_r)
    //                             {
    //                                 q2 = pGoal.norm() * pFarSafeInfl / pFarSafeInfl.norm();
    //                             } else if (larger_than_l && !larger_than_r)
    //                             {
    //                                 q2 = pGoal;
    //                             } else
    //                             {
    //                                 ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "Goal point is in the wrong region. [Orientation is within gap 2 l side]");
    //                                 success = false;
    //                             }
    //                         }
    //                     } else
    //                     {
    //                         ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Within 2 r side: ");
    //                         // Eigen::Vector2f l_normal_vec(-pFarSafe[1], pFarSafe[0]);
    //                         Eigen::Vector2f eFar = pFarSafe.normalized();
    //                         Eigen::Vector2f leftAngularInflDir = Rnegpi2 * eFar; 

    //                         Eigen::Vector2f pFarSafeInfl = pFarSafe + scaledMinDim * leftAngularInflDir;

    //                         ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "q1: " << q1.transpose());
    //                         ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "pFarSafe: " << pFarSafe.transpose());
    //                         ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "pFarSafeInfl: " << pFarSafeInfl.transpose());

    //                         // I THINK THIS IS WRONG OR UNNECESSARY
    //                         // if (isLargerAngle(pFarSafeInfl, rbt_orient_vec))
    //                         // {
    //                         //     success = false;
    //                         //     ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "The union region does not exist. [Orientation is within gap 2 r side]");
    //                         //     ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "The union region does not exist. [Orientation is within gap 2 r side]");
    //                         // } else
    //                         // {
    //                         bool larger_than_l = isLargerAngle(pGoal, pFarSafeInfl);
    //                         bool larger_than_r = isLargerAngle(pGoal, rbt_orient_vec);

    //                         if (!larger_than_l)
    //                         {
    //                             q2 = pGoal.norm() * pFarSafeInfl / pFarSafeInfl.norm();
    //                         } else if (larger_than_l && larger_than_r)
    //                         {
    //                             q2 = pGoal.norm() * rbt_orient_vec;
    //                         } else if(larger_than_l && !larger_than_r)
    //                         {
    //                             q2 = pGoal;
    //                         } else
    //                         {
    //                             ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Goal point is in the wrong region. [Orientation is within gap 2 r side]");
    //                             ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "Goal point is in the wrong region. [Orientation is within gap 2 r side]");
    //                             success = false;
    //                         }
    //                         // }
    //                     }
    //                 }
    //             } else // dist to other inter < thresh
    //             {
    //                 ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Distances to the both intersections are smaller than diagonal / 2.");
    //                 ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "Distances to the both intersections are smaller than diagonal / 2.");
    //                 success = false;
    //             }
    //         }
    //     } else // the closest inter point has angle larger than pi/2
    //     {
    //         ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   (Potentially) non-convex gap");

    //         q1 = q1IdealNorm * rbt_orient_vec;

    //         success = true;

    //         // ROS_INFO_STREAM("Within 1 larger: " << cp[0] << " " << cp[1] << " " << l_vec[0] << " " << l_vec[1] << " " << l_new_vec[0] << " " << l_new_vec[1] << " " << r_vec[0] << " " << r_vec[1] << " " << r_new_vec[0] << " " << r_new_vec[1]);

    //         Eigen::Vector2f l_used_vec;
    //         Eigen::Vector2f r_used_vec;
    //         if (left)
    //         {
    //             l_used_vec = pCloseSafe;
    //             r_used_vec = pFarSafe;
    //         } else
    //         {
    //             l_used_vec = pFarSafe;
    //             r_used_vec = pCloseSafe;
    //         }

    //         Eigen::Vector2f l_normal_vec(-l_used_vec[1], l_used_vec[0]);
    //         Eigen::Vector2f r_normal_vec(r_used_vec[1], -r_used_vec[0]);

    //         Eigen::Vector2f l_new = l_used_vec + scaledMinDim * l_normal_vec / l_normal_vec.norm();
    //         Eigen::Vector2f r_new = r_used_vec + scaledMinDim * r_normal_vec / r_normal_vec.norm();

    //         ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Within 1 larger: " << q1[0] << " " << q1[1] << " " << l_used_vec[0] << " " << l_used_vec[1] << " " << l_new[0] << " " << l_new[1] << " " << r_used_vec[0] << " " << r_used_vec[1] << " " << r_new[0] << " " << r_new[1]);

    //         if (!isLargerAngle(r_new, l_new))
    //         {
    //             success = false;
    //             ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "The union region does not exist. [Orientation is within gap larger pi/2]");
    //         } else
    //         {
    //             bool larger_than_l = isLargerAngle(pGoal, l_new);
    //             bool larger_than_r = isLargerAngle(pGoal, r_new);

    //             if (!larger_than_l)
    //             {
    //                 q2 = pGoal.norm() * l_new / l_new.norm();
    //             } else if(larger_than_l && larger_than_r)
    //             {
    //                 q2 = pGoal.norm() * r_new / r_new.norm();
    //             } else if(larger_than_l && !larger_than_r)
    //             {
    //                 q2 = pGoal;
    //             } else
    //             {
    //                 ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "Goal point is in the wrong region. [Orientation is within gap larger pi/2");
    //                 success = false;
    //             }
    //         }
    //     }
    // }

    // void GapTrajGenerator::findBackFacingBezierControlPts(const bool & left,
    //                                                         const Eigen::Vector2f pCloseSafe,
    //                                                         const Eigen::Vector2f pFarSafe,
    //                                                         const float & scaledMinDim,
    //                                                         const float & robot_geo_diagonal_thresh,
    //                                                         const float & q1IdealNorm,
    //                                                         const float & q1MaxNorm,
    //                                                         const Eigen::Vector2f & rbt_orient_vec,
    //                                                         const Eigen::Vector2f & pGoal,
    //                                                         Eigen::Vector2f & q1,
    //                                                         Eigen::Vector2f & q2,
    //                                                         bool & success)
    // {
    //     ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Backward-facing gap");
            
    //     if (q1IdealNorm > q1MaxNorm)
    //         q1 = q1MaxNorm * rbt_orient_vec;
    //     else
    //         q1 = q1IdealNorm * rbt_orient_vec;

    //     // By default, we will calculate r side geo
    //     Eigen::Vector2f cp_inter_vec = pCloseSafe - q1;
    //     success = true;

    //     if (!left)
    //     {
    //         ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   Right side gap");

    //         Eigen::Vector2f cp_inter_normal_vec(cp_inter_vec[1], -cp_inter_vec[0]);
    //         Eigen::Vector2f cp_new_inter_vec = scaledMinDim * cp_inter_normal_vec / cp_inter_normal_vec.norm() + cp_inter_vec;
    //         Eigen::Vector2f new_inter_vec = cp_new_inter_vec + q1;

    //         Eigen::Vector2f l_used = pFarSafe;
    //         Eigen::Vector2f pLeftSafe_normal_vec(-l_used[1], l_used[0]);
    //         Eigen::Vector2f l_new_inter_vec = scaledMinDim * pLeftSafe_normal_vec / pLeftSafe_normal_vec.norm() + l_used;

    //         ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Not within r side: " << q1[0] << " " << q1[1] << " " << pCloseSafe[0] << " " << pCloseSafe[1] << " " << new_inter_vec[0] << " " << new_inter_vec[1] << " " << l_used[0] << " " << l_used[1] << " " << l_new_inter_vec[0] << " " << l_new_inter_vec[1]);
            
    //         // Get intersection point p1 = (0, 0), p2 = l_new_inter_vec, p3 = cp, p4 = new_inter_vec
    //         float denominator = (0. - l_new_inter_vec[0]) * (q1[1] - new_inter_vec[1]) - (0. - l_new_inter_vec[1]) * (q1[0] - new_inter_vec[0]);
    //         float nom_x = 0. - (0. - l_new_inter_vec[0]) * (q1[0] * new_inter_vec[1] - q1[1] * new_inter_vec[0]);
    //         float nom_y = 0. - (0. - l_new_inter_vec[1]) * (q1[0] * new_inter_vec[1] - q1[1] * new_inter_vec[0]);

    //         bool has_inter = true;
    //         float inter_x, inter_y;
    //         if (denominator == 0)
    //             has_inter = false;
    //         else
    //         {
    //             inter_x = nom_x / denominator;
    //             inter_y = nom_y / denominator;
    //         }

    //         bool inter_left_to_cp = isLeftofLine(Eigen::Vector2f(0, 0), q1, Eigen::Vector2f(inter_x, inter_y));
    //         bool left_to_cp_line = isLeftofLine(Eigen::Vector2f(0, 0), q1, pGoal);
    //         bool left_to_inter_line = isLeftofLine(Eigen::Vector2f(inter_x, inter_y), Eigen::Vector2f(inter_x, inter_y) + q1, pGoal);
    //         bool left_to_cp_inter = isLeftofLine(q1, new_inter_vec, pGoal);
    //         bool left_to_new_inter = isLeftofLine(Eigen::Vector2f(0, 0), l_new_inter_vec, pGoal);

    //         if (!has_inter || (has_inter && inter_left_to_cp) || (has_inter && !inter_left_to_cp && left_to_inter_line))
    //         {
    //             if(left_to_cp_line || (!left_to_cp_line && left_to_cp_inter))
    //             {
    //                 Eigen::Vector2f goal_cp_vec = pGoal - q1;
    //                 q2 = goal_cp_vec.norm() * cp_new_inter_vec / cp_new_inter_vec.norm() + q1;
    //             } else if(!left_to_cp_line && !left_to_new_inter)
    //             {
    //                 q2 = pGoal.norm() * l_new_inter_vec / l_new_inter_vec.norm();
    //             } else if(!left_to_cp_line && left_to_new_inter && !left_to_cp_inter)
    //             {
    //                 q2 = pGoal;
    //             } else
    //             {
    //                 ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "Goal point is in the wrong region. [Orientation is not within gap r side, parallel]");
    //                 success = false;
    //             }
    //         } else if((has_inter && !inter_left_to_cp && !left_to_inter_line))
    //         {
    //             q2 = Eigen::Vector2f(inter_x, inter_y);
    //         } else
    //         {
    //             ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "Goal point is in the wrong region. [Orientation is not within gap r side, not desired]");
    //             success = false;
    //         }
            
    //     } else
    //     {
    //         ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   Left side gap");

    //         Eigen::Vector2f cp_inter_normal_vec(-cp_inter_vec[1], cp_inter_vec[0]);
    //         Eigen::Vector2f cp_new_inter_vec = scaledMinDim * cp_inter_normal_vec / cp_inter_normal_vec.norm() + cp_inter_vec;
    //         Eigen::Vector2f new_inter_vec = cp_new_inter_vec + q1;

    //         Eigen::Vector2f r_used = pFarSafe;
    //         Eigen::Vector2f r_inter_normal_vec(r_used[1], -r_used[0]);
    //         Eigen::Vector2f r_new_inter_vec = scaledMinDim * r_inter_normal_vec / r_inter_normal_vec.norm() + r_used;

    //         ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Not within l side: " << q1[0] << " " << q1[1] << " " << pCloseSafe[0] << " " << pCloseSafe[1] << " " << new_inter_vec[0] << " " << new_inter_vec[1] << " " << r_used[0] << " " << r_used[1] << " " << r_new_inter_vec[0] << " " << r_new_inter_vec[1]);

    //         // Get intersection point p1 = (0, 0), p2 = r_new_inter_vec, p3 = cp, p4 = new_inter_vec
    //         float denominator = (0. - r_new_inter_vec[0]) * (q1[1] - new_inter_vec[1]) - (0. - r_new_inter_vec[1]) * (q1[0] - new_inter_vec[0]);
    //         float nom_x = 0. - (0. - r_new_inter_vec[0]) * (q1[0] * new_inter_vec[1] - q1[1] * new_inter_vec[0]);
    //         float nom_y = 0. - (0. - r_new_inter_vec[1]) * (q1[0] * new_inter_vec[1] - q1[1] * new_inter_vec[0]);

    //         bool has_inter = true;
    //         float inter_x, inter_y;
    //         if (denominator == 0)
    //             has_inter = false;
    //         else
    //         {
    //             inter_x = nom_x / denominator;
    //             inter_y = nom_y / denominator;
    //         }

    //         bool inter_left_to_cp = isLeftofLine(Eigen::Vector2f(0, 0), q1, Eigen::Vector2f(inter_x, inter_y));
    //         bool left_to_cp_line = isLeftofLine(Eigen::Vector2f(0, 0), q1, pGoal);
    //         bool left_to_inter_line = isLeftofLine(Eigen::Vector2f(inter_x, inter_y), Eigen::Vector2f(inter_x, inter_y) + q1, pGoal);
    //         bool left_to_cp_inter = isLeftofLine(q1, new_inter_vec, pGoal);
    //         bool left_to_new_inter = isLeftofLine(Eigen::Vector2f(0, 0), r_new_inter_vec, pGoal);

    //         if (!has_inter || (has_inter && !inter_left_to_cp) || (has_inter && inter_left_to_cp && !left_to_inter_line))
    //         {
    //             if (!left_to_cp_line || (left_to_cp_line && !left_to_cp_inter))
    //             {
    //                 Eigen::Vector2f goal_cp_vec = pGoal - q1;
    //                 q2 = goal_cp_vec.norm() * cp_new_inter_vec / cp_new_inter_vec.norm() + q1;
    //             } else if (left_to_cp_line && left_to_new_inter)
    //             {
    //                 q2 = pGoal.norm() * r_new_inter_vec / r_new_inter_vec.norm();
    //             } else if (left_to_cp_line && !left_to_new_inter && left_to_cp_inter)
    //             {
    //                 q2 = pGoal;
    //             } else
    //             {
    //                 ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "Goal point is in the wrong region. [Orientation is not within gap l side, parallel]");
    //                 success = false;
    //             }
    //         } else if ((has_inter && inter_left_to_cp && left_to_inter_line))
    //         {
    //             q2 = Eigen::Vector2f(inter_x, inter_y);
    //         } else
    //         {
    //             ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "Goal point is in the wrong region. [Orientation is not within gap l side, not desired]");
    //             success = false;
    //         }
    //     }
    // }

    Trajectory GapTrajGenerator::generateBezierTrajectory(Gap * gap, 
                                                            const geometry_msgs::TwistStamped & rbtVelRbtFrame)
    {
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "[generateBezierTrajectory()]");
        geometry_msgs::PoseArray pathRbtFrame;
        pathRbtFrame.header.stamp = ros::Time::now();
        
        pathRbtFrame.header.frame_id = cfg_->robot_frame_id;

        // if (gap->goal.discard) 
        // {
        //     ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "This waypoint is discard.");
        //     return posearr;
        // }

        Bezier::Bezier<2> quadraBezier;
        bool success = findBezierControlPts(gap, quadraBezier, rbtVelRbtFrame);
        
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Bezier control points:");
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "P0: " << quadraBezier[0][0] << " " << quadraBezier[0][1]);
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "P1: " << quadraBezier[1][0] << " " << quadraBezier[1][1]);
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "P2: " << quadraBezier[2][0] << " " << quadraBezier[2][1]);

        if (!success)
        {
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "No path is generated.");
            Trajectory traj(pathRbtFrame);
            return traj;
        }
        else
        {
            if (!cfg_->traj.bezier_interp)
            {
                for (float t = 0; t <= 1; t+=0.02)
                {
                    geometry_msgs::Pose pose;
                    pose.position.x = quadraBezier.valueAt(t, 0);
                    pose.position.y = quadraBezier.valueAt(t, 1);
                    pathRbtFrame.poses.push_back(pose);
                }
                Trajectory traj(pathRbtFrame);
                return traj;            
            }
            else
            {
                // float des_dist = robotGeoProc_->getRobotAvgLinSpeed() * cfg_->traj.bezier_unit_time;
                float entire_dist = getBezierDist(quadraBezier, 0, 1, 30);
                // int num_sampled_pts = int(round(entire_dist / des_dist));
                // num_sampled_pts = num_sampled_pts >= 2 ? num_sampled_pts : 2;

                int num_sampled_pts = cfg_->traj.bezier_num_sampled_pts;

                float des_dist = entire_dist / num_sampled_pts;

                int steps = 5;
                float dist_thresh = des_dist / 10;
                float t_step = 1. / (num_sampled_pts - 1);
                float t_kmin1 = 0;
                for (size_t i = 0; i < num_sampled_pts; i++)
                {
                    float t_k = i * t_step;
                    float cur_dist = getBezierDist(quadraBezier, t_kmin1, t_k, steps);
                    if(abs(cur_dist - des_dist) < dist_thresh)
                    {
                        geometry_msgs::Pose pose;
                        pose.position.x = quadraBezier.valueAt(t_k, 0);
                        pose.position.y = quadraBezier.valueAt(t_k, 1);
                        pathRbtFrame.poses.push_back(pose);
                        t_kmin1 = t_k;
                    }
                    else if(cur_dist > des_dist)
                    {
                        float t_prev = (i - 1) * t_step;
                        float t_interp = (t_k + t_prev) / 2;
                        float interp_dist = getBezierDist(quadraBezier, t_kmin1, t_interp, steps);

                        float t_high = t_k;
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
                            interp_dist = getBezierDist(quadraBezier, t_kmin1, t_interp, steps);
                            if (abs(t_interp - t_low) <= 1e-3 && abs(t_interp - t_high) <= 1e-3)
                                break;
                            // ROS_INFO_STREAM(t_interp << " " << t_low << " " << t_high << " " << interp_dist << " " << abs(interp_dist - des_dist) << " " << dist_thresh);
                        }
                        // ROS_INFO_STREAM("exit");
                        geometry_msgs::Pose pose;
                        pose.position.x = quadraBezier.valueAt(t_interp, 0);
                        pose.position.y = quadraBezier.valueAt(t_interp, 1);
                        pathRbtFrame.poses.push_back(pose);
                        t_kmin1 = t_interp;
                    }
                }

                if (pathRbtFrame.poses.size() < num_sampled_pts)
                {
                    geometry_msgs::Pose pose;
                    pose.position.x = quadraBezier.valueAt(1, 0);
                    pose.position.y = quadraBezier.valueAt(1, 1);
                    pathRbtFrame.poses.push_back(pose);
                }

                Trajectory traj(pathRbtFrame);
                return traj;
            }
        }
    }

    Trajectory GapTrajGenerator::processTrajectory(const Trajectory & traj)
    {
        // geometry_msgs::PoseArray new_pose_arr;

        geometry_msgs::PoseArray rawPath = traj.getPathRbtFrame();
        
        geometry_msgs::PoseArray processedPath;
        processedPath.header = rawPath.header;

        geometry_msgs::Pose old_pose;
        old_pose.position.x = 0;
        old_pose.position.y = 0;
        old_pose.position.z = 0;
        old_pose.orientation.x = 0;
        old_pose.orientation.y = 0;
        old_pose.orientation.z = 0;
        old_pose.orientation.w = 1;
        float dx, dy, result;

        float delta = 0.05;
        geometry_msgs::Pose back_pose;
        // std::vector<geometry_msgs::Pose> shortened;
        processedPath.poses.push_back(old_pose);
        for (const geometry_msgs::Pose & pose : rawPath.poses)
        {
            back_pose = processedPath.poses.back();
            dx = pose.position.x - back_pose.position.x;
            dy = pose.position.y - back_pose.position.y;
            result = sqrt(pow(dx, 2) + pow(dy, 2));
            
            if (result > 0.05)
                processedPath.poses.push_back(pose);
        }

        // new_pose_arr.header = pose_arr.header;
        // new_pose_arr.poses = shortened;

        // Fix rotation
        Eigen::Quaternionf q;
        geometry_msgs::Pose new_pose;
        for (int idx = 1; idx < processedPath.poses.size(); idx++)
        {
            new_pose = processedPath.poses[idx];
            old_pose = processedPath.poses[idx - 1];
            
            dx = new_pose.position.x - old_pose.position.x;
            dy = new_pose.position.y - old_pose.position.y;
            result = std::atan2(dy, dx);
            
            q = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(result, Eigen::Vector3f::UnitZ());
            q.normalize();

            processedPath.poses[idx - 1].orientation.x = q.x();
            processedPath.poses[idx - 1].orientation.y = q.y();
            processedPath.poses[idx - 1].orientation.z = q.z();
            processedPath.poses[idx - 1].orientation.w = q.w();
        }
        processedPath.poses.pop_back();

        Trajectory processedTraj(processedPath);
        return processedTraj;
    }

    void GapTrajGenerator::getOrientDecayedPath(Trajectory & traj)
    {
        geometry_msgs::PoseArray rawPath = traj.getPathRbtFrame();

        // The original path should be in robot frame
        assert(rawPath.header.frame_id == cfg_->robot_frame_id);

        geometry_msgs::PoseArray orientedPath;

        if (rawPath.poses.size() <= 1)
        {
            ROS_WARN_STREAM("[getOrientDecayedPath] Original path is too short with size [ " << rawPath.poses.size() << " ].");
            orientedPath = rawPath;
            traj.setOrientedPathRbtFrame(orientedPath);
            return;
        }
        
        if (robotGeoProc_->robot_.shape == RobotShape::circle || !cfg_->planning.virtual_path_decay_enable)
        {
            orientedPath = rawPath;
        } else if (robotGeoProc_->robot_.shape == RobotShape::box)
        {
            orientedPath.header = rawPath.header;
            geometry_msgs::Pose first_pose = rawPath.poses[0];
            geometry_msgs::Quaternion init_quat;
            init_quat.w = 1;
            first_pose.orientation = init_quat;
            orientedPath.poses.push_back(first_pose);
            float length = 0;
            for (size_t i = 1; i < rawPath.poses.size(); i++)
            {
                if (!cfg_->planning.robot_path_orient_linear_decay)
                {
                    geometry_msgs::Pose curr_pose = rawPath.poses[i];
                    curr_pose.orientation = init_quat;
                    orientedPath.poses.push_back(curr_pose);
                } else
                {
                    geometry_msgs::Pose curr_pose = rawPath.poses[i];
                    geometry_msgs::Pose prev_pose = rawPath.poses[i-1];
                    float x_diff = curr_pose.position.x - prev_pose.position.x;
                    float y_diff = curr_pose.position.y - prev_pose.position.y;
                    float dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
                    length += dist;

                    float avg_speed = 0.2;
                    float t = length / avg_speed;
                    float avg_ang = cfg_->rbt.vang_absmax / cfg_->rbt.speed_factor;

                    Eigen::Quaternionf q(curr_pose.orientation.w, curr_pose.orientation.x, curr_pose.orientation.y, curr_pose.orientation.z);
                    Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
                    float ang_diff = std::abs(euler[2]);
                    float decayed_ang = avg_ang * t;
                    decayed_ang = decayed_ang <= ang_diff ? decayed_ang : ang_diff;
                    if (euler[2] <= 0)
                        decayed_ang = -decayed_ang;
                    
                    float roll = 0, pitch = 0;    
                    Eigen::Quaternionf e;
                    e = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
                        * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
                        * Eigen::AngleAxisf(decayed_ang, Eigen::Vector3f::UnitZ());
                    
                    curr_pose.orientation.w = e.w();
                    curr_pose.orientation.x = e.x();
                    curr_pose.orientation.y = e.y();
                    curr_pose.orientation.z = e.z();

                    orientedPath.poses.push_back(curr_pose);
                }
                
            }
        }
        else
        {
            ROS_WARN("Doesn't support robot shape, use original path.");
            orientedPath = rawPath;
        }

        traj.setOrientedPathRbtFrame(orientedPath);
        // Trajectory orientedTraj(orientedPath);
        // return orientedTrajgetOrientDecayedPath;

        return;
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

    // Eigen::Vector2f GapTrajGenerator::getRotatedVec(const Eigen::Vector2f & orig_vec, const float & chord_length, const bool & ccw)
    // {
    //     float r = orig_vec.norm();
    //     float rotate_angle = acos((2 * r * r - chord_length * chord_length) / (2 * r * r));

    //     Eigen::Matrix2f rotate_mat;
    //     if(ccw)
    //         rotate_mat << cos(rotate_angle), -sin(rotate_angle), sin(rotate_angle), cos(rotate_angle);
    //     else
    //         rotate_mat << cos(rotate_angle), sin(rotate_angle), -sin(rotate_angle), cos(rotate_angle);

    //     Eigen::Vector2f rotated_vec = rotate_mat * orig_vec;

    //     return rotated_vec;
    // }

}