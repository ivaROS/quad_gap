#include <quad_gap/trajectory_generation/GapGoalPlacer.h>

namespace quad_gap 
{
    void GapGoalPlacer::updateEgoCircle(std::shared_ptr<sensor_msgs::msg::LaserScan const> msg) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        scan_ = msg;
        // num_of_scan = (int)(scan_.get()->ranges.size());
    }

    void GapGoalPlacer::setGapWaypoint(Gap * gap, const geometry_msgs::msg::PoseStamped & globalPathLocalWaypointRobotFrame)
    {
        // // RCLCPP_INFO_STREAM(logger_,  "[setGapWaypoint()]");

        // TODO: assume there is no idx that will pass 0
        float xLeft, xRight, yLeft, yRight;

        int idxLeft = gap->manipLeftIdx();
        int idxRight = gap->manipRightIdx();
        float leftRange = gap->manipLeftRange();
        float rightRange = gap->manipRightRange();

        Eigen::Vector2f pLeft = gap->getManipLPosition(); // (xLeft, yLeft);
        Eigen::Vector2f pRight = gap->getManipRPosition(); // (xRight, yRight);
        Eigen::Vector2f pGoal(globalPathLocalWaypointRobotFrame.pose.position.x, 
                                globalPathLocalWaypointRobotFrame.pose.position.y);

        xLeft = pLeft[0];     // (gap->convex.leftRange_) * cos(idx2theta(gap->convex.leftIdx_));
        yLeft = pLeft[1];         // (gap->convex.leftRange_) * sin(idx2theta(gap->convex.leftIdx_));
        xRight = pRight[0];            // (gap->convex.rightRange_) * cos(idx2theta(gap->convex.rightIdx_));
        yRight = pRight[1];            // (gap->convex.rightRange_) * sin(idx2theta(gap->convex.rightIdx_));
        
        // if agc. then the shorter side need to be further in
        
        // Get the equivalent passing length
        // Eigen::Vector2f orient_vec(1, 0);
        // Eigen::Vector2f pMid = 0.5 * (pLeft + pRight);
        // float epl = robotGeoProc_.getDecayEquivalentPL(orient_vec, pMid, pMid.norm());
        // float epl = robotGeoProc_->getLinearDecayEquivalentPL(pMid);
        
        // Eigen::Vector2f lr = (pLeft - pRight) / (pLeft - pRight).norm() * (epl / 2) * cfg_->traj.inf_ratio + pRight;
        // float thetaRight = car2pol(lr)(1);
        // if (pRight[1] >= 0 && lr[1] < 0 && pRight[0] <= 0 && lr[0] < 0)
        //     thetaRight = thetaRight + 2 * M_PI;
        
        // Eigen::Vector2f rl = (pRight - pLeft) / (pRight - pLeft).norm() * (epl / 2) * cfg_->traj.inf_ratio + pLeft;
        // float thetaLeft = car2pol(rl)(1);
        // if (pLeft[1] <= 0 && rl[1] > 0 && pLeft[0] <= 0 && rl[0] < 0)
        //     thetaLeft = thetaLeft - 2 * M_PI;
        
        float thetaLeft = idx2theta(idxLeft);
        float thetaRight = idx2theta(idxRight); 
        float thetaGoal = std::atan2(pGoal[1], pGoal[0]);
        int idxGoal = theta2idx(thetaGoal);

        // Second condition: if angle smaller than M_PI / 3
        // Check if arc length < 3 robot width
        float leftToRightAngle = getSweptLeftToRightAngle(pLeft, pRight);

        bool smallGap = (leftToRightAngle < M_PI && 
                            sqrt(pow(xLeft - xRight, 2) + pow(yLeft - yRight, 2)) < 4 * cfg_->rbt.r_inscr);

        // // RCLCPP_INFO_STREAM(logger_,  gap->mode.reduced << " " << gap->convex.rightIdx_ << " " << gap->convex.leftIdx_ << " " << pRight[0] << " " << pRight[1] << " " << pLeft[0] << " " << pLeft[1] << " " << thetaLeft << " " << thetaRight);

        // thetaLeft < thetaRight || 

        if (smallGap) 
        {
            float leftToRightAngle = getSweptLeftToRightAngle(pLeft, pRight);
                
            float thetaLeft = std::atan2(pLeft[1], pLeft[0]);

            // // RCLCPP_INFO_STREAM(logger_,  "leftToRightAngle: " << leftToRightAngle);
            float thetaCenter = (thetaLeft - 0.5 * leftToRightAngle); 
            float rangeCenter = 0.5 * (pLeft.norm() + pRight.norm());
            Eigen::Vector2f centerGoal(rangeCenter * std::cos(thetaCenter), rangeCenter * std::sin(thetaCenter));
            // // RCLCPP_INFO_STREAM(logger_,  "thetaLeft: " << thetaLeft << ", thetaRight: " << thetaRight << ", thetaCenter: " << thetaCenter);

            gap->setGoalPos(centerGoal[0], centerGoal[1]);

            // // RCLCPP_INFO_STREAM(logger_,  "        Option 1: small gap");
            // // RCLCPP_INFO_STREAM(logger_,  "            goal: " << centerGoal[0] << ", " << centerGoal[1]);

            return;
        }

        if (checkWaypointVisibility(pGoal) // pLeft, pRight, 
            && isGlobalPathLocalWaypointWithinGapAngle(idxGoal, idxRight, idxLeft)) 
        {
            // // RCLCPP_INFO_STREAM(logger_,  "Goal is visible, setting goal within gap");
            gap->setGoalPos(pGoal[0], pGoal[1]);
            // // RCLCPP_INFO_STREAM(logger_,  "Goal set to: " << gap->getGoalX() << ", " << gap->getGoalY());
            // gap->goal.x = localgoal.pose.position.x;
            // gap->goal.y = localgoal.pose.position.y;
            // gap->goal.set = true;
            // gap->goal.goalwithin = true;
            gap->setGoalWithin(); // will use later during trajectory generation
            return;
        }

        // // RCLCPP_INFO_STREAM(logger_,  "Biasing goal position");
        
        float globalPathLocalWaypointTheta = std::atan2(pGoal[1], pGoal[0]);
        
        float leftToWaypointAngle = getSweptLeftToRightAngle(pLeft, pGoal);
        float rightToWaypointAngle = getSweptLeftToRightAngle(pRight, pGoal);

        float biasedGapGoalTheta = setBiasedGapGoalTheta(thetaLeft, thetaRight, globalPathLocalWaypointTheta,
                                                            leftToRightAngle, leftToWaypointAngle, rightToWaypointAngle);
        Eigen::Vector2f biasedGapGoalUnitNorm(std::cos(biasedGapGoalTheta), std::sin(biasedGapGoalTheta));

        float leftToGapGoalAngle = getSweptLeftToRightAngle(pLeft, biasedGapGoalUnitNorm); 

        float biasedGapGoalRange = leftRange + (rightRange - leftRange) * leftToGapGoalAngle / leftToRightAngle;

        Eigen::Vector2f biasedGapGoal(biasedGapGoalRange * cos(biasedGapGoalTheta), biasedGapGoalRange * sin(biasedGapGoalTheta));

        // float confined_theta = std::min(thetaLeft, std::max(thetaRight, globalPathLocalWaypointTheta));
        // float confined_r = (gap->manipLeftRange() - gap->manipRightRange()) * (confined_theta - thetaRight) / (thetaLeft - thetaRight)
        //                     + gap->manipRightRange();

        // float xg = confined_r * cos(confined_theta);
        // float yg = confined_r * sin(confined_theta);
        // Eigen::Vector2f anchor(xg, yg);
        // Eigen::Matrix2f r_negpi2;
        //     r_negpi2 << 0,1,-1,0;
        // auto offset = r_negpi2 * (pLeft - pRight);
        // auto goal_pt = offset / offset.norm() * (epl / 2) * cfg_->traj.inf_ratio + anchor;
        // Eigen::Vector2f goal_pt;
        // float waypoint_dist_thresh = (epl * 1.5) * cfg_->traj.inf_ratio; // 0.1 // TODO: change this 1.5 to param
        
        // if ((goal_orientation - thetaRight) > 0 && 
        //     (goal_orientation - thetaLeft) < 0 && 
        //     (anchor - pRight).norm() >= waypoint_dist_thresh && 
        //     (anchor - pLeft).norm() >= waypoint_dist_thresh)
        // {
        //     goal_pt = anchor;
        // }
        // else
        // {
        //     // Eigen::Vector2f mid_pt = (pRight + pLeft) / 2;
        //     float thetaMid = atan2(pMid[1], pMid[0]);
        //     float rangeMid = (pMid - pRight).norm();

        //     float ang_anchor_pRight = abs(goal_orientation - thetaRight);
        //     float ang_anchor_pLeft = abs(goal_orientation - thetaLeft);

        //     if (ang_anchor_pRight <= ang_anchor_pLeft)
        //     {
        //         Eigen::Vector2f offset_anchor = waypoint_dist_thresh * (pLeft - pRight) / (pLeft - pRight).norm() + anchor;
        //         float offset_anchor_angle = atan2(offset_anchor[1], offset_anchor[0]);
        //         if (offset_anchor_angle < thetaMid)
        //         {
        //             goal_pt = offset_anchor;
        //         } else
        //         {
        //             goal_pt = pMid;
        //         }
        //     }
        //     else
        //     {
        //         Eigen::Vector2f offset_anchor = waypoint_dist_thresh * (pRight - pLeft) / (pRight - pLeft).norm() + anchor;
        //         float offset_anchor_angle = atan2(offset_anchor[1], offset_anchor[0]);
        //         if (offset_anchor_angle > thetaMid)
        //         {
        //             goal_pt = offset_anchor;
        //         } else
        //         {
        //             goal_pt = pMid;
        //         }
        //     }
        // }
        // Eigen::Matrix2f r_negpi2;
        // r_negpi2 << 0,1,-1,0;
        
        // Eigen::Vector2f offset = r_negpi2 * (pLeft - pRight);
        // goal_pt += robotGeoProc_->getRobotMaxRadius() * offset / offset.norm();

        // gap->goal.x = goal_pt(0);
        // gap->goal.y = goal_pt(1);
        // gap->goal.set = true;
        gap->setGoalPos(biasedGapGoal(0), biasedGapGoal(1));
        // // RCLCPP_INFO_STREAM(logger_,  "Goal set to: " << gap->getGoalX() << ", " << gap->getGoalY());
        return;
    }

    float GapGoalPlacer::setBiasedGapGoalTheta(const float & leftTheta, const float & rightTheta, const float & globalGoalTheta,
                                                const float & leftToRightAngle, const float & leftToWaypointAngle,  const float & rightToWaypointAngle)
    {
        float biasedGapGoalTheta = 0.0;
        if (leftTheta > rightTheta) // gap is not behind robot
        { 
            biasedGapGoalTheta = std::min(leftTheta, std::max(rightTheta, globalGoalTheta));
        } else // gap is behind
        { 
            if (0 < leftToWaypointAngle && leftToWaypointAngle < leftToRightAngle)
                biasedGapGoalTheta = globalGoalTheta;
            else if (std::abs(leftToWaypointAngle) < std::abs(rightToWaypointAngle))
                biasedGapGoalTheta = leftTheta;
            else
                biasedGapGoalTheta = rightTheta;
        }

        // ROS_INFO_STREAM("            leftTheta: " << leftTheta << ", rightTheta: " << rightTheta << ", globalGoalTheta: " << globalGoalTheta);
        // ROS_INFO_STREAM("            leftToRightAngle: " << leftToRightAngle << ", leftToWaypointAngle: " << leftToWaypointAngle << ", rightToWaypointAngle: " << rightToWaypointAngle);

        return biasedGapGoalTheta;
    }

    // const Eigen::Vector2f & pLeft, 
    // const Eigen::Vector2f & pRight,
                                                    
    bool GapGoalPlacer::checkWaypointVisibility(const Eigen::Vector2f & globalGoal) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        // with robot as 0,0 (globalGoal in robot frame as well)
        float dist2goal = globalGoal.norm(); // sqrt(pow(globalGoal.pose.position.x, 2) + pow(globalGoal.pose.position.y, 2));

        sensor_msgs::msg::LaserScan scan = *scan_.get();
        auto minScanRange = *std::min_element(scan.ranges.begin(), scan.ranges.end());

        // If sufficiently close to robot
        if (dist2goal < 2 * cfg_->rbt.r_inscr)
            return true;

        // If within closest configuration space
        if (dist2goal < minScanRange - cfg_->traj.inf_ratio * cfg_->rbt.r_inscr)
            return true;

        // Should be sufficiently far, otherwise we are in trouble
        float globalGoalAngle = std::atan2(globalGoal[1], globalGoal[0]);
        int globalGoalIdx = theta2idx(globalGoalAngle);

        // Should be sufficiently far, otherwise we are in trouble

        // get gap's range at globalGoal idx

        // float leftToRightAngle = getSweptLeftToRightAngle(pLeft, pRight);
        // float leftToWaypointAngle = getSweptLeftToRightAngle(pLeft, globalGoal);
        // float gapGoapRightange = (pRight.norm() - pLeft.norm()) * epsilonDivide(leftToWaypointAngle, leftToRightAngle) + pLeft.norm();

        float rangeAtGoalIdx = scan.ranges.at(globalGoalIdx);

        return dist2goal < rangeAtGoalIdx;
    }

    Eigen::Vector2f GapGoalPlacer::car2pol(const Eigen::Vector2f & a) 
    {
        return Eigen::Vector2f(a.norm(), float(std::atan2(a(1), a(0))));
    }    
}