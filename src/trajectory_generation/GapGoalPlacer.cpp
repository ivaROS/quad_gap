#include <quad_gap/trajectory_generation/GapGoalPlacer.h>

namespace quad_gap 
{
    void GapGoalPlacer::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        scan_ = msg;
        // num_of_scan = (int)(scan_.get()->ranges.size());
    }

    void GapGoalPlacer::setGapWaypoint(Gap * gap, const geometry_msgs::PoseStamped & globalPathLocalWaypoint)
    {
        ROS_INFO_STREAM_NAMED("GapGoalPlacer", "[setGapWaypoint()]");

        // TODO: assume there is no idx that will pass 0
        float xLeft, xRight, yLeft, yRight;

        Eigen::Vector2f pLeft = gap->getManipLCartesian(); // (xLeft, yLeft);
        Eigen::Vector2f pRight = gap->getManipRCartesian(); // (xRight, yRight);
        Eigen::Vector2f pGoal(globalPathLocalWaypoint.pose.position.x, globalPathLocalWaypoint.pose.position.y);

        xLeft = pLeft[0];     // (gap->convex.leftRange_) * cos(idx2theta(gap->convex.leftIdx_));
        yLeft = pLeft[1];         // (gap->convex.leftRange_) * sin(idx2theta(gap->convex.leftIdx_));
        xRight = pRight[0];            // (gap->convex.rightRange_) * cos(idx2theta(gap->convex.rightIdx_));
        yRight = pRight[1];            // (gap->convex.rightRange_) * sin(idx2theta(gap->convex.rightIdx_));
        
        // if agc. then the shorter side need to be further in
        
        // Get the equivalent passing length
        // Eigen::Vector2f orient_vec(1, 0);
        Eigen::Vector2f pMid = (pLeft + pRight) / 2;
        // float epl = robotGeoProc_.getDecayEquivalentPL(orient_vec, pMid, pMid.norm());
        float epl = robotGeoProc_->getLinearDecayEquivalentPL(robotOrientationVector, pMid, pMid.norm());
        
        // Eigen::Vector2f lr = (pLeft - pRight) / (pLeft - pRight).norm() * (epl / 2) * cfg_->traj.inf_ratio + pRight;
        // float thetaRight = car2pol(lr)(1);
        // if (pRight[1] >= 0 && lr[1] < 0 && pRight[0] <= 0 && lr[0] < 0)
        //     thetaRight = thetaRight + 2 * M_PI;
        
        // Eigen::Vector2f rl = (pRight - pLeft) / (pRight - pLeft).norm() * (epl / 2) * cfg_->traj.inf_ratio + pLeft;
        // float thetaLeft = car2pol(rl)(1);
        // if (pLeft[1] <= 0 && rl[1] > 0 && pLeft[0] <= 0 && rl[0] < 0)
        //     thetaLeft = thetaLeft - 2 * M_PI;
        
        float thetaLeft = idx2theta(gap->manipLeftIdx());
        float thetaRight = idx2theta(gap->manipRightIdx()); 

        // Second condition: if angle smaller than M_PI / 3
        // Check if arc length < 3 robot width
        float leftToRightAngle = getSweptLeftToRightAngle(leftPt, rightPt);

        bool smallGap = (leftToRightAngle < M_PI && sqrt(pow(xLeft - xRight, 2) + pow(yLeft - yRight, 2)) < 4 * cfg_->rbt.r_inscr);
        // float dist = 0;
        // bool small_gap = false;
        // if (gap_size_check) //  && !cfg_->planning.planning_inflated
        // {
        //     // if smaller than M_PI/3
        //     dist = sqrt(pow(xLeft - xRight, 2) + pow(yLeft - yRight, 2));
        //     small_gap = dist < 2 * epl;
        // }

        // ROS_INFO_STREAM(gap->mode.reduced << " " << gap->convex.rightIdx_ << " " << gap->convex.leftIdx_ << " " << pRight[0] << " " << pRight[1] << " " << pLeft[0] << " " << pLeft[1] << " " << thetaLeft << " " << thetaRight);

        // thetaLeft < thetaRight || 

        if (smallGap) 
        {
            float leftToRightAngle = getSweptLeftToRightAngle(leftPt, rightPt);
                
            float thetaLeft = std::atan2(leftPt[1], leftPt[0]);

            // ROS_INFO_STREAM_NAMED("GapManipulator", "leftToRightAngle: " << leftToRightAngle);
            float thetaCenter = (thetaLeft - 0.5 * leftToRightAngle); 
            float rangeCenter = (leftPt.norm() + rightPt.norm()) / 2.0;
            Eigen::Vector2f centerGoal(rangeCenter * std::cos(thetaCenter), rangeCenter * std::sin(thetaCenter));
            // ROS_INFO_STREAM_NAMED("GapManipulator", "thetaLeft: " << thetaLeft << ", thetaRight: " << thetaRight << ", thetaCenter: " << thetaCenter);

            gap->setGoalPos(centerGoal[0], centerGoal[1]);

            ROS_INFO_STREAM_NAMED("GapManipulator", "        Option 1: small gap");
            ROS_INFO_STREAM_NAMED("GapManipulator", "            goal: " << centerGoal[0] << ", " << centerGoal[1]);

            return;
        }

        // ROS_INFO_STREAM("l gap [" << pRight[0] << " , " << pRight[1] << "], r gap [" << pLeft[0] << " , " << pLeft[1] << "], thetaRight: " << thetaRight << " thetaLeft: " << thetaLeft << " goal orient: " << goal_orientation << " Anchor [" << anchor[0] << " , " << anchor[1] << "], Waypoint [" << goal_pt[0] << " , " << goal_pt[1] << "]");
        // float half_max_r = robotGeoProc_.getRobotMaxRadius() / 2;
        // auto goal_pt = offset * half_max_r * cfg_->traj.inf_ratio + anchor;

        // float r1 = gap->convex.rightRange_;
        // float r2 = gap->convex.leftRange_;
        // float r_close = (float) std::min(r1, r2);
        // float goal_dist = sqrt(
        //     pow(localgoal.pose.position.y, 2) + 
        //     pow(localgoal.pose.position.x, 2)
        // );

        if (checkWaypointVisibility(pLeft, pRight, pGoal)) 
        {
            ROS_INFO_STREAM_NAMED("GapGoalPlacer", "Goal is visible, setting goal within gap");
            gap->setGoalPos(pGoal[0], pGoal[1]);
            ROS_INFO_STREAM_NAMED("GapGoalPlacer", "Goal set to: " << gap->getGoalX() << ", " << gap->getGoalY());
            // gap->goal.x = localgoal.pose.position.x;
            // gap->goal.y = localgoal.pose.position.y;
            // gap->goal.set = true;
            // gap->goal.goalwithin = true;
            gap->setGoalWithin();
            return;
        }

        ROS_INFO_STREAM_NAMED("GapGoalPlacer", "Biasing goal position");
        
        float goal_orientation = std::atan2(pGoal[1], pGoal[0]);
        float confined_theta = std::min(thetaLeft, std::max(thetaRight, goal_orientation));
        float confined_r = (gap->manipLeftRange() - gap->manipRightRange()) * (confined_theta - thetaRight) / (thetaLeft - thetaRight)
                            + gap->manipRightRange();

        float xg = confined_r * cos(confined_theta);
        float yg = confined_r * sin(confined_theta);
        Eigen::Vector2f anchor(xg, yg);
        // Eigen::Matrix2f r_negpi2;
        //     r_negpi2 << 0,1,-1,0;
        // auto offset = r_negpi2 * (pLeft - pRight);
        // auto goal_pt = offset / offset.norm() * (epl / 2) * cfg_->traj.inf_ratio + anchor;
        Eigen::Vector2f goal_pt;
        float waypoint_dist_thresh = (epl * 1.5) * cfg_->traj.inf_ratio; // 0.1 // TODO: change this 1.5 to param
        
        if ((goal_orientation - thetaRight) > 0 && (goal_orientation - thetaLeft) < 0 && (anchor - lr).norm() >= waypoint_dist_thresh && (anchor - rl).norm() >= waypoint_dist_thresh)
        {
            goal_pt = anchor;
        }
        else
        {
            Eigen::Vector2f mid_pt = (lr + rl) / 2;
            float mid_pt_angle = atan2(mid_pt[1], mid_pt[0]);
            float mid_pt_side_length = (mid_pt - lr).norm();

            float ang_anchor_lr = abs(goal_orientation - thetaRight);
            float ang_anchor_rl = abs(goal_orientation - thetaLeft);

            if (ang_anchor_lr <= ang_anchor_rl)
            {
                Eigen::Vector2f offset_anchor = waypoint_dist_thresh * (rl - lr) / (rl - lr).norm() + anchor;
                float offset_anchor_angle = atan2(offset_anchor[1], offset_anchor[0]);
                if (offset_anchor_angle < mid_pt_angle)
                {
                    goal_pt = offset_anchor;
                } else
                {
                    goal_pt = mid_pt;
                }
            }
            else
            {
                Eigen::Vector2f offset_anchor = waypoint_dist_thresh * (lr - rl) / (lr - rl).norm() + anchor;
                float offset_anchor_angle = atan2(offset_anchor[1], offset_anchor[0]);
                if (offset_anchor_angle > mid_pt_angle)
                {
                    goal_pt = offset_anchor;
                } else
                {
                    goal_pt = mid_pt;
                }
            }
        }
        Eigen::Matrix2f r_negpi2;
        r_negpi2 << 0,1,-1,0;
        
        Eigen::Vector2f offset = r_negpi2 * (pLeft - pRight);
        goal_pt += robotGeoProc_->getRobotMaxRadius() * offset / offset.norm();

        // gap->goal.x = goal_pt(0);
        // gap->goal.y = goal_pt(1);
        // gap->goal.set = true;
        gap->setGoalPos(goal_pt(0), goal_pt(1));
        ROS_INFO_STREAM_NAMED("GapGoalPlacer", "Goal set to: " << gap->getGoalX() << ", " << gap->getGoalY());
        return;
    }

    bool GapGoalPlacer::checkWaypointVisibility(const Eigen::Vector2f & leftPt, 
                                                const Eigen::Vector2f & rightPt,
                                                const Eigen::Vector2f & globalGoal) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        // with robot as 0,0 (globalGoal in robot frame as well)
        float dist2goal = globalGoal.norm(); // sqrt(pow(globalGoal.pose.position.x, 2) + pow(globalGoal.pose.position.y, 2));

        sensor_msgs::LaserScan scan = *scan_.get();
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

        // float leftToRightAngle = getSweptLeftToRightAngle(leftPt, rightPt);
        // float leftToWaypointAngle = getSweptLeftToRightAngle(leftPt, globalGoal);
        // float gapGoalRange = (rightPt.norm() - leftPt.norm()) * epsilonDivide(leftToWaypointAngle, leftToRightAngle) + leftPt.norm();

        float rangeAtGoalIdx = scan.ranges.at(globalGoalIdx);

        return dist2goal < rangeAtGoalIdx;
    }

    Eigen::Vector2f GapGoalPlacer::car2pol(const Eigen::Vector2f & a) 
    {
        return Eigen::Vector2f(a.norm(), float(std::atan2(a(1), a(0))));
    }    
}