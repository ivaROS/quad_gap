#include <quad_gap/trajectory_generation/GapGoalPlacer.h>

namespace quad_gap 
{
    void GapGoalPlacer::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg) 
    {
        boost::mutex::scoped_lock lock(egolock);
        scan_ = msg;
        num_of_scan = (int)(scan_.get()->ranges.size());
    }

    void GapGoalPlacer::setGapWaypoint(Gap * gap, const geometry_msgs::PoseStamped & localgoal)
    {
        // TODO: assume there is no idx that will pass 0
        float xLeft, xRight, yLeft, yRight;

        Eigen::Vector2f pLeft = gap->getManipLCartesian(); // (xLeft, yLeft);
        Eigen::Vector2f pRight = gap->getManipRCartesian(); // (xRight, yRight);

        xLeft = pLeft[0];     // (gap->convex.leftRange_) * cos(idx2theta(gap->convex.leftIdx_));
        yLeft = pLeft[1];         // (gap->convex.leftRange_) * sin(idx2theta(gap->convex.leftIdx_));
        xRight = pRight[0];            // (gap->convex.rightRange_) * cos(idx2theta(gap->convex.rightIdx_));
        yRight = pRight[1];            // (gap->convex.rightRange_) * sin(idx2theta(gap->convex.rightIdx_));
        
        // if agc. then the shorter side need to be further in
        
        // Get the equivalent passing length
        Eigen::Vector2f orient_vec(1, 0);
        Eigen::Vector2f m_pt_vec = (pRight.cast<float>() + pLeft.cast<float>()) / 2;
        // float epl = robot_geo_proc_.getDecayEquivalentPL(orient_vec, m_pt_vec, m_pt_vec.norm());
        float epl = robot_geo_proc_.getLinearDecayEquivalentPL(orient_vec, m_pt_vec, m_pt_vec.norm());
        
        Eigen::Vector2f lr = (pLeft - pRight) / (pLeft - pRight).norm() * (epl / 2) * cfg_->traj.inf_ratio + pRight;
        float thetalr = car2pol(lr)(1);
        if(pRight[1] >= 0 && lr[1] < 0 && pRight[0] <= 0 && lr[0] < 0)
            thetalr = thetalr + 2 * M_PI;
        
        Eigen::Vector2f rl = (pRight - pLeft) / (pRight - pLeft).norm() * (epl / 2) * cfg_->traj.inf_ratio + pLeft;
        float thetarl = car2pol(rl)(1);
        if (pLeft[1] <= 0 && rl[1] > 0 && pLeft[0] <= 0 && rl[0] < 0)
            thetarl = thetarl - 2 * M_PI;
        
        float left_ori = idx2theta(gap->manipLeftIdx());
        float right_ori = idx2theta(gap->manipRightIdx()); 

        // Second condition: if angle smaller than M_PI / 3
        // Check if arc length < 3 robot width
        bool gap_size_check = (right_ori - left_ori) < M_PI;
        float dist = 0;
        bool small_gap = false;
        if (gap_size_check) //  && !cfg_->planning.planning_inflated
        {
            // if smaller than M_PI/3
            dist = sqrt(pow(xLeft - xRight, 2) + pow(yLeft - yRight, 2));
            small_gap = dist < 2 * epl;
        }

        // ROS_INFO_STREAM(gap->mode.reduced << " " << gap->convex.rightIdx_ << " " << gap->convex.leftIdx_ << " " << pRight[0] << " " << pRight[1] << " " << pLeft[0] << " " << pLeft[1] << " " << thetarl << " " << thetalr);

        // thetarl < thetalr || 

        if (small_gap) 
        {
            gap->setGoalPos((xLeft + xRight) / 2, (yLeft + yRight) / 2);
            // gap->goal.x = (xLeft + xRight) / 2;
            // gap->goal.y = (yLeft + yRight) / 2;
            // gap->goal.discard = thetarl < thetalr;
            // gap->goal.set = true;
            return;
        }
        
        float goal_orientation = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        float confined_theta = std::min(thetarl, std::max(thetalr, goal_orientation));
        float confined_r = (gap->manipLeftRange() - gap->manipRightRange()) * (confined_theta - thetalr) / (thetarl - thetalr)
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
        
        if ((goal_orientation - thetalr) > 0 && (goal_orientation - thetarl) < 0 && (anchor - lr).norm() >= waypoint_dist_thresh && (anchor - rl).norm() >= waypoint_dist_thresh)
        {
            goal_pt = anchor;
        }
        else
        {
            Eigen::Vector2f mid_pt = (lr + rl) / 2;
            float mid_pt_angle = atan2(mid_pt[1], mid_pt[0]);
            float mid_pt_side_length = (mid_pt - lr).norm();

            float ang_anchor_lr = abs(goal_orientation - thetalr);
            float ang_anchor_rl = abs(goal_orientation - thetarl);

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
        goal_pt += robot_geo_proc_.getRobotMaxRadius() * offset / offset.norm();

        // ROS_INFO_STREAM("l gap [" << pRight[0] << " , " << pRight[1] << "], r gap [" << pLeft[0] << " , " << pLeft[1] << "], thetalr: " << thetalr << " thetarl: " << thetarl << " goal orient: " << goal_orientation << " Anchor [" << anchor[0] << " , " << anchor[1] << "], Waypoint [" << goal_pt[0] << " , " << goal_pt[1] << "]");
        // float half_max_r = robot_geo_proc_.getRobotMaxRadius() / 2;
        // auto goal_pt = offset * half_max_r * cfg_->traj.inf_ratio + anchor;

        // float r1 = gap->convex.rightRange_;
        // float r2 = gap->convex.leftRange_;
        // float r_close = (float) std::min(r1, r2);
        // float goal_dist = sqrt(
        //     pow(localgoal.pose.position.y, 2) + 
        //     pow(localgoal.pose.position.x, 2)
        // );

        if (checkGoalVisibility(localgoal)) 
        {
            gap->setGoalPos(localgoal.pose.position.x, localgoal.pose.position.y);
            // gap->goal.x = localgoal.pose.position.x;
            // gap->goal.y = localgoal.pose.position.y;
            // gap->goal.set = true;
            // gap->goal.goalwithin = true;
            gap->setGoalWithin();
            return;
        }


        // gap->goal.x = goal_pt(0);
        // gap->goal.y = goal_pt(1);
        // gap->goal.set = true;
        gap->setGoalPos(goal_pt(0), goal_pt(1));

    }

    bool GapGoalPlacer::checkGoalVisibility(const geometry_msgs::PoseStamped & localgoal) 
    {
        boost::mutex::scoped_lock lock(egolock);
        float dist2goal = sqrt(pow(localgoal.pose.position.x, 2) + pow(localgoal.pose.position.y, 2));

        sensor_msgs::LaserScan scan = *scan_.get();
        float min_val = *std::min_element(scan.ranges.begin(), scan.ranges.end());

        // If sufficiently close to robot
        Eigen::Vector2f orient_vec(1, 0);
        Eigen::Vector2f goal_vec(localgoal.pose.position.x, localgoal.pose.position.y);
        float er = robot_geo_proc_.getRobotMaxRadius();
        if (dist2goal < 2 * er) {
            return true;
        }

        // If within closest configuration space
        float er_max = robot_geo_proc_.getRobotMaxRadius(); //TODO: check
        if (dist2goal < min_val - cfg_->traj.inf_ratio * er_max) {
            return true;
        }

        // Should be sufficiently far, otherwise we are in trouble
        float goal_angle = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        int incident_angle = (int) round((goal_angle - scan.angle_min) / scan.angle_increment);

        // float half_angle = std::asin(cfg_->rbt.r_inscr / dist2goal);
        // int index = std::ceil(half_angle / scan.angle_increment) * 1.5;
        int index = (int)(scan.ranges.size()) / 8;
        int lower_bound = std::max(incident_angle - index, 0);
        int upper_bound = std::min(incident_angle + index, int(scan.ranges.size() - 1));
        float min_val_round_goal = *std::min_element(scan.ranges.begin() + lower_bound, scan.ranges.begin() + upper_bound);
        return dist2goal < min_val_round_goal;
    }

    Eigen::Vector2f GapGoalPlacer::car2pol(const Eigen::Vector2f & a) 
    {
        return Eigen::Vector2f(a.norm(), float(std::atan2(a(1), a(0))));
    }    
}