#include <quad_gap/gap_manip.h>

namespace quad_gap {
    void GapManipulator::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_) {
        boost::mutex::scoped_lock lock(egolock);
        msg = msg_;
        num_of_scan = (int)(msg.get()->ranges.size());
    }

    void GapManipulator::setGapWaypoint(quad_gap::Gap& gap, geometry_msgs::PoseStamped localgoal){
        // TODO: assume there is no idx that will pass 0
        auto half_num_scan = gap.half_scan;
        float x1, x2, y1, y2;
        x1 = (gap.convex.convex_ldist) * cos(-((float) half_num_scan - gap.convex.convex_lidx) / half_num_scan * M_PI);
        y1 = (gap.convex.convex_ldist) * sin(-((float) half_num_scan - gap.convex.convex_lidx) / half_num_scan * M_PI);

        x2 = (gap.convex.convex_rdist) * cos(-((float) half_num_scan - gap.convex.convex_ridx) / half_num_scan * M_PI);
        y2 = (gap.convex.convex_rdist) * sin(-((float) half_num_scan - gap.convex.convex_ridx) / half_num_scan * M_PI);

        Eigen::Vector2f pl(x1, y1);
        Eigen::Vector2f pr(x2, y2);
        
        // if agc. then the shorter side need to be further in
        
        // Get the equivalent passing length
        Eigen::Vector2d orient_vec(1, 0);
        Eigen::Vector2d m_pt_vec = (pl.cast<double>() + pr.cast<double>()) / 2;
        // double epl = robot_geo_proc_.getDecayEquivalentPL(orient_vec, m_pt_vec, m_pt_vec.norm());
        double epl = robot_geo_proc_.getLinearDecayEquivalentPL(orient_vec, m_pt_vec, m_pt_vec.norm());
        auto lr = (pr - pl) / (pr - pl).norm() * (epl / 2) * cfg_->traj.inf_ratio + pl;
        auto thetalr = car2pol(lr)(1);
        if(pl[1] >= 0 && lr[1] < 0 && pl[0] <= 0 && lr[0] < 0)
            thetalr = thetalr + 2 * M_PI;
        auto rl = (pl - pr) / (pl - pr).norm() * (epl / 2) * cfg_->traj.inf_ratio + pr;
        auto thetarl = car2pol(rl)(1);
        if(pr[1] <= 0 && rl[1] > 0 && pr[0] <= 0 && rl[0] < 0)
            thetarl = thetarl - 2 * M_PI;
        
        auto left_ori = gap.convex.convex_lidx * msg.get()->angle_increment + msg.get()->angle_min;
        auto right_ori = gap.convex.convex_ridx * msg.get()->angle_increment + msg.get()->angle_min;

        // Second condition: if angle smaller than M_PI / 3
        // Check if arc length < 3 robot width
        bool gap_size_check = (right_ori - left_ori) < M_PI;
        float dist = 0;
        bool small_gap = false;
        if (gap_size_check) //  && !cfg_->planning.planning_inflated
        {
            // if smaller than M_PI/3
            dist = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
            small_gap = dist < 2 * epl;
        }

        // ROS_INFO_STREAM(gap.mode.reduced << " " << gap.convex.convex_lidx << " " << gap.convex.convex_ridx << " " << pl[0] << " " << pl[1] << " " << pr[0] << " " << pr[1] << " " << thetarl << " " << thetalr);

        if (thetarl < thetalr || small_gap) {
            gap.goal.x = (x1 + x2) / 2;
            gap.goal.y = (y1 + y2) / 2;
            gap.goal.discard = thetarl < thetalr;
            gap.goal.set = true;
            return;
        }
        
        float goal_orientation = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        float confined_theta = std::min(thetarl, std::max(thetalr, goal_orientation));
        float confined_r = (gap.convex.convex_rdist - gap.convex.convex_ldist) * (confined_theta - thetalr) / (thetarl - thetalr)
            + gap.convex.convex_ldist;
        float xg = confined_r * cos(confined_theta);
        float yg = confined_r * sin(confined_theta);
        Eigen::Vector2f anchor(xg, yg);
        // Eigen::Matrix2f r_negpi2;
        //     r_negpi2 << 0,1,-1,0;
        // auto offset = r_negpi2 * (pr - pl);
        // auto goal_pt = offset / offset.norm() * (epl / 2) * cfg_->traj.inf_ratio + anchor;
        Eigen::Vector2f goal_pt;
        float waypoint_dist_thresh = (epl * 1.5) * cfg_->traj.inf_ratio; // 0.1 // TODO: change this 1.5 to param
        
        if((goal_orientation - thetalr) > 0 && (goal_orientation - thetarl) < 0 && (anchor - lr).norm() >= waypoint_dist_thresh && (anchor - rl).norm() >= waypoint_dist_thresh)
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

            if(ang_anchor_lr <= ang_anchor_rl)
            {
                Eigen::Vector2f offset_anchor = waypoint_dist_thresh * (rl - lr) / (rl - lr).norm() + anchor;
                float offset_anchor_angle = atan2(offset_anchor[1], offset_anchor[0]);
                if(offset_anchor_angle < mid_pt_angle)
                {
                    goal_pt = offset_anchor;
                }
                else
                {
                    goal_pt = mid_pt;
                }
            }
            else
            {
                Eigen::Vector2f offset_anchor = waypoint_dist_thresh * (lr - rl) / (lr - rl).norm() + anchor;
                float offset_anchor_angle = atan2(offset_anchor[1], offset_anchor[0]);
                if(offset_anchor_angle > mid_pt_angle)
                {
                    goal_pt = offset_anchor;
                }
                else
                {
                    goal_pt = mid_pt;
                }
            }
        }
        Eigen::Matrix2f r_negpi2;
            r_negpi2 << 0,1,-1,0;
        auto offset = r_negpi2 * (pr - pl);
        goal_pt += robot_geo_proc_.getRobotMaxRadius() * offset / offset.norm();

        // ROS_INFO_STREAM("l gap [" << pl[0] << " , " << pl[1] << "], r gap [" << pr[0] << " , " << pr[1] << "], thetalr: " << thetalr << " thetarl: " << thetarl << " goal orient: " << goal_orientation << " Anchor [" << anchor[0] << " , " << anchor[1] << "], Waypoint [" << goal_pt[0] << " , " << goal_pt[1] << "]");
        // float half_max_r = robot_geo_proc_.getRobotMaxRadius() / 2;
        // auto goal_pt = offset * half_max_r * cfg_->traj.inf_ratio + anchor;

        // float r1 = gap.convex.convex_ldist;
        // float r2 = gap.convex.convex_rdist;
        // double r_close = (double) std::min(r1, r2);
        // double goal_dist = sqrt(
        //     pow(localgoal.pose.position.y, 2) + 
        //     pow(localgoal.pose.position.x, 2)
        // );

        if (checkGoalVisibility(localgoal)) {
            gap.goal.x = localgoal.pose.position.x;
            gap.goal.y = localgoal.pose.position.y;
            gap.goal.set = true;
            gap.goal.goalwithin = true;
            return;
        }


        gap.goal.x = goal_pt(0);
        gap.goal.y = goal_pt(1);
        gap.goal.set = true;

    }

    bool GapManipulator::checkGoalVisibility(geometry_msgs::PoseStamped localgoal) {
        boost::mutex::scoped_lock lock(egolock);
        double dist2goal = sqrt(pow(localgoal.pose.position.x, 2) + pow(localgoal.pose.position.y, 2));

        auto scan = *msg.get();
        auto min_val = *std::min_element(scan.ranges.begin(), scan.ranges.end());

        // If sufficiently close to robot
        Eigen::Vector2d orient_vec(1, 0);
        Eigen::Vector2d goal_vec(localgoal.pose.position.x, localgoal.pose.position.y);
        double er = robot_geo_proc_.getRobotMaxRadius();
        if (dist2goal < 2 * er) {
            return true;
        }

        // If within closest configuration space
        double er_max = robot_geo_proc_.getRobotMaxRadius(); //TODO: check
        if (dist2goal < min_val - cfg_->traj.inf_ratio * er_max) {
            return true;
        }

        // Should be sufficiently far, otherwise we are in trouble
        double goal_angle = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        int incident_angle = (int) round((goal_angle - scan.angle_min) / scan.angle_increment);

        // double half_angle = std::asin(cfg_->rbt.r_inscr / dist2goal);
        // int index = std::ceil(half_angle / scan.angle_increment) * 1.5;
        int index = (int)(scan.ranges.size()) / 8;
        int lower_bound = std::max(incident_angle - index, 0);
        int upper_bound = std::min(incident_angle + index, int(scan.ranges.size() - 1));
        auto min_val_round_goal = *std::min_element(scan.ranges.begin() + lower_bound, scan.ranges.begin() + upper_bound);
        return dist2goal < min_val_round_goal;
    }

    // In place modification
    void GapManipulator::reduceGap(quad_gap::Gap& gap, geometry_msgs::PoseStamped localgoal) {
        int lidx = gap.LIdx();
        int ridx = gap.RIdx();
        if (!msg) return; 
        double angular_size = (ridx - lidx) * (msg.get()->angle_increment);

        if (angular_size < cfg_->gap_manip.reduction_threshold){
            return;
        }

        int gap_size = cfg_->gap_manip.reduction_target / msg.get()->angle_increment;
        int l_biased_r = lidx + gap_size;
        int r_biased_l = ridx - gap_size;

        double goal_orientation = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        int goal_idx = goal_orientation / (M_PI / (num_of_scan / 2)) + (num_of_scan / 2);

        int acceptable_dist = int(round(gap_size / 2));

        int new_l, new_r;
        if (goal_idx + acceptable_dist > ridx){
            // r-biased Gap
            new_r = ridx;
            new_l = r_biased_l;
        } else if (goal_idx - acceptable_dist < lidx) {
            // l-biased gap
            new_r = l_biased_r;
            new_l = lidx;
        } else {
            // Lingering in center
            new_l = goal_idx - acceptable_dist;
            new_r = goal_idx + acceptable_dist;
        }

        // ROS_INFO_STREAM(lidx << " " << ridx << " " << l_biased_r << " " << r_biased_l << " " << goal_idx + acceptable_dist << " " << goal_idx - acceptable_dist << " " << new_l << " " << new_r);

        float ldist = gap.LDist();
        float rdist = gap.RDist();
        float new_ldist = float(new_l - lidx) / float(ridx - lidx) * (rdist - ldist) + ldist;
        float new_rdist = float(new_r - lidx) / float(ridx - lidx) * (rdist - ldist) + ldist;
        gap.convex.convex_lidx = new_l;
        gap.convex.convex_ridx = new_r;
        gap.convex.convex_ldist = new_ldist + cfg_->gap_viz.viz_jitter;
        gap.convex.convex_rdist = new_rdist + cfg_->gap_viz.viz_jitter;
        gap.life_time = 50;
        gap.mode.reduced = true;
        return;
    }

    void GapManipulator::convertAxialGap(quad_gap::Gap& gap) {
        // Return if not axial gap or disabled
        if (!gap.isAxial() || !cfg_->gap_manip.axial_convert) {
            // ROS_INFO_STREAM("Swept gap.");
            return;
        }

        auto stored_scan_msgs = *msg.get();
        
        bool left = gap.isLeftType();
        // Extend of rotation to the radial gap 
        // amp-ed by a **small** ratio to ensure the local goal does not exactly fall on the
        // visibility line
        // float rot_val = (float) std::atan2(cfg_->gap_manip.epsilon2 * cfg_->gap_manip.rot_ratio, cfg_->gap_manip.epsilon1);

        auto half_num_scan = gap.half_scan;

        int l_idx, r_idx;
        float l_dist, r_dist;
        if(gap.mode.reduced)
        {
            l_idx = gap.convex.convex_lidx;
            l_dist = gap.convex.convex_ldist;
            r_idx = gap.convex.convex_ridx;
            r_dist = gap.convex.convex_rdist;
        }
        else
        {
            l_idx = gap.LIdx();
            l_dist = gap.LDist();
            r_idx = gap.RIdx();
            r_dist = gap.RDist();
        }

        float x1, x2, y1, y2;

        x1 = (l_dist) * cos(-((float) half_num_scan - l_idx) / half_num_scan * M_PI);
        y1 = (l_dist) * sin(-((float) half_num_scan - l_idx) / half_num_scan * M_PI);

        x2 = (r_dist) * cos(-((float) half_num_scan - r_idx) / half_num_scan * M_PI);
        y2 = (r_dist) * sin(-((float) half_num_scan - r_idx) / half_num_scan * M_PI);

        Eigen::Vector2d l_vec(x1, y1);
        Eigen::Vector2d r_vec(x2, y2);
        Eigen::Vector2d mid = (l_vec + r_vec) / 2;
        Eigen::Vector2d robot_orient(1,0);
        float robot_el = float(robot_geo_proc_.getLinearDecayEquivalentPL(robot_orient, mid, mid.norm()));
        float robot_er = float(robot_geo_proc_.getLinearDecayEquivalentRL(robot_orient, mid, mid.norm()));
        
        float rot_val = (float) std::atan2(robot_el / 2 * cfg_->gap_manip.rot_ratio, robot_er / 2);
        float theta = left ? (rot_val + 1e-3): -(rot_val + 1e-3);
        int near_idx, far_idx;
        float near_dist, far_dist;
        
        if (left) {
            // near_idx = gap.LIdx();
            // far_idx = gap.RIdx();
            // near_dist = gap.LDist();
            // far_dist = gap.RDist();
            near_idx = l_idx;
            far_idx = r_idx;
            near_dist = l_dist;
            far_dist = r_dist;
        } else {
            far_idx = l_idx;
            near_idx = r_idx;
            far_dist = l_dist;
            near_dist = r_dist;
        }
        
        Eigen::Matrix3f rot_mat;
        rot_mat << cos(theta), -sin(theta), 0,
                    sin(theta), cos(theta), 0,
                    0, 0, 1;

        
        
        Eigen::Matrix3f near_rbt;
        near_rbt << 1, 0, near_dist * cos(M_PI / half_num_scan * (near_idx - half_num_scan)),
                    0, 1, near_dist * sin(M_PI / half_num_scan * (near_idx - half_num_scan)),
                    0, 0, 1;
        Eigen::Matrix3f far_rbt;
        far_rbt  << 1, 0, far_dist * cos(M_PI / half_num_scan * (far_idx - half_num_scan)),
                    0, 1, far_dist * sin(M_PI / half_num_scan * (far_idx - half_num_scan)),
                    0, 0, 1;
        
        Eigen::Matrix3f rot_rbt = near_rbt * (rot_mat * (near_rbt.inverse() * far_rbt));

        float r = float(sqrt(pow(rot_rbt(0, 2), 2) + pow(rot_rbt(1, 2), 2)));
        int idx = int (std::atan2(rot_rbt(1, 2), rot_rbt(0, 2)) / M_PI * half_num_scan) + half_num_scan;

        // Rotation Completed
        // Get minimum dist range val from start to target index location
        // For wraparound
        int offset = left ? gap._right_idx : idx;
        int upperbound = left ? idx : gap._left_idx;
        int intermediate_pt = offset + 1;
        int second_inter_pt = intermediate_pt;
        int size = upperbound - offset;


        if ((upperbound - offset) < 3) {
            // Arbitrary value
            gap.goal.discard = true;
            return;
        }

        offset = std::max(offset, 0);
        upperbound = std::min(upperbound, num_of_scan - 1);
        std::vector<float> min_dist(upperbound - offset);

        if (size == 0) {
            // This shouldn't happen
            return;
        }


        if (stored_scan_msgs.ranges.size() < 500) {
            ROS_FATAL_STREAM("Scan range incorrect gap manip");
        }

        try{
            for (int i = 0; i < min_dist.size(); i++) {
                min_dist.at(i) = sqrt(pow(near_dist, 2) + pow(stored_scan_msgs.ranges.at(i + offset), 2) -
                    2 * near_dist * stored_scan_msgs.ranges.at(i + offset) * cos((i + offset - near_idx) * stored_scan_msgs.angle_increment));
            }
        } catch(...) {
            ROS_FATAL_STREAM("convertAxialGap outofBound");
        }

        auto farside_iter = std::min_element(min_dist.begin(), min_dist.end());
        float farside = *farside_iter;

        Eigen::Matrix3f far_near = near_rbt.inverse() * far_rbt;
        float coefs = far_near.block<2, 1>(0, 2).norm();
        // ROS_INFO_STREAM()
        far_near(0, 2) *= farside / coefs;
        far_near(1, 2) *= farside / coefs;
        Eigen::Matrix3f short_pt = near_rbt * (rot_mat * far_near);

        r = float(sqrt(pow(short_pt(0, 2), 2) + pow(short_pt(1, 2), 2)));
        idx = int (std::atan2(short_pt(1, 2), short_pt(0, 2)) / M_PI * half_num_scan) + half_num_scan;


        // Recalculate end point location based on length
        gap.convex.convex_lidx = left ? near_idx : idx;
        gap.convex.convex_ldist = left ? near_dist : r;
        gap.convex.convex_ridx = left ? idx : near_idx;
        gap.convex.convex_rdist = left ? r : near_dist;

        if (left && gap.convex.convex_ridx < gap.convex.convex_lidx) {
            gap.goal.discard = true;
        }

        if (!left && gap.convex.convex_ridx < gap.convex.convex_lidx) {
            gap.goal.discard = true;
        }

        gap.mode.agc = true;
    }

    void GapManipulator::radialExtendGap(quad_gap::Gap& selected_gap) {
        if (!cfg_->gap_manip.radial_extend) {
            ROS_DEBUG_STREAM_THROTTLE(1, "Radial Extension is off");
            return;
        }
        // TODO: check if the idx are correct when they cross the 0.
        int half_num_scan = (int)(msg.get()->ranges.size()) / 2;
        float s = selected_gap.getMinSafeDist();

        float x1, x2, y1, y2;
        x1 = (selected_gap.convex.convex_ldist) * cos(-((float) half_num_scan - selected_gap.convex.convex_lidx) / half_num_scan * M_PI);
        y1 = (selected_gap.convex.convex_ldist) * sin(-((float) half_num_scan - selected_gap.convex.convex_lidx) / half_num_scan * M_PI);

        x2 = (selected_gap.convex.convex_rdist) * cos(-((float) half_num_scan - selected_gap.convex.convex_ridx) / half_num_scan * M_PI);
        y2 = (selected_gap.convex.convex_rdist) * sin(-((float) half_num_scan - selected_gap.convex.convex_ridx) / half_num_scan * M_PI);

        Eigen::Vector2f gL(x1, y1);
        Eigen::Vector2f gR(x2, y2);

        Eigen::Vector2f eL = gL / gL.norm();
        Eigen::Vector2f eR = gR / gR.norm();

        Eigen::Vector2f eB = (eL + eR) / 2;
        eB /= eB.norm();
        float gap_size = std::acos(eL.dot(eR));

        Eigen::Vector2f qB = -s * eB;
        
        // Shifted Back Frame
        Eigen::Vector2f qLp = gL - qB;
        Eigen::Vector2f qRp = gR - qB;

        Eigen::Vector2f pLp = car2pol(qLp);
        // pLp(1) += M_PI;
        Eigen::Vector2f pRp = car2pol(qRp);
        // pRp(1) += M_PI;

        float phiB = pRp(1) - pLp(1);

        Eigen::Vector2f pB = car2pol(-qB);
        // pB(2) += M_PI;

        float thL = pB(1) - gap_size / 4;
        float thR = pB(1) + gap_size / 4;

        Eigen::Vector2f pLn = pTheta(thL, phiB, pLp, pRp);
        Eigen::Vector2f pRn = pTheta(thR, phiB, pLp, pRp);

        Eigen::Vector2f qLn = pol2car(pLn) + qB;
        Eigen::Vector2f qRn = pol2car(pRn) + qB;

        // Store info back to original gap;
        Eigen::Vector2f polqLn = car2pol(qLn);
        Eigen::Vector2f polqRn = car2pol(qRn);

        selected_gap.convex.convex_lidx = polqLn(1) / M_PI * half_num_scan + half_num_scan;
        selected_gap.convex.convex_ridx = polqRn(1) / M_PI * half_num_scan + half_num_scan;
        selected_gap.convex.convex_ldist = polqLn(0);
        selected_gap.convex.convex_rdist = polqRn(0);
        selected_gap.mode.convex = true;

        selected_gap.qB = qB;
        ROS_DEBUG_STREAM("l: " << selected_gap._left_idx << " to " << selected_gap.convex.convex_lidx
         << ", r: " << selected_gap._right_idx << " to " << selected_gap.convex.convex_ridx);
        
        ROS_DEBUG_STREAM("ldist: " << selected_gap._ldist << " to " << selected_gap.convex.convex_ldist
        << ", rdist: " << selected_gap._rdist << " to " << selected_gap.convex.convex_rdist);

        return;
    }

    Eigen::Vector2f GapManipulator::car2pol(Eigen::Vector2f a) {
        return Eigen::Vector2f(a.norm(), float(std::atan2(a(1), a(0))));
    }

    Eigen::Vector2f GapManipulator::pol2car(Eigen::Vector2f a) {
        return Eigen::Vector2f(cos(a(1)) * a(0), sin(a(1)) * a(0));
    }

    Eigen::Vector2f GapManipulator::pTheta(
        float th, float phiB, Eigen::Vector2f pRp, Eigen::Vector2f pLp) {
        return pLp * (th - pRp(1)) / phiB + pRp * (pLp(1) - th) / phiB;
    }

}
