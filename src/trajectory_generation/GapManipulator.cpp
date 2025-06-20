#include <quad_gap/trajectory_generation/GapManipulator.h>

namespace quad_gap 
{
    void GapManipulator::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg) 
    {
        boost::mutex::scoped_lock lock(egolock);
        scan_ = msg;
        num_of_scan = (int)(scan_.get()->ranges.size());
    }

    // In place modification
    void GapManipulator::reduceGap(Gap * gap, const geometry_msgs::PoseStamped & localgoal) 
    {
        ROS_INFO_STREAM_NAMED("GapManipulator", "    [reduceGap()]");

        int left_idx = gap->LIdx();
        int right_idx = gap->RIdx();

        // ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-RGC gap in polar. left: (" << leftIdx << ", " << leftRange << "), right: (" << rightIdx << ", " << rightRange << ")");
        // ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-AGC gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");    

        
        if (!scan_) 
            return; 

        float angular_size = (left_idx - right_idx) * (scan_.get()->angle_increment);

        if (angular_size < cfg_->gap_manip.reduction_threshold)
        {
            return;
        }

        int gap_size = cfg_->gap_manip.reduction_target / scan_.get()->angle_increment;
        int l_biased_r = right_idx + gap_size;
        int r_biased_l = left_idx - gap_size;

        float goal_orientation = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        int goal_idx = goal_orientation / (M_PI / (num_of_scan / 2)) + (num_of_scan / 2);

        int acceptable_dist = int(round(gap_size / 2));

        int new_l, new_r;
        if (goal_idx + acceptable_dist > left_idx)
        {
            // r-biased Gap
            new_r = left_idx;
            new_l = r_biased_l;
        } else if (goal_idx - acceptable_dist < right_idx) 
        {
            // l-biased gap
            new_r = l_biased_r;
            new_l = right_idx;
        } else 
        {
            // Lingering in center
            new_l = goal_idx - acceptable_dist;
            new_r = goal_idx + acceptable_dist;
        }

        // ROS_INFO_STREAM(right_idx << " " << left_idx << " " << l_biased_r << " " << r_biased_l << " " << goal_idx + acceptable_dist << " " << goal_idx - acceptable_dist << " " << new_l << " " << new_r);

        float left_dist = gap->LRange();
        float right_dist = gap->RRange();
        float newLeftRange = float(new_r - right_idx) / float(left_idx - right_idx) * (left_dist - right_dist) + right_dist;
        float newRightRange = float(new_l - right_idx) / float(left_idx - right_idx) * (left_dist - right_dist) + right_dist;

        gap->convex.convexLeftIdx_ = new_r;
        gap->convex.convexRightIdx_ = new_l;

        gap->convex.convexLeftDist_ = newLeftRange;
        gap->convex.convexRightDist_ = newRightRange;

        gap->life_time = 50;
        gap->mode.reduced = true;
        return;
    }

    void GapManipulator::convertAxialGap(Gap * gap) 
    {
        // Return if not radial gap or disabled
        if (!gap->isRadial() || !cfg_->gap_manip.radial_convert) 
        {
            // ROS_INFO_STREAM("Swept gap.");
            return;
        }

        sensor_msgs::LaserScan stored_scan_msgs = *scan_.get();
        
        bool left = gap->isRightType();
        // Extend of rotation to the radial gap 
        // amp-ed by a **small** ratio to ensure the local goal does not exactly fall on the
        // visibility line

        int right_idx, left_idx;
        float right_dist, left_dist;
        if (gap->mode.reduced)
        {
            left_idx = gap->convex.convexLeftIdx_;
            left_dist = gap->convex.convexLeftDist_;
            right_idx = gap->convex.convexRightIdx_;
            right_dist = gap->convex.convexRightDist_;
        } else
        {
            left_idx = gap->LIdx();
            left_dist = gap->LRange();
            right_idx = gap->RIdx();
            right_dist = gap->RRange();
        }

        float x1, x2, y1, y2;

        x1 = (right_dist) * cos(idx2theta(right_idx));
        y1 = (right_dist) * sin(idx2theta(right_idx));

        x2 = (left_dist) * cos(idx2theta(left_idx));
        y2 = (left_dist) * sin(idx2theta(left_idx));

        Eigen::Vector2f l_vec(x1, y1);
        Eigen::Vector2f r_vec(x2, y2);
        Eigen::Vector2f mid = (l_vec + r_vec) / 2;
        Eigen::Vector2f robot_orient(1,0);
        float robot_el = float(robot_geo_proc_.getLinearDecayEquivalentPL(robot_orient, mid, mid.norm()));
        float robot_er = float(robot_geo_proc_.getLinearDecayEquivalentRL(robot_orient, mid, mid.norm()));
        
        float rot_val = (float) std::atan2(robot_el / 2 * cfg_->gap_manip.rot_ratio, robot_er / 2);
        float signed_rot_vel = left ? (rot_val + 1e-3): -(rot_val + 1e-3);
        int near_idx, far_idx;
        float near_dist, far_dist;
        
        if (left) 
        {
            // near_idx = gap->RIdx();
            // far_idx = gap->LIdx();
            // near_dist = gap->RRange();
            // far_dist = gap->LRange();
            near_idx = right_idx;
            far_idx = left_idx;
            near_dist = right_dist;
            far_dist = left_dist;
        } else 
        {
            far_idx = right_idx;
            near_idx = left_idx;
            far_dist = right_dist;
            near_dist = left_dist;
        }
        
        Eigen::Matrix3f rot_mat;
        rot_mat << cos(signed_rot_vel), -sin(signed_rot_vel), 0,
                    sin(signed_rot_vel), cos(signed_rot_vel), 0,
                    0, 0, 1;

        Eigen::Matrix3f near_rbt;
        near_rbt << 1, 0, near_dist * cos(idx2theta(near_idx)),
                    0, 1, near_dist * sin(idx2theta(near_idx)),
                    0, 0, 1;
        Eigen::Matrix3f far_rbt;
        far_rbt  << 1, 0, far_dist * cos(idx2theta(far_idx)),
                    0, 1, far_dist * sin(idx2theta(far_idx)),
                    0, 0, 1;
        
        Eigen::Matrix3f rot_rbt = near_rbt * (rot_mat * (near_rbt.inverse() * far_rbt));

        float r = float(sqrt(pow(rot_rbt(0, 2), 2) + pow(rot_rbt(1, 2), 2)));
        float theta = std::atan2(rot_rbt(1, 2), rot_rbt(0, 2));
        int idx = theta2idx(theta);

        // Rotation Completed
        // Get minimum dist range val from start to target index location
        // For wraparound
        int offset = left ? gap->LIdx() : idx;
        int upperbound = left ? idx : gap->RIdx();
        int intermediate_pt = offset + 1;
        int second_inter_pt = intermediate_pt;
        int size = upperbound - offset;


        if ((upperbound - offset) < 3) {
            // Arbitrary value
            gap->goal.discard = true;
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

        // auto farside_iter = ;
        float farside = *std::min_element(min_dist.begin(), min_dist.end());

        Eigen::Matrix3f far_near = near_rbt.inverse() * far_rbt;
        float coefs = far_near.block<2, 1>(0, 2).norm();
        // ROS_INFO_STREAM()
        far_near(0, 2) *= farside / coefs;
        far_near(1, 2) *= farside / coefs;
        Eigen::Matrix3f short_pt = near_rbt * (rot_mat * far_near);

        r = float(sqrt(pow(short_pt(0, 2), 2) + pow(short_pt(1, 2), 2)));
        theta = std::atan2(short_pt(1, 2), short_pt(0, 2));
        idx = theta2idx(theta);

        // Recalculate end point location based on length
        gap->convex.convexRightIdx_ = left ? near_idx : idx;
        gap->convex.convexRightDist_ = left ? near_dist : r;
        gap->convex.convexLeftIdx_ = left ? idx : near_idx;
        gap->convex.convexLeftDist_ = left ? r : near_dist;

        if (left && gap->convex.convexLeftIdx_ < gap->convex.convexRightIdx_) 
        {
            gap->goal.discard = true;
        }

        if (!left && gap->convex.convexLeftIdx_ < gap->convex.convexRightIdx_) 
        {
            gap->goal.discard = true;
        }

        gap->mode.agc = true;
    }

    void GapManipulator::radialExtendGap(Gap * selected_gap) 
    {
        if (!cfg_->gap_manip.radial_extend) 
        {
            ROS_DEBUG_STREAM_THROTTLE(1, "Radial Extension is off");
            return;
        }
        // TODO: check if the idx are correct when they cross the 0.

        float s = selected_gap->getMinSafeDist();

        float x1, x2, y1, y2;
        x1 = (selected_gap->convex.convexRightDist_) * cos(idx2theta(selected_gap->convex.convexRightIdx_));
        y1 = (selected_gap->convex.convexRightDist_) * sin(idx2theta(selected_gap->convex.convexRightIdx_));

        x2 = (selected_gap->convex.convexLeftDist_) * cos(idx2theta(selected_gap->convex.convexLeftIdx_));
        y2 = (selected_gap->convex.convexLeftDist_) * sin(idx2theta(selected_gap->convex.convexLeftIdx_));

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

        selected_gap->convex.convexRightIdx_ = theta2idx(polqLn(1));
        selected_gap->convex.convexLeftIdx_ = theta2idx(polqRn(1));
        selected_gap->convex.convexRightDist_ = polqLn(0);
        selected_gap->convex.convexLeftDist_ = polqRn(0);
        selected_gap->mode.convex = true;

        selected_gap->qB = qB;
        ROS_DEBUG_STREAM("l: " << selected_gap->RIdx() << " to " << selected_gap->convex.convexRightIdx_
         << ", r: " << selected_gap->LIdx() << " to " << selected_gap->convex.convexLeftIdx_);
        
        ROS_DEBUG_STREAM("right_dist: " << selected_gap->RRange() << " to " << selected_gap->convex.convexRightDist_
        << ", left_dist: " << selected_gap->LRange() << " to " << selected_gap->convex.convexLeftDist_);

        return;
    }

    Eigen::Vector2f GapManipulator::car2pol(const Eigen::Vector2f & a) 
    {
        return Eigen::Vector2f(a.norm(), float(std::atan2(a(1), a(0))));
    }

    Eigen::Vector2f GapManipulator::pol2car(const Eigen::Vector2f & a) 
    {
        return Eigen::Vector2f(cos(a(1)) * a(0), sin(a(1)) * a(0));
    }

    Eigen::Vector2f GapManipulator::pTheta(const float & th, const float & phiB, 
                                            const Eigen::Vector2f & pRp, const Eigen::Vector2f & pLp) 
    {
        return pLp * (th - pRp(1)) / phiB + pRp * (pLp(1) - th) / phiB;
    }

}
