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

        int leftIdx = gap->manipLeftIdx();
        float leftRange = gap->manipLeftRange();
        int rightIdx = gap->manipRightIdx();
        float rightRange = gap->manipRightRange();

        float leftTheta = idx2theta(leftIdx);
        float rightTheta = idx2theta(rightIdx);
        float xLeft = leftRange * cos(leftTheta);
        float yLeft = leftRange * sin(leftTheta);
        float xRight = rightRange * cos(rightTheta);
        float yRight = rightRange * sin(rightTheta);

        ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-reduce gap in polar. left: (" << leftIdx << ", " << leftRange << "), right: (" << rightIdx << ", " << rightRange << ")");
        ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-reduce gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");    

        if (!scan_) 
            return; 

        float angularSize = (leftIdx - rightIdx) * (scan_.get()->angle_increment);

        if (angularSize < cfg_->gap_manip.reduction_threshold)
        {
            return;
        }

        int gap_size = cfg_->gap_manip.reduction_target / scan_.get()->angle_increment;
        int leftBiasedRightIdx = rightIdx + gap_size;
        int rightBiasedLeftIdx = leftIdx - gap_size;

        float goal_orientation = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        int goal_idx = theta2idx(goal_orientation);

        int acceptable_dist = int(round(gap_size / 2));

        int newLeftIdx, newRightIdx;
        if (goal_idx + acceptable_dist > leftIdx)
        {
            // r-biased Gap
            newLeftIdx = leftIdx;
            newRightIdx = rightBiasedLeftIdx;
        } else if (goal_idx - acceptable_dist < rightIdx) 
        {
            // l-biased gap
            newLeftIdx = leftBiasedRightIdx;
            newRightIdx = rightIdx;
        } else 
        {
            // Lingering in center
            newLeftIdx = goal_idx + acceptable_dist;
            newRightIdx = goal_idx - acceptable_dist;
        }

        // ROS_INFO_STREAM(rightIdx << " " << leftIdx << " " << leftBiasedRightIdx << " " << rightBiasedLeftIdx << " " << goal_idx + acceptable_dist << " " << goal_idx - acceptable_dist << " " << newRightIdx << " " << newLeftIdx);

        float newLeftRange = float(newLeftIdx - rightIdx) / float(leftIdx - rightIdx) * (leftRange - rightRange) + rightRange;
        float newRightRange = float(newRightIdx - rightIdx) / float(leftIdx - rightIdx) * (leftRange - rightRange) + rightRange;

        // gap->convex.leftIdx_ = newLeftIdx;
        // gap->convex.rightIdx_ = newRightIdx;

        // gap->convex.leftRange_ = newLeftRange;
        // gap->convex.rightRange_ = newRightRange;
        gap->setManipPoints(newLeftIdx, newLeftRange, newRightIdx, newRightRange);

        float newLeftTheta = idx2theta(newLeftIdx);
        float newRightTheta = idx2theta(newRightIdx);
        float newXLeft = (newLeftRange) * cos(newLeftTheta);
        float newYLeft = (newLeftRange) * sin(newLeftTheta);
        float newXRight = (newRightRange) * cos(newRightTheta);
        float newYRight = (newRightRange) * sin(newRightTheta);

        ROS_INFO_STREAM_NAMED("GapManipulator", "        post-reduce gap in polar. left: (" << newLeftIdx << ", " << newLeftRange << "), right: (" << newRightIdx << ", " << newRightRange << ")");
        ROS_INFO_STREAM_NAMED("GapManipulator", "        post-reduce gap in cart. left: (" << newXLeft << ", " << newYLeft << "), right: (" << newXRight << ", " << newYRight << ")");    

        gap->setReduced();
        return;
    }

    void GapManipulator::convertAxialGap(Gap * gap) 
    {
        ROS_INFO_STREAM_NAMED("GapManipulator", "    [convertRadialGap()]");

        // Return if not radial gap or disabled
        if (!gap->isRadial() || !cfg_->gap_manip.radial_convert) 
        {
            // ROS_INFO_STREAM("Swept gap.");
            return;
        }

        sensor_msgs::LaserScan stored_scan_msgs = *scan_.get();
        
        bool right = gap->isRightType();
        // Extend of rotation to the radial gap 
        // amp-ed by a **small** ratio to ensure the local goal does not exactly fall on the
        // visibility line

        int leftIdx = gap->manipLeftIdx();
        int rightIdx = gap->manipRightIdx();
        float leftRange = gap->manipLeftRange();
        float rightRange = gap->manipRightRange();

        float leftTheta = idx2theta(leftIdx);
        float rightTheta = idx2theta(rightIdx);
        float xLeft = leftRange * cos(leftTheta);
        float yLeft = leftRange * sin(leftTheta);
        float xRight = rightRange * cos(rightTheta);
        float yRight = rightRange * sin(rightTheta);

        ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-RGC gap in polar. left: (" << leftIdx << ", " << leftRange << "), right: (" << rightIdx << ", " << rightRange << ")");
        ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-RGC gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");

        Eigen::Vector2f leftPt(xLeft, yLeft);
        Eigen::Vector2f rightPt(xRight, yRight);

        Eigen::Vector2f mid = (rightPt + leftPt) / 2;
        Eigen::Vector2f robot_orient(1,0);
        float robot_el = float(robot_geo_proc_.getLinearDecayEquivalentPL(robot_orient, mid, mid.norm()));
        float robot_er = float(robot_geo_proc_.getLinearDecayEquivalentRL(robot_orient, mid, mid.norm()));
        
        float nomPivotAngle = (float) std::atan2(robot_el / 2 * cfg_->gap_manip.rot_ratio, robot_er / 2);
        int nearIdx = 0.0, farIdx = 0.0;
        float nearRange = 0.0, farRange = 0.0;
        float signedNomPivotAngle = 0.0; //  = right ? (rot_val + 1e-3): -(rot_val + 1e-3);
        
        if (right) 
        {
            nearIdx = rightIdx;
            farIdx = leftIdx;
            nearRange = rightRange;
            farRange = leftRange;
            signedNomPivotAngle = nomPivotAngle;
        } else 
        {
            farIdx = rightIdx;
            nearIdx = leftIdx;
            farRange = rightRange;
            nearRange = leftRange;
            signedNomPivotAngle = -nomPivotAngle;
        }
        
        Eigen::Matrix3f rot_mat;
        rot_mat << cos(signedNomPivotAngle), -sin(signedNomPivotAngle), 0,
                    sin(signedNomPivotAngle), cos(signedNomPivotAngle), 0,
                    0, 0, 1;

        Eigen::Matrix3f near_rbt;
        near_rbt << 1, 0, nearRange * cos(idx2theta(nearIdx)),
                    0, 1, nearRange * sin(idx2theta(nearIdx)),
                    0, 0, 1;
        Eigen::Matrix3f far_rbt;
        far_rbt  << 1, 0, farRange * cos(idx2theta(farIdx)),
                    0, 1, farRange * sin(idx2theta(farIdx)),
                    0, 0, 1;
        
        Eigen::Matrix3f rot_rbt = near_rbt * (rot_mat * (near_rbt.inverse() * far_rbt));

        float r = float(sqrt(pow(rot_rbt(0, 2), 2) + pow(rot_rbt(1, 2), 2)));
        float theta = std::atan2(rot_rbt(1, 2), rot_rbt(0, 2));
        int idx = theta2idx(theta);

        // Rotation Completed
        // Get minimum dist range val from start to target index location
        // For wraparound
        int offset = right ? gap->LIdx() : idx;
        int upperbound = right ? idx : gap->RIdx();
        int intermediate_pt = offset + 1;
        int second_inter_pt = intermediate_pt;
        int size = upperbound - offset;

        // if ((upperbound - offset) < 3) 
        // {
        //     // Arbitrary value
        //     gap->goal.discard = true;
        //     return;
        // }

        offset = std::max(offset, 0);
        upperbound = std::min(upperbound, num_of_scan - 1);
        std::vector<float> min_dist(upperbound - offset);

        if (size == 0) 
        {
            // This shouldn't happen
            return;
        }


        if (stored_scan_msgs.ranges.size() < 500) 
        {
            ROS_FATAL_STREAM("Scan range incorrect gap manip");
        }

        try
        {
            for (int i = 0; i < min_dist.size(); i++) 
            {
                min_dist.at(i) = sqrt(pow(nearRange, 2) + pow(stored_scan_msgs.ranges.at(i + offset), 2) -
                    2 * nearRange * stored_scan_msgs.ranges.at(i + offset) * cos((i + offset - nearIdx) * stored_scan_msgs.angle_increment));
            }
        } catch(...) 
        {
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
        if (right)
        {
            gap->setManipPoints(idx, r, nearIdx, nearRange);

            // gap->convex.leftIdx_ = idx;
            // gap->convex.leftRange_ = r;
            // gap->convex.rightIdx_ = nearIdx;
            // gap->convex.rightRange_ = nearRange;
        } else
        {
            gap->setManipPoints(nearIdx, nearRange, idx, r);

            // gap->convex.leftIdx_ = nearIdx;
            // gap->convex.leftRange_ = nearRange;
            // gap->convex.rightIdx_ = idx;
            // gap->convex.rightRange_ = r;
        }

        // if (right && gap->convex.leftIdx_ < gap->convex.rightIdx_) 
        // {
        //     gap->goal.discard = true;
        // }

        // if (!right && gap->convex.leftIdx_ < gap->convex.rightIdx_) 
        // {
        //     gap->goal.discard = true;
        // }
        // gap->mode.agc = true;
        
        gap->setAGC();


        Eigen::Vector2f pLeft = gap->getManipLCartesian(); // (xLeft, yLeft);
        Eigen::Vector2f pRight = gap->getManipRCartesian(); // (xRight, yRight);

        xLeft = pLeft[0];     // (gap->convex.leftRange_) * cos(idx2theta(gap->convex.leftIdx_));
        yLeft = pLeft[1];         // (gap->convex.leftRange_) * sin(idx2theta(gap->convex.leftIdx_));
        xRight = pRight[0];            // (gap->convex.rightRange_) * cos(idx2theta(gap->convex.rightIdx_));
        yRight = pRight[1];            // (gap->convex.rightRange_) * sin(idx2theta(gap->convex.rightIdx_));
                
        ROS_INFO_STREAM_NAMED("GapManipulator", "        post-RGC gap in polar. left: (" << gap->manipLeftIdx() << ", " 
                                                                                        << gap->manipLeftRange() << "), right: (" 
                                                                                        << gap->manipRightIdx() << ", " 
                                                                                        << gap->manipRightRange() << ")");
        ROS_INFO_STREAM_NAMED("GapManipulator", "        post-RGC gap in cart. left: (" << xLeft << ", " 
                                                                                        << yLeft << "), right: (" 
                                                                                        << xRight << ", " 
                                                                                        << yRight << ")");
    }

    void GapManipulator::radialExtendGap(Gap * gap) 
    {
        ROS_INFO_STREAM_NAMED("GapManipulator", "    [radialExtendGap()]");

        if (!cfg_->gap_manip.radial_extend) 
        {
            ROS_DEBUG_STREAM_THROTTLE(1, "Radial Extension is off");
            return;
        }
        // TODO: check if the idx are correct when they cross the 0.

        float s = gap->getMinSafeDist();

        int leftIdx = gap->manipLeftIdx();
        int rightIdx = gap->manipRightIdx();
        float leftRange = gap->manipLeftRange();
        float rightRange = gap->manipRightRange();

        float leftTheta = idx2theta(leftIdx);
        float rightTheta = idx2theta(rightIdx);
        float xLeft = leftRange * cos(leftTheta);
        float yLeft = leftRange * sin(leftTheta);
        float xRight = rightRange * cos(rightTheta);
        float yRight = rightRange * sin(rightTheta);

        ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-radial extend gap in polar. left: (" << leftIdx << ", " << leftRange << "), right: (" << rightIdx << ", " << rightRange << ")");
        ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-radial extend gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");

        Eigen::Vector2f ptLeft(xLeft, yLeft);
        Eigen::Vector2f ptRight(xRight, yRight);

        Eigen::Vector2f eRight = ptRight / ptRight.norm();
        Eigen::Vector2f eLeft = ptLeft / ptLeft.norm();

        Eigen::Vector2f eB = (eLeft + eRight) / 2;
        eB.normalize();
        float gap_size = std::acos(eLeft.dot(eRight));

        Eigen::Vector2f qB = -s * eB;
        
        // Shifted Back Frame
        Eigen::Vector2f qLp = ptRight - qB;
        Eigen::Vector2f qRp = ptLeft - qB;

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

        gap->setManipPoints(theta2idx(polqRn(1)), polqRn(0), 
                            theta2idx(polqLn(1)), polqLn(0));
        // gap->convex.leftIdx_ = theta2idx(polqRn(1));
        // gap->convex.rightIdx_ = theta2idx(polqLn(1));
        // gap->convex.leftRange_ = polqRn(0);
        // gap->convex.rightRange_ = polqLn(0);
        // gap->mode.extended = true;
        gap->setExtended();

        // gap->qB = qB;
        gap->setQB(qB);

        Eigen::Vector2f pLeft = gap->getManipLCartesian(); // (xLeft, yLeft);
        Eigen::Vector2f pRight = gap->getManipRCartesian(); // (xRight, yRight);

        xLeft = pLeft[0];     // (gap->convex.leftRange_) * cos(idx2theta(gap->convex.leftIdx_));
        yLeft = pLeft[1];         // (gap->convex.leftRange_) * sin(idx2theta(gap->convex.leftIdx_));
        xRight = pRight[0];            // (gap->convex.rightRange_) * cos(idx2theta(gap->convex.rightIdx_));
        yRight = pRight[1];            // (gap->convex.rightRange_) * sin(idx2theta(gap->convex.rightIdx_));
        
        ROS_INFO_STREAM_NAMED("GapManipulator", "        post-radial extend gap in polar. left: (" << gap->manipLeftIdx() << ", " 
                                                                                                    << gap->manipLeftRange() << "), right: (" 
                                                                                                    << gap->manipRightIdx() << ", " 
                                                                                                    << gap->manipRightRange() << ")");
        ROS_INFO_STREAM_NAMED("GapManipulator", "        post-radial extend gap in cart. left: (" << xLeft << ", " 
                                                                                                    << yLeft << "), right: (" 
                                                                                                    << xRight << ", " 
                                                                                                    << yRight << ")");
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
