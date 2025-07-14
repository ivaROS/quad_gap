#include <quad_gap/trajectory_generation/GapManipulator.h>

namespace quad_gap 
{
    void GapManipulator::updateEgoCircle(boost::shared_ptr<sensor_msgs::msg::LaserScan const> msg) 
    {
        boost::mutex::scoped_lock lock(egolock);
        scan_ = msg;
        // num_of_scan = (int)(scan_.get()->ranges.size());
    }

    // In place modification
    void GapManipulator::reduceGap(Gap * gap, const geometry_msgs::msg::PoseStamped & globalPathLocalWaypoint) 
    {
        RCLCPP_INFO_STREAM(logger_,  "    [reduceGap()]");

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

        RCLCPP_INFO_STREAM(logger_,  "        pre-reduce gap in polar. left: (" << leftIdx << ", " << leftRange << "), right: (" << rightIdx << ", " << rightRange << ")");
        RCLCPP_INFO_STREAM(logger_,  "        pre-reduce gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");    

        if (!scan_) 
        {
            RCLCPP_INFO_STREAM(logger_,  "No scan available, cannot reduce gap.");
            RCLCPP_WARN_STREAM(logger_,  "No scan available, cannot reduce gap.");
            return; 
        }

        float gapIdxSpan = (leftIdx - rightIdx);
        if (gapIdxSpan < 0.0)
            gapIdxSpan += cfg_->scan.full_scan_f; // (2*gap->half_scan);

        float gapAngle = gapIdxSpan * cfg_->scan.angle_increment;

        if (gapAngle < cfg_->gap_manip.reduction_threshold)
        {
            RCLCPP_INFO_STREAM(logger_,  "Gap is convex, not reducing.");
            return;
        }

        int targetGapIdxSpan = cfg_->gap_manip.reduction_target / cfg_->scan.angle_increment;

        int leftIdxBiasedRight = subtractAndWrapScanIndices(leftIdx - targetGapIdxSpan, cfg_->scan.full_scan);
        int rightIdxBiasedLeft = (rightIdx + targetGapIdxSpan) % cfg_->scan.full_scan; // num_of_scan is int version of 2*half_scan

        float globalPathLocalWaypointTheta = std::atan2(globalPathLocalWaypoint.pose.position.y, globalPathLocalWaypoint.pose.position.x);
        int globalPathLocalWaypointIdx = theta2idx(globalPathLocalWaypointTheta); // globalPathLocalWaypointTheta / (M_PI / gap->half_scan) + gap->half_scan;
        RCLCPP_INFO_STREAM(logger_,  "        globalPathLocalWaypointIdx: " << globalPathLocalWaypointIdx);
        int halfTargetGapIdxSpan = 0.5 * targetGapIdxSpan; // distance in scan indices
        
        leftIdxBiasedRight = subtractAndWrapScanIndices(leftIdx - halfTargetGapIdxSpan, cfg_->scan.full_scan);
        int leftIdxBiasedLeft = (leftIdx + halfTargetGapIdxSpan) % cfg_->scan.full_scan;

        int rightIdxBiasedRight = subtractAndWrapScanIndices(rightIdx - halfTargetGapIdxSpan, cfg_->scan.full_scan);
        rightIdxBiasedLeft = (rightIdx + halfTargetGapIdxSpan) % cfg_->scan.full_scan;

        bool isLocalWaypointLeftBiased = isGlobalPathLocalWaypointWithinGapAngle(globalPathLocalWaypointIdx, leftIdxBiasedRight, leftIdxBiasedLeft); 
        bool isLocalWaypointRightBiased = isGlobalPathLocalWaypointWithinGapAngle(globalPathLocalWaypointIdx, rightIdxBiasedRight, rightIdxBiasedLeft); 
        int newLeftIdx, newRightIdx;
        if (isLocalWaypointLeftBiased) // left biased
        {
            newLeftIdx = leftIdx;
            newRightIdx = leftIdxBiasedRight;   
            RCLCPP_INFO_STREAM(logger_,  "        creating left-biased gap: " << newLeftIdx << ", " << newRightIdx);
        } else if (isLocalWaypointRightBiased) // right biased
        {
            newLeftIdx = rightIdxBiasedLeft;
            newRightIdx = rightIdx;
            RCLCPP_INFO_STREAM(logger_,  "        creating right-biased gap: " << newLeftIdx << ", " << newRightIdx);
        } else // Lingering in center 
        { 
            //RCLCPP_INFO_STREAM(logger_,   "central gap" << std::endl;
            newLeftIdx = (globalPathLocalWaypointIdx + halfTargetGapIdxSpan) % cfg_->scan.full_scan;
            newRightIdx = subtractAndWrapScanIndices(globalPathLocalWaypointIdx - halfTargetGapIdxSpan, cfg_->scan.full_scan);
            RCLCPP_INFO_STREAM(logger_,  "        creating goal-centered gap: " << newLeftIdx << ", " << newRightIdx);
        }

        // removed some float casting here
        float leftToNewLeftIdxSpan = subtractAndWrapScanIndices(leftIdx - newLeftIdx, cfg_->scan.full_scan);
        float leftToNewRightIdxSpan = subtractAndWrapScanIndices(leftIdx - newRightIdx, cfg_->scan.full_scan);

        // RCLCPP_INFO_STREAM(logger_,  "orig_gap_size: " << orig_gap_size);
        // RCLCPP_INFO_STREAM(logger_,  "leftToNewRightIdxSpan: " << leftToNewRightIdxSpan << ", leftToNewLeftIdxSpan: " << leftToNewLeftIdxSpan);

        float newLeftRange = leftRange + (rightRange - leftRange) * epsilonDivide(leftToNewLeftIdxSpan, gapIdxSpan);
        float newRightRange = leftRange +  (rightRange - leftRange) * epsilonDivide(leftToNewRightIdxSpan, gapIdxSpan);

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

        RCLCPP_INFO_STREAM(logger_,  "        post-reduce gap in polar. left: (" << newLeftIdx << ", " << newLeftRange << "), right: (" << newRightIdx << ", " << newRightRange << ")");
        RCLCPP_INFO_STREAM(logger_,  "        post-reduce gap in cart. left: (" << newXLeft << ", " << newYLeft << "), right: (" << newXRight << ", " << newYRight << ")");    

        gap->setReduced();
        return;
    }

    void GapManipulator::convertRadialGap(Gap * gap) 
    {
        RCLCPP_INFO_STREAM(logger_,  "    [convertRadialGap()]");

        sensor_msgs::msg::LaserScan desScan = *scan_.get();
        
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

        RCLCPP_INFO_STREAM(logger_,  "        pre-RGC gap in polar. left: (" << leftIdx << ", " << leftRange << "), right: (" << rightIdx << ", " << rightRange << ")");
        RCLCPP_INFO_STREAM(logger_,  "        pre-RGC gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");

        // Return if not radial gap or disabled
        if (!gap->isRadial()) 
        {
            RCLCPP_INFO_STREAM(logger_,  "        gap is not radial, no conversion needed");
            return;
        }

        if (gap->isReduced()) 
        {
            RCLCPP_INFO_STREAM(logger_,  "        gap has been reduced, no conversion needed");
            return;
        }

        if (!cfg_->gap_manip.radial_convert)
        {
            RCLCPP_INFO_STREAM(logger_,  "        gap radial conversion disabled, no conversion needed");
            return;
        }

        Eigen::Vector2f leftPt(xLeft, yLeft);
        Eigen::Vector2f rightPt(xRight, yRight);

        Eigen::Vector2f mid = 0.5 * (rightPt + leftPt);
        Eigen::Vector2f eMid = mid.normalized(); // normalized vector of mid point
        // Eigen::Vector2f robot_orient(1,0);
        float equivPassingLength = robotGeoProc_->getLinearDecayEquivalentPL(mid);
        float equivRadialLength = float(robotGeoProc_->getLinearDecayEquivalentRL(eMid, mid.norm()));

        float nomPivotAngle = (float) std::atan2(0.5 * equivPassingLength * cfg_->gap_manip.rot_ratio, 0.5 * equivRadialLength);
        
        int nearIdx = 0.0, farIdx = 0.0;
        float nearRange = 0.0, farRange = 0.0;
        float nearTheta = 0.0, farTheta = 0.0;
        float signedNomPivotAngle = 0.0; //  = right ? (rot_val + 1e-3): -(rot_val + 1e-3);
        
        if (right) 
        {
            nearIdx = rightIdx;
            nearRange = rightRange;
            farIdx = leftIdx;
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
        
        Eigen::Matrix2f nomPivotAngleRotationMatrix;
        // nomPivotAngleRotationMatrix: SE(3) matrix that represents desired rotation amount
        nomPivotAngleRotationMatrix << cos(signedNomPivotAngle), -sin(signedNomPivotAngle),
                                        sin(signedNomPivotAngle), cos(signedNomPivotAngle);
        
        nearTheta = idx2theta(nearIdx);
        farTheta = idx2theta(farIdx);   

        Eigen::Vector2f nearPt(nearRange * cos(nearTheta), nearRange * sin(nearTheta));
        Eigen::Vector2f farPt(farRange * cos(farTheta), farRange * sin(farTheta));
                                
        Eigen::Vector2f nearToFar = farPt - nearPt;
        
        Eigen::Vector2f rotatedNearToFar = nomPivotAngleRotationMatrix * nearToFar;

        // Eigen::Matrix3f rot_rbt = near_rbt * (rot_mat * (near_rbt.inverse() * far_rbt));

        Eigen::Vector2f pivotedPt = nearPt + rotatedNearToFar;

        // float r = float(sqrt(pow(rot_rbt(0, 2), 2) + pow(rot_rbt(1, 2), 2)));
        // float theta = std::atan2(rot_rbt(1, 2), rot_rbt(0, 2));
        // int idx = theta2idx(theta);

        // Extracting theta and idx of pivoted point
        float nomPivotedTheta = std::atan2(pivotedPt[1], pivotedPt[0]);
        int nomPivotedIdx = theta2idx(nomPivotedTheta);

        RCLCPP_INFO_STREAM(logger_,  "        nomPivotedTheta: " << nomPivotedTheta);
        RCLCPP_INFO_STREAM(logger_,  "        nomPivotedIdx: " << nomPivotedIdx);

        // Rotation Completed
        // Get minimum dist range val from start to target index location
        // For wraparound
        // int offset = right ? gap->LIdx() : idx;
        // int upperbound = right ? idx : gap->RIdx();
        // int intermediate_pt = offset + 1;
        // int second_inter_pt = intermediate_pt;
        // int size = upperbound - offset;

        // if ((upperbound - offset) < 3) 
        // {
        //     // Arbitrary value
        //     gap->goal.discard = true;
        //     return;
        // }

        int scanSearchStartIdx = 0, scanSearchEndIdx = 0;
        if (right)
        {   
            scanSearchStartIdx = leftIdx;
            scanSearchEndIdx = nomPivotedIdx;
        } else
        {
            scanSearchStartIdx = nomPivotedIdx;
            scanSearchEndIdx = rightIdx;
        }

        int scanSearchSize = scanSearchEndIdx - scanSearchStartIdx;

        // offset = std::max(offset, 0);
        // upperbound = std::min(upperbound, num_of_scan - 1);
        // std::vector<float> min_dist(upperbound - offset);

        // if (size == 0) 
        // {
        //     // This shouldn't happen
        //     return;
        // }

        if (scanSearchSize == 0)
        {
            RCLCPP_WARN_STREAM(logger_,  "        scanSearchSize is 0, SHOULD NOT BE HAPPENING");
            return;
        } else if (scanSearchSize < 0)
        {
            scanSearchSize += cfg_->scan.full_scan; // int(2*gap->half_scan);
        }

        // if (stored_scan_msgs.ranges.size() < 500) 
        // {
        //     ROS_FATAL_STREAM_NAMED("GapManipulator", "Scan range incorrect gap manip");
        // }

        int gapIdxSpan = (leftIdx - rightIdx);
        if (gapIdxSpan < 0)
            gapIdxSpan += cfg_->scan.full_scan;

        std::vector<float> nearPtToScanDists(scanSearchSize);

        int checkIdx = 0;
        float checkRange = 0.0, checkIdxSpan = 0.0;
        for (int i = 0; i < nearPtToScanDists.size(); i++) 
        {
            checkIdx = (i + scanSearchStartIdx) % cfg_->scan.full_scan; // int(2 * gap->half_scan);
            checkRange = desScan.ranges.at(checkIdx);
            checkIdxSpan = gapIdxSpan + (scanSearchSize - i);
            nearPtToScanDists.at(i) = sqrt(pow(nearRange, 2) + pow(checkRange, 2) -
                                        2.0 * nearRange * checkRange * cos(checkIdxSpan * cfg_->scan.angle_increment));
            // RCLCPP_INFO_STREAM(logger_,  "checking idx: " << checkIdx << ", range of: " << range << ", diff in idx: " << checkIdxSpan << ", dist of " << dist.at(i));
        }

        auto minDistIter = std::min_element(nearPtToScanDists.begin(), nearPtToScanDists.end());
        int minDistIdx = (scanSearchStartIdx + std::distance(nearPtToScanDists.begin(), minDistIter)) % cfg_->scan.full_scan; // int(2*gap->half_scan);
        
        float minDist = *minDistIter;

        RCLCPP_INFO_STREAM(logger_,  "        from " << scanSearchStartIdx << " to " << scanSearchEndIdx << ", min dist of " << minDist << " at " << minDistIdx);         

        // Eigen::Vector2f rotatedNearToFarVector = rotatedNearToFarTranslationMatrix.col(2).head(2);
        // Eigen::Vector2f rotatedNearToFarDirection = rotatedNearToFarVector.normalized();    

        // Eigen::Vector2f nearPt = Eigen::Vector2f(nearRange * cos(nearTheta), nearRange * sin(nearTheta));
        // Eigen::Vector2f pivotedPt = nearPt + rotatedNearToFarDirection * minDist;

        Eigen::Vector2f adjustedNearToFar = pivotedPt - nearPt; 

        RCLCPP_INFO_STREAM(logger_,  "        adjustedNearToFar: " << adjustedNearToFar[0] << ", " << adjustedNearToFar[1]);
        Eigen::Vector2f adjustedNearToFarDirection = adjustedNearToFar.normalized();
        RCLCPP_INFO_STREAM(logger_,  "        adjustedNearToFarDirection: " << adjustedNearToFarDirection[0] << ", " << adjustedNearToFarDirection[1]);

        Eigen::Vector2f convertedPt = nearPt + adjustedNearToFarDirection * minDist;

        RCLCPP_INFO_STREAM(logger_,  "        convertedPt: " << convertedPt[0] << ", " << convertedPt[1]);

        float convertedPtTheta = std::atan2(convertedPt[1], convertedPt[0]);
        int convertedPtIdx = theta2idx(convertedPtTheta);
        float convertedPtRange = convertedPt.norm();

        // try
        // {
        //     for (int i = 0; i < min_dist.size(); i++) 
        //     {
        //         min_dist.at(i) = sqrt(pow(nearRange, 2) + pow(stored_scan_msgs.ranges.at(i + offset), 2) -
        //             2 * nearRange * stored_scan_msgs.ranges.at(i + offset) * cos((i + offset - nearIdx) * stored_scan_msgs.angle_increment));
        //     }
        // } catch(...) 
        // {
        //     ROS_FATAL_STREAM_NAMED("GapManipulator", "convertRadialGap outofBound");
        // }

        // // auto farside_iter = ;
        // float farside = *std::min_element(min_dist.begin(), min_dist.end());

        // Eigen::Matrix3f far_near = near_rbt.inverse() * far_rbt;
        // float coefs = far_near.block<2, 1>(0, 2).norm();
        // // RCLCPP_INFO_STREAM(logger_,  )
        // far_near(0, 2) *= farside / coefs;
        // far_near(1, 2) *= farside / coefs;
        // Eigen::Matrix3f short_pt = near_rbt * (rot_mat * far_near);

        // r = float(sqrt(pow(short_pt(0, 2), 2) + pow(short_pt(1, 2), 2)));
        // theta = std::atan2(short_pt(1, 2), short_pt(0, 2));
        // idx = theta2idx(theta);

        // Recalculate end point location based on length
        float newLeftIdx = 0.0, newRightIdx = 0.0, newLeftRange = 0.0, newRightRange = 0.0;
        if (right)
        {
            newLeftIdx = convertedPtIdx;
            newLeftRange = convertedPtRange;
            newRightIdx = nearIdx;
            newRightRange = nearRange;
        } else
        {
            newLeftIdx = nearIdx;
            newLeftRange = nearRange;
            newRightIdx = convertedPtIdx;
            newRightRange = convertedPtRange;
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

        gap->setManipPoints(newLeftIdx, newLeftRange, newRightIdx, newRightRange);

        gap->getManipLCartesian(xLeft, yLeft);
        gap->getManipRCartesian(xRight, yRight);

        // Eigen::Vector2f pLeft = gap->getManipLPosition(); // (xLeft, yLeft);
        // Eigen::Vector2f pRight = gap->getManipRPosition(); // (xRight, yRight);

        // xLeft = pLeft[0];     // (gap->convex.leftRange_) * cos(idx2theta(gap->convex.leftIdx_));
        // yLeft = pLeft[1];         // (gap->convex.leftRange_) * sin(idx2theta(gap->convex.leftIdx_));
        // xRight = pRight[0];            // (gap->convex.rightRange_) * cos(idx2theta(gap->convex.rightIdx_));
        // yRight = pRight[1];            // (gap->convex.rightRange_) * sin(idx2theta(gap->convex.rightIdx_));
            
        RCLCPP_INFO_STREAM(logger_,  "        post-RGC gap in polar. left: (" << newLeftIdx << ", " << newLeftRange << "), right: (" << newRightIdx << ", " << newRightRange << ")");
        RCLCPP_INFO_STREAM(logger_,  "        post-AGC gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");
    }

    void GapManipulator::radialExtendGap(Gap * gap) 
    {
        RCLCPP_INFO_STREAM(logger_,  "    [radialExtendGap()]");

        if (!cfg_->gap_manip.radial_extend) 
        {
            RCLCPP_DEBUG_STREAM(logger_, "Radial Extension is off");
            return;
        }
        // TODO: check if the idx are correct when they cross the 0.

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

        RCLCPP_INFO_STREAM(logger_,  "        pre-radial extend gap in polar. left: (" << leftIdx << ", " << leftRange << "), right: (" << rightIdx << ", " << rightRange << ")");
        RCLCPP_INFO_STREAM(logger_,  "        pre-radial extend gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");

        // Eigen::Vector2f ptLeft(xLeft, yLeft);
        // Eigen::Vector2f ptRight(xRight, yRight);

        // Eigen::Vector2f eRight = ptRight / ptRight.norm();
        // Eigen::Vector2f eLeft = ptLeft / ptLeft.norm();

        // Eigen::Vector2f eB = (eLeft + eRight) / 2;
        // eB.normalize();
        // float gap_size = std::acos(eLeft.dot(eRight));

        // Eigen::Vector2f qB = -s * eB;
        
        // // Shifted Back Frame
        // Eigen::Vector2f qLp = ptRight - qB;
        // Eigen::Vector2f qRp = ptLeft - qB;

        // Eigen::Vector2f pLp = car2pol(qLp);
        // // pLp(1) += M_PI;
        // Eigen::Vector2f pRp = car2pol(qRp);
        // // pRp(1) += M_PI;

        // float phiB = pRp(1) - pLp(1);

        // Eigen::Vector2f pB = car2pol(-qB);
        // // pB(2) += M_PI;

        // float thL = pB(1) - gap_size / 4;
        // float thR = pB(1) + gap_size / 4;

        // Eigen::Vector2f pLn = pTheta(thL, phiB, pLp, pRp);
        // Eigen::Vector2f pRn = pTheta(thR, phiB, pLp, pRp);

        // Eigen::Vector2f qLn = pol2car(pLn) + qB;
        // Eigen::Vector2f qRn = pol2car(pRn) + qB;

        // // Store info back to original gap;
        // Eigen::Vector2f polqLn = car2pol(qLn);
        // Eigen::Vector2f polqRn = car2pol(qRn);

        // gap->setManipPoints(theta2idx(polqRn(1)), polqRn(0), 
        //                     theta2idx(polqLn(1)), polqLn(0));

        Eigen::Vector2f leftPt(xLeft, yLeft);
        Eigen::Vector2f rightPt(xRight, yRight);

        float leftToRightAngle = getSweptLeftToRightAngle(leftPt, rightPt);

        float thetaCenter = (leftTheta - 0.5*leftToRightAngle);

        // middle of gap direction
        Eigen::Vector2f eB(std::cos(thetaCenter), std::sin(thetaCenter));
        // RCLCPP_INFO_STREAM(logger_,  "eB: (" << eB[0] << ", " << eB[1] << ")");

        Eigen::Vector2f norm_eB = eB.normalized();         

        float s = gap->getMinSafeDist();

        Eigen::Vector2f qB = -s * norm_eB; // Shifted Back Frame

        // gap->convex.leftIdx_ = theta2idx(polqRn(1));
        // gap->convex.rightIdx_ = theta2idx(polqLn(1));
        // gap->convex.leftRange_ = polqRn(0);
        // gap->convex.rightRange_ = polqLn(0);
        // gap->mode.extended = true;
        gap->setExtended();

        // gap->qB = qB;
        gap->setQB(qB);

        Eigen::Vector2f pLeft = gap->getManipLPosition(); // (xLeft, yLeft);
        Eigen::Vector2f pRight = gap->getManipRPosition(); // (xRight, yRight);

        xLeft = pLeft[0];     // (gap->convex.leftRange_) * cos(idx2theta(gap->convex.leftIdx_));
        yLeft = pLeft[1];         // (gap->convex.leftRange_) * sin(idx2theta(gap->convex.leftIdx_));
        xRight = pRight[0];            // (gap->convex.rightRange_) * cos(idx2theta(gap->convex.rightIdx_));
        yRight = pRight[1];            // (gap->convex.rightRange_) * sin(idx2theta(gap->convex.rightIdx_));
        
        RCLCPP_INFO_STREAM(logger_,  "        post-radial extend gap in polar. left: (" << gap->manipLeftIdx() << ", " 
                                                                                                    << gap->manipLeftRange() << "), right: (" 
                                                                                                    << gap->manipRightIdx() << ", " 
                                                                                                    << gap->manipRightRange() << ")");
        RCLCPP_INFO_STREAM(logger_,  "        post-radial extend gap in cart. left: (" << xLeft << ", " 
                                                                                                    << yLeft << "), right: (" 
                                                                                                    << xRight << ", " 
                                                                                                    << yRight << ")");
        return;
    }

    void GapManipulator::inflateGapSides(Gap * gap) 
    {
        // get points

        float inf_ratio = cfg_->traj.inf_ratio;

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
        
        Eigen::Vector2f leftPt(xLeft, yLeft);
        Eigen::Vector2f rightPt(xRight, yRight);

        Eigen::Vector2f midPt = 0.5 * (leftPt + rightPt);
        // float epl = robotGeoProc_.getDecayEquivalentPL(orient_vec, pMid, pMid.norm());
        float epl = robotGeoProc_->getLinearDecayEquivalentPL(midPt);

        RCLCPP_INFO_STREAM(logger_,  "    [inflateGapSides()]");
        RCLCPP_INFO_STREAM(logger_,  "        pre-inflate gap in polar. left: (" << leftIdx << ", " << leftRange << "), right: (" << rightIdx << ", " << rightRange << ")");
        RCLCPP_INFO_STREAM(logger_,  "        pre-inflate gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");

        Eigen::Vector2f leftUnitNorm = leftPt.normalized();
        Eigen::Vector2f rightUnitNorm = rightPt.normalized();
        float leftToRightAngle = getSweptLeftToRightAngle(leftUnitNorm, rightUnitNorm);

        RCLCPP_INFO_STREAM(logger_,  "        leftToRightAngle: " << leftToRightAngle);;

        float epl_radius = 0.5 * epl;
        float newLeftToRightAngle = leftToRightAngle;
        float inflatedLeftTheta = leftTheta;
        float inflatedRightTheta = rightTheta;
        int inflatedLeftIdx = leftIdx;
        int inflatedRightIdx = rightIdx;
        float inflatedLeftRange = leftRange;
        float inflatedRightRange = rightRange;
        bool successful_inflation = false;
        while (!successful_inflation && inf_ratio >= 1.0) // try current inflation ratio, scale it down if that fails
        {
            ///////////////////////
            // ANGULAR INFLATION //
            ///////////////////////
            RCLCPP_INFO_STREAM(logger_,  "        inflating gap sides with ratio: " << inf_ratio);
    
            if (epl_radius * inf_ratio > leftRange)
            {
                RCLCPP_WARN_STREAM(logger_,  "        inflation ratio is too large, aborting");

                gap->setManipPoints(leftIdx, leftRange, rightIdx, rightRange);

                return;
                // return false;
            }

            float alpha_left = std::asin(epl_radius * inf_ratio / leftPt.norm() );
            float alpha_right = std::asin(epl_radius * inf_ratio / rightPt.norm() );
    
            float beta_left = (M_PI_OVER_TWO) - alpha_left;
            float beta_right = (M_PI_OVER_TWO) - alpha_right;
    
            float r_infl_left = epl_radius * inf_ratio / sin(beta_left);
            float r_infl_right = epl_radius * inf_ratio / sin(beta_right);
    
            Eigen::Vector2f leftAngularInflDir = Rnegpi2 * leftUnitNorm;
            Eigen::Vector2f rightAngularInflDir = Rpi2 * rightUnitNorm;
    
            RCLCPP_INFO_STREAM(logger_,  "        leftAngularInflDir: (" << leftAngularInflDir.transpose() << ")");
            RCLCPP_INFO_STREAM(logger_,  "        rightAngularInflDir: (" << rightAngularInflDir.transpose() << ")");
    
            // perform inflation
            Eigen::Vector2f inflatedLeftPt = leftPt + leftAngularInflDir * r_infl_left;
            Eigen::Vector2f inflatedRightPt = rightPt + rightAngularInflDir * r_infl_right;
    
            RCLCPP_INFO_STREAM(logger_,  "        inflatedLeftPt: (" << inflatedLeftPt.transpose() << ")");
            RCLCPP_INFO_STREAM(logger_,  "        inflatedRightPt: (" << inflatedRightPt.transpose() << ")");
    
            inflatedLeftTheta = std::atan2(inflatedLeftPt[1], inflatedLeftPt[0]);
            inflatedRightTheta = std::atan2(inflatedRightPt[1], inflatedRightPt[0]);
    
            // Check if inflation worked properly
            Eigen::Vector2f inflatedLeftUnitNorm(std::cos(inflatedLeftTheta), std::sin(inflatedLeftTheta));
            Eigen::Vector2f inflatedRightUnitNorm(std::cos(inflatedRightTheta), std::sin(inflatedRightTheta));
            float newLeftToRightAngle = getSweptLeftToRightAngle(inflatedLeftUnitNorm, inflatedRightUnitNorm);
    
            // update gap points
            inflatedLeftIdx = theta2idx(inflatedLeftTheta);
            inflatedRightIdx = theta2idx(inflatedRightTheta);
            
            // float leftToInflatedLeftAngle = getSweptLeftToRightAngle(leftUnitNorm, inflatedLeftUnitNorm);
            // float leftToInflatedRightAngle = getSweptLeftToRightAngle(leftUnitNorm, inflatedRightUnitNorm);
            inflatedLeftRange = inflatedLeftPt.norm(); // leftRange + (rightRange - leftRange) * epsilonDivide(leftToInflatedLeftAngle, leftToRightAngle);
            inflatedRightRange = inflatedRightPt.norm(); // leftRange + (rightRange - leftRange) * epsilonDivide(leftToInflatedRightAngle, leftToRightAngle);
    
            // if gap is too small, mark it to be discarded
            if (newLeftToRightAngle > leftToRightAngle)
            {
                RCLCPP_INFO_STREAM(logger_,  "        inflation has failed, new points in polar. left: (" << inflatedLeftIdx << ", " << inflatedLeftRange << "), right: (" << inflatedRightIdx << ", " << inflatedRightRange << ")");
    
                inf_ratio -= 0.1; // = std::max(1.0, inf_ratio - 0.1);

            } else
            {
                successful_inflation = true;
                RCLCPP_INFO_STREAM(logger_,  "        inflation succeeded, new points in polar. left: (" << inflatedLeftIdx << ", " << inflatedLeftRange << "), right: (" << inflatedRightIdx << ", " << inflatedRightRange << ")");
            }
        }

        if (! successful_inflation)
        {
            RCLCPP_INFO_STREAM(logger_,  "        inflation has failed for good.");
         
            gap->setManipPoints(leftIdx, leftRange, rightIdx, rightRange);

            return;
            // return false;
        }

        if (inflatedRightIdx == inflatedLeftIdx) // // RCLCPP_INFO_STREAM(logger_,  "manipulated indices are same");
            inflatedLeftIdx++;

        gap->setManipPoints(inflatedLeftIdx, inflatedLeftRange, inflatedRightIdx, inflatedRightRange);

        Eigen::Vector2f pLeft = gap->getManipLPosition(); // (xLeft, yLeft);
        Eigen::Vector2f pRight = gap->getManipRPosition(); // (xRight, yRight);

        xLeft = pLeft[0];     // (gap->convex.leftRange_) * cos(idx2theta(gap->convex.leftIdx_));
        yLeft = pLeft[1];         // (gap->convex.leftRange_) * sin(idx2theta(gap->convex.leftIdx_));
        xRight = pRight[0];            // (gap->convex.rightRange_) * cos(idx2theta(gap->convex.rightIdx_));
        yRight = pRight[1];            // (gap->convex.rightRange_) * sin(idx2theta(gap->convex.rightIdx_));
        
        RCLCPP_INFO_STREAM(logger_,  "        post-inflate gap in polar. left: (" << inflatedLeftIdx << ", " << inflatedLeftRange << "), right: (" << inflatedRightIdx << ", " << inflatedRightRange << ")");
        RCLCPP_INFO_STREAM(logger_,  "        post-inflate gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");

        return;
        // return true;
    }

    // Eigen::Vector2f GapManipulator::car2pol(const Eigen::Vector2f & a) 
    // {
    //     return Eigen::Vector2f(a.norm(), float(std::atan2(a(1), a(0))));
    // }

    // Eigen::Vector2f GapManipulator::pol2car(const Eigen::Vector2f & a) 
    // {
    //     return Eigen::Vector2f(cos(a(1)) * a(0), sin(a(1)) * a(0));
    // }

    // Eigen::Vector2f GapManipulator::pTheta(const float & th, const float & phiB, 
    //                                         const Eigen::Vector2f & pRp, const Eigen::Vector2f & pLp) 
    // {
    //     return pLp * (th - pRp(1)) / phiB + pRp * (pLp(1) - th) / phiB;
    // }
}
