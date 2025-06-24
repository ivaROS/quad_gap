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
    void GapManipulator::reduceGap(Gap * gap, const geometry_msgs::PoseStamped & globalPathLocalWaypoint) 
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
        {
            ROS_INFO_STREAM_NAMED("GapManipulator", "No scan available, cannot reduce gap.");
            ROS_WARN_STREAM_NAMED("GapManipulator", "No scan available, cannot reduce gap.");
            return; 
        }

        float gapIdxSpan = (leftIdx - rightIdx);
        if (gapIdxSpan < 0.0)
            gapIdxSpan += cfg_->scan.full_scan_f; // (2*gap->half_scan);

        float gapAngle = gapIdxSpan * cfg_->scan.angle_increment;

        if (gapAngle < cfg_->gap_manip.reduction_threshold)
        {
            ROS_INFO_STREAM_NAMED("GapManipulator", "Gap is convex, not reducing.");
            return;
        }

        int targetGapIdxSpan = cfg_->gap_manip.reduction_target / cfg_->scan.angle_increment;

        int leftIdxBiasedRight = subtractAndWrapScanIndices(leftIdx - targetGapIdxSpan, cfg_->scan.full_scan);
        int rightIdxBiasedLeft = (rightIdx + targetGapIdxSpan) % cfg_->scan.full_scan; // num_of_scan is int version of 2*half_scan

        float globalPathLocalWaypointTheta = std::atan2(globalPathLocalWaypoint.pose.position.y, globalPathLocalWaypoint.pose.position.x);
        int globalPathLocalWaypointIdx = theta2idx(globalPathLocalWaypointTheta); // globalPathLocalWaypointTheta / (M_PI / gap->half_scan) + gap->half_scan;
        ROS_INFO_STREAM_NAMED("GapManipulator", "        globalPathLocalWaypointIdx: " << globalPathLocalWaypointIdx);
        int halfTargetGapIdxSpan = targetGapIdxSpan / 2; // distance in scan indices
        
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
            ROS_INFO_STREAM_NAMED("GapManipulator", "        creating left-biased gap: " << newLeftIdx << ", " << newRightIdx);
        } else if (isLocalWaypointRightBiased) // right biased
        {
            newLeftIdx = rightIdxBiasedLeft;
            newRightIdx = rightIdx;
            ROS_INFO_STREAM_NAMED("GapManipulator", "        creating right-biased gap: " << newLeftIdx << ", " << newRightIdx);
        } else // Lingering in center 
        { 
            //ROS_INFO_STREAM_NAMED("GapManipulator",  "central gap" << std::endl;
            newLeftIdx = (globalPathLocalWaypointIdx + halfTargetGapIdxSpan) % cfg_->scan.full_scan;
            newRightIdx = subtractAndWrapScanIndices(globalPathLocalWaypointIdx - halfTargetGapIdxSpan, cfg_->scan.full_scan);
            ROS_INFO_STREAM_NAMED("GapManipulator", "        creating goal-centered gap: " << newLeftIdx << ", " << newRightIdx);
        }

        // removed some float casting here
        float leftToNewLeftIdxSpan = subtractAndWrapScanIndices(leftIdx - newLeftIdx, cfg_->scan.full_scan);
        float leftToNewRightIdxSpan = subtractAndWrapScanIndices(leftIdx - newRightIdx, cfg_->scan.full_scan);

        // ROS_INFO_STREAM_NAMED("GapManipulator", "orig_gap_size: " << orig_gap_size);
        // ROS_INFO_STREAM_NAMED("GapManipulator", "leftToNewRightIdxSpan: " << leftToNewRightIdxSpan << ", leftToNewLeftIdxSpan: " << leftToNewLeftIdxSpan);

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

        ROS_INFO_STREAM_NAMED("GapManipulator", "        post-reduce gap in polar. left: (" << newLeftIdx << ", " << newLeftRange << "), right: (" << newRightIdx << ", " << newRightRange << ")");
        ROS_INFO_STREAM_NAMED("GapManipulator", "        post-reduce gap in cart. left: (" << newXLeft << ", " << newYLeft << "), right: (" << newXRight << ", " << newYRight << ")");    

        gap->setReduced();
        return;
    }

    void GapManipulator::convertRadialGap(Gap * gap) 
    {
        ROS_INFO_STREAM_NAMED("GapManipulator", "    [convertRadialGap()]");

        sensor_msgs::LaserScan desScan = *scan_.get();
        
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

        // Return if not radial gap or disabled
        if (!gap->isRadial()) 
        {
            ROS_INFO_STREAM_NAMED("GapManipulator", "        gap is not radial, no conversion needed");
            return;
        }

        if (!cfg_->gap_manip.radial_convert)
        {
            ROS_INFO_STREAM_NAMED("GapManipulator", "        gap radial conversion disabled, no conversion needed");
            return;
        }

        Eigen::Vector2f leftPt(xLeft, yLeft);
        Eigen::Vector2f rightPt(xRight, yRight);

        Eigen::Vector2f mid = (rightPt + leftPt) / 2;
        // Eigen::Vector2f robot_orient(1,0);
        float equivPassingLength = float(robot_geo_proc_->getLinearDecayEquivalentPL(robotOrientationVector, mid, mid.norm()));
        float equivRadialLength = float(robot_geo_proc_->getLinearDecayEquivalentRL(robotOrientationVector, mid, mid.norm()));
        
        float nomPivotAngle = (float) std::atan2(equivPassingLength / 2 * cfg_->gap_manip.rot_ratio, equivRadialLength / 2);
        
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

        ROS_INFO_STREAM_NAMED("GapManipulator", "        nomPivotedTheta: " << nomPivotedTheta);
        ROS_INFO_STREAM_NAMED("GapManipulator", "        nomPivotedIdx: " << nomPivotedIdx);

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
            ROS_WARN_STREAM_NAMED("GapManipulator", "        scanSearchSize is 0, SHOULD NOT BE HAPPENING");
            return;
        } else if (scanSearchSize < 0)
        {
            scanSearchSize += cfg_->scan.full_scan; // int(2*gap->half_scan);
        }

        // if (stored_scan_msgs.ranges.size() < 500) 
        // {
        //     ROS_FATAL_STREAM("Scan range incorrect gap manip");
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
            // // ROS_INFO_STREAM("checking idx: " << checkIdx << ", range of: " << range << ", diff in idx: " << checkIdxSpan << ", dist of " << dist.at(i));
        }

        auto minDistIter = std::min_element(nearPtToScanDists.begin(), nearPtToScanDists.end());
        int minDistIdx = (scanSearchStartIdx + std::distance(nearPtToScanDists.begin(), minDistIter)) % cfg_->scan.full_scan; // int(2*gap->half_scan);
        
        float minDist = *minDistIter;

        ROS_INFO_STREAM_NAMED("GapManipulator", "        from " << scanSearchStartIdx << " to " << scanSearchEndIdx << ", min dist of " << minDist << " at " << minDistIdx);         

        // Eigen::Vector2f rotatedNearToFarVector = rotatedNearToFarTranslationMatrix.col(2).head(2);
        // Eigen::Vector2f rotatedNearToFarDirection = rotatedNearToFarVector.normalized();    

        // Eigen::Vector2f nearPt = Eigen::Vector2f(nearRange * cos(nearTheta), nearRange * sin(nearTheta));
        // Eigen::Vector2f pivotedPt = nearPt + rotatedNearToFarDirection * minDist;

        Eigen::Vector2f adjustedNearToFar = pivotedPt - nearPt; 

        ROS_INFO_STREAM_NAMED("GapManipulator", "        adjustedNearToFar: " << adjustedNearToFar[0] << ", " << adjustedNearToFar[1]);
        Eigen::Vector2f adjustedNearToFarDirection = adjustedNearToFar.normalized();
        ROS_INFO_STREAM_NAMED("GapManipulator", "        adjustedNearToFarDirection: " << adjustedNearToFarDirection[0] << ", " << adjustedNearToFarDirection[1]);

        Eigen::Vector2f convertedPt = nearPt + adjustedNearToFarDirection * minDist;

        ROS_INFO_STREAM_NAMED("GapManipulator", "        convertedPt: " << convertedPt[0] << ", " << convertedPt[1]);

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
        //     ROS_FATAL_STREAM("convertRadialGap outofBound");
        // }

        // // auto farside_iter = ;
        // float farside = *std::min_element(min_dist.begin(), min_dist.end());

        // Eigen::Matrix3f far_near = near_rbt.inverse() * far_rbt;
        // float coefs = far_near.block<2, 1>(0, 2).norm();
        // // ROS_INFO_STREAM()
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

        // Eigen::Vector2f pLeft = gap->getManipLCartesian(); // (xLeft, yLeft);
        // Eigen::Vector2f pRight = gap->getManipRCartesian(); // (xRight, yRight);

        // xLeft = pLeft[0];     // (gap->convex.leftRange_) * cos(idx2theta(gap->convex.leftIdx_));
        // yLeft = pLeft[1];         // (gap->convex.leftRange_) * sin(idx2theta(gap->convex.leftIdx_));
        // xRight = pRight[0];            // (gap->convex.rightRange_) * cos(idx2theta(gap->convex.rightIdx_));
        // yRight = pRight[1];            // (gap->convex.rightRange_) * sin(idx2theta(gap->convex.rightIdx_));
            
        ROS_INFO_STREAM_NAMED("GapManipulator", "        post-RGC gap in polar. left: (" << newLeftIdx << ", " << newLeftRange << "), right: (" << newRightIdx << ", " << newRightRange << ")");
        ROS_INFO_STREAM_NAMED("GapManipulator", "        post-AGC gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");
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
        // ROS_INFO_STREAM_NAMED("GapManipulator", "eB: (" << eB[0] << ", " << eB[1] << ")");

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
