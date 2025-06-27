#include <quad_gap/gap_detection/GapDetector.h>

namespace quad_gap 
{
    GapDetector::GapDetector(const QuadGapConfig& cfg, RobotGeometryProcessor& robot_geo_proc) 
    {
        cfg_ = & cfg;
        robot_geo_proc_ = & robot_geo_proc;
    }

    ///////////// SCAN PRE-PROCESSING ///////////////////////

    void GapDetector::preprocessScan(boost::shared_ptr<sensor_msgs::LaserScan> scan)
    {
        // pre-process scan (turning nan's and inf's into max ranges)
        float eps = 0.00001f;
        for (int i = 0; i < scan->ranges.size(); i++)
        {
            if (std::isnan(scan->ranges.at(i)))
            {
                // ROS_WARN_STREAM_THROTTLE_NAMED(1.0, "GapDetector", "NaN detected in scan, replacing with max range");
                scan->ranges.at(i) = cfg_->scan.range_max - eps;
            }

            if (std::isinf(scan->ranges.at(i)))
            {
                // ROS_WARN_STREAM_THROTTLE_NAMED(1.0, "GapDetector", "Inf detected in scan, replacing with max range");
                scan->ranges.at(i) = cfg_->scan.range_max - eps;
            }
        }
    }

    ////////////////// GAP DETECTION ///////////////////////

    bool GapDetector::isFinite(const float & range)
    {
        return range < maxScanDist_;
    }

    bool GapDetector::sweptGapStartedOrEnded(const float & currRange, const float & prevRange)
    {
        return isFinite(prevRange) != isFinite(currRange);
    }

    bool GapDetector::radialGapSizeCheck(const float & currRange, 
                                            const float & prevRange, 
                                            const float & gapAngle)
    {
        if (!(prevRange < maxScanDist_ && currRange < maxScanDist_))
        return false;

        // Euclidean distance between current and previous points
        float consecScanPointDist = sqrt(pow(prevRange, 2) + pow(currRange, 2) - 2 * prevRange * currRange * cos(gapAngle));

        bool canRobotFit = consecScanPointDist > 3 * cfg_->rbt.r_inscr;

        return canRobotFit;
    }  

    bool GapDetector::equivalentPLDistcheck(Gap * rawGap)
    {
        // Inscribed radius gets enforced here, or unless using inflated egocircle,
        // then no need for range diff
        // Find equivalent passing length
        // Eigen::Vector2f orient_vec(1, 0);
        Eigen::Vector2f m_pt_vec = rawGap->get_middle_pt_vec();
        // float epl = robot_geo_proc_->getDecayEquivalentPL(orient_vec, m_pt_vec, m_pt_vec.norm());
        float epl = robot_geo_proc_->getLinearDecayEquivalentPL(robotOrientationVector, m_pt_vec, m_pt_vec.norm());

        return rawGap->get_dist_side() > epl;
    }

    bool GapDetector::bridgeCondition(const std::vector<Gap *> & rawGaps)
    {
        bool multipleGaps = rawGaps.size() > 1;
        
        if (!multipleGaps)
            return false;

        // only defined behavior if there are multiple gaps
        bool firstAndLastGapsBorder = (rawGaps.front()->RIdx() == 0 && 
                                          rawGaps.back()->LIdx() == (fullScanRayCount_ - 1));
        
        return firstAndLastGapsBorder;
    }

    std::vector<Gap *> GapDetector::gapDetection(boost::shared_ptr<sensor_msgs::LaserScan const> scanPtr)
    {
        std::vector<Gap *> rawGaps;
        // rawGaps.clear();

        scan_ = *scanPtr.get();

        fullScanRayCount_ = scan_.ranges.size();
        ROS_WARN_STREAM_COND_NAMED(fullScanRayCount_ != cfg_->scan.full_scan, "GapDetector", "Scan is wrong size, should be " << cfg_->scan.full_scan);

        halfScanRayCount_ = float(0.5 * fullScanRayCount_);

        minScanDist_ = *std::min_element(scan_.ranges.begin(), scan_.ranges.end());
        maxScanDist_ = *std::max_element(scan_.ranges.begin(), scan_.ranges.end());
        ROS_INFO_STREAM_NAMED("GapDetector", "gapDetection min_dist: " << minScanDist_);

        std::string frame = scan_.header.frame_id;

        // bool prev = true;
        // auto max_dist_iter = std::max_element(scan_.ranges.begin(), scan_.ranges.end());
        // float max_scan_dist = *max_dist_iter;
        // auto min_dist = *std::min_element(scan_.ranges.begin(), scan_.ranges.end());

        // int gap_size = 0;
        int gapRIdx = 0;
        float gapRRange = scan_.ranges[0];

        bool withinSweptGap = gapRRange >= maxScanDist_;

        float currRange = scan_.ranges[0];
        float prevRange = currRange; // First range is always valid, so no need to check
        
        float scan_diff;
        // int wrap = 0;

        for (int currIdx = 1; currIdx < scan_.ranges.size(); currIdx++)
        {
            currRange = scan_.ranges[currIdx];
            scan_diff = currRange - prevRange;
            
            // Arbitrary small threshold for a range difference to be considered
            // if (std::abs(scan_diff) > 0.2) 
            // {

            // If both current and last values are not infinity, meaning this is not a swept gap
            if (radialGapSizeCheck(currRange, prevRange, scan_.angle_increment)) 
            {
                Gap * rawGap = new Gap(frame, currIdx - 1, prevRange, true);
                rawGap->addLeftInformation(currIdx, currRange);
                rawGap->setMinSafeDist(minScanDist_);

                if (equivalentPLDistcheck(rawGap))
                {
                    rawGaps.push_back(rawGap); //  || cfg_->planning.planning_inflated
                } else
                {
                    delete rawGap; // If not equivalent, delete the gap
                }
            }
                
            // }

            // Beginning or the end of a reading into infinity => swept gap
            if (sweptGapStartedOrEnded(currRange, prevRange))
            {
                // If previously marked gap, meaning ending of a gap
                if (withinSweptGap)
                {
                    withinSweptGap = false;
                    Gap * rawGap = new Gap(frame, gapRIdx, gapRRange);
                    rawGap->addLeftInformation(currIdx, currRange);
                    rawGap->setMinSafeDist(minScanDist_);

                    if (equivalentPLDistcheck(rawGap))
                    {
                        rawGaps.push_back(rawGap); //  || cfg_->planning.planning_inflated
                    } else
                    {
                        delete rawGap; // If not equivalent, delete the gap
                    }
                } else // previously not marked a gap, not marking the gap
                {
                    gapRIdx = currIdx - 1;
                    gapRRange = prevRange;
                    withinSweptGap = true;
                }
            }
            prevRange = currRange;
        }

        // Catch the last gap
        if (withinSweptGap) 
        {
            Gap * rawGap = new Gap(frame, gapRIdx, gapRRange);
            rawGap->addLeftInformation(int(scan_.ranges.size() - 1), *(scan_.ranges.end() - 1));
            rawGap->setMinSafeDist(minScanDist_);

            if (equivalentPLDistcheck(rawGap) || rawGap->LIdx() - rawGap->RIdx() > 500)
            {
                rawGaps.push_back(rawGap); //  || cfg_->planning.planning_inflated
            } else
            {
                delete rawGap; // If not equivalent, delete the gap
            } 
        }
        
        // Bridge the last gap around
        // Bridge the last gap around
        if (bridgeCondition(rawGaps))
        {
            rawGaps.back()->addLeftInformation(rawGaps.front()->LIdx(), rawGaps.front()->LRange());

            // delete first gap
            delete *rawGaps.begin();
            rawGaps.erase(rawGaps.begin());

            // // Both ends
            // float start_side_dist = rawGaps[0].LRange();
            // float end_side_dist = rawGaps[rawGaps.size() - 1].RRange();
            // int start_side_idx = rawGaps[0].LIdx();
            // int end_side_idx = rawGaps[rawGaps.size() - 1].RIdx();

            // // float result = (end_side_dist - start_side_dist) * start_side_idx / (rawGaps.size() - end_side_idx + start_side_idx) + start_side_dist;
            // int total_size = 511 - end_side_idx + start_side_idx;
            // float result = (end_side_dist - start_side_dist) * (float (start_side_idx) / float (total_size)) + start_side_dist;
            // rawGaps[0].setRightObs();
            // rawGaps[rawGaps.size() - 1].setLeftObs();
            // rawGaps[rawGaps.size() - 1].addLeftInformation(511, result);
            // rawGaps[0].setRRange(result);
        }

        return rawGaps;
    }

    ////////////////// GAP SIMPLIFICATION ///////////////////////

    int GapDetector::checkSimplifiedGapsMergeability(Gap * rawGap, 
                                                     const std::vector<Gap *> & simplifiedGaps)
    {
        int lastMergeable = -1;

        float currLRange = rawGap->LRange();
        // int erase_counter = 0;

        int startIdx = -1, endIdx = -1;
        // float coefs = cfg_->planning.planning_inflated ? 0 : 1;
        for (int j = (int) (simplifiedGaps.size() - 1); j >= 0; j--)
        {
            startIdx = std::min(simplifiedGaps.at(j)->LIdx(), rawGap->RIdx());
            endIdx = std::max(simplifiedGaps.at(j)->LIdx(), rawGap->RIdx());

            auto minIntergapRangeIter = std::min_element(scan_.ranges.begin() + startIdx, scan_.ranges.begin() + endIdx);
            float minIntergapRange = *minIntergapRangeIter;
            int minIntergapIdx = minIntergapRangeIter - scan_.ranges.begin();
            
            float farside_angle = idx2theta(minIntergapIdx);
            Eigen::Vector2f farside_vec(cos(farside_angle), sin(farside_angle));

            // TODO: what number to use? Currently, use the max radius. The merging will not happen frequently.
            // float max_r_er = robot_geo_proc_->getRobotMaxRadius();
            

            float erlLRange = robot_geo_proc_->getLinearDecayEquivalentRL(robotOrientationVector, farside_vec, currLRange);
            float erlRRange = robot_geo_proc_->getLinearDecayEquivalentRL(robotOrientationVector, farside_vec, simplifiedGaps[j]->RRange());
            bool second_test = currLRange <= (minIntergapRange - erlLRange) && simplifiedGaps[j]->RRange() <= (minIntergapRange - erlRRange);
            
            
            // 2. Checking if current simplified gap is either right dist < left dist or swept 
            bool rightTypeOrSweptGap = simplifiedGaps.at(j)->isRightType() || !simplifiedGaps.at(j)->isRadial();
            
            // bool idx_diff = rawGap->LIdx() - simplifiedGaps[j]->RIdx() < cfg_->gap_manip.max_idx_diff;

            if (second_test && rightTypeOrSweptGap) //  && idx_diff 
            {
                lastMergeable = j;
            } 
        }

        return lastMergeable;
    }

    bool GapDetector::mergeSweptGapCondition(Gap * rawGap, 
                                             const std::vector<Gap *> & simplifiedGaps)
    {
        // checking if difference between raw gap left dist and simplified gap right (widest distances, encompassing both gaps)
        // dist is sufficiently small (to fit robot)
        bool adjacentGapPtDistDiffCheck = std::abs(rawGap->LRange() - simplifiedGaps.back()->RRange()) < 3 * cfg_->rbt.r_inscr;

        // checking if difference is sufficiently small, and that current simplified gap is radial and right dist < left dist
        return adjacentGapPtDistDiffCheck && simplifiedGaps.back()->isRadial() && simplifiedGaps.back()->isRightType();
    }


    std::vector<Gap *> GapDetector::gapSimplification(const std::vector<Gap *> & rawGaps)
    {
        std::vector<Gap *> simplifiedGaps;

        // int right_idx = -1;
        // int left_idx = -1;
        // float left_dist = 3;
        // float right_dist = 3; // TODO: Make this reconfigurable
        // int observed_size = (int) rawGaps.size();

        // sensor_msgs::LaserScan scan_ = *scanPtr.get();
        // Termination Condition

        // Insert first
        bool markToStart = true;
        // bool last_type_left = true;
        // int left_counter = 0;
        // bool changed = true;

        int lastMergeable = -1;

        // for (int i = 0; i < (int) rawGaps.size(); i++)
        for (Gap * rawGap : rawGaps)
        {
            if (markToStart)
            {
                if (rawGap->isRadial() && rawGap->isRightType())
                {
                    // Wait until the first mergable gap aka swept left type gap
                    markToStart = false;
                }
                
                simplifiedGaps.push_back(rawGap);
            } else 
            {
                if (rawGap->isRadial())
                {
                    if (rawGap->isRightType())
                    {
                        simplifiedGaps.push_back(rawGap);
                    } else
                    {
                        lastMergeable = checkSimplifiedGapsMergeability(rawGap, simplifiedGaps);

                        if (lastMergeable != -1) 
                        {
                            for (auto gapIter = simplifiedGaps.begin() + lastMergeable + 1; gapIter != simplifiedGaps.end(); gapIter++)
                                delete *gapIter;

                            simplifiedGaps.erase(simplifiedGaps.begin() + lastMergeable + 1, simplifiedGaps.end());
                            simplifiedGaps.back()->addLeftInformation(rawGap->LIdx(), rawGap->LRange());
                        } else 
                        {
                            simplifiedGaps.push_back(rawGap);
                        }
                    }
                } else
                {
                    if (mergeSweptGapCondition(rawGap, simplifiedGaps))
                    {
                        simplifiedGaps.back()->addLeftInformation(rawGap->LIdx(), rawGap->LRange());
                    } else {
                        simplifiedGaps.push_back(rawGap);
                    }
                }
            }
                // else
                // {
                    // // A swept gap solely on its own
                    // simplifiedGaps.push_back(rawGap);
                // }
            // }
            // last_type_left = rawGap->isRightType();
        }

        // rawGaps.clear();
        // rawGaps = simplifiedGaps;
        return simplifiedGaps;
    }


}