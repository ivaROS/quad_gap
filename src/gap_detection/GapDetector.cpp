#include <quad_gap/gap_detection/GapDetector.h>

namespace quad_gap 
{
    GapDetector::GapDetector(const QuadGapConfig& cfg, RobotGeometryProcessor& robot_geo_proc) 
    {
        cfg_ = & cfg;
        robot_geo_proc_ = robot_geo_proc;
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

    bool GapDetector::equivalentCheck(const Gap * detected_gap)
    {
        // Inscribed radius gets enforced here, or unless using inflated egocircle,
        // then no need for range diff
        // Find equivalent passing length
        Eigen::Vector2d orient_vec(1, 0);
        Eigen::Vector2d m_pt_vec = detected_gap->get_middle_pt_vec();
        // double epl = robot_geo_proc_.getDecayEquivalentPL(orient_vec, m_pt_vec, m_pt_vec.norm());
        double epl = robot_geo_proc_.getLinearDecayEquivalentPL(orient_vec, m_pt_vec, m_pt_vec.norm());

        return detected_gap->get_dist_side() > epl;
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
                Gap * detected_gap = new Gap(frame, currIdx - 1, prevRange, true);
                detected_gap->addLeftInformation(currIdx, currRange);
                detected_gap->setMinSafeDist(minScanDist_);

                if (equivalentCheck(detected_gap))
                {
                    rawGaps.push_back(detected_gap); //  || cfg_->planning.planning_inflated
                } else
                {
                    delete detected_gap; // If not equivalent, delete the gap
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
                    Gap * detected_gap = new Gap(frame, gapRIdx, gapRRange);
                    detected_gap->addLeftInformation(currIdx, currRange);
                    detected_gap->setMinSafeDist(minScanDist_);

                    if (equivalentCheck(detected_gap))
                    {
                        rawGaps.push_back(detected_gap); //  || cfg_->planning.planning_inflated
                    } else
                    {
                        delete detected_gap; // If not equivalent, delete the gap
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
            Gap * detected_gap = new Gap(frame, gapRIdx, gapRRange);
            detected_gap->addLeftInformation(int(scan_.ranges.size() - 1), *(scan_.ranges.end() - 1));
            detected_gap->setMinSafeDist(minScanDist_);

            if (equivalentCheck(detected_gap) || detected_gap->_left_idx - detected_gap->_right_idx > 500)
            {
                rawGaps.push_back(detected_gap); //  || cfg_->planning.planning_inflated
            } else
            {
                delete detected_gap; // If not equivalent, delete the gap
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

    int GapDetector::checkSimplifiedGapsMergeability(const Gap * rawGap, 
                                                     const std::vector<Gap *> & simpGaps)
    {
        int last_mergable = -1;

        float curr_left_dist = rawGap->LRange();
        // int erase_counter = 0;

        // float coefs = cfg_->planning.planning_inflated ? 0 : 1;
        for (int j = (int) (simpGaps.size() - 1); j >= 0; j--)
        {
            int start_idx = std::min(simpGaps[j]->LIdx(), rawGap->RIdx());
            int end_idx = std::max(simpGaps[j]->LIdx(), rawGap->RIdx());
            auto farside_iter = std::min_element(scan_.ranges.begin() + start_idx, scan_.ranges.begin() + end_idx);
            int farside_idx = farside_iter - scan_.ranges.begin();
            // TODO: what number to use? Currently, use the max radius. The merging will not happen frequently.
            // double max_r_er = robot_geo_proc_.getRobotMaxRadius();
            double farside_angle = farside_idx * scan_.angle_increment + scan_.angle_min;
            Eigen::Vector2d farside_vec(cos(farside_angle), sin(farside_angle));
            Eigen::Vector2d orient_vec(1, 0);
            double erl_left_dist = robot_geo_proc_.getLinearDecayEquivalentRL(orient_vec, farside_vec, curr_left_dist);
            double erl_right_dist = robot_geo_proc_.getLinearDecayEquivalentRL(orient_vec, farside_vec, simpGaps[j]->RRange());
            bool second_test = curr_left_dist <= (*farside_iter - erl_left_dist) && simpGaps[j]->RRange() <= (*farside_iter - erl_right_dist);
            bool dist_diff = simpGaps[j]->isRightType() || !simpGaps[j]->isRadial();
            bool idx_diff = rawGap->LIdx() - simpGaps[j]->RIdx() < cfg_->gap_manip.max_idx_diff;
            if (second_test && dist_diff && idx_diff) 
            {
                last_mergable = j;
            } 
        }
    }

    bool GapDetector::mergeSweptGapCondition(const Gap * rawGap, 
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
        std::vector<Gap *> simpGaps;

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

        int last_mergable = -1;

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
                
                simpGaps.push_back(rawGap);
            } else 
            {
                if (rawGap->isRadial())
                {
                    if (rawGap->isRightType())
                    {
                        simpGaps.push_back(rawGap);
                    }
                    else
                    {
                        last_mergable = checkSimplifiedGapsMergeability(rawGap, simpGaps);

                        if (last_mergable != -1) 
                        {
                            simpGaps.erase(simpGaps.begin() + last_mergable + 1, simpGaps.end());
                            simpGaps.back()->addLeftInformation(rawGap->LIdx(), rawGap->LRange());
                        } else 
                        {
                            simpGaps.push_back(rawGap);
                        }
                    }
                } else
                {
                    if (mergeSweptGapCondition(rawGap, simpGaps))
                    {
                        simpGaps.back()->addLeftInformation(rawGap->LIdx(), rawGap->LRange());
                    } else {
                        simpGaps.push_back(rawGap);
                    }
                }
            }
                // else
                // {
                    // // A swept gap solely on its own
                    // simpGaps.push_back(rawGap);
                // }
            // }
            // last_type_left = rawGap->isRightType();
        }

        // rawGaps.clear();
        // rawGaps = simpGaps;
        return simpGaps;
    }


}