#include <quad_gap/gap_detection/GapDetector.h>

namespace quad_gap 
{
    GapDetector::GapDetector(const QuadGapConfig& cfg, RobotGeometryProcessor& robot_geo_proc) 
    {
        cfg_ = & cfg;
        robot_geo_proc_ = robot_geo_proc;
    }

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

    std::vector<Gap> GapDetector::gapDetection(boost::shared_ptr<sensor_msgs::LaserScan const> scanPtr)
    {
        std::vector<Gap> observed_gaps;
        // observed_gaps.clear();

        scan_ = *scanPtr.get();

        bool prev = true;
        auto max_dist_iter = std::max_element(scan_.ranges.begin(), scan_.ranges.end());
        float max_scan_dist = *max_dist_iter;
        auto min_dist = *std::min_element(scan_.ranges.begin(), scan_.ranges.end());
        int gap_size = 0;
        std::string frame = scan_.header.frame_id;
        int gap_right_idx = 0;
        float gap_right_dist = scan_.ranges[0];
        float last_scan = scan_.ranges[0];
        bool prev_right_gap = gap_right_dist >= max_scan_dist;
        float scan_dist;
        float scan_diff;
        int wrap = 0;

        for (std::vector<float>::size_type it = 1; it < scan_.ranges.size(); ++it)
        {
            scan_dist = scan_.ranges[it];
            scan_diff = scan_dist - last_scan;
            
            // Arbitrary small threshold for a range difference to be considered
            if (std::abs(scan_diff) > 0.2) 
            {
                // If both current and last values are not infinity, meaning this is not a swept gap
                if (scan_dist < max_scan_dist && last_scan < max_scan_dist) 
                {
                    Gap detected_gap(frame, it - 1, last_scan, true);
                    detected_gap.addLeftInformation(it, scan_dist);
                    detected_gap.setMinSafeDist(min_dist);
                    // Inscribed radius gets enforced here, or unless using inflated egocircle,
                    // then no need for range diff
                    // Find equivalent passing length
                    Eigen::Vector2d orient_vec(1, 0);
                    Eigen::Vector2d m_pt_vec = detected_gap.get_middle_pt_vec();
                    // double epl = robot_geo_proc_.getDecayEquivalentPL(orient_vec, m_pt_vec, m_pt_vec.norm());
                    double epl = robot_geo_proc_.getLinearDecayEquivalentPL(orient_vec, m_pt_vec, m_pt_vec.norm());
                    if (detected_gap.get_dist_side() > epl) observed_gaps.push_back(detected_gap); //  || cfg_->planning.planning_inflated
                }
                
            }

            // Beginning or the end of a reading into infinity => swept gap
            if (last_scan < max_scan_dist != scan_dist < max_scan_dist)
            {
                // If previously marked gap, meaning ending of a gap
                if (prev_right_gap)
                {
                    prev_right_gap = false;
                    Gap detected_gap(frame, gap_right_idx, gap_right_dist);
                    detected_gap.addLeftInformation(it, scan_dist);
                    detected_gap.setMinSafeDist(min_dist);
                    // Inscribed radius gets enforced here, or unless using inflated egocircle,
                    // then no need for range diff
                    // Find equivalent passing length
                    Eigen::Vector2d orient_vec(1, 0);
                    Eigen::Vector2d m_pt_vec = detected_gap.get_middle_pt_vec();
                    // double epl = robot_geo_proc->.getDecayEquivalentPL(orient_vec, m_pt_vec, m_pt_vec.norm());
                    double epl = robot_geo_proc_.getLinearDecayEquivalentPL(orient_vec, m_pt_vec, m_pt_vec.norm());
                    if (detected_gap.get_dist_side() > epl) observed_gaps.push_back(detected_gap); //  || cfg_->planning.planning_inflated
                }
                else // previously not marked a gap, not marking the gap
                {
                    gap_right_idx = it - 1;
                    gap_right_dist = last_scan;
                    prev_right_gap = true;
                }
            }
            last_scan = scan_dist;
        }

        // Catch the last gap
        if (prev_right_gap) 
        {
            Gap detected_gap(frame, gap_right_idx, gap_right_dist);
            detected_gap.addLeftInformation(int(scan_.ranges.size() - 1), *(scan_.ranges.end() - 1));
            detected_gap.setMinSafeDist(min_dist);
            Eigen::Vector2d orient_vec(1, 0);
            Eigen::Vector2d m_pt_vec = detected_gap.get_middle_pt_vec();
            // double epl = robot_geo_proc_.getDecayEquivalentPL(orient_vec, m_pt_vec, m_pt_vec.norm());
            double epl = robot_geo_proc_.getLinearDecayEquivalentPL(orient_vec, m_pt_vec, m_pt_vec.norm());
            if (detected_gap._left_idx - detected_gap._right_idx > 500 || detected_gap.get_dist_side() > epl) observed_gaps.push_back(detected_gap);
        }
        
        // Bridge the last gap around
        if (observed_gaps.size() > 1)
        {
            if (observed_gaps[0].RIdx() == 0 && observed_gaps[observed_gaps.size() - 1].LIdx() == scan_.ranges.size() - 1) // Magic number?
            {
                // Both ends
                float start_side_dist = observed_gaps[0].LDist();
                float end_side_dist = observed_gaps[observed_gaps.size() - 1].RDist();
                int start_side_idx = observed_gaps[0].LIdx();
                int end_side_idx = observed_gaps[observed_gaps.size() - 1].RIdx();

                // float result = (end_side_dist - start_side_dist) * start_side_idx / (observed_gaps.size() - end_side_idx + start_side_idx) + start_side_dist;
                int total_size = 511 - end_side_idx + start_side_idx;
                float result = (end_side_dist - start_side_dist) * (float (start_side_idx) / float (total_size)) + start_side_dist;
                observed_gaps[0].setRightObs();
                observed_gaps[observed_gaps.size() - 1].setLeftObs();
                observed_gaps[observed_gaps.size() - 1].addLeftInformation(511, result);
                observed_gaps[0].setRDist(result);
            }
        }

        return observed_gaps;
    }

    std::vector<Gap> GapDetector::gapSimplification(const std::vector<Gap> & observed_gaps)
    {
        std::vector<Gap> second_gap;

        // int right_idx = -1;
        // int left_idx = -1;
        // float left_dist = 3;
        // float right_dist = 3; // TODO: Make this reconfigurable
        int observed_size = (int) observed_gaps.size();

        // sensor_msgs::LaserScan scan_ = *scanPtr.get();
        // Termination Condition

        // Insert first
        bool mark_to_start = true;
        bool last_type_left = true;
        int left_counter = 0;
        bool changed = true;
        for (int i = 0; i < (int) observed_gaps.size(); i++)
        {
            if (mark_to_start && observed_gaps.at(i).isRadial() && observed_gaps.at(i).isRightType())
            {
                // Wait until the first mergable gap aka swept left type gap
                mark_to_start = false;
                second_gap.push_back(observed_gaps[i]);
            } else {
                if (!mark_to_start)
                {
                    if (observed_gaps.at(i).isRadial())
                    {
                        if (observed_gaps.at(i).isRightType())
                        {
                            second_gap.push_back(observed_gaps[i]);
                        }
                        else
                        {
                            float curr_left_dist = observed_gaps[i].LDist();
                            int erase_counter = 0;
                            int last_mergable = -1;

                            // float coefs = cfg_->planning.planning_inflated ? 0 : 1;
                            for (int j = (int) (second_gap.size() - 1); j >= 0; j--)
                            {
                                int start_idx = std::min(second_gap[j].LIdx(), observed_gaps[i].RIdx());
                                int end_idx = std::max(second_gap[j].LIdx(), observed_gaps[i].RIdx());
                                auto farside_iter = std::min_element(scan_.ranges.begin() + start_idx, scan_.ranges.begin() + end_idx);
                                int farside_idx = farside_iter - scan_.ranges.begin();
                                // TODO: what number to use? Currently, use the max radius. The merging will not happen frequently.
                                // double max_r_er = robot_geo_proc_.getRobotMaxRadius();
                                double farside_angle = farside_idx * scan_.angle_increment + scan_.angle_min;
                                Eigen::Vector2d farside_vec(cos(farside_angle), sin(farside_angle));
                                Eigen::Vector2d orient_vec(1, 0);
                                double erl_left_dist = robot_geo_proc_.getLinearDecayEquivalentRL(orient_vec, farside_vec, curr_left_dist);
                                double erl_right_dist = robot_geo_proc_.getLinearDecayEquivalentRL(orient_vec, farside_vec, second_gap[j].RDist());
                                bool second_test = curr_left_dist <= (*farside_iter - erl_left_dist) && second_gap[j].RDist() <= (*farside_iter - erl_right_dist);
                                bool dist_diff = second_gap[j].isRightType() || !second_gap[j].isRadial();
                                bool idx_diff = observed_gaps[i].LIdx() - second_gap[j].RIdx() < cfg_->gap_manip.max_idx_diff;
                                if (second_test && dist_diff && idx_diff) {
                                    last_mergable = j;
                                } 
                            }

                            if (last_mergable != -1) {
                                second_gap.erase(second_gap.begin() + last_mergable + 1, second_gap.end());
                                second_gap.back().addLeftInformation(observed_gaps[i].LIdx(), observed_gaps[i].LDist());
                            } else {
                                second_gap.push_back(observed_gaps.at(i));
                            }
                        }
                    }
                    else
                    {
                        // If not radial gap, 
                        float curr_left_dist = observed_gaps.at(i).LDist();
                        if (std::abs(curr_left_dist - second_gap.back().RDist()) < 0.2 && second_gap.back().isRadial() && second_gap.back().isRightType())
                        {
                            second_gap.back().addLeftInformation(observed_gaps[i].LIdx(), observed_gaps[i].LDist());
                        } else {
                            second_gap.push_back(observed_gaps[i]);
                        }
                    }
                }
                else
                {
                    // A swept gap solely on its own
                    second_gap.push_back(observed_gaps[i]);
                }
            }
            last_type_left = observed_gaps[i].isRightType();
        }

        // observed_gaps.clear();
        // observed_gaps = second_gap;
        return second_gap;
    }


}