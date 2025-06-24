#include <quad_gap/trajectory_evaluation/TrajectoryEvaluator.h>

namespace quad_gap 
{
    TrajectoryEvaluator::TrajectoryEvaluator(const QuadGapConfig& cfg, RobotGeometryProcessor& robot_geo_proc)
    {
        cfg_ = & cfg;
        robot_geo_proc_ = & robot_geo_proc;
    }

    void TrajectoryEvaluator::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        scan_ = msg;
    }
    void TrajectoryEvaluator::updateGapContainer(const std::vector<Gap *> & observed_gaps) 
    {
        boost::mutex::scoped_lock lock(gap_mutex);
        gaps.clear();
        gaps = observed_gaps;
    }

    void TrajectoryEvaluator::transformGlobalPathLocalWaypointToRbtFrame(const geometry_msgs::PoseStamped & globalPathLocalWaypointOdomFrame, 
                                                                            const geometry_msgs::TransformStamped & odom2rbt) 
    {
        boost::mutex::scoped_lock lock(globalPlanMutex_);
        tf2::doTransform(globalPathLocalWaypointOdomFrame, globalPathLocalWaypointRobotFrame_, odom2rbt);
    }

    float TrajectoryEvaluator::costFn(Gap * g, int goal_idx)
    {
        // This is in rbt frame
        int leftdist = std::abs(g->RIdx() - goal_idx);
        int rightdist = std::abs(g->LIdx() - goal_idx);
        return std::min(leftdist, rightdist);
    }

    // Does things in rbt frame
    std::vector<float> TrajectoryEvaluator::scoreGaps()
    {
        boost::mutex::scoped_lock planlock(globalPlanMutex_);
        boost::mutex::scoped_lock egolock(scanMutex_);
        if (gaps.size() < 1) 
        {
            ROS_WARN_STREAM("Observed num of gap: 0");
            return std::vector<float>(0);
        }

        // How fix this
        int num_of_scan = scan_.get()->ranges.size();
        float goal_orientation = std::atan2(globalPathLocalWaypointRobotFrame_.pose.position.y, globalPathLocalWaypointRobotFrame_.pose.position.x);
        int idx = goal_orientation / (M_PI / (num_of_scan / 2)) + (num_of_scan / 2);
        ROS_DEBUG_STREAM("Goal Orientation: " << goal_orientation << ", idx: " << idx);
        ROS_DEBUG_STREAM(globalPathLocalWaypointRobotFrame_.pose.position);

        std::vector<float> cost(gaps.size());
        for (int i = 0; i < cost.size(); i++) 
        {
            cost.at(i) = costFn(gaps.at(i), idx);
        }

        return cost;
    }

    // Again, in rbt frame
    std::vector<float> TrajectoryEvaluator::scoreTrajectories(const std::vector<geometry_msgs::PoseArray> & sample_traj) 
    {
        // This will be in robot frame
        
        return std::vector<float>(sample_traj.size());
    }

    void TrajectoryEvaluator::scoreTrajectory(const geometry_msgs::PoseArray & traj,
                                                std::vector<float> & posewiseCosts,
                                                float & terminalPoseCost) 
    {
        ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "[scoreTrajectory()]");

        // Requires LOCAL FRAME
        // Should be no racing condition

        sensor_msgs::LaserScan scan = *scan_.get();

        posewiseCosts = std::vector<float>(traj.poses.size());
        for (int i = 0; i < posewiseCosts.size(); i++) 
        {
            ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "  Pose " << i);
            posewiseCosts.at(i) = scorePose(traj.poses.at(i), scan);
        }

        // float total_val = std::accumulate(cost_val.begin(), cost_val.end(), float(0));

        if (!traj.poses.empty()) // && ! cost_val.at(0) == -std::numeric_limits<float>::infinity())
        {
            terminalPoseCost = cfg_->traj.terminal_weight * terminalGoalCost(traj.poses.back());
            // if (terminal_cost < 1 && total_val > -10) return std::vector<float>(traj.poses.size(), 100);
            // Should be safe
            // cost_val.at(0) -= terminal_cost;
        } else
        {
            terminalPoseCost = -std::numeric_limits<float>::infinity();
            // ROS_WARN_STREAM("Empty trajectory, terminal cost set to -inf");
            // return std::vector<float>(traj.poses.size(), -std::numeric_limits<float>::infinity());
        }
        
        return;
    }

    float TrajectoryEvaluator::terminalGoalCost(const geometry_msgs::Pose & pose) 
    {
        boost::mutex::scoped_lock planlock(globalPlanMutex_);
        // ROS_INFO_STREAM(pose);
        float dx = pose.position.x - globalPathLocalWaypointRobotFrame_.pose.position.x;
        float dy = pose.position.y - globalPathLocalWaypointRobotFrame_.pose.position.y;
        return sqrt(pow(dx, 2) + pow(dy, 2));
    }

    float TrajectoryEvaluator::dist2Pose(const float & theta, const float & dist, const geometry_msgs::Pose & pose) 
    {
        float x = dist * std::cos(theta);
        float y = dist * std::sin(theta);
        return sqrt(pow(pose.position.x - x, 2) + pow(pose.position.y - y, 2));
    }

    float TrajectoryEvaluator::scorePose(const geometry_msgs::Pose & pose, const sensor_msgs::LaserScan & scan) 
    {
        // boost::mutex::scoped_lock lock(scanMutex_);

        ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "[scorePose()]");

        // float pose_ori = std::atan2(pose.position.y + 1e-3, pose.position.x + 1e-3);
        // int center_idx = (int) std::round((pose_ori + M_PI) / msg.get()->angle_increment);
        
        // int scan_size = (int) ;
        std::vector<float> dist(scan.ranges.size());
        // std::vector<float> rmax_offset(scan_size);

        // This size **should** be ensured
        // if (scan.ranges.size() < 500) 
        // {
        //     ROS_FATAL_STREAM("Scan range incorrect scorePose");
        // }

        // Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        // Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
        float yaw = quaternionToYaw(pose.orientation);
        Eigen::Vector2f orient_vec(cos(yaw), sin(yaw));
        Eigen::Vector2f poseVec(pose.position.x, pose.position.y); // TODO: pose should be in robot frame

        ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "  Pose: " << poseVec.transpose() << 
                                                     ", Orientation: " << orient_vec.transpose());

        float poseTheta = atan2(poseVec[1], poseVec[0]);
        int poseIdx = theta2idx(poseTheta);

        ROS_INFO_STREAM("Pose Index: " << poseIdx << ", Pose Theta: " << poseTheta);

        float poseIdxRange = scan.ranges.at(poseIdx);

        ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "  Pose Idx Range: " << poseIdxRange);

        if (poseVec.norm() >= poseIdxRange)
            return -std::numeric_limits<float>::infinity();

        float range_i = 0.0;
        float theta_i = 0.0;
        Eigen::Vector2f scanPt;
        Eigen::Vector2f rel_pt_vec;
        // float nearest_dist = 0.0;
        for (int i = 0; i < dist.size(); i++) 
        {
            float range_i = scan.ranges.at(i);
            float theta_i = idx2theta(i);
            scanPt << range_i * cos(theta_i), range_i * sin(theta_i);
            
            // range_i = range_i == 3 ? range_i + cfg_->traj.rmax : range_i;

            // Iterate through robot boundary
            
            // float pt_ang = i * scan.angle_increment - M_PI;
            // pt_ang = (pt_ang >= -M_PI) ? pt_ang : -M_PI;
            // pt_ang = (pt_ang <= M_PI) ? pt_ang : M_PI;

            // Eigen::Vector2f pt_vec(cos(pt_ang), sin(pt_ang));
            // pt_vec = range_i * pt_vec;
            
            rel_pt_vec = scanPt - poseVec;
            // nearest_dist = 
            dist.at(i) = robot_geo_proc_->getNearestDistance(orient_vec, rel_pt_vec);
            // ROS_INFO_STREAM(dist.at(i));
            // rmax_offset.at(i) = rmax - robot_geo_proc_.getRobotMaxRadius() * cfg_->traj.inf_ratio;
            
            // Get the robot equivalent radius
            
            // Eigen::Vector2f pose_position_vec(pose.position.x, pose.position.y);
            // Eigen::Vector2f scan_pt_vec(range_i * cos(i * scan.angle_increment - M_PI), range_i * sin(i * scan.angle_increment - M_PI));
            // Eigen::Vector2f relative_vec = scan_pt_vec - pose_position_vec;
            // relative_vec = relative_vec / relative_vec.norm();
            // float robot_er = robot_geo_proc_.getEquivalentR(orient_vec, relative_vec);
            // dist.at(i) = dist2Pose(i * scan.angle_increment - M_PI,
            //     range_i, pose);
            // dist.at(i) -= robot_er * cfg_->traj.inf_ratio;
            // rmax_offset.at(i) = rmax - robot_er * cfg_->traj.inf_ratio;
        }

        auto iter = std::min_element(dist.begin(), dist.end());

        ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "  Min dist: " << *iter);

        // float rmax_offset_val = rmax_offset[iter - dist.begin()];
        float rmax_offset_val = cfg_->traj.rmax - robot_geo_proc_->getRobotMaxRadius() * cfg_->traj.inf_ratio;
        return chapterScore(*iter, rmax_offset_val);
    }

    float TrajectoryEvaluator::chapterScore(const float & d, const float & rmax_offset_val) 
    {
        if (d <= 0) 
            return -std::numeric_limits<float>::infinity();
        
        if (d > rmax_offset_val) 
            return 0;
        
        return cfg_->traj.cobs * std::exp(- cfg_->traj.w * (d));
    }

    Gap * TrajectoryEvaluator::returnAndScoreGaps() 
    {
        boost::mutex::scoped_lock gaplock(gap_mutex);
        std::vector<float> cost = scoreGaps();
        auto decision_iter = std::min_element(cost.begin(), cost.end());
        int gap_idx = std::distance(cost.begin(), decision_iter);
        // ROS_INFO_STREAM("Selected Gap Index " << gap_idx);
        Gap * selected_gap = gaps.at(gap_idx);
        return selected_gap;
    }

    
}