#include <quad_gap/global_plan_management/GlobalPlanManager.h>

namespace quad_gap 
{
    void GlobalPlanManager::updateGlobalPathMapFrame(const std::vector<geometry_msgs::msg::PoseStamped> & globalPlanMapFrame) 
    {
        // Incoming plan is in map frame
        boost::mutex::scoped_lock lock(goalSelectMutex_);
        boost::mutex::scoped_lock gplock(globalPlanMutex_);

        globalPlanMapFrame_ = globalPlanMapFrame;
        return;
    }

    void GlobalPlanManager::updateEgoCircle(std::shared_ptr<sensor_msgs::msg::LaserScan const> scan) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        scan_ = scan;
    }

    void GlobalPlanManager::generateGlobalPathLocalWaypoint(const geometry_msgs::msg::TransformStamped & map2rbt) 
    {
        boost::mutex::scoped_lock glock(goalSelectMutex_);
        boost::mutex::scoped_lock llock(scanMutex_);
        
        if (globalPlanMapFrame_.size() < 2) // No Global Path
            return;

        // ROS_INFO_STREAM_NAMED("GlobalPlanManager", "running generateGlobalPathLocalWaypoint");
        // getting snippet of global trajectory in robot frame (snippet is whatever part of global trajectory is within laser scan)
        std::vector<geometry_msgs::msg::PoseStamped> globalPlanSnippetRobotFrame = getVisibleGlobalPlanSnippetRobotFrame(map2rbt);
        
        if (globalPlanSnippetRobotFrame.size() < 1) // relevant global plan snippet
            return;

        globalPathLocalWaypointRobotFrame_ = globalPlanSnippetRobotFrame.back();
    }

    std::vector<geometry_msgs::msg::PoseStamped> GlobalPlanManager::getVisibleGlobalPlanSnippetRobotFrame(const geometry_msgs::msg::TransformStamped & map2rbt) 
    {
        // ROS_INFO_STREAM_NAMED("GlobalPlanManager", "getVisibleGlobalPlanSnippetRobotFrame");
        boost::mutex::scoped_lock gplock(globalPlanMutex_);
        std::vector<geometry_msgs::msg::PoseStamped> globalPlan = globalPlanMapFrame_;
        // where is globalPlanMapFrame_ coming from?
        // globalPlan = globalPlanMapFrame_;

        // if (globalPlan.size() == 0) {
        //     ROS_FATAL_STREAM_NAMED("GlobalPlanManager", "Global Plan Length = 0");
        // }

        // transforming plan into robot frame
        for (int i = 0; i < globalPlan.size(); i++)
            tf2::doTransform(globalPlan.at(i), globalPlan.at(i), map2rbt);

        // ROS_INFO_STREAM_NAMED("GlobalPlanManager", "mod plan size: " << globalPlan.size());
        std::vector<float> planPoseNorms(globalPlan.size());
        std::vector<float> scanDistsAtPlanIndices(globalPlan.size());
        std::vector<float> scanMinusPlanPoseNormDiffs(globalPlan.size());

        for (int i = 0; i < planPoseNorms.size(); i++) 
        {
            planPoseNorms.at(i) = poseNorm(globalPlan.at(i)); // calculating distance to robot at each step of plan
            scanDistsAtPlanIndices.at(i) = calculateScanRangesAtPlanIndices(globalPlan.at(i));
            scanMinusPlanPoseNormDiffs.at(i) = (scanDistsAtPlanIndices.at(i) - (0.5 * cfg_->rbt.r_inscr)) - planPoseNorms.at(i);
        }

        // Find closest pose to robot to start the global plan snippet
        auto closestPlanPose = std::min_element(planPoseNorms.begin(), planPoseNorms.end());
        int closestPlanPoseIdx = std::distance(planPoseNorms.begin(), closestPlanPose);
        // ROS_INFO_STREAM_NAMED("GlobalPlanManager", "closestPlanPoseIdx: " << closestPlanPoseIdx);

        // find_if returns iterator for which the predicate (second input) is true within range.

        // firstNotVisibleGlobalPlanPose is the first point within the global plan that lies beyond the current scan.
        auto firstNotVisibleGlobalPlanPose = std::find_if(scanMinusPlanPoseNormDiffs.begin() + closestPlanPoseIdx, 
                                                          scanMinusPlanPoseNormDiffs.end(),
                                                          std::bind1st(std::mem_fun(&GlobalPlanManager::isNegative), this));

        if (closestPlanPose == scanMinusPlanPoseNormDiffs.end()) 
        {
            // ROS_ERROR_STREAM_NAMED("GlobalPlanManager", "No Global Plan pose within Robot scan");
            return std::vector<geometry_msgs::msg::PoseStamped>(0);
        }

        int firstNotVisibleGlobalPlanPoseIdx = std::distance(scanMinusPlanPoseNormDiffs.begin(), firstNotVisibleGlobalPlanPose);

        std::vector<geometry_msgs::msg::PoseStamped> visibleGlobalPlanSnippetRobotFrame(globalPlan.begin() + closestPlanPoseIdx, 
                                                                                   globalPlan.begin() + firstNotVisibleGlobalPlanPoseIdx);
        return visibleGlobalPlanSnippetRobotFrame;
    }

    int GlobalPlanManager::poseIdxInScan(const geometry_msgs::msg::PoseStamped & pose) 
    {
        float orientation = getPoseOrientation(pose);
        int index = theta2idx(orientation);
        return index;
    }

    float GlobalPlanManager::getPoseOrientation(const geometry_msgs::msg::PoseStamped & pose) 
    {
        return std::atan2(pose.pose.position.y + 1e-3, pose.pose.position.x + 1e-3);
    }

    float GlobalPlanManager::poseNorm(const geometry_msgs::msg::PoseStamped & pose) 
    {
        return sqrt(pow(pose.pose.position.x, 2) + pow(pose.pose.position.y, 2));
    }

    float GlobalPlanManager::calculateScanRangesAtPlanIndices(const geometry_msgs::msg::PoseStamped & pose) 
    {
        sensor_msgs::msg::LaserScan scan = *scan_.get();

        int poseIdx = poseIdxInScan(pose);

        float scanRangeAtPoseIdx = scan.ranges.at(poseIdx);

        return scanRangeAtPoseIdx;
    }

    bool GlobalPlanManager::isNegative(const float dist) 
    {
        return dist <= 0.0;
    }

    // This should return something in odom frame
    geometry_msgs::msg::PoseStamped GlobalPlanManager::getGlobalPathLocalWaypointOdomFrame(const geometry_msgs::msg::TransformStamped & rbt2odom) 
    {
        geometry_msgs::msg::PoseStamped globalPathLocalWaypointOdomFrame;
        tf2::doTransform(globalPathLocalWaypointRobotFrame_, globalPathLocalWaypointOdomFrame, rbt2odom);
        
        return globalPathLocalWaypointOdomFrame;
    }

    std::vector<geometry_msgs::msg::PoseStamped> GlobalPlanManager::getGlobalPathOdomFrame() 
    {
        return globalPlanMapFrame_;
    }
}