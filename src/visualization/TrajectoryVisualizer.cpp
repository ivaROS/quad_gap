#include <quad_gap/visualization/TrajectoryVisualizer.h>

namespace quad_gap
{
TrajectoryVisualizer::TrajectoryVisualizer(const rclcpp::Node::SharedPtr & node, const QuadGapConfig& cfg)
    {
        cfg_ = &cfg;
        trajSwitchIdxPublisher = node->create_publisher<visualization_msgs::msg::Marker>("trajectory_switch", 10);
        planLoopIdxPublisher = node->create_publisher<visualization_msgs::msg::Marker>("planning_loop_idx", 10);

        currentTrajectoryPublisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("curr_exec_dg_traj", 1);

        globalPlanPublisher = node->create_publisher<visualization_msgs::msg::MarkerArray>("entire_global_plan", 10);
        
        gapTrajectoriesPublisher = node->create_publisher<visualization_msgs::msg::MarkerArray>("candidate_trajectories", 1000);

        globalPlanSnippetPublisher = node->create_publisher<visualization_msgs::msg::MarkerArray>("relevant_global_plan_snippet", 10);
    }


    void TrajectoryVisualizer::drawCurrentTrajectory(const Trajectory & traj)
    {
        // First, clearing topic.
        clearMarkerArrayPublisher(currentTrajectoryPublisher_);

        visualization_msgs::msg::MarkerArray trajMarkerArray;
        visualization_msgs::msg::Marker trajMarker;

        if (traj.getPathRbtFrame().header.frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawCurrentTrajectory] Trajectory frame_id is empty");
            return;
        }

        trajMarker.header.frame_id = traj.getPathRbtFrame().header.frame_id;
        trajMarker.header.stamp = traj.getPathRbtFrame().header.stamp;
        trajMarker.ns = "currentTraj";
        trajMarker.type = visualization_msgs::msg::Marker::ARROW;
        trajMarker.action = visualization_msgs::msg::Marker::ADD;
        trajMarker.scale.x = 0.1;
        trajMarker.scale.y = 0.08; // 0.01;
        trajMarker.scale.z = 0.0001;
        trajMarker.color.a = 1;
        trajMarker.color.r = 1.0;
        trajMarker.color.g = 0.0;
        trajMarker.color.b = 0.0;

        trajMarker.lifetime = ros::Duration(0);     
        
        geometry_msgs::msg::PoseArray path = traj.getPathRbtFrame();
        for (const geometry_msgs::msg::Pose & pose : path.poses) 
        {
            trajMarker.id = int (trajMarkerArray.markers.size());
            trajMarker.pose = pose;
            trajMarkerArray.markers.push_back(trajMarker);
        }
    
        currentTrajectoryPublisher_.publish(trajMarkerArray);
    }

    void TrajectoryVisualizer::drawPlanningLoopIdx(const int & planningLoopIdx) 
    {
        // First, clearing topic.
        clearMarkerPublisher(planLoopIdxPublisher);

        visualization_msgs::msg::Marker trajSwitchIdxMarker;

        if (cfg_->robot_frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawPlanningLoopIdx] Trajectory frame_id is empty");
            return; 
        }

        trajSwitchIdxMarker.header.frame_id = cfg_->robot_frame_id;
        trajSwitchIdxMarker.header.stamp = rclcpp::Time::now();

        trajSwitchIdxMarker.ns = "planning_loop_idx";
        trajSwitchIdxMarker.id = 0;
        trajSwitchIdxMarker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        trajSwitchIdxMarker.action = visualization_msgs::msg::Marker::ADD;
        trajSwitchIdxMarker.pose.position.x = 0.0;
        trajSwitchIdxMarker.pose.position.y = 0.0;
        trajSwitchIdxMarker.pose.position.z = 0.05;
        trajSwitchIdxMarker.pose.orientation.w = 1.0;
        trajSwitchIdxMarker.pose.orientation.x = 0.0;
        trajSwitchIdxMarker.pose.orientation.y = 0.0;
        trajSwitchIdxMarker.pose.orientation.z = 0.0;

        trajSwitchIdxMarker.scale.z = 0.3;
        trajSwitchIdxMarker.color.a = 1.0; // Don't forget to set the alpha!
        trajSwitchIdxMarker.color.r = 0.0;
        trajSwitchIdxMarker.color.g = 0.0;
        trajSwitchIdxMarker.color.b = 0.0;
        trajSwitchIdxMarker.text = "PLAN: " + std::to_string(planningLoopIdx);
        planLoopIdxPublisher.publish(trajSwitchIdxMarker);
    }

    void TrajectoryVisualizer::drawTrajectorySwitchCount(const int & trajSwitchIndex, const Trajectory & traj) 
    {
        // First, clearing topic.
        clearMarkerPublisher(trajSwitchIdxPublisher);

        geometry_msgs::msg::PoseArray path = traj.getPathRbtFrame();

        // geometry_msgs::msg::PoseArray path = chosenTraj.getPathRbtFrame();
        geometry_msgs::msg::Pose lastTrajPose = (path.poses.size() > 0) ? path.poses.back() : geometry_msgs::msg::Pose();

        if (path.header.frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawTrajectorySwitchCount] Trajectory frame_id is empty");
            return; 
        }

        visualization_msgs::msg::Marker trajSwitchIdxMarker;
        trajSwitchIdxMarker.header = path.header;
        trajSwitchIdxMarker.ns = "traj_switch_count";
        trajSwitchIdxMarker.id = 0;
        trajSwitchIdxMarker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        trajSwitchIdxMarker.action = visualization_msgs::msg::Marker::ADD;
        trajSwitchIdxMarker.pose.position = lastTrajPose.position;
        trajSwitchIdxMarker.pose.orientation = lastTrajPose.orientation;;
        trajSwitchIdxMarker.scale.z = 0.3;
        trajSwitchIdxMarker.color.a = 1.0; // Don't forget to set the alpha!
        trajSwitchIdxMarker.color.r = 0.0;
        trajSwitchIdxMarker.color.g = 0.0;
        trajSwitchIdxMarker.color.b = 0.0;
        trajSwitchIdxMarker.text = "SWITCH: " + std::to_string(trajSwitchIndex);
        trajSwitchIdxPublisher.publish(trajSwitchIdxMarker);
    }

    void TrajectoryVisualizer::drawGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped> & globalPlan) 
    {
        // First, clearing topic.
        clearMarkerArrayPublisher(globalPlanPublisher);

        if (globalPlan.empty()) 
            ROS_WARN_STREAM_NAMED("Visualizer", "Goal Selector Returned Trajectory Size 0");

        if (globalPlan.at(0).header.frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawGlobalPlan] Trajectory frame_id is empty");
            return;
        }

        visualization_msgs::msg::MarkerArray globalPlanMarkerArray;
        visualization_msgs::msg::Marker globalPlanMarker;

        globalPlanMarker.header.frame_id = globalPlan.at(0).header.frame_id;
        globalPlanMarker.header.stamp = globalPlan.at(0).header.stamp;
        globalPlanMarker.ns = "globalPlan";
        globalPlanMarker.type = visualization_msgs::msg::Marker::ARROW;
        globalPlanMarker.action = visualization_msgs::msg::Marker::ADD;
        globalPlanMarker.scale.x = 0.1;
        globalPlanMarker.scale.y = 0.04; // 0.01;
        globalPlanMarker.scale.z = 0.0001;
        globalPlanMarker.color.a = 1;
        globalPlanMarker.color.r = 1.0;
        globalPlanMarker.color.g = 0.0;
        globalPlanMarker.color.b = 0.0;

        globalPlanMarker.lifetime = ros::Duration(0);     
        
        for (const geometry_msgs::msg::PoseStamped & poseStamped : globalPlan) 
        {
            globalPlanMarker.id = int (globalPlanMarkerArray.markers.size());
            globalPlanMarker.pose = poseStamped.pose;
            globalPlanMarkerArray.markers.push_back(globalPlanMarker);
        }

        // geometry_msgs::msg::PoseArray globalPlanPoseArray;
        // globalPlanPoseArray.header = globalPlan.at(0).header;
        // for (const geometry_msgs::msg::PoseStamped & pose : globalPlan) 
            // globalPlanPoseArray.poses.push_back(pose.pose);

        globalPlanPublisher.publish(globalPlanMarkerArray);
    }

    void TrajectoryVisualizer::drawGapTrajectories(const std::vector<Trajectory> & trajs) 
    {
        // First, clearing topic.
        clearMarkerArrayPublisher(gapTrajectoriesPublisher);
        if (trajs.size() == 0)
        {
            // ROS_WARN_STREAM_NAMED("Visualizer", "no trajectories to visualize");
            return;
        }
        
        visualization_msgs::msg::MarkerArray gapTrajMarkerArray;
        visualization_msgs::msg::Marker gapTrajMarker;

        Trajectory traj = trajs.at(0);

        if (traj.getPathRbtFrame().header.frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawGapTrajectories] Trajectory frame_id is empty");
            return;
        }

        // The above makes this safe
        gapTrajMarker.header.frame_id = traj.getPathRbtFrame().header.frame_id;
        gapTrajMarker.header.stamp = traj.getPathRbtFrame().header.stamp;
        gapTrajMarker.ns = "allTraj";
        gapTrajMarker.type = visualization_msgs::msg::Marker::ARROW;
        gapTrajMarker.action = visualization_msgs::msg::Marker::ADD;
        gapTrajMarker.scale.x = 0.1;
        gapTrajMarker.scale.y = 0.04; // 0.01;
        gapTrajMarker.scale.z = 0.0001;
        gapTrajMarker.color.a = 1;
        gapTrajMarker.color.b = 1.0;
        gapTrajMarker.color.g = 1.0;
        gapTrajMarker.lifetime = ros::Duration(0);

        for (const Trajectory & traj : trajs) 
        {
            geometry_msgs::msg::PoseArray path = traj.getPathRbtFrame();
            for (const geometry_msgs::msg::Pose & pose : path.poses) 
            {
                gapTrajMarker.id = int (gapTrajMarkerArray.markers.size());
                gapTrajMarker.pose = pose;
                gapTrajMarkerArray.markers.push_back(gapTrajMarker);
            }
        }

        gapTrajectoriesPublisher.publish(gapTrajMarkerArray);
    }

    void TrajectoryVisualizer::drawRelevantGlobalPlanSnippet(const std::vector<geometry_msgs::msg::PoseStamped> & globalPlanSnippet) 
    {
        // First, clearing topic.
        clearMarkerArrayPublisher(globalPlanSnippetPublisher);

        if (globalPlanSnippet.empty())             // Should be safe with this check
        {
            ROS_WARN_STREAM_NAMED("TrajectoryVisualizer", "Goal Selector Returned Trajectory Size " << globalPlanSnippet.size() << " < 1");
            return;
        }    
        
        if (globalPlanSnippet.at(0).header.frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("TrajectoryVisualizer", "[drawRelevantGlobalPlanSnippet] Trajectory frame_id is empty");
            return;
        }

        visualization_msgs::msg::MarkerArray globalPlanSnippetMarkerArray;
        visualization_msgs::msg::Marker globalPlanSnippetMarker;

        // The above makes this safe
        globalPlanSnippetMarker.header.frame_id = globalPlanSnippet.at(0).header.frame_id;
        globalPlanSnippetMarker.header.stamp = globalPlanSnippet.at(0).header.stamp;
        globalPlanSnippetMarker.ns = "globalPlanSnippet";
        globalPlanSnippetMarker.type = visualization_msgs::msg::Marker::ARROW;
        globalPlanSnippetMarker.action = visualization_msgs::msg::Marker::ADD;
        globalPlanSnippetMarker.scale.x = 0.1;
        globalPlanSnippetMarker.scale.y = 0.04; // 0.01;
        globalPlanSnippetMarker.scale.z = 0.0001;
        globalPlanSnippetMarker.color.a = 1;
        globalPlanSnippetMarker.color.r = 1.0;
        globalPlanSnippetMarker.lifetime = ros::Duration(0);

        for (const geometry_msgs::msg::PoseStamped & poseStamped : globalPlanSnippet) 
        {
            globalPlanSnippetMarker.id = int (globalPlanSnippetMarkerArray.markers.size());
            globalPlanSnippetMarker.pose = poseStamped.pose;
            globalPlanSnippetMarkerArray.markers.push_back(globalPlanSnippetMarker);
        }

        globalPlanSnippetPublisher.publish(globalPlanSnippetMarkerArray);

        // geometry_msgs::msg::PoseArray globalPlanSnippetPoseArray;

        // globalPlanSnippetPoseArray.header = globalPlanSnippet.at(0).header;

        // for (const geometry_msgs::msg::PoseStamped & pose : globalPlanSnippet) 
        //     globalPlanSnippetPoseArray.poses.push_back(pose.pose);

        // globalPlanSnippetPublisher.publish(globalPlanSnippetPoseArray);
    }

}