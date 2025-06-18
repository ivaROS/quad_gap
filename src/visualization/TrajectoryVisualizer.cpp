#include <quad_gap/visualization/TrajectoryVisualizer.h>

namespace quad_gap
{
TrajectoryVisualizer::TrajectoryVisualizer(ros::NodeHandle& nh, const QuadGapConfig& cfg)
    {
        cfg_ = &cfg;
        trajSwitchIdxPublisher = nh.advertise<visualization_msgs::Marker>("trajectory_switch", 10);
        planLoopIdxPublisher = nh.advertise<visualization_msgs::Marker>("planning_loop_idx", 10);

        currentTrajectoryPublisher_ = nh.advertise<visualization_msgs::MarkerArray>("curr_exec_dg_traj", 1);

        globalPlanPublisher = nh.advertise<visualization_msgs::MarkerArray>("entire_global_plan", 10);
        
        gapTrajectoriesPublisher = nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1000);

        globalPlanSnippetPublisher = nh.advertise<visualization_msgs::MarkerArray>("relevant_global_plan_snippet", 10);

    }


    void TrajectoryVisualizer::drawCurrentTrajectory(const geometry_msgs::PoseArray & path)
    {
        // First, clearing topic.
        clearMarkerArrayPublisher(currentTrajectoryPublisher_);

        visualization_msgs::MarkerArray trajMarkerArray;
        visualization_msgs::Marker trajMarker;

        if (path.header.frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawCurrentTrajectory] Trajectory frame_id is empty");
            return;
        }

        trajMarker.header.frame_id = path.header.frame_id;
        trajMarker.header.stamp = path.header.stamp;
        trajMarker.ns = "currentTraj";
        trajMarker.type = visualization_msgs::Marker::ARROW;
        trajMarker.action = visualization_msgs::Marker::ADD;
        trajMarker.scale.x = 0.1;
        trajMarker.scale.y = 0.08; // 0.01;
        trajMarker.scale.z = 0.0001;
        trajMarker.color.a = 1;
        trajMarker.color.r = 1.0;
        trajMarker.color.g = 0.0;
        trajMarker.color.b = 0.0;

        trajMarker.lifetime = ros::Duration(0);     
        
        // geometry_msgs::PoseArray path = traj.getPathRbtFrame();
        for (const geometry_msgs::Pose & pose : path.poses) 
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

        visualization_msgs::Marker trajSwitchIdxMarker;

        if (cfg_->robot_frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawPlanningLoopIdx] Trajectory frame_id is empty");
            return; 
        }

        trajSwitchIdxMarker.header.frame_id = cfg_->robot_frame_id;
        trajSwitchIdxMarker.header.stamp = ros::Time::now();

        trajSwitchIdxMarker.ns = "planning_loop_idx";
        trajSwitchIdxMarker.id = 0;
        trajSwitchIdxMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        trajSwitchIdxMarker.action = visualization_msgs::Marker::ADD;
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

    void TrajectoryVisualizer::drawTrajectorySwitchCount(const int & trajSwitchIndex, const geometry_msgs::PoseArray & path) 
    {
        // First, clearing topic.
        clearMarkerPublisher(trajSwitchIdxPublisher);

        // geometry_msgs::PoseArray path = chosenTraj.getPathRbtFrame();
        geometry_msgs::Pose lastTrajPose = (path.poses.size() > 0) ? path.poses.back() : geometry_msgs::Pose();

        if (path.header.frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawTrajectorySwitchCount] Trajectory frame_id is empty");
            return; 
        }

        visualization_msgs::Marker trajSwitchIdxMarker;
        trajSwitchIdxMarker.header = path.header;
        trajSwitchIdxMarker.ns = "traj_switch_count";
        trajSwitchIdxMarker.id = 0;
        trajSwitchIdxMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        trajSwitchIdxMarker.action = visualization_msgs::Marker::ADD;
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

    void TrajectoryVisualizer::drawGlobalPlan(const std::vector<geometry_msgs::PoseStamped> & globalPlan) 
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

        visualization_msgs::MarkerArray globalPlanMarkerArray;
        visualization_msgs::Marker globalPlanMarker;

        globalPlanMarker.header.frame_id = globalPlan.at(0).header.frame_id;
        globalPlanMarker.header.stamp = globalPlan.at(0).header.stamp;
        globalPlanMarker.ns = "globalPlan";
        globalPlanMarker.type = visualization_msgs::Marker::ARROW;
        globalPlanMarker.action = visualization_msgs::Marker::ADD;
        globalPlanMarker.scale.x = 0.1;
        globalPlanMarker.scale.y = 0.04; // 0.01;
        globalPlanMarker.scale.z = 0.0001;
        globalPlanMarker.color.a = 1;
        globalPlanMarker.color.r = 1.0;
        globalPlanMarker.color.g = 0.0;
        globalPlanMarker.color.b = 0.0;

        globalPlanMarker.lifetime = ros::Duration(0);     
        
        for (const geometry_msgs::PoseStamped & poseStamped : globalPlan) 
        {
            globalPlanMarker.id = int (globalPlanMarkerArray.markers.size());
            globalPlanMarker.pose = poseStamped.pose;
            globalPlanMarkerArray.markers.push_back(globalPlanMarker);
        }

        // geometry_msgs::PoseArray globalPlanPoseArray;
        // globalPlanPoseArray.header = globalPlan.at(0).header;
        // for (const geometry_msgs::PoseStamped & pose : globalPlan) 
            // globalPlanPoseArray.poses.push_back(pose.pose);

        globalPlanPublisher.publish(globalPlanMarkerArray);
    }

    void TrajectoryVisualizer::drawGapTrajectories(const std::vector<geometry_msgs::PoseArray> & pose_arrays) 
    {
        // First, clearing topic.
        clearMarkerArrayPublisher(gapTrajectoriesPublisher);

        if (pose_arrays.size() == 0)
        {
            // ROS_WARN_STREAM_NAMED("Visualizer", "no trajectories to visualize");
            return;
        }
        
        visualization_msgs::MarkerArray gapTrajMarkerArray;
        visualization_msgs::Marker gapTrajMarker;

        geometry_msgs::PoseArray pose_array = pose_arrays.at(0);

        if (pose_array.header.frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawGapTrajectories] Trajectory frame_id is empty");
            return;
        }

        // The above makes this safe
        gapTrajMarker.header.frame_id = pose_array.header.frame_id;
        gapTrajMarker.header.stamp = pose_array.header.stamp;
        gapTrajMarker.ns = "allTraj";
        gapTrajMarker.type = visualization_msgs::Marker::ARROW;
        gapTrajMarker.action = visualization_msgs::Marker::ADD;
        gapTrajMarker.scale.x = 0.1;
        gapTrajMarker.scale.y = 0.04; // 0.01;
        gapTrajMarker.scale.z = 0.0001;
        gapTrajMarker.color.a = 1;
        gapTrajMarker.color.b = 1.0;
        gapTrajMarker.color.g = 1.0;
        gapTrajMarker.lifetime = ros::Duration(0);

        for (const geometry_msgs::PoseArray & pose_array : pose_arrays) 
        {
            // geometry_msgs::PoseArray path = traj.getPathRbtFrame();
            for (const geometry_msgs::Pose & pose : pose_array.poses) 
            {
                gapTrajMarker.id = int (gapTrajMarkerArray.markers.size());
                gapTrajMarker.pose = pose;
                gapTrajMarkerArray.markers.push_back(gapTrajMarker);
            }
        }

        gapTrajectoriesPublisher.publish(gapTrajMarkerArray);
    }

    // void TrajectoryVisualizer::pubAllTraj(const std::vector<geometry_msgs::PoseArray> & prr) 
    // {
    //     // First, clearing topic.
    //     clearMarkerArrayPublisher(gapTrajectoriesPublisher);

    //     if (!cfg_->gap_viz.debug_viz) return;
    //     visualization_msgs::MarkerArray vis_traj_arr;
    //     visualization_msgs::Marker lg_marker;
    //     if (prr.size() == 0)
    //     {
    //         ROS_WARN_STREAM("traj count length 0");
    //         return;
    //     }

    //     // The above makes this safe
    //     lg_marker.header.frame_id = prr.at(0).header.frame_id;
    //     lg_marker.header.stamp = ros::Time::now();
    //     lg_marker.ns = "allTraj";
    //     lg_marker.type = visualization_msgs::Marker::ARROW;
    //     lg_marker.action = visualization_msgs::Marker::ADD;
    //     lg_marker.scale.x = 0.1;
    //     lg_marker.scale.y = cfg_->gap_viz.fig_gen ? 0.02 : 0.01;// 0.01;
    //     lg_marker.scale.z = 0.1;
    //     lg_marker.color.a = 1;
    //     lg_marker.color.r = 0.5;
    //     lg_marker.color.g = 0.5;
    //     lg_marker.lifetime = ros::Duration(0.25);

    //     for (auto & arr : prr) {
    //         for (auto pose : arr.poses) {
    //             lg_marker.id = int (vis_traj_arr.markers.size());
    //             lg_marker.pose = pose;
    //             vis_traj_arr.markers.push_back(lg_marker);
    //         }
    //     }
    //     gapTrajectoriesPublisher.publish(vis_traj_arr);
    // }

    void TrajectoryVisualizer::drawRelevantGlobalPlanSnippet(const std::vector<geometry_msgs::PoseStamped> & globalPlanSnippet) 
    {
        // First, clearing topic.
        clearMarkerArrayPublisher(globalPlanSnippetPublisher);

        if (globalPlanSnippet.empty())             // Should be safe with this check
        {
            ROS_WARN_STREAM("Goal Selector Returned Trajectory Size " << globalPlanSnippet.size() << " < 1");
            return;
        }    
        
        if (globalPlanSnippet.at(0).header.frame_id.empty())
        {
            ROS_WARN_STREAM("[drawRelevantGlobalPlanSnippet] Trajectory frame_id is empty");
            return;
        }

        visualization_msgs::MarkerArray globalPlanSnippetMarkerArray;
        visualization_msgs::Marker globalPlanSnippetMarker;

        // The above makes this safe
        globalPlanSnippetMarker.header.frame_id = globalPlanSnippet.at(0).header.frame_id;
        globalPlanSnippetMarker.header.stamp = globalPlanSnippet.at(0).header.stamp;
        globalPlanSnippetMarker.ns = "globalPlanSnippet";
        globalPlanSnippetMarker.type = visualization_msgs::Marker::ARROW;
        globalPlanSnippetMarker.action = visualization_msgs::Marker::ADD;
        globalPlanSnippetMarker.scale.x = 0.1;
        globalPlanSnippetMarker.scale.y = 0.04; // 0.01;
        globalPlanSnippetMarker.scale.z = 0.0001;
        globalPlanSnippetMarker.color.a = 1;
        globalPlanSnippetMarker.color.r = 1.0;
        globalPlanSnippetMarker.lifetime = ros::Duration(0);

        for (const geometry_msgs::PoseStamped & poseStamped : globalPlanSnippet) 
        {
            globalPlanSnippetMarker.id = int (globalPlanSnippetMarkerArray.markers.size());
            globalPlanSnippetMarker.pose = poseStamped.pose;
            globalPlanSnippetMarkerArray.markers.push_back(globalPlanSnippetMarker);
        }

        globalPlanSnippetPublisher.publish(globalPlanSnippetMarkerArray);

        // geometry_msgs::PoseArray globalPlanSnippetPoseArray;

        // globalPlanSnippetPoseArray.header = globalPlanSnippet.at(0).header;

        // for (const geometry_msgs::PoseStamped & pose : globalPlanSnippet) 
        //     globalPlanSnippetPoseArray.poses.push_back(pose.pose);

        // globalPlanSnippetPublisher.publish(globalPlanSnippetPoseArray);
    }

}