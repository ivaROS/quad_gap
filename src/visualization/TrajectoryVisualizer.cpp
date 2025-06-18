#include <quad_gap/visualization/TrajectoryVisualizer.h>

namespace quad_gap
{
TrajectoryVisualizer::TrajectoryVisualizer(ros::NodeHandle& nh, const QuadGapConfig& cfg)
    {
        cfg_ = &cfg;
        goal_selector_traj_vis = nh.advertise<geometry_msgs::PoseArray>("goal_select_traj", 1000);
        trajectory_score = nh.advertise<visualization_msgs::MarkerArray>("traj_score", 1000);
        all_traj_viz = nh.advertise<visualization_msgs::MarkerArray>("all_traj_vis", 1000);
        globalPlanSnippetPublisher = nh.advertise<visualization_msgs::MarkerArray>("relevant_global_plan_snippet", 10);

    }

    void TrajectoryVisualizer::rawGlobalPlan(const std::vector<geometry_msgs::PoseStamped> & plan) 
    {
        if (!cfg_->gap_viz.debug_viz) return;
        if (plan.size() < 1) {
            ROS_WARN_STREAM("Goal Selector Returned Trajectory Size " << plan.size() << " < 1");
        }

        geometry_msgs::PoseArray vis_arr;
        vis_arr.header = plan.at(0).header;
        for (auto & pose : plan) {
            vis_arr.poses.push_back(pose.pose);
        }
        goal_selector_traj_vis.publish(vis_arr);
    }

    void TrajectoryVisualizer::trajScore(const geometry_msgs::PoseArray & p_arr, const std::vector<double> & p_score) 
    {
        // First, clearing topic.
        clearMarkerArrayPublisher(trajectory_score);

        if (!cfg_->gap_viz.debug_viz) return;

        ROS_FATAL_STREAM_COND(!p_score.size() == p_arr.poses.size(), "trajScore size mismatch, p_arr: "
            << p_arr.poses.size() << ", p_score: " << p_score.size());

        visualization_msgs::MarkerArray score_arr;
        visualization_msgs::Marker lg_marker;
        lg_marker.header.frame_id = p_arr.header.frame_id;
        lg_marker.header.stamp = ros::Time::now();
        lg_marker.ns = "trajScore";
        lg_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        lg_marker.action = visualization_msgs::Marker::ADD;
        lg_marker.pose.orientation.w = 1;
        lg_marker.scale.x = 0.1;
        lg_marker.scale.y = 0.1;
        lg_marker.scale.z = 0.1;

        lg_marker.color.a = 1;
        lg_marker.color.r = 1;
        lg_marker.color.g = 1;
        lg_marker.color.b = 1;

        for (int i = 0; i < p_score.size(); i++) {
            lg_marker.id = int (score_arr.markers.size());
            lg_marker.pose.position.x = p_arr.poses.at(i).position.x;
            lg_marker.pose.position.y = p_arr.poses.at(i).position.y;
            lg_marker.pose.position.z = 0.5;
            lg_marker.text = std::to_string(p_score.at(i));
            score_arr.markers.push_back(lg_marker);
        }

        trajectory_score.publish(score_arr);
    }

    void TrajectoryVisualizer::pubAllScore(const std::vector<geometry_msgs::PoseArray> & prr, 
                                            const std::vector<std::vector<double>> & cost) 
    {
        // First, clearing topic.
        clearMarkerArrayPublisher(trajectory_score);

        if (!cfg_->gap_viz.debug_viz) return;
        visualization_msgs::MarkerArray score_arr;
        visualization_msgs::Marker lg_marker;
        if (prr.size() == 0)
        {
            ROS_WARN_STREAM("traj count length 0");
            return;
        }

        // The above ensures this is safe
        lg_marker.header.frame_id = prr.at(0).header.frame_id;
        lg_marker.header.stamp = ros::Time::now();
        lg_marker.ns = "trajScore";
        lg_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        lg_marker.action = visualization_msgs::Marker::ADD;
        lg_marker.pose.orientation.w = 1;
        lg_marker.scale.x = 0.1;
        lg_marker.scale.y = 0.1;
        lg_marker.scale.z = 0.05;

        lg_marker.color.a = 1;
        lg_marker.color.r = 1;
        lg_marker.color.g = 1;
        lg_marker.color.b = 1;
        lg_marker.lifetime = ros::Duration(0.25);


        ROS_FATAL_STREAM_COND(!prr.size() == cost.size(), "pubAllScore size mismatch, prr: "
            << prr.size() << ", cost: " << cost.size());

        for (int i = 0; i < prr.size(); i++) {

            ROS_FATAL_STREAM_COND(!prr.at(i).poses.size() == cost.at(i).size(), "pubAllScore size mismatch," << i << "th "
                << prr.at(i).poses.size() << ", cost: " << cost.at(i).size());
            
            for (int j = 0; j < prr.at(i).poses.size(); j++) {
                lg_marker.id = int (score_arr.markers.size());
                lg_marker.pose = prr.at(i).poses.at(j);

                std::stringstream stream;
                stream << std::fixed << std::setprecision(2) << cost.at(i).at(j);
                lg_marker.text = stream.str();

                score_arr.markers.push_back(lg_marker);
            }
        }
        trajectory_score.publish(score_arr);
    }

    void TrajectoryVisualizer::pubAllTraj(const std::vector<geometry_msgs::PoseArray> & prr) 
    {
        // First, clearing topic.
        clearMarkerArrayPublisher(all_traj_viz);

        if (!cfg_->gap_viz.debug_viz) return;
        visualization_msgs::MarkerArray vis_traj_arr;
        visualization_msgs::Marker lg_marker;
        if (prr.size() == 0)
        {
            ROS_WARN_STREAM("traj count length 0");
            return;
        }

        // The above makes this safe
        lg_marker.header.frame_id = prr.at(0).header.frame_id;
        lg_marker.header.stamp = ros::Time::now();
        lg_marker.ns = "allTraj";
        lg_marker.type = visualization_msgs::Marker::ARROW;
        lg_marker.action = visualization_msgs::Marker::ADD;
        lg_marker.scale.x = 0.1;
        lg_marker.scale.y = cfg_->gap_viz.fig_gen ? 0.02 : 0.01;// 0.01;
        lg_marker.scale.z = 0.1;
        lg_marker.color.a = 1;
        lg_marker.color.r = 0.5;
        lg_marker.color.g = 0.5;
        lg_marker.lifetime = ros::Duration(0.25);

        for (auto & arr : prr) {
            for (auto pose : arr.poses) {
                lg_marker.id = int (vis_traj_arr.markers.size());
                lg_marker.pose = pose;
                vis_traj_arr.markers.push_back(lg_marker);
            }
        }
        all_traj_viz.publish(vis_traj_arr);
    }


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