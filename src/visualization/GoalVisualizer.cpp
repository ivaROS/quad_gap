#include <quad_gap/visualization/GoalVisualizer.h>

namespace quad_gap
{
    GoalVisualizer::GoalVisualizer(ros::NodeHandle& nh, const QuadGapConfig& cfg)
    {
        cfg_ = &cfg;
        globalGoalPublisher = nh.advertise<visualization_msgs::Marker>("global_goal", 10);
        globalPathLocalWaypointPublisher = nh.advertise<visualization_msgs::Marker>("goals", 10);
        gapwp_pub = nh.advertise<visualization_msgs::MarkerArray>("gap_goals", 1000);


        gapwp_color.r = 0.5;
        gapwp_color.g = 1;
        gapwp_color.b = 0.5;
        gapwp_color.a = 1;

        globalPathLocalWaypointColor.r = 0;
        globalPathLocalWaypointColor.g = 1;
        globalPathLocalWaypointColor.b = 0;
        globalPathLocalWaypointColor.a = 1;

        globalGoalColor.r = 1.;
        globalGoalColor.g = 1.;
        globalGoalColor.b = 0.;
        globalGoalColor.a = 1;        
    }

    void GoalVisualizer::drawGlobalGoal(const geometry_msgs::PoseStamped & globalGoalOdomFrame)
    {
        // First, clearing topic.
        clearMarkerPublisher(globalGoalPublisher);

        visualization_msgs::Marker globalGoalMarker;

        if (globalGoalOdomFrame.header.frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawGlobalGoal] Global goal frame_id is empty");
            return;
        }

        globalGoalMarker.header.frame_id = globalGoalOdomFrame.header.frame_id;
        globalGoalMarker.header.stamp = globalGoalOdomFrame.header.stamp;
        globalGoalMarker.ns = "global_goal";
        globalGoalMarker.id = 0;
        globalGoalMarker.type = visualization_msgs::Marker::SPHERE;
        globalGoalMarker.action = visualization_msgs::Marker::ADD;
        globalGoalMarker.pose.position.x = globalGoalOdomFrame.pose.position.x;
        globalGoalMarker.pose.position.y = globalGoalOdomFrame.pose.position.y;
        globalGoalMarker.pose.position.z = 0.0005;
        globalGoalMarker.pose.orientation.w = 1;
        globalGoalMarker.scale.x = 0.1;
        globalGoalMarker.scale.y = 0.1;
        globalGoalMarker.scale.z = 0.1;
        globalGoalMarker.color = globalGoalColor;
        globalGoalPublisher.publish(globalGoalMarker);        
    }

    void GoalVisualizer::drawGlobalPathLocalWaypoint(const geometry_msgs::PoseStamped & globalPathLocalWaypoint)
    {
        // First, clearing topic.
        clearMarkerPublisher(globalPathLocalWaypointPublisher);

        visualization_msgs::Marker globalPathLocalWaypointMarker;

        if (globalPathLocalWaypoint.header.frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawGlobalPathLocalWaypoint] Global path local waypoint frame_id is empty");
            return;
        }

        globalPathLocalWaypointMarker.header.frame_id = globalPathLocalWaypoint.header.frame_id;
        globalPathLocalWaypointMarker.header.stamp = globalPathLocalWaypoint.header.stamp;
        globalPathLocalWaypointMarker.ns = "local_goal";
        globalPathLocalWaypointMarker.id = 0;
        globalPathLocalWaypointMarker.type = visualization_msgs::Marker::SPHERE;
        globalPathLocalWaypointMarker.action = visualization_msgs::Marker::ADD;
        globalPathLocalWaypointMarker.pose.position.x = globalPathLocalWaypoint.pose.position.x;
        globalPathLocalWaypointMarker.pose.position.y = globalPathLocalWaypoint.pose.position.y;
        globalPathLocalWaypointMarker.pose.position.z = 0.0005;
        globalPathLocalWaypointMarker.pose.orientation.w = 1;
        globalPathLocalWaypointMarker.scale.x = 0.1;
        globalPathLocalWaypointMarker.scale.y = 0.1;
        globalPathLocalWaypointMarker.scale.z = 0.1;
        globalPathLocalWaypointMarker.color = globalPathLocalWaypointColor;
        globalPathLocalWaypointPublisher.publish(globalPathLocalWaypointMarker);
    }


    void GoalVisualizer::drawGapGoal(visualization_msgs::MarkerArray& vis_arr, const Gap & g) 
    {
        if (!cfg_->gap_viz.debug_viz) return;
        if (!g.goal.set) {
            return;
        }

        visualization_msgs::Marker lg_marker;
        lg_marker.header.frame_id = g._frame;
        lg_marker.header.stamp = ros::Time::now();
        lg_marker.ns = "gap_goal";
        lg_marker.id = int (vis_arr.markers.size());
        lg_marker.type = visualization_msgs::Marker::SPHERE;
        lg_marker.action = visualization_msgs::Marker::ADD;
        lg_marker.pose.position.x = g.goal.x;
        lg_marker.pose.position.y = g.goal.y;
        lg_marker.pose.position.z = 0.5;
        lg_marker.pose.orientation.w = 1;
        lg_marker.scale.x = 0.1;
        lg_marker.scale.y = 0.1;
        lg_marker.scale.z = 0.1;
        lg_marker.color = gapwp_color;
        lg_marker.lifetime = ros::Duration(0.5);
        vis_arr.markers.push_back(lg_marker);

    }

    void GoalVisualizer::drawGapGoals(const std::vector<Gap> & gs) 
    {
        // First, clearing topic.
        clearMarkerArrayPublisher(gapwp_pub);

        if (!cfg_->gap_viz.debug_viz) return;

        visualization_msgs::MarkerArray vis_arr;
        for (const Gap & gap : gs) 
        {
            drawGapGoal(vis_arr, gap);
        }
        gapwp_pub.publish(vis_arr);
        return;
    }
}