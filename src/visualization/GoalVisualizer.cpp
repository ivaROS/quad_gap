#include <quad_gap/visualization/GoalVisualizer.h>

namespace quad_gap
{
    GoalVisualizer::GoalVisualizer(const rclcpp::Node::SharedPtr & node, const QuadGapConfig& cfg)
    {
        node_ = node;
        cfg_ = &cfg;
        globalGoalPublisher = node->create_publisher<visualization_msgs::msg::Marker>("global_goal", 10);
        globalPathLocalWaypointPublisher = node->create_publisher<visualization_msgs::msg::Marker>("global_path_local_waypoint", 10);
        gapGoalPublisher = node->create_publisher<visualization_msgs::msg::Marker>("gap_goals", 1000);

        gapGoalsColor.r = 1.0;
        gapGoalsColor.g = 0.5;
        gapGoalsColor.b = 0.0;
        gapGoalsColor.a = 1;

        globalPathLocalWaypointColor.a = 1;
        globalPathLocalWaypointColor.r = 0;
        globalPathLocalWaypointColor.g = 1;
        globalPathLocalWaypointColor.b = 0;

        globalGoalColor.a = 1;        
        globalGoalColor.r = 1.;
        globalGoalColor.g = 1.;
        globalGoalColor.b = 0.;
    }

    void GoalVisualizer::drawGlobalGoal(const geometry_msgs::msg::PoseStamped & globalGoalOdomFrame)
    {
        // First, clearing topic.
        clearMarkerPublisher(globalGoalPublisher);

        visualization_msgs::msg::Marker globalGoalMarker;

        if (globalGoalOdomFrame.header.frame_id.empty())
        {
            RCLCPP_WARN_STREAM(node_->get_logger(),  "[drawGlobalGoal] Global goal frame_id is empty");
            return;
        }

        globalGoalMarker.header.frame_id = globalGoalOdomFrame.header.frame_id;
        globalGoalMarker.header.stamp = globalGoalOdomFrame.header.stamp;
        globalGoalMarker.ns = "global_goal";
        globalGoalMarker.id = 0;
        globalGoalMarker.type = visualization_msgs::msg::Marker::SPHERE;
        globalGoalMarker.action = visualization_msgs::msg::Marker::ADD;
        globalGoalMarker.pose.position.x = globalGoalOdomFrame.pose.position.x;
        globalGoalMarker.pose.position.y = globalGoalOdomFrame.pose.position.y;
        globalGoalMarker.pose.position.z = 0.0005;
        globalGoalMarker.pose.orientation.w = 1;
        globalGoalMarker.scale.x = 0.1;
        globalGoalMarker.scale.y = 0.1;
        globalGoalMarker.scale.z = 0.1;
        globalGoalMarker.color = globalGoalColor;
        globalGoalPublisher->publish(globalGoalMarker);        
    }

    void GoalVisualizer::drawGlobalPathLocalWaypoint(const geometry_msgs::msg::PoseStamped & globalPathLocalWaypoint)
    {
        // First, clearing topic.
        clearMarkerPublisher(globalPathLocalWaypointPublisher);

        visualization_msgs::msg::Marker globalPathLocalWaypointMarker;

        if (globalPathLocalWaypoint.header.frame_id.empty())
        {
            RCLCPP_WARN_STREAM(node_->get_logger(),  "[drawGlobalPathLocalWaypoint] Global path local waypoint frame_id is empty");
            return;
        }

        globalPathLocalWaypointMarker.header.frame_id = globalPathLocalWaypoint.header.frame_id;
        globalPathLocalWaypointMarker.header.stamp = globalPathLocalWaypoint.header.stamp;
        globalPathLocalWaypointMarker.ns = "local_goal";
        globalPathLocalWaypointMarker.id = 0;
        globalPathLocalWaypointMarker.type = visualization_msgs::msg::Marker::SPHERE;
        globalPathLocalWaypointMarker.action = visualization_msgs::msg::Marker::ADD;
        globalPathLocalWaypointMarker.pose.position.x = globalPathLocalWaypoint.pose.position.x;
        globalPathLocalWaypointMarker.pose.position.y = globalPathLocalWaypoint.pose.position.y;
        globalPathLocalWaypointMarker.pose.position.z = 0.0005;
        globalPathLocalWaypointMarker.pose.orientation.w = 1;
        globalPathLocalWaypointMarker.scale.x = 0.1;
        globalPathLocalWaypointMarker.scale.y = 0.1;
        globalPathLocalWaypointMarker.scale.z = 0.1;
        globalPathLocalWaypointMarker.color = globalPathLocalWaypointColor;
        globalPathLocalWaypointPublisher->publish(globalPathLocalWaypointMarker);
    }


    void GoalVisualizer::drawGapGoal(visualization_msgs::msg::Marker & marker, Gap * gap) 
    {
        if (!gap->isGoalSet()) 
        {
            RCLCPP_INFO_STREAM(node_->get_logger(),  "[drawGapGoal] Gap goal is not set");
            RCLCPP_WARN_STREAM(node_->get_logger(),  "[drawGapGoal] Gap goal is not set");
            return;
        }

        // visualization_msgs::msg::Marker lg_marker;

        geometry_msgs::msg::Point lg_point;
        lg_point.x = gap->getGoalX(); // gap->goal.x;
        lg_point.y = gap->getGoalY(); // gap->goal.y;
        lg_point.z = 0.0;

        RCLCPP_INFO_STREAM(node_->get_logger(),  "[drawGapGoal] Gap goal position: " << lg_point.x << ", " << lg_point.y);

        marker.points.push_back(lg_point);
        // marker.colors.push_back(gapGoalsColor);
        
        // lg_marker.color = gapGoalColors;
        // vis_arr.markers.push_back(lg_marker);

    }

    void GoalVisualizer::drawGapGoals(const std::vector<Gap *> & gaps) 
    {
        // First, clearing topic.
        clearMarkerPublisher(gapGoalPublisher);

        if (gaps.empty()) 
        {
            RCLCPP_INFO_STREAM(node_->get_logger(),  "[drawGapGoals] No gaps to visualize");
            RCLCPP_WARN_STREAM(node_->get_logger(),  "[drawGapGoals] No gaps to visualize");
            return;
        }

        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = gaps.at(0)->getFrame();
        marker.header.stamp = gaps.at(0)->getTimeStamp();
        marker.ns = "gap_goal";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.02;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.0001;
        marker.color = gapGoalsColor; // gapGoalsColor;
        // marker.lifetime = ros::Duration(0);

        for (Gap * gap : gaps) 
        {
            drawGapGoal(marker, gap);
        }

        gapGoalPublisher->publish(marker);
        return;
    }
}