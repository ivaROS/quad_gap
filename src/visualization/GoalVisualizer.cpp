#include <quad_gap/visualization/GoalVisualizer.h>

namespace quad_gap
{
    GoalVisualizer::GoalVisualizer(ros::NodeHandle& nh, const quad_gap::QuadGapConfig& cfg)
    {
        cfg_ = &cfg;
        goal_pub = nh.advertise<visualization_msgs::Marker>("goals", 1000);
        gapwp_pub = nh.advertise<visualization_msgs::MarkerArray>("gap_goals", 1000);
        gapwp_color.r = 0.5;
        gapwp_color.g = 1;
        gapwp_color.b = 0.5;
        gapwp_color.a = 1;
        localGoal_color.r = 0;
        localGoal_color.g = 1;
        localGoal_color.b = 0;
        localGoal_color.a = 1;
    }

    void GoalVisualizer::localGoal(geometry_msgs::PoseStamped localGoal)
    {
        if (!cfg_->gap_viz.debug_viz) return;
        visualization_msgs::Marker lg_marker;
        lg_marker.header.frame_id = localGoal.header.frame_id;
        lg_marker.header.stamp = ros::Time::now();
        lg_marker.ns = "local_goal";
        lg_marker.id = 0;
        lg_marker.type = visualization_msgs::Marker::SPHERE;
        lg_marker.action = visualization_msgs::Marker::ADD;
        lg_marker.pose.position.x = localGoal.pose.position.x;
        lg_marker.pose.position.y = localGoal.pose.position.y;
        lg_marker.pose.position.z = 2;
        lg_marker.pose.orientation.w = 1;
        lg_marker.scale.x = 0.1;
        lg_marker.scale.y = 0.1;
        lg_marker.scale.z = 0.1;
        lg_marker.color = localGoal_color;
        goal_pub.publish(lg_marker);
    }

    void GoalVisualizer::drawGapGoal(visualization_msgs::MarkerArray& vis_arr, quad_gap::Gap g) {
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

    void GoalVisualizer::drawGapGoals(std::vector<quad_gap::Gap> gs) {
        if (!cfg_->gap_viz.debug_viz) return;
        visualization_msgs::MarkerArray vis_arr;
        for (auto & gap : gs) {
            drawGapGoal(vis_arr, gap);
        }
        gapwp_pub.publish(vis_arr);
        return;
    }
}