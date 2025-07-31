#pragma once

// #include <ros/ros.h>
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>

// move_base_flex
// #include <mbf_costmap_core/costmap_controller.h>
// #include <nav_core/base_local_planner.h>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/header.hpp>
// #include <navfn/navfn_ros.h>
#include <boost/shared_ptr.hpp>
// #include <quad_gap/utils/Gap.h>
#include <geometry_msgs/msg/pose_array.hpp>

// #include <tf2_ros/transform_listener.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <geometry_msgs/msg/transform_stamped.h>

#include <boost/numeric/odeint.hpp>

#include <nav2_core/controller.hpp>

#include <quad_gap/Planner.h>

// #include <dynamic_reconfigure/server.h>
// #include <quad_gap/qgConfig.h>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace quad_gap 
{
    class QuadGapPlanner : public nav2_core::Controller
    {
        public: 

            QuadGapPlanner() = default;
            ~QuadGapPlanner() override = default;

            void configure(
                const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
                const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;


            void cleanup() override;
            void activate() override;
            void deactivate() override;
            void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

            geometry_msgs::msg::TwistStamped computeVelocityCommands(
                const geometry_msgs::msg::PoseStamped & pose,
                const geometry_msgs::msg::Twist & velocity,
                nav2_core::GoalChecker * goal_checker) override;

            void setPlan(const nav_msgs::msg::Path & path) override;

        private:

            rclcpp::Node::SharedPtr node_;
            rclcpp_lifecycle::LifecycleNode::WeakPtr lifecycle_node_;

            rclcpp::Logger logger_ {rclcpp::get_logger("QuadGapPlanner")};

            // qgConfig loadRosParamFromNodeHandle(const ros::NodeHandle& nh);

            Planner planner;
            std::string planner_name;
            // ros::NodeHandle nh, pnh;

            // std::shared_ptr<dynamic_reconfigure::Server<qgConfig> > dynamic_recfg_server;
            // dynamic_reconfigure::Server<qgConfig>::CallbackType f;
    };
}