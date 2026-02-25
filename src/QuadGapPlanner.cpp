// #include <ros/ros.h>
#include <quad_gap/QuadGapPlanner.h>

// MBF return codes
// #include <mbf_msgs/ExePathResult.h>

// #include <quad_gap/utils/Gap.h>
// #include <pluginlib/class_list_macros.h>

// #include <visualization_msgs/msg/marker.h>
// #include <visualization_msgs/msg/marker_array.h>

// #include <boost/numeric/odeint.hpp>
// #include <Eigen/Core>
// #include <Eigen/Geometry>
// #include <cmath>
// #include <math.h>

// using namespace boost::numeric::odeint;
// namespace pl = std::placeholders;

namespace quad_gap 
{
    void QuadGapPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                                    std::string name, 
                                    const std::shared_ptr<tf2_ros::Buffer> tf,
                                    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        RCLCPP_INFO_STREAM(logger_,  "Initializing Planner with name: " << name);
        
        lifecycle_node_ = parent;
        rclcpp_lifecycle::LifecycleNode::SharedPtr node = lifecycle_node_.lock();
        // logger_ = lifecycle_logger_;

        plugin_name_ = name;

        // planner_name = name;
        planner.initialize(node, plugin_name_);

        // ros::NodeHandle pnh("~/" + planner_name);

        // // Setup dynamic reconfigure
        // dynamic_recfg_server = std::make_shared<dynamic_reconfigure::Server <qgConfig> > (pnh);
        // f = boost::bind(&Planner::rcfgCallback, &planner, _1, _2);
        // dynamic_recfg_server->setCallback(f);
        RCLCPP_INFO_STREAM(logger_,  "void QuadGapPlanner::configure ended");
    }

    void QuadGapPlanner::cleanup()
    {
        // print initalize message
        RCLCPP_INFO_STREAM(logger_,  "void QuadGapPlanner::cleanup started");
        // RCLCPP_INFO_STREAM(logger_,  "Cleaning up Planner");
    }

    void QuadGapPlanner::activate()
    {
        // print initalize message
        RCLCPP_INFO_STREAM(logger_,  "void QuadGapPlanner::activate started");
        // RCLCPP_INFO_STREAM(logger_,  "Activating Planner");
    }

    void QuadGapPlanner::deactivate()
    {
        // print initalize message
        RCLCPP_INFO_STREAM(logger_,  "void QuadGapPlanner::deactivate started");
        // RCLCPP_INFO_STREAM(logger_,  "Deactivating Planner");
    }

    void QuadGapPlanner::setSpeedLimit(const double& speed_limit, 
                                        const bool& percentage)
    {
        // print initalize message
        RCLCPP_INFO_STREAM(logger_,  "void QuadGapPlanner::setSpeedLimit started");

        (void) speed_limit;
        (void) percentage;

        RCLCPP_INFO_STREAM(logger_,  "void QuadGapPlanner::setSpeedLimit ended");
    }

    geometry_msgs::msg::TwistStamped QuadGapPlanner::computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
                                                                                const geometry_msgs::msg::Twist & velocity,
                                                                                nav2_core::GoalChecker * goal_checker)
    {
        // // print initalize message
        RCLCPP_INFO_STREAM(logger_,  "[QuadGapPlanner::computeVelocityCommands(twist)]");

        geometry_msgs::msg::TwistStamped cmd_vel;

        if (!planner.initialized())
        {
            // planner.initialize(planner_name);
            RCLCPP_WARN_STREAM(logger_, "computerVelocity called before initializing planner");
            return cmd_vel;
        }

        // std::string dummy_message;
        // geometry_msgs::msg::PoseStamped dummy_pose;
        // geometry_msgs::msg::TwistStamped dummy_velocity, cmd_vel_stamped;

        // bool outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);

        // cmdVel = cmd_vel_stamped.twist;

        planner.setReachedGlobalGoal(false);

        Trajectory finalTraj = planner.runPlanningLoop();

        if (planner.isGoalReached())
        {
            cmd_vel.twist = geometry_msgs::msg::Twist();
            return cmd_vel;
        }        

        cmd_vel.twist = planner.ctrlGeneration(finalTraj);

        bool acceptedCmdVel = planner.recordAndCheckVel(cmd_vel);  

        // RCLCPP_INFO_STREAM(logger_,  "computeVelocityCommands cmdVel: ");
        // RCLCPP_INFO_STREAM(logger_,  "                linear: ");
        // RCLCPP_INFO_STREAM(logger_,  "                  x: " << cmd_vel.twist.linear.x << ", y: " << cmd_vel.twist.linear.y << ", z: " << cmd_vel.twist.linear.z);
        // RCLCPP_INFO_STREAM(logger_,  "                angular: ");
        // RCLCPP_INFO_STREAM(logger_,  "                  x: " << cmd_vel.twist.angular.x << ", y: " << cmd_vel.twist.angular.y << ", z: " << cmd_vel.twist.angular.z);

        // // print initalize message
        RCLCPP_INFO_STREAM(logger_,  "[QuadGapPlanner::computeVelocityCommands(twist)]   ended");

        return cmd_vel;
    }

    void QuadGapPlanner::setPlan(const nav_msgs::msg::Path & path)
    {
        // print initalize message
        RCLCPP_INFO_STREAM(logger_,  "void QuadGapPlanner::setPlan started");

        // RCLCPP_INFO_STREAM(logger_,  "[QuadGapPlanner::setPlan()]");

        if (!planner.initialized())
        {
            return;
        } else
        {
            return planner.setPlan(path);
        }

        // print initalize message
        RCLCPP_INFO_STREAM(logger_,  "void QuadGapPlanner::setPlan ended");
    }

}

PLUGINLIB_EXPORT_CLASS(quad_gap::QuadGapPlanner, nav2_core::Controller)