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
        // RCLCPP_INFO_STREAM(logger_,  "Initializing Planner with name: " << name);
        
        node_ = parent;
        auto node = node_.lock();
        logger_ = node->get_logger();

        // planner_name = name;
        // planner.initialize(planner_name);

        // ros::NodeHandle pnh("~/" + planner_name);

        // // Setup dynamic reconfigure
        // dynamic_recfg_server = boost::make_shared<dynamic_reconfigure::Server <qgConfig> > (pnh);
        // f = boost::bind(&Planner::rcfgCallback, &planner, _1, _2);
        // dynamic_recfg_server->setCallback(f);
    }

    void QuadGapPlanner::cleanup()
    {
        RCLCPP_INFO_STREAM(logger_,  "Cleaning up Planner");
    }

    void QuadGapPlanner::activate()
    {
        RCLCPP_INFO_STREAM(logger_,  "Activating Planner");
    }

    void QuadGapPlanner::deactivate()
    {
        RCLCPP_INFO_STREAM(logger_,  "Deactivating Planner");
    }

    void QuadGapPlanner::setSpeedLimit(const double& speed_limit, 
                                        const bool& percentage)
    {
        (void) speed_limit;
        (void) percentage;
    }

    geometry_msgs::msg::TwistStamped QuadGapPlanner::computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
                                                                                const geometry_msgs::msg::Twist & velocity,
                                                                                nav2_core::GoalChecker * goal_checker)
    {
        RCLCPP_INFO_STREAM(logger_,  "[QuadGapPlanner::computeVelocityCommands(twist)]");

        geometry_msgs::msg::TwistStamped cmd_vel;

        // if (!planner.initialized())
        // {
        //     planner.initialize(planner_name);
        //     ROS_WARN_STREAM_NAMED("QuadGapPlanner", "computerVelocity called before initializing planner");
        // }

        // // std::string dummy_message;
        // // geometry_msgs::PoseStamped dummy_pose;
        // // geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;

        // // bool outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);

        // // cmdVel = cmd_vel_stamped.twist;

        // planner.setReachedGlobalGoal(false);

        // Trajectory finalTraj = planner.runPlanningLoop();

        // if (planner.isGoalReached())
        // {
        //     cmd_vel.twist = geometry_msgs::Twist();
        //     return mbf_msgs::ExePathResult::SUCCESS;
        // }        

        // cmd_vel.twist = planner.ctrlGeneration(finalTraj);

        // bool acceptedCmdVel = planner.recordAndCheckVel(cmd_vel);  

        // RCLCPP_INFO_STREAM(logger_,  "computeVelocityCommands cmdVel: ");
        // RCLCPP_INFO_STREAM(logger_,  "                linear: ");
        // RCLCPP_INFO_STREAM(logger_,  "                  x: " << cmd_vel.twist.linear.x << ", y: " << cmd_vel.twist.linear.y << ", z: " << cmd_vel.twist.linear.z);
        // RCLCPP_INFO_STREAM(logger_,  "                angular: ");
        // RCLCPP_INFO_STREAM(logger_,  "                  x: " << cmd_vel.twist.angular.x << ", y: " << cmd_vel.twist.angular.y << ", z: " << cmd_vel.twist.angular.z);

        // // TODO: just hardcoding this now, need to revise
        // bool success = 1;

        // return success;
        return cmd_vel;
    }

    // uint32_t QuadGapPlanner::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
    //                                                     const geometry_msgs::TwistStamped& velocity,
    //                                                     geometry_msgs::TwistStamped &cmd_vel,
    //                                                     std::string &message)
    // {
    //     if (!planner.initialized())
    //     {
    //         planner.initialize(planner_name);
    //         ROS_WARN_STREAM_NAMED("QuadGapPlanner", "computerVelocity called before initializing planner");
    //     }

    //     // if (planner.ccEnabled() && !planner.getCCWrapper()->isReady())
    //     // {
    //     //     ROS_ERROR("CC NOT READY");
    //     //     return false;
    //     // }

    //     planner.setReachedGlobalGoal(false);

    //     Trajectory finalTraj = planner.runPlanningLoop();

    //     if (planner.isGoalReached())
    //     {
    //         cmd_vel.twist = geometry_msgs::Twist();
    //         return mbf_msgs::ExePathResult::SUCCESS;
    //     }        

    //     geometry_msgs::Twist cmdVelNoStamp = planner.ctrlGeneration(finalTraj);

    //     cmd_vel.twist = cmdVelNoStamp;

    //     bool acceptedCmdVel = planner.recordAndCheckVel(cmdVelNoStamp);  
        
    //     /*
    //     *         SUCCESS           = 0
    //     *         1..9 are reserved as plugin specific non-error results
    //     *         FAILURE           = 100  # Unspecified failure, only used for old, non-mfb_core based plugins
    //     *         CANCELED          = 101
    //     *         NO_VALID_CMD      = 102
    //     *         PAT_EXCEEDED      = 103
    //     *         COLLISION         = 104
    //     *         OSCILLATION       = 105
    //     *         ROBOT_STUCK       = 106
    //     *         MISSED_GOAL       = 107
    //     *         MISSED_PATH       = 108
    //     *         BLOCKED_GOAL      = 109
    //     *         BLOCKED_PATH      = 110
    //     *         INVALID_PATH      = 111
    //     *         TF_ERROR          = 112
    //     *         NOT_INITIALIZED   = 113
    //     *         INVALID_PLUGIN    = 114
    //     *         INTERNAL_ERROR    = 115
    //     *         OUT_OF_MAP        = 116  # The start and / or the goal are outside the map
    //     *         MAP_ERROR         = 117  # The map is not running properly
    //     *         STOPPED           = 118  # The controller execution has been stopped rigorously
    //     */        
    //     if (acceptedCmdVel)
    //         return mbf_msgs::ExePathResult::SUCCESS;
    //     else
    //         return mbf_msgs::ExePathResult::FAILURE;        
    // }

    // bool QuadGapPlanner::isGoalReached()
    // {
    //     // RCLCPP_INFO_STREAM(logger_,  "[QuadGapPlanner::isGoalReached()]");

    //     return planner.isGoalReached();
    // }

    void QuadGapPlanner::setPlan(const nav_msgs::msg::Path & path)
    {
        // RCLCPP_INFO_STREAM(logger_,  "[QuadGapPlanner::setPlan()]");

        // if (!planner.initialized())
        // {
        //     return false;
        // } else
        // {
        //     return planner.setPlan(path.poses);
        // }

        return;

        // 0: fail, 1: success
        // return 1;
    }

}

PLUGINLIB_EXPORT_CLASS(quad_gap::QuadGapPlanner, nav2_core::Controller)