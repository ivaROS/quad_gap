// #include <ros/ros.h>
#include <quad_gap/QuadGapPlanner.h>

// MBF return codes
#include <mbf_msgs/ExePathResult.h>

// #include <quad_gap/utils/Gap.h>
#include <pluginlib/class_list_macros.h>

// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>

// #include <boost/numeric/odeint.hpp>
// #include <Eigen/Core>
// #include <Eigen/Geometry>
// #include <cmath>
// #include <math.h>

// using namespace boost::numeric::odeint;
// namespace pl = std::placeholders;

PLUGINLIB_EXPORT_CLASS(quad_gap::QuadGapPlanner, nav_core::BaseLocalPlanner)

namespace quad_gap 
{
    void QuadGapPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_INFO_STREAM_NAMED("Planner", "Initializing Planner with name: " << name);
        
        planner_name = name;
        planner.initialize(name);

        // ros::NodeHandle pnh("~/" + planner_name);

        // // Setup dynamic reconfigure
        // dynamic_recfg_server = boost::make_shared<dynamic_reconfigure::Server <qgConfig> > (pnh);
        // f = boost::bind(&Planner::rcfgCallback, &planner, _1, _2);
        // dynamic_recfg_server->setCallback(f);
    }

    bool QuadGapPlanner::computeVelocityCommands(geometry_msgs::Twist & cmdVel)
    {
        ROS_INFO_STREAM("[QuadGapPlanner::computeVelocityCommands(twist)]");

        std::string dummy_message;
        geometry_msgs::PoseStamped dummy_pose;
        geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;

        bool outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);

        cmdVel = cmd_vel_stamped.twist;

        ROS_INFO_STREAM_NAMED("QuadGapPlanner", "computeVelocityCommands cmdVel: ");
        ROS_INFO_STREAM_NAMED("QuadGapPlanner", "                linear: ");
        ROS_INFO_STREAM_NAMED("QuadGapPlanner", "                  x: " << cmdVel.linear.x << ", y: " << cmdVel.linear.y << ", z: " << cmdVel.linear.z);
        ROS_INFO_STREAM_NAMED("QuadGapPlanner", "                angular: ");
        ROS_INFO_STREAM_NAMED("QuadGapPlanner", "                  x: " << cmdVel.angular.x << ", y: " << cmdVel.angular.y << ", z: " << cmdVel.angular.z);

        // TODO: just hardcoding this now, need to revise
        bool success = 1;

        return success;
    }

    uint32_t QuadGapPlanner::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                        const geometry_msgs::TwistStamped& velocity,
                                                        geometry_msgs::TwistStamped &cmd_vel,
                                                        std::string &message)
    {
        if (!planner.initialized())
        {
            planner.initialize(planner_name);
            ROS_WARN_STREAM("computerVelocity called before initializing planner");
        }

        // if (planner.ccEnabled() && !planner.getCCWrapper()->isReady())
        // {
        //     ROS_ERROR("CC NOT READY");
        //     return false;
        // }

        planner.setReachedGlobalGoal(false);

        geometry_msgs::PoseArray final_traj = planner.runPlanningLoop();

        if (planner.isGoalReached())
        {
            cmd_vel.twist = geometry_msgs::Twist();
            return mbf_msgs::ExePathResult::SUCCESS;
        }        

        geometry_msgs::Twist cmdVelNoStamp = planner.ctrlGeneration(final_traj);

        // cmd_vel.twist = cmdVelNoStamp;

        bool acceptedCmdVel = planner.recordAndCheckVel(cmdVelNoStamp);  
        
        /*
        *         SUCCESS           = 0
        *         1..9 are reserved as plugin specific non-error results
        *         FAILURE           = 100  # Unspecified failure, only used for old, non-mfb_core based plugins
        *         CANCELED          = 101
        *         NO_VALID_CMD      = 102
        *         PAT_EXCEEDED      = 103
        *         COLLISION         = 104
        *         OSCILLATION       = 105
        *         ROBOT_STUCK       = 106
        *         MISSED_GOAL       = 107
        *         MISSED_PATH       = 108
        *         BLOCKED_GOAL      = 109
        *         BLOCKED_PATH      = 110
        *         INVALID_PATH      = 111
        *         TF_ERROR          = 112
        *         NOT_INITIALIZED   = 113
        *         INVALID_PLUGIN    = 114
        *         INTERNAL_ERROR    = 115
        *         OUT_OF_MAP        = 116  # The start and / or the goal are outside the map
        *         MAP_ERROR         = 117  # The map is not running properly
        *         STOPPED           = 118  # The controller execution has been stopped rigorously
        */        
        if (acceptedCmdVel)
            return mbf_msgs::ExePathResult::SUCCESS;
        else
            return mbf_msgs::ExePathResult::FAILURE;        
        
    }

    bool QuadGapPlanner::isGoalReached()
    {
        // ROS_INFO_STREAM("[QuadGapPlanner::isGoalReached()]");

        return planner.isGoalReached();
    }

    bool QuadGapPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> & globalPlanMapFrame)
    {
        // ROS_INFO_STREAM("[QuadGapPlanner::setPlan()]");

        if (!planner.initialized())
        {
            return false;
        } else
        {
            return planner.setPlan(globalPlanMapFrame);
        }

        // 0: fail, 1: success
        // return 1;
    }

}