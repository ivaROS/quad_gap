#include <ros/ros.h>
#include <quad_gap/quad_gap.h>
#include <quad_gap/gap.h>
#include <pluginlib/class_list_macros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// MBF return codes
#include <mbf_msgs/ExePathResult.h>

#include <boost/numeric/odeint.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <math.h>

// using namespace boost::numeric::odeint;
namespace pl = std::placeholders;

#define JANKY_PID false
#define NEAR_IDENTITY true

PLUGINLIB_EXPORT_CLASS(quad_gap::QuadGapPlanner, nav_core::BaseLocalPlanner)

namespace quad_gap 
{
    void QuadGapPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_INFO_STREAM_NAMED("Planner", "Initializing Planner with name: " << name);
        planner_name = name;
        ros::NodeHandle pnh("~/" + planner_name);

        // laser_sub = pnh.subscribe("/scan", 100, &Planner::laserScanCB, &planner);
        // // inflated_laser_sub = pnh.subscribe("/inflated_point_scan", 100, &Planner::inflatedlaserScanCB, &planner);
        // // feasi_laser_sub = pnh.subscribe("/inflated_point_scan", 100, &Planner::inflatedlaserScanCB, &planner);
        // pose_sub = pnh.subscribe("/odom",10, &Planner::poseCB, &planner);
        // planner.initialize(name);
        // initialized = true;

        // // Setup dynamic reconfigure
        // dynamic_recfg_server = boost::make_shared<dynamic_reconfigure::Server <quad_gap::qgConfig> > (pnh);
        // f = boost::bind(&quad_gap::Planner::rcfgCallback, &planner, _1, _2);
        // dynamic_recfg_server->setCallback(f);
    }

    bool QuadGapPlanner::computeVelocityCommands(geometry_msgs::Twist & cmdVel)
    {
        // ROS_INFO_STREAM("[QuadGapPlanner::computeVelocityCommands(short)]");

        // std::string dummy_message;
        // geometry_msgs::PoseStamped dummy_pose;
        // geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;

        // bool outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);

        // cmdVel = cmd_vel_stamped.twist;

        // ROS_INFO_STREAM_NAMED("QuadGapPlanner", "computeVelocityCommands cmdVel: ");
        // ROS_INFO_STREAM_NAMED("QuadGapPlanner", "                linear: ");
        // ROS_INFO_STREAM_NAMED("QuadGapPlanner", "                  x: " << cmdVel.linear.x << ", y: " << cmdVel.linear.y << ", z: " << cmdVel.linear.z);
        // ROS_INFO_STREAM_NAMED("QuadGapPlanner", "                angular: ");
        // ROS_INFO_STREAM_NAMED("QuadGapPlanner", "                  x: " << cmdVel.angular.x << ", y: " << cmdVel.angular.y << ", z: " << cmdVel.angular.z);

        // TODO: just hardcoding this now, need to revise
        bool success = 1;

        return success;
    }

    uint32_t QuadGapPlanner::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                        const geometry_msgs::TwistStamped& velocity,
                                                        geometry_msgs::TwistStamped &cmd_vel,
                                                        std::string &message)
    {
        // if (!planner.initialized())
        // {
        //     ros::NodeHandle pnh("~/" + planner_name);
        //     planner.initialize(planner_name);
        //     ROS_WARN_STREAM("computerVelocity called before initializing planner");
        // }

        // if(planner.ccEnabled() && !planner.getCCWrapper()->isReady())
        // {
        //     ROS_ERROR("CC NOT READY");
        //     return false;
        // }

        // auto final_traj = planner.getPlanTrajectory();

        // geometry_msgs::Twist cmdVelNoStamp = planner.ctrlGeneration(final_traj);

        // cmd_vel.twist = cmdVelNoStamp;

        // bool acceptedCmdVel = planner.recordAndCheckVel(cmdVelNoStamp);  
        
        bool acceptedCmdVel = 1;

        if (acceptedCmdVel)
            return mbf_msgs::ExePathResult::SUCCESS;
        else
            return mbf_msgs::ExePathResult::FAILURE;        
        
    }

    bool QuadGapPlanner::isGoalReached()
    {
        // ROS_INFO_STREAM("[QuadGapPlanner::isGoalReached()]");

        return false;
        // return planner.isGoalReached();
    }

    bool QuadGapPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> & globalPlanMapFrame)
    {
        // ROS_INFO_STREAM("[QuadGapPlanner::setPlan()]");

        return 1;
        // if (!planner.initialized())
        // {
        //     return false;
        // } else
        // {
        //     return planner.setGoal(globalPlanMapFrame);
        // }
        // 0: fail, 1: success
        // return 1;
    }

}