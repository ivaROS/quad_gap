#include <ros/ros.h>
#include <quad_gap/quad_gap.h>
#include <quad_gap/gap.h>
#include <pluginlib/class_list_macros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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
    QuadGapPlanner::QuadGapPlanner()
    {
        
    }

    QuadGapPlanner::~QuadGapPlanner()
    {
        ROS_INFO_STREAM("Planner terminated");
    }

    bool QuadGapPlanner::isGoalReached()
    {
        return planner.isGoalReached();
    }

    bool QuadGapPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> & plan)
    {
        return planner.setGoal(plan);
    }

    void QuadGapPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        planner_name = name;
        ros::NodeHandle pnh("~/" + planner_name);

        laser_sub = pnh.subscribe("/point_scan", 100, &Planner::laserScanCB, &planner);
        inflated_laser_sub = pnh.subscribe("/inflated_point_scan", 100, &Planner::inflatedlaserScanCB, &planner);
        feasi_laser_sub = pnh.subscribe("/inflated_point_scan", 100, &Planner::inflatedlaserScanCB, &planner);
        pose_sub = pnh.subscribe("/odom",10, &Planner::poseCB, &planner);
        planner.initialize(pnh);
        initialized = true;

        // Setup dynamic reconfigure
        dynamic_recfg_server = boost::make_shared<dynamic_reconfigure::Server <quad_gap::qgConfig> > (pnh);
        f = boost::bind(&quad_gap::Planner::rcfgCallback, &planner, _1, _2);
        dynamic_recfg_server->setCallback(f);
    }

    bool QuadGapPlanner::computeVelocityCommands(geometry_msgs::Twist & cmd_vel)
    {
        if (!planner.initialized())
        {
            ros::NodeHandle pnh("~/" + planner_name);
            planner.initialize(pnh);
            ROS_WARN_STREAM("computerVelocity called before initializing planner");
        }

        if(planner.ccEnabled() && !planner.getCCWrapper()->isReady())
        {
            ROS_ERROR("CC NOT READY");
            return false;
        }

        auto final_traj = planner.getPlanTrajectory();

        cmd_vel = planner.ctrlGeneration(final_traj);

        return planner.recordAndCheckVel(cmd_vel);
    }

    void QuadGapPlanner::reset()
    {
        planner.reset();
        return;
    }

}