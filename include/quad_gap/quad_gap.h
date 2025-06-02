#pragma once

#include <ros/ros.h>
#include <ros/console.h>

// move_base_flex
#include <mbf_costmap_core/costmap_controller.h>
#include <nav_core/base_local_planner.h>

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Header.h>
#include <navfn/navfn_ros.h>
#include <boost/shared_ptr.hpp>
#include <quad_gap/gap.h>
#include <quad_gap/trajectory_generation/helper.h>
#include <geometry_msgs/PoseArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <boost/numeric/odeint.hpp>

#include <quad_gap/planner.h>

#include <dynamic_reconfigure/server.h>
#include <quad_gap/qgConfig.h>

namespace quad_gap {

    class QuadGapPlanner : public nav_core::BaseLocalPlanner , public mbf_costmap_core::CostmapController
    {
        public: 

            /**
            * \brief Initializes the dynamic gap plugin, this is a virtual function for move_base plugins
            *        that must be overloaded
            * \param name planner name (used for ROS topic namespaces)
            * \param tf Pointer to a tf buffer
            * \param costmap_ros Cost map representing occupied and free space
            */
            void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

            /**
            * \brief Compute velocity commands, this is a virtual function for move_base plugins
            *        that must be overloaded. Given the current position, orientation, and velocity 
            *        of the robot, compute velocity commands to send to the base.
            * \param pose the current pose of the robot.
            * \param velocity the current velocity of the robot.
            * \param cmd_vel Will be filled with the velocity command to be passed to the robot base.
            * \param message Optional more detailed outcome as a string
            * \return Result code as described on ExePath action result:
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
            *         121..149 are reserved as plugin specific errors
            */
            uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                const geometry_msgs::TwistStamped& velocity,
                                                geometry_msgs::TwistStamped &cmd_vel,
                                                std::string &message);

            /**
            * \brief Actual entry point for move_base plugin, uses local path planner to compute 
            *        next command velocity, this is a virtual function for move_base plugins
            *        that must be overloaded.
            * \param cmdVel command velocity to update
            * \return boolean for if command velocity was successfully computed
            */
            bool computeVelocityCommands(geometry_msgs::Twist & cmdVel);

            /**
            * \brief Check if the global goal pose has been achieved, this is a virtual function for move_base plugins
            *        that must be overloaded.
            * \return True if achieved, false otherwise
            */
            bool isGoalReached();

            /**
            * \brief Dummy version to satisfy MBF API
            */
            bool isGoalReached(double xy_tolerance, double yaw_tolerance) { return isGoalReached(); };

            /**
            * @brief Set the plan that the local planner is following, this is a virtual function for move_base plugins
            *        that must be overloaded.
            * @param globalPlanMapFrame The global plan to pass to the local planner
            * @return True if the plan was updated successfully, false otherwise
            */
            bool setPlan(const std::vector<geometry_msgs::PoseStamped> & globalPlanMapFrame);

            /**
            * @brief Requests the planner to cancel, e.g. if it takes too much time, 
            *        this is a virtual function for move_base plugins that must be overloaded.
            * @remark New on MBF API
            * @return True if a cancel has been successfully requested, false if not implemented.
            */
            bool cancel() { return true; };

            // void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

            void reset();

        private:
            // quad_gap::qgConfig loadRosParamFromNodeHandle(const ros::NodeHandle& nh);

            quad_gap::Planner planner;
            std::string planner_name;
            ros::NodeHandle nh, pnh;

            ros::Subscriber laser_sub, inflated_laser_sub;
            ros::Subscriber pose_sub;
            ros::Subscriber feasi_laser_sub;

            bool initialized = false;

            boost::shared_ptr<dynamic_reconfigure::Server<quad_gap::qgConfig> > dynamic_recfg_server;
            dynamic_reconfigure::Server<quad_gap::qgConfig>::CallbackType f;
    };
}