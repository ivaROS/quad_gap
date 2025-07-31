#pragma once

// #include <ros/console.h>
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

// #include <quad_gap/qgConfig.h>
#include <Eigen/Core>
#include <boost/thread/mutex.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <boost/shared_ptr.hpp>

#include <quad_gap/utils/Utils.h>  

namespace quad_gap 
{
    class QuadGapConfig 
    {
        public:
            std::string map_frame_id = "map"; /**< Map frame ID */
            std::string odom_frame_id = "TBD"; /**< Odometry frame ID */
            std::string robot_frame_id = "TBD"; /**< Robot frame ID */
            std::string sensor_frame_id = "TBD"; /**< Sensor frame ID */
            std::string odom_topic = "TBD"; /**< Odometry ROS topic */
            // std::string acc_topic = "TBD"; /**< IMU ROS topic */
            std::string scan_topic = "TBD"; /**< Laser scan ROS topic */

            struct Robot 
            {
                float r_inscr = 0.225; /**< Inscribed radius of the robot (for circle geom) */
                float length = 0.45; /**< Robot length (for box geom) */
                float width = 0.45; /**< Robot width (for box geom) */
                float avg_lin_speed = 0.2; /**< Average linear speed */
                float avg_rot_speed = 0.5; /**< Average rotational speed */
                float vx_absmax = 1.0; /**< Maximum linear speed in x-direction for robot */
                float vy_absmax = 1.0; /**< Maximum linear speed in y-direction for robot */
                float vang_absmax = 1.0; /**< Maximum angular speed for robot */          
                float speed_factor = 2.0; /**< Speed factor for robot to divide max angular velocity by for average speed */
                int shape_id = 0; /**< Robot shape ID, 0: circle, 1: box */
                bool use_geo_storage = false; /**< Use precomputed robot geometry storage */
            } rbt;

            struct Goal 
            {
                float xy_global_goal_tolerance = 0.2; /**< Distance threshold for global goal */
                float yaw_global_goal_tolerance = M_PI; /**< Angular distance threshold for global goal */
                float xy_waypoint_tolerance = 0.1; /**< Distance threshold for global path local waypoint */
            } goal;

            /**
            * \brief Hyperparameters for laser scan
            */
            struct Scan
            {
                // will get overriden in updateParamFromScan
                float angle_min = -M_PI; /**< minimum angle value in scan */
                float angle_max = M_PI; /**< maximum angle value in scan */
                int half_scan = 256; /**< Half of total rays in scan (integer) */
                float half_scan_f = 256.; /**< Half of total rays in scan (float) */
                int full_scan = 512; /**< Total ray count in scan (integer) */
                float full_scan_f = 512.; /**< Total ray count in scan (float) */
                float angle_increment = (2 * M_PI) / (full_scan_f - 1); /**< Angular increment between consecutive scan indices */
                float range_min = 0.03; /**< Minimum detectable range in scan */
                float range_max = -1e10; /**< Maximum detectable range in scan */
            } scan;            

            struct PlanningMode 
            {
                bool holonomic = false; /**< Boolean for if robot is holonomic or not */
                bool heading = false; /**< Boolean for if robot tracks path headings or not */
                bool projection_operator = true; /**< Boolean for if planner should apply projection operator */
                bool robot_path_orient_linear_decay = true; /**< Enable linear decay of robot path orientation */
                bool virtual_path_decay_enable = true; /**< Enable virtual path decay */    
                float decay_factor = 0.0; /**< Decay factor for virtual path decay */
                bool use_bezier = true; /**< Use Bezier curve for trajectory generation */
            } planning;

            struct ControlParams 
            {
                float Kpx = 3.5; /**< Proportional gain for feedback controller for x-dir velocity */
                float Kpy = 3.5; /**< Proportional gain for feedback controller for y-dir velocity */
                float Kpz = 0.5; /**< Proportional gain for feedback controller for angular velocity */
                int ctrl_ahead_pose = 2; /**< Number of poses ahead of closest pose in current trajectory to track */
            } control;
            
            struct ManualControl 
            {
                bool man_ctrl = false;
                float man_x = 0;
                float man_y = 0;
                float man_theta = 0;
            } man;

            struct GapManipulation 
            {
                float reduction_threshold = M_PI / 2; /**< Minimum span of gap for which we will reduce */
                float reduction_target = M_PI / 4; /**< Target span of gap after reduction */
                // int max_idx_diff = 256;
                bool radial_extend = true;
                bool radial_convert = true;
                float rot_ratio = 1.5;
            } gap_manip;

            struct ProjectionParam 
            {
                float k_po_x = 1.0; /**< Proportional gain in x-direction for projection operator */
                float r_unity = 0.5; /**< Robot to environment distance at which projection operator takes on a value of 1 */
                float r_zero = 1.0; /**< Robot to environment distance at which projection operator takes on a value of 0 */
            } projection;

            struct Trajectory 
            {
                float integrate_maxt = 50;
                float integrate_stept = 1e-2;
                float rmax = 0.5;
                float sigma = 1.0; /**< Sigma used in exp term for circular potential field based trajectory synthesis */
                float inf_ratio = 1.2;
                float Q = 1.0;
                float pen_exp_weight = 3;
                float Q_f = 1.0; /**< Terminal weight for trajectory evaluation */
                float robot_geo_scale = 1;
                bool bezier_interp = true;
                float bezier_unit_time = 1.0; // 0.1;      
                int bezier_num_sampled_pts = 10; /**< Number of sampled points for Bezier curve */       
            } traj;


            struct CollisionChecker
            {
                bool collision_checker_enable = false;
                int cc_type = -1; // assuming depth: 0, depth_ego: 1, egocircle: 2

            } collision_checker;

            void loadRosParamFromNodeHandle(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

            // void reconfigure(qgConfig& cfg);

            boost::mutex & configMutex() {return config_mutex;}

            /**
            * \brief Load in hyperparameters from current laser scan
            */
            void updateParamFromScan(std::shared_ptr<sensor_msgs::msg::LaserScan const> scanPtr);            

        private: 
            boost::mutex config_mutex; 
    };
}