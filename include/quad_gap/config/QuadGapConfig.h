#pragma once

#include <ros/console.h>
#include <ros/ros.h>
#include <quad_gap/qgConfig.h>
#include <Eigen/Core>
#include <boost/thread/mutex.hpp>

#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

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
            std::string acc_topic = "TBD"; /**< IMU ROS topic */
            std::string scan_topic = "TBD"; /**< Laser scan ROS topic */

            struct GapVisualization 
            {
                int min_resoln = 1;
                bool close_gap_vis = false;
                bool follow_the_gap_vis = false;
                bool fig_gen = false;
                double viz_jitter = 0.1;
                bool debug_viz = true;
            } gap_viz;

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

            struct GapManipulation 
            {
                double gap_diff = 0.1;
                double epsilon2 = 0.18;
                double epsilon1 = 0.18;
                double sigma = 1.0;
                double reduction_threshold = M_PI / 2;
                double reduction_target = M_PI / 4;
                int max_idx_diff = 256;
                bool radial_extend = true;
                bool axial_convert = true;
                double rot_ratio = 1.5;
            } gap_manip;

            struct ControlParams 
            {
                double k_drive_x = 3.5;
                double k_drive_y = 3.5;
                double k_turn = 0.5;
                double v_ang_const = 0.0;
                double v_lin_x_const = 0.0;
                double v_lin_y_const = 0.0;
                int ctrl_ahead_pose = 2;
                double vx_absmax = 0.5;
                double vy_absmax = 0.5;
                double ang_absmax = 0.2;             
            } control;
            
            struct ProjectionParam 
            {
                double k_po = 0.8;
                double r_min = 0.5;
                double r_norm = 0.75;
                double r_norm_offset = 0.5;
                double k_po_turn = 1;
            } projection;

            struct Waypoint 
            {
                int global_plan_lookup_increment = 75;
                double global_plan_change_tolerance = 0.1;
            } waypoint;

            struct PlanningMode 
            {
                bool feasi_inflated = false;  
                bool projection_inflated = false;
                // bool planning_inflated;
                bool holonomic = false;
                bool full_fov = false;
                bool projection_operator = true;
                bool niGen_s = false;
                bool far_feasible = false;
                int num_feasi_check = 10;
                int halt_size = 5;              
            } planning;

            struct Goal 
            {
                double lin_goal_tolerance = 0.2;
                double waypoint_tolerance = 0.1;     
                double yaw_goal_tolerance = 0.1;        
            } goal;

            struct Trajectory 
            {
                bool synthesized_frame = false;
                double scale = 1;
                double integrate_maxt = 50;
                double integrate_stept = 1e-2;
                double rmax = 0.3;
                double cobs = -1;
                double w = 3;
                double inf_ratio = 1.2;
                double terminal_weight = 10;
                double waypoint_ratio = 1.5;
                double bezier_cp_scale = 1;
                double robot_geo_scale = 1;
                bool bezier_interp = true;
                double bezier_unit_time = 0.1;             
            } traj;

            struct Robot 
            {
                float r_inscr = 0.18;
            } rbt;

            struct ManualControl 
            {
                bool man_ctrl = false;
                float man_x = 0;
                float man_y = 0;
                float man_theta = 0;
                bool line = false;
            } man;

            void loadRosParamFromNodeHandle(const std::string & name);

            void reconfigure(qgConfig& cfg);

            boost::mutex & configMutex() {return config_mutex;}

            /**
            * \brief Load in hyperparameters from current laser scan
            */
            void updateParamFromScan(boost::shared_ptr<sensor_msgs::LaserScan const> scanPtr);            

        private: 
            boost::mutex config_mutex; 
    };
}