#pragma once

#include <ros/console.h>
#include <ros/ros.h>
#include <quad_gap/qgConfig.h>
#include <Eigen/Core>
#include <boost/thread/mutex.hpp>

#include <sensor_msgs/LaserScan.h>
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
                float r_inscr = 0.18;
            } rbt;

            struct Goal 
            {
                double lin_goal_tolerance = 0.2;
                double waypoint_tolerance = 0.1;     
                double yaw_goal_tolerance = 0.1;        
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
                bool holonomic = false;
                bool projection_operator = true;
                int num_feasi_check = 10;
                int halt_size = 5;              
            } planning;

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
            
            struct ManualControl 
            {
                bool man_ctrl = false;
                float man_x = 0;
                float man_y = 0;
                float man_theta = 0;
                bool line = false;
            } man;

            struct GapManipulation 
            {
                double sigma = 1.0;
                double reduction_threshold = M_PI / 2;
                double reduction_target = M_PI / 4;
                int max_idx_diff = 256;
                bool radial_extend = true;
                bool radial_convert = true;
                double rot_ratio = 1.5;
            } gap_manip;

            struct ProjectionParam 
            {
                double k_po = 0.8;
                double r_min = 0.5;
                double r_norm = 0.75;
                double r_norm_offset = 0.5;
                double k_po_turn = 1;
            } projection;

            struct Trajectory 
            {
                double integrate_maxt = 50;
                double integrate_stept = 1e-2;
                double rmax = 0.3;
                double cobs = -1;
                double w = 3;
                double inf_ratio = 1.2;
                double terminal_weight = 10;
                double robot_geo_scale = 1;
                bool bezier_interp = true;
                double bezier_unit_time = 0.1;             
            } traj;


            struct CollisionChecker
            {
                bool collision_checker_enable = true;
                int cc_type = -1; // assuming depth: 0, depth_ego: 1, egocircle: 2

            } collision_checker;

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