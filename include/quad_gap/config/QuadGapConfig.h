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
                float length = 0.7; /**< Robot length */
                float width = 0.3; /**< Robot width */
                float avg_lin_speed = 0.2; /**< Average linear speed */
                float avg_rot_speed = 0.5; /**< Average rotational speed */
                int shape_id = 1; /**< Robot shape ID, 1 for rectangle,
                                    2 for circle, 3 for triangle */
                bool use_geo_storage = false; /**< Use precomputed robot geometry storage */
            } rbt;

            struct Goal 
            {
                float xy_global_goal_tolerance = 0.2;
                float yaw_global_goal_tolerance = 0.1;    
                float xy_waypoint_tolerance = 0.1;
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
                int halt_size = 5;           
                bool robot_path_orient_linear_decay = true; /**< Enable linear decay of robot path orientation */
                bool virtual_path_decay_enable = true; /**< Enable virtual path decay */    
                float decay_factor = 0.0; /**< Decay factor for virtual path decay */
                bool use_bezier = true; /**< Use Bezier curve for trajectory generation */
            } planning;

            struct ControlParams 
            {
                float k_drive_x = 3.5;
                float k_drive_y = 3.5;
                float k_turn = 0.5;
                float v_ang_const = 0.0;
                float v_lin_x_const = 0.0;
                float v_lin_y_const = 0.0;
                int ctrl_ahead_pose = 2;
                float vx_absmax = 0.5;
                float vy_absmax = 0.5;
                float ang_absmax = 0.2;       
                float speed_factor = 2.0;
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
                float sigma = 1.0;
                float reduction_threshold = M_PI / 2;
                float reduction_target = M_PI / 4;
                int max_idx_diff = 256;
                bool radial_extend = true;
                bool radial_convert = true;
                float rot_ratio = 1.5;
            } gap_manip;

            struct ProjectionParam 
            {
                float k_po = 0.8;
                float r_min = 0.5;
                float r_norm = 0.75;
                float r_norm_offset = 0.5;
                float k_po_turn = 1;
            } projection;

            struct Trajectory 
            {
                float integrate_maxt = 50;
                float integrate_stept = 1e-2;
                float rmax = 0.3;
                float cobs = -1;
                float w = 3;
                float inf_ratio = 1.2;
                float terminal_weight = 10;
                float robot_geo_scale = 1;
                bool bezier_interp = true;
                float bezier_unit_time = 0.1;             
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