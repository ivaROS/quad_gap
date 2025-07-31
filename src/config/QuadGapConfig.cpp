#include <quad_gap/config/QuadGapConfig.h>

namespace quad_gap 
{
    void QuadGapConfig::loadRosParamFromNodeHandle(const rclcpp::Node::SharedPtr & node)
    {
        // ros::NodeHandle nh("~/" + name);

        // auto node = node_.lock();
        rclcpp::Logger logger_ = node->get_logger();

        // RCLCPP_INFO_STREAM(logger_, "Setting nh to: " << "~/" << name);

        std::string model;
        node->get_parameter("/model", model);

        if (model == "rto")
        {
            RCLCPP_INFO_STREAM(logger_, "Setting model to: " << model);

            RCLCPP_INFO_STREAM(logger_, "map_frame_id is: " << map_frame_id);

            odom_frame_id = model + "/odom";
            RCLCPP_INFO_STREAM(logger_, "Setting odom_frame_id to: " << odom_frame_id);

            robot_frame_id = model + "/base_link";
            RCLCPP_INFO_STREAM(logger_, "Setting robot_frame_id to: " << robot_frame_id);

            sensor_frame_id = model + "/hokuyo_link";
            RCLCPP_INFO_STREAM(logger_, "Setting sensor_frame_id to: " << sensor_frame_id);

            odom_topic = "odom"; // model + "/odom";
            RCLCPP_INFO_STREAM(logger_, "Setting odom_topic to: " << odom_topic);

            scan_topic = "scan"; // model + "/scan";
            RCLCPP_INFO_STREAM(logger_, "Setting scan_topic to: " << scan_topic);

            ///////////
            // Robot //
            ///////////
            ros_throw_param_load(node, "r_inscr", rbt.r_inscr);
            ros_throw_param_load(node, "length", rbt.length);
            ros_throw_param_load(node, "width", rbt.width);
            ros_throw_param_load(node, "avg_lin_speed", rbt.avg_lin_speed);
            ros_throw_param_load(node, "avg_rot_speed", rbt.avg_rot_speed);
            ros_throw_param_load(node, "vx_absmax", rbt.vx_absmax);
            ros_throw_param_load(node, "vy_absmax", rbt.vy_absmax);
            ros_throw_param_load(node, "vang_absmax", rbt.vang_absmax);
            ros_throw_param_load(node, "speed_factor", rbt.speed_factor);            
            ros_throw_param_load(node, "shape_id", rbt.shape_id);
            ros_throw_param_load(node, "use_geo_storage", rbt.use_geo_storage);

            ///////////
            // Goal //
            ///////////
            ros_throw_param_load(node, "xy_global_goal_tolerance", goal.xy_global_goal_tolerance);
            ros_throw_param_load(node, "xy_waypoint_tolerance", goal.xy_waypoint_tolerance);
            ros_throw_param_load(node, "yaw_global_goal_tolerance", goal.yaw_global_goal_tolerance);

            //////////
            // Scan //
            //////////

            // Populated later in updateParamFromScan

            ///////////////////
            // Planning Mode //
            ///////////////////
            ros_throw_param_load(node, "holonomic", planning.holonomic);
            ros_throw_param_load(node, "heading", planning.heading);
            ros_throw_param_load(node, "projection_operator", planning.projection_operator);
            ros_throw_param_load(node, "robot_path_orient_linear_decay", planning.robot_path_orient_linear_decay);
            ros_throw_param_load(node, "virtual_path_decay_enable", planning.virtual_path_decay_enable);
            ros_throw_param_load(node, "decay_factor", planning.decay_factor);
            ros_throw_param_load(node, "use_bezier", planning.use_bezier);

            ////////////////////
            // Control Params //
            ////////////////////
            ros_throw_param_load(node, "Kpx", control.Kpx);
            ros_throw_param_load(node, "Kpy", control.Kpy);
            ros_throw_param_load(node, "Kpz", control.Kpz);
            // ros_throw_param_load(node, "v_ang_const", control.v_ang_const);
            // ros_throw_param_load(node, "v_lin_x_const", control.v_lin_x_const);
            // ros_throw_param_load(node, "v_lin_y_const", control.v_lin_y_const);
            ros_throw_param_load(node, "ctrl_ahead_pose", control.ctrl_ahead_pose);

            ///////////////////////////
            // Manual Control Params //
            ///////////////////////////
            ros_throw_param_load(node, "man_ctrl", man.man_ctrl);
            ros_throw_param_load(node, "man_x", man.man_x);
            ros_throw_param_load(node, "man_y", man.man_y);
            ros_throw_param_load(node, "man_theta", man.man_theta);

            ///////////////////////
            // Gap Manipulation //
            ///////////////////////
            ros_throw_param_load(node, "sigma", traj.sigma);
            ros_throw_param_load(node, "rot_ratio", gap_manip.rot_ratio);
            ros_throw_param_load(node, "reduction_threshold", gap_manip.reduction_threshold);
            ros_throw_param_load(node, "reduction_target", gap_manip.reduction_target);
            // ros_throw_param_load(node, "max_idx_diff", gap_manip.max_idx_diff);
            ros_throw_param_load(node, "radial_extend", gap_manip.radial_extend);
            ros_throw_param_load(node, "radial_convert", gap_manip.radial_convert);

            ///////////////////////
            // Projection Params //
            ///////////////////////
            ros_throw_param_load(node, "k_po_x", projection.k_po_x);
            ros_throw_param_load(node, "r_unity", projection.r_unity);
            ros_throw_param_load(node, "r_zero", projection.r_zero);

            ///////////////////////
            // Trajectory Params //
            ///////////////////////
            ros_throw_param_load(node, "integrate_maxt", traj.integrate_maxt);
            ros_throw_param_load(node, "integrate_stept", traj.integrate_stept);
            ros_throw_param_load(node, "rmax", traj.rmax);
            ros_throw_param_load(node, "inf_ratio", traj.inf_ratio);
            ros_throw_param_load(node, "Q", traj.Q);
            ros_throw_param_load(node, "pen_exp_weight", traj.pen_exp_weight);
            ros_throw_param_load(node, "Q_f", traj.Q_f);
            ros_throw_param_load(node, "robot_geo_scale", traj.robot_geo_scale);
            ros_throw_param_load(node, "bezier_interp", traj.bezier_interp);
            ros_throw_param_load(node, "bezier_unit_time", traj.bezier_unit_time);
            ros_throw_param_load(node, "bezier_num_sampled_pts", traj.bezier_num_sampled_pts);

            ///////////////////////
            // Collision Checker //
            ///////////////////////
            ros_throw_param_load(node, "collision_checker_enable", collision_checker.collision_checker_enable);
            ros_throw_param_load(node, "cc_type", collision_checker.cc_type);

        } else
        {
            throw std::runtime_error("Model " + model + " not implemented!");
        }
    }

    void QuadGapConfig::updateParamFromScan(std::shared_ptr<sensor_msgs::msg::LaserScan const> scanPtr)
    {
        sensor_msgs::msg::LaserScan incomingScan = *scanPtr.get();
        scan.angle_min = incomingScan.angle_min;
        scan.angle_max = incomingScan.angle_max;
        scan.full_scan = incomingScan.ranges.size();
        scan.full_scan_f = float(scan.full_scan);
        scan.half_scan = 0.5 * scan.full_scan;
        scan.half_scan_f = float(scan.half_scan);        
        scan.angle_increment = (2 * M_PI) / (scan.full_scan_f - 1);

        scan.range_max = incomingScan.range_max; // maximum detectable range, not max range within a particular scan
        scan.range_min = incomingScan.range_min; // minimum detectable range, not min range within a particular scan
    }
}