#include <quad_gap/config/QuadGapConfig.h>

namespace quad_gap 
{
    void QuadGapConfig::loadRosParamFromNodeHandle(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
                                                    const std::string & name)
    {
        // ros::NodeHandle nh("~/" + name);

        // auto node = node_.lock();
        rclcpp::Logger logger_ = node->get_logger();

        // // RCLCPP_INFO_STREAM(logger_, "Setting nh to: " << "~/" << name);

        // // RCLCPP_INFO_STREAM(logger_, "Setting model to: " << model);

        // auto parameters_and_prefixes = node->list_parameters({}, 10);

        // for (auto & name : parameters_and_prefixes.names) {
        //     std::cout << "Parameter name: " << name << std::endl;
        // }
        // for (auto & prefix : parameters_and_prefixes.prefixes) {
        //     std::cout << "Parameter prefix: " << prefix << std::endl;
        // }

        // map_frame_id = "map"; // model + "/map";
        ros_throw_param_load(node, name + ".map_frame_id", map_frame_id);
        RCLCPP_INFO_STREAM(logger_, "map_frame_id is: " << map_frame_id);

        // odom_frame_id = "odom";
        ros_throw_param_load(node, name + ".odom_frame_id", odom_frame_id);
        RCLCPP_INFO_STREAM(logger_, "Setting odom_frame_id to: " << odom_frame_id);

        // robot_frame_id = "base_link";
        ros_throw_param_load(node, name + ".robot_frame_id", robot_frame_id);
        RCLCPP_INFO_STREAM(logger_, "Setting robot_frame_id to: " << robot_frame_id);

        // sensor_frame_id = "laser";
        ros_throw_param_load(node, name + ".sensor_frame_id", sensor_frame_id);
        RCLCPP_INFO_STREAM(logger_, "Setting sensor_frame_id to: " << sensor_frame_id);

        // odom_topic = "/ground_truth/state"; // model + "/odom";
        ros_throw_param_load(node, name + ".odom_topic", odom_topic);
        RCLCPP_INFO_STREAM(logger_, "Setting odom_topic to: " << odom_topic);

        // scan_topic = "/scan"; // model + "/scan";
        ros_throw_param_load(node, name + ".scan_topic", scan_topic);
        RCLCPP_INFO_STREAM(logger_, "Setting scan_topic to: " << scan_topic);

        ///////////
        // Robot //
        ///////////
        
        ros_throw_param_load(node, name + ".r_inscr", rbt.r_inscr);
        ros_throw_param_load(node, name + ".length", rbt.length);
        ros_throw_param_load(node, name + ".width", rbt.width);
        ros_throw_param_load(node, name + ".avg_lin_speed", rbt.avg_lin_speed);
        ros_throw_param_load(node, name + ".avg_rot_speed", rbt.avg_rot_speed);
        ros_throw_param_load(node, name + ".vx_absmax", rbt.vx_absmax);
        ros_throw_param_load(node, name + ".vy_absmax", rbt.vy_absmax);
        ros_throw_param_load(node, name + ".vang_absmax", rbt.vang_absmax);
        ros_throw_param_load(node, name + ".speed_factor", rbt.speed_factor);
        ros_throw_param_load(node, name + ".shape_id", rbt.shape_id);
        ros_throw_param_load(node, name + ".use_geo_storage", rbt.use_geo_storage);

        ///////////
        // Goal //
        ///////////
        ros_throw_param_load(node, name + ".xy_global_goal_tolerance", goal.xy_global_goal_tolerance);
        ros_throw_param_load(node, name + ".xy_waypoint_tolerance", goal.xy_waypoint_tolerance);
        ros_throw_param_load(node, name + ".yaw_global_goal_tolerance", goal.yaw_global_goal_tolerance);

        //////////
        // Scan //
        //////////

        // Populated later in updateParamFromScan

        ///////////////////
        // Planning Mode //
        ///////////////////
        ros_throw_param_load(node, name + ".holonomic", planning.holonomic);
        ros_throw_param_load(node, name + ".heading", planning.heading);
        ros_throw_param_load(node, name + ".projection_operator", planning.projection_operator);
        ros_throw_param_load(node, name + ".robot_path_orient_linear_decay", planning.robot_path_orient_linear_decay);
        ros_throw_param_load(node, name + ".virtual_path_decay_enable", planning.virtual_path_decay_enable);
        ros_throw_param_load(node, name + ".decay_factor", planning.decay_factor);
        ros_throw_param_load(node, name + ".use_bezier", planning.use_bezier);

        ////////////////////
        // Control Params //
        ////////////////////
        ros_throw_param_load(node, name + ".Kpx", control.Kpx);
        ros_throw_param_load(node, name + ".Kpy", control.Kpy);
        ros_throw_param_load(node, name + ".Kpz", control.Kpz);
        // ros_throw_param_load(node, name + ".v_ang_const", control.v_ang_const);
        // ros_throw_param_load(node, name + ".v_lin_x_const", control.v_lin_x_const);
        // ros_throw_param_load(node, name + ".v_lin_y_const", control.v_lin_y_const);
        ros_throw_param_load(node, name + ".ctrl_ahead_pose", control.ctrl_ahead_pose);

        ///////////////////////////
        // Manual Control Params //
        ///////////////////////////
        ros_throw_param_load(node, name + ".man_ctrl", man.man_ctrl);
        ros_throw_param_load(node, name + ".man_x", man.man_x);
        ros_throw_param_load(node, name + ".man_y", man.man_y);
        ros_throw_param_load(node, name + ".man_theta", man.man_theta);

        ///////////////////////
        // Gap Manipulation //
        ///////////////////////
        ros_throw_param_load(node, name + ".sigma", traj.sigma);
        ros_throw_param_load(node, name + ".rot_ratio", gap_manip.rot_ratio);
        ros_throw_param_load(node, name + ".reduction_threshold", gap_manip.reduction_threshold);
        ros_throw_param_load(node, name + ".reduction_target", gap_manip.reduction_target);
        // ros_throw_param_load(node, name + ".max_idx_diff", gap_manip.max_idx_diff);
        ros_throw_param_load(node, name + ".radial_extend", gap_manip.radial_extend);
        ros_throw_param_load(node, name + ".radial_convert", gap_manip.radial_convert);

        ///////////////////////
        // Projection Params //
        ///////////////////////
        ros_throw_param_load(node, name + ".k_po_x", projection.k_po_x);
        ros_throw_param_load(node, name + ".r_unity", projection.r_unity);
        ros_throw_param_load(node, name + ".r_zero", projection.r_zero);

        ///////////////////////
        // Trajectory Params //
        ///////////////////////
        ros_throw_param_load(node, name + ".integrate_maxt", traj.integrate_maxt);
        ros_throw_param_load(node, name + ".integrate_stept", traj.integrate_stept);
        ros_throw_param_load(node, name + ".rmax", traj.rmax);
        ros_throw_param_load(node, name + ".inf_ratio", traj.inf_ratio);
        ros_throw_param_load(node, name + ".Q", traj.Q);
        ros_throw_param_load(node, name + ".pen_exp_weight", traj.pen_exp_weight);
        ros_throw_param_load(node, name + ".Q_f", traj.Q_f);
        ros_throw_param_load(node, name + ".robot_geo_scale", traj.robot_geo_scale);
        ros_throw_param_load(node, name + ".bezier_interp", traj.bezier_interp);
        ros_throw_param_load(node, name + ".bezier_unit_time", traj.bezier_unit_time);
        ros_throw_param_load(node, name + ".bezier_num_sampled_pts", traj.bezier_num_sampled_pts);

        ///////////////////////
        // Collision Checker //
        ///////////////////////
        ros_throw_param_load(node, name + ".collision_checker_enable", collision_checker.collision_checker_enable);
        ros_throw_param_load(node, name + ".cc_type", collision_checker.cc_type);
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