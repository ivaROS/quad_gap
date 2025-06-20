#include <quad_gap/config/QuadGapConfig.h>

namespace quad_gap 
{
    void QuadGapConfig::loadRosParamFromNodeHandle(const std::string & name)
    {
        ros::NodeHandle nh("~/" + name);

        ROS_INFO_STREAM_NAMED("Parameters", "Setting nh to: " << "~/" << name);

        std::string model;
        nh.param("/model", model, model); // Must write as "/model" with leading slash

        if (model == "rto")
        {
            ROS_INFO_STREAM_NAMED("Parameters", "Setting model to: " << model);

            ROS_INFO_STREAM_NAMED("Parameters", "map_frame_id is: " << map_frame_id);

            odom_frame_id = model + "/odom";
            ROS_INFO_STREAM_NAMED("Parameters", "Setting odom_frame_id to: " << odom_frame_id);

            robot_frame_id = model + "/base_link";
            ROS_INFO_STREAM_NAMED("Parameters", "Setting robot_frame_id to: " << robot_frame_id);

            sensor_frame_id = model + "/hokuyo_link";
            ROS_INFO_STREAM_NAMED("Parameters", "Setting sensor_frame_id to: " << sensor_frame_id);

            odom_topic = "odom"; // model + "/odom";
            ROS_INFO_STREAM_NAMED("Parameters", "Setting odom_topic to: " << odom_topic);

            scan_topic = "scan"; // model + "/scan";
            ROS_INFO_STREAM_NAMED("Parameters", "Setting scan_topic to: " << scan_topic);

            ///////////
            // Robot //
            ///////////
            ros_throw_param_load(nh, "robot_radius", rbt.r_inscr);
            ros_throw_param_load(nh, "length", rbt.length);
            ros_throw_param_load(nh, "width", rbt.width);
            ros_throw_param_load(nh, "avg_lin_speed", rbt.avg_lin_speed);
            ros_throw_param_load(nh, "avg_rot_speed", rbt.avg_rot_speed);
            ros_throw_param_load(nh, "shape_id", rbt.shape_id);
            ros_throw_param_load(nh, "use_geo_storage", rbt.use_geo_storage);

            ///////////
            // Goal //
            ///////////
            ros_throw_param_load(nh, "xy_global_goal_tolerance", goal.xy_global_goal_tolerance);
            ros_throw_param_load(nh, "xy_waypoint_tolerance", goal.xy_waypoint_tolerance);
            ros_throw_param_load(nh, "yaw_global_goal_tolerance", goal.yaw_global_goal_tolerance);

            //////////
            // Scan //
            //////////

            // Populated later in updateParamFromScan

            ///////////////////
            // Planning Mode //
            ///////////////////
            ros_throw_param_load(nh, "holonomic", planning.holonomic);
            ros_throw_param_load(nh, "projection_operator", planning.projection_operator);
            ros_throw_param_load(nh, "halt_size", planning.halt_size);
            ros_throw_param_load(nh, "robot_path_orient_linear_decay", planning.robot_path_orient_linear_decay);
            ros_throw_param_load(nh, "virtual_path_decay_enable", planning.virtual_path_decay_enable);
            ros_throw_param_load(nh, "decay_factor", planning.decay_factor);
            ros_throw_param_load(nh, "use_bezier", planning.use_bezier);

            ////////////////////
            // Control Params //
            ////////////////////
            ros_throw_param_load(nh, "k_drive_x", control.k_drive_x);
            ros_throw_param_load(nh, "k_drive_y", control.k_drive_y);
            ros_throw_param_load(nh, "k_turn", control.k_turn);
            ros_throw_param_load(nh, "v_ang_const", control.v_ang_const);
            ros_throw_param_load(nh, "v_lin_x_const", control.v_lin_x_const);
            ros_throw_param_load(nh, "v_lin_y_const", control.v_lin_y_const);
            ros_throw_param_load(nh, "ctrl_ahead_pose", control.ctrl_ahead_pose);
            ros_throw_param_load(nh, "vx_absmax", control.vx_absmax);
            ros_throw_param_load(nh, "vy_absmax", control.vy_absmax);
            ros_throw_param_load(nh, "ang_absmax", control.ang_absmax);
            ros_throw_param_load(nh, "speed_factor", control.speed_factor);

            ///////////////////////////
            // Manual Control Params //
            ///////////////////////////
            ros_throw_param_load(nh, "man_ctrl", man.man_ctrl);
            ros_throw_param_load(nh, "man_x", man.man_x);
            ros_throw_param_load(nh, "man_y", man.man_y);
            ros_throw_param_load(nh, "man_theta", man.man_theta);
            ros_throw_param_load(nh, "line", man.line);

            ///////////////////////
            // Gap Manipulation //
            ///////////////////////
            ros_throw_param_load(nh, "sigma", gap_manip.sigma);
            ros_throw_param_load(nh, "rot_ratio", gap_manip.rot_ratio);
            ros_throw_param_load(nh, "reduction_threshold", gap_manip.reduction_threshold);
            ros_throw_param_load(nh, "reduction_target", gap_manip.reduction_target);
            ros_throw_param_load(nh, "max_idx_diff", gap_manip.max_idx_diff);
            ros_throw_param_load(nh, "radial_extend", gap_manip.radial_extend);
            ros_throw_param_load(nh, "radial_convert", gap_manip.radial_convert);

            ///////////////////////
            // Projection Params //
            ///////////////////////
            ros_throw_param_load(nh, "k_po", projection.k_po);
            ros_throw_param_load(nh, "k_po_turn", projection.k_po_turn);
            ros_throw_param_load(nh, "r_min", projection.r_min);
            ros_throw_param_load(nh, "r_norm", projection.r_norm);
            ros_throw_param_load(nh, "r_norm_offset", projection.r_norm_offset);

            ///////////////////////
            // Trajectory Params //
            ///////////////////////
            ros_throw_param_load(nh, "integrate_maxt", traj.integrate_maxt);
            ros_throw_param_load(nh, "integrate_stept", traj.integrate_stept);
            ros_throw_param_load(nh, "rmax", traj.rmax);
            ros_throw_param_load(nh, "inf_ratio", traj.inf_ratio);
            ros_throw_param_load(nh, "terminal_weight", traj.terminal_weight);
            ros_throw_param_load(nh, "robot_geo_scale", traj.robot_geo_scale);
            ros_throw_param_load(nh, "bezier_interp", traj.bezier_interp);
            ros_throw_param_load(nh, "bezier_unit_time", traj.bezier_unit_time);

            ///////////////////////
            // Collision Checker //
            ///////////////////////
            ros_throw_param_load(nh, "collision_checker_enable", collision_checker.collision_checker_enable);
            ros_throw_param_load(nh, "cc_type", collision_checker.cc_type);

        } else
        {
            throw std::runtime_error("Model " + model + " not implemented!");
        }

    }

    void QuadGapConfig::updateParamFromScan(boost::shared_ptr<sensor_msgs::LaserScan const> scanPtr)
    {
        sensor_msgs::LaserScan incomingScan = *scanPtr.get();
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