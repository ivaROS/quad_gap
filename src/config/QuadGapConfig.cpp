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

            ///////////
            // Goal //
            ///////////
            ros_throw_param_load(nh, "lin_goal_tolerance", goal.lin_goal_tolerance);
            ros_throw_param_load(nh, "waypoint_tolerance", goal.waypoint_tolerance);
            ros_throw_param_load(nh, "yaw_goal_tolerance", goal.yaw_goal_tolerance);

            // Gap Visualization
            nh.param("min_resoln", gap_viz.min_resoln, gap_viz.min_resoln);
            nh.param("close_gap", gap_viz.close_gap_vis, gap_viz.close_gap_vis);
            nh.param("follow_the_gap", gap_viz.follow_the_gap_vis, gap_viz.follow_the_gap_vis);
            nh.param("fig_gen", gap_viz.fig_gen, gap_viz.fig_gen);
            nh.param("viz_jitter", gap_viz.viz_jitter, gap_viz.viz_jitter);
            nh.param("debug_viz", gap_viz.debug_viz, gap_viz.debug_viz);

            // Gap Manipulation
            nh.param("gap_diff", gap_manip.gap_diff, gap_manip.gap_diff);
            nh.param("epsilon2", gap_manip.epsilon2, gap_manip.epsilon2);
            nh.param("epsilon1", gap_manip.epsilon1, gap_manip.epsilon1);
            nh.param("sigma", gap_manip.sigma, gap_manip.sigma);
            nh.param("rot_ratio", gap_manip.rot_ratio, gap_manip.rot_ratio);
            nh.param("reduction_threshold", gap_manip.reduction_threshold, gap_manip.reduction_threshold);
            nh.param("reduction_target", gap_manip.reduction_target, gap_manip.reduction_target);        
            nh.param("max_idx_diff", gap_manip.max_idx_diff, gap_manip.max_idx_diff);
            nh.param("radial_extend", gap_manip.radial_extend, gap_manip.radial_extend);
            nh.param("radial_convert", gap_manip.radial_convert, gap_manip.radial_convert);

            // Control Params
            nh.param("k_drive_x",control.k_drive_x, control.k_drive_x);
            nh.param("k_drive_y",control.k_drive_y, control.k_drive_y);
            nh.param("k_turn",control.k_turn, control.k_turn);
            nh.param("v_ang_const",control.v_ang_const, control.v_ang_const);
            nh.param("v_lin_x_const",control.v_lin_x_const, control.v_lin_x_const);
            nh.param("v_lin_y_const",control.v_lin_y_const, control.v_lin_y_const);
            nh.param("ctrl_ahead_pose",control.ctrl_ahead_pose, control.ctrl_ahead_pose);

            nh.param("vx_absmax",control.vx_absmax, control.vx_absmax);
            nh.param("vy_absmax",control.vy_absmax, control.vy_absmax);
            nh.param("ang_absmax",control.ang_absmax, control.ang_absmax);

            // Projection Params
            nh.param("k_po", projection.k_po, projection.k_po);
            nh.param("k_po_turn", projection.k_po, projection.k_po);
            nh.param("r_min", projection.r_min, projection.r_min);
            nh.param("r_norm", projection.r_norm, projection.r_norm);
            nh.param("r_norm_offset", projection.r_norm_offset, projection.r_norm_offset);

            // Waypoint Params
            nh.param("global_plan_lookup_increment", waypoint.global_plan_lookup_increment, waypoint.global_plan_lookup_increment);
            nh.param("global_plan_change_tolerance", waypoint.global_plan_change_tolerance, waypoint.global_plan_change_tolerance);

            // Goal Param
            nh.param("lin_goal_tolerance", goal.lin_goal_tolerance, goal.lin_goal_tolerance);
            nh.param("waypoint_tolerance", goal.waypoint_tolerance, goal.waypoint_tolerance);

            // General Planning Mode Params
            nh.param("feasi_inflated", planning.feasi_inflated, planning.feasi_inflated);
            nh.param("projection_inflated", planning.projection_inflated, planning.projection_inflated);
            // nh.param("planning_inflated", planning.planning_inflated, planning.planning_inflated);
            nh.param("holonomic", planning.holonomic, planning.holonomic);
            nh.param("full_fov", planning.full_fov, planning.full_fov);
            nh.param("projection_operator", planning.projection_operator, planning.projection_operator);
            nh.param("niGen_s", planning.niGen_s, planning.niGen_s);
            nh.param("num_feasi_check", planning.num_feasi_check, planning.num_feasi_check);
            nh.param("num_feasi_check", planning.far_feasible, planning.far_feasible);

            // Trajectory
            nh.param("synthesized_frame", traj.synthesized_frame, traj.synthesized_frame);
            nh.param("scale", traj.scale, traj.scale);
            nh.param("integrate_maxt", traj.integrate_maxt, traj.integrate_maxt);
            nh.param("integrate_stept", traj.integrate_stept, traj.integrate_stept);
            nh.param("rmax", traj.rmax, traj.rmax);
            nh.param("inf_ratio", traj.inf_ratio, traj.inf_ratio);
            nh.param("terminal_weight", traj.terminal_weight, traj.terminal_weight);
            nh.param("waypoint_ratio", traj.waypoint_ratio, traj.waypoint_ratio);
            nh.param("bezier_cp_scale", traj.bezier_cp_scale, traj.bezier_cp_scale);
            nh.param("robot_geo_scale", traj.robot_geo_scale, traj.robot_geo_scale);
            nh.param("bezier_interp", traj.bezier_interp, traj.bezier_interp);
            nh.param("bezier_unit_time", traj.bezier_unit_time, traj.bezier_unit_time);
            
            // Robot
            nh.param("r_inscr", rbt.r_inscr, rbt.r_inscr);
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