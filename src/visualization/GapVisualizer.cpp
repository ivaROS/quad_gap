#include <quad_gap/visualization/GapVisualizer.h>

namespace quad_gap
{
    GapVisualizer::GapVisualizer(ros::NodeHandle& nh, const QuadGapConfig& cfg) 
    {
        initialize(nh, cfg);
    }

    void GapVisualizer::initialize(ros::NodeHandle& nh, const QuadGapConfig& cfg) 
    {
        cfg_ = &cfg;
        gaparc_publisher = nh.advertise<visualization_msgs::MarkerArray>("qg_arcs", 1000);
        gapside_publisher = nh.advertise<visualization_msgs::MarkerArray>("qg_sides", 100);
        gaqgoal_publisher = nh.advertise<visualization_msgs::MarkerArray>("qg_markers", 10);

        std_msgs::ColorRGBA std_color;
        std::vector<std_msgs::ColorRGBA> raw_radial;
        std::vector<std_msgs::ColorRGBA> raw_axial;
        std::vector<std_msgs::ColorRGBA> fin_radial;
        std::vector<std_msgs::ColorRGBA> fin_axial;
        std::vector<std_msgs::ColorRGBA> extent;
        std::vector<std_msgs::ColorRGBA> agc;

        // Raw Therefore Alpha halved
        std_color.a = 0.5;
        std_color.r = 0.7;
        std_color.g = 0.1;
        std_color.b = 0.5;
        raw_radial.push_back(std_color);
        raw_radial.push_back(std_color);
        std_color.r = 0.6;
        std_color.g = 0.2;
        std_color.b = 0.1;
        raw_axial.push_back(std_color);
        raw_axial.push_back(std_color);
        std_color.a = 1;
        std_color.r = 1;
        std_color.g = 0.9;
        std_color.b = 0.3;
        fin_radial.push_back(std_color);
        fin_radial.push_back(std_color);
        std_color.r = 0.4;
        std_color.g = 0;
        std_color.b = 0.9;
        fin_axial.push_back(std_color);
        fin_axial.push_back(std_color);
        std_color.r = 0;
        std_color.g = 1;
        std_color.b = 0;
        extent.push_back(std_color);
        extent.push_back(std_color);
        std_color.r = 1;
        std_color.g = 0;
        std_color.b = 0;
        agc.push_back(std_color);
        agc.push_back(std_color);
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("raw_axial", raw_axial));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("raw_radial", raw_radial));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("fin_radial", fin_radial));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("fin_axial", fin_axial));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("fin_extent", extent));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("fin_agc", agc));

    }

    void GapVisualizer::drawGap(visualization_msgs::MarkerArray & vis_arr, quad_gap::Gap g, std::string ns, std::string color) {
        // ROS_INFO_STREAM(g._left_idx << ", " << g._ldist << ", " << g._right_idx << ", " << g._rdist << ", " << g._frame);
        if (!cfg_->gap_viz.debug_viz) return;

        int viz_offset = 0;
        double viz_jitter = cfg_->gap_viz.viz_jitter;
        if (viz_jitter > 0 && g.isAxial()){
            viz_offset = g.isLeftType() ? -2 : 2;
        }

        int num_gaps = (g._right_idx - g._left_idx) / cfg_->gap_viz.min_resoln + 1;
        float dist_step = (g._rdist - g._ldist) / num_gaps;
        int sub_gap_lidx = g._left_idx + viz_offset;
        float sub_gap_ldist = g._ldist;

        visualization_msgs::Marker this_marker;
        this_marker.header.frame_id = g._frame;
        this_marker.header.stamp = ros::Time();
        this_marker.ns = ns;
        this_marker.type = visualization_msgs::Marker::LINE_STRIP;
        this_marker.action = visualization_msgs::Marker::ADD;

        std::string local_ns = ns;
        if (g.isAxial()) {
            local_ns.append("_axial");
        } else {
            local_ns.append("_radial");
        }

        auto color_value = colormap.find(local_ns);
        if (color_value == colormap.end()) {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }

        this_marker.colors = color_value->second;
        double thickness = cfg_->gap_viz.fig_gen ? 0.05 : 0.01;
        this_marker.scale.x = thickness;
        this_marker.scale.y = 0.1;
        this_marker.scale.z = 0.1;
        bool finNamespace = (ns.compare("fin") == 0);

        geometry_msgs::Point linel;
        geometry_msgs::Point liner;
        std::vector<geometry_msgs::Point> lines;

        if (finNamespace) {
            this_marker.colors.at(0).a = 1;
            this_marker.colors.at(1).a = 1;
            linel.z = 0.1;
            liner.z = 0.1;
        }

        int id = (int) vis_arr.markers.size();

        for (int i = 0; i < num_gaps - 1; i++)
        {
            lines.clear();
            linel.x = (sub_gap_ldist + viz_jitter) * cos(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            linel.y = (sub_gap_ldist + viz_jitter) * sin(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            sub_gap_lidx += cfg_->gap_viz.min_resoln;
            sub_gap_ldist += dist_step;
            liner.x = (sub_gap_ldist + viz_jitter) * cos(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            liner.y = (sub_gap_ldist + viz_jitter) * sin(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            lines.push_back(linel);
            lines.push_back(liner);

            this_marker.points = lines;
            this_marker.id = id++;
            this_marker.lifetime = ros::Duration(g.life_time);
            vis_arr.markers.push_back(this_marker);
        }

        // close the last
        lines.clear();
        linel.x = (sub_gap_ldist + viz_jitter) * cos(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
        linel.y = (sub_gap_ldist + viz_jitter) * sin(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
        liner.x = (g._rdist + viz_jitter) * cos(-( (float) g.half_scan - g._right_idx) / g.half_scan * M_PI);
        liner.y = (g._rdist + viz_jitter) * sin(-( (float) g.half_scan - g._right_idx) / g.half_scan * M_PI);
        lines.push_back(linel);
        lines.push_back(liner);
        this_marker.points = lines;
        this_marker.id = id++;
        this_marker.lifetime = ros::Duration(g.life_time);
        vis_arr.markers.push_back(this_marker);
    }
    
    void GapVisualizer::drawGaps(std::vector<quad_gap::Gap> g, std::string ns, std::string color) {
        if (!cfg_->gap_viz.debug_viz) return;
        visualization_msgs::MarkerArray vis_arr;
        for (auto & gap : g) {
            drawGap(vis_arr, gap, ns);
        }
        gaparc_publisher.publish(vis_arr);
    }

    void GapVisualizer::drawManipGap(visualization_msgs::MarkerArray & vis_arr, quad_gap::Gap g, bool & circle) {
        // if AGC: Color is Red
        // if Convex: color is Brown, viz_jitter + 0.1
        // if RadialExtension: color is green, draw additional circle
        if (!cfg_->gap_viz.debug_viz) return;

        if (!g.mode.reduced && !g.mode.convex && !g.mode.agc) {
            return;
        }

        float viz_jitter = (float) cfg_->gap_viz.viz_jitter;
        int viz_offset = 0;
        if (viz_jitter > 0 && g.isAxial()){
            viz_offset = g.isLeftType() ? -2 : 2;
        }

        std::string ns;
        if (g.mode.reduced) {
            ns = "fin_radial";
            viz_jitter += 0.1;
        }
        
        if (g.mode.convex) {
            ns = "fin_extent";
        }

        if (g.mode.agc) {
            ns = "fin_agc";
        }
        
        int num_gaps = (g.convex.convex_ridx - g.convex.convex_lidx) / cfg_->gap_viz.min_resoln + 1;
        float dist_step = (g.convex.convex_rdist - g.convex.convex_ldist) / num_gaps;
        int sub_gap_lidx = g.convex.convex_lidx + viz_offset;
        float sub_gap_ldist = g.convex.convex_ldist;

        visualization_msgs::Marker this_marker;
        this_marker.header.frame_id = g._frame;
        this_marker.header.stamp = ros::Time();
        this_marker.ns = ns;
        this_marker.type = visualization_msgs::Marker::LINE_STRIP;
        this_marker.action = visualization_msgs::Marker::ADD;

        auto color_value = colormap.find(ns);
        if (color_value == colormap.end()) {
            ROS_FATAL_STREAM("[drawManipGaps] Visualization Color not found, return without drawing");
            return;
        }

        this_marker.colors = color_value->second;
        double thickness = cfg_->gap_viz.fig_gen ? 0.05 : 0.01;
        this_marker.scale.x = thickness;
        this_marker.scale.y = 0.1;
        this_marker.scale.z = 0.1;

        geometry_msgs::Point linel;
        geometry_msgs::Point liner;
        liner.z = 0.1;
        linel.z = 0.1;
        std::vector<geometry_msgs::Point> lines;

        int id = (int) vis_arr.markers.size();
        // ROS_INFO_STREAM("ID: "<< id);

        this_marker.lifetime = ros::Duration(0.25);

        for (int i = 0; i < num_gaps - 1; i++)
        {
            lines.clear();
            linel.x = (sub_gap_ldist + viz_jitter) * cos(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            linel.y = (sub_gap_ldist + viz_jitter) * sin(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            sub_gap_lidx += cfg_->gap_viz.min_resoln;
            sub_gap_ldist += dist_step;
            liner.x = (sub_gap_ldist + viz_jitter) * cos(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            liner.y = (sub_gap_ldist + viz_jitter) * sin(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            lines.push_back(linel);
            lines.push_back(liner);

            this_marker.points = lines;
            this_marker.id = id++;
            vis_arr.markers.push_back(this_marker);
        }

        // close the last
        lines.clear();
        linel.x = (sub_gap_ldist + viz_jitter) * cos(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
        linel.y = (sub_gap_ldist + viz_jitter) * sin(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
        liner.x = (g.convex.convex_rdist + viz_jitter) * cos(-( (float) g.half_scan - g.convex.convex_ridx) / g.half_scan * M_PI);
        liner.y = (g.convex.convex_rdist + viz_jitter) * sin(-( (float) g.half_scan - g.convex.convex_ridx) / g.half_scan * M_PI);
        lines.push_back(linel);
        lines.push_back(liner);
        this_marker.points = lines;
        this_marker.id = id++;
        this_marker.lifetime = ros::Duration(g.life_time);
        vis_arr.markers.push_back(this_marker);

        if (g.mode.convex) {
            float r = g.getMinSafeDist();
            if (r < 0) {
                ROS_WARN_STREAM("Gap min safe dist not recorded");
            }

            auto convex_color = colormap["fin_extent"];

            this_marker.ns = "fin_extent";

            // The Circle
            if (!circle) {
                for (int i = 0; i < 50; i++) {
                    lines.clear();
                    linel.x = r * cos(M_PI / 25 * float(i));
                    linel.y = r * sin(M_PI / 25 * float(i));
                    liner.x = r * cos(M_PI / 25 * float(i + 1));
                    liner.y = r * sin(M_PI / 25 * float(i + 1));
                    lines.push_back(linel);
                    lines.push_back(liner);
                    this_marker.points = lines;
                    this_marker.colors = convex_color;
                    this_marker.id = id++;
                    this_marker.lifetime = ros::Duration();
                    vis_arr.markers.push_back(this_marker);
                }
                circle = true;
            }

            auto getline = [] (int idx, float dist,
                                Eigen::Vector2f& qB,
                                std::vector<geometry_msgs::Point>& lines,
                                geometry_msgs::Point& linel,
                                geometry_msgs::Point& liner,
                                visualization_msgs::Marker& this_marker,
                                std::vector<std_msgs::ColorRGBA> & convex_color,
                                visualization_msgs::MarkerArray& vis_arr,
                                int half_num_scan,
                                int id
                                ) -> void {
                                    lines.clear();
                                    linel.x = qB(0);
                                    linel.y = qB(1);
                                    linel.z = 0.1;
                                    liner.x = dist * cos(M_PI / half_num_scan * (idx - half_num_scan));
                                    liner.y = dist * sin(M_PI / half_num_scan * (idx - half_num_scan));
                                    liner.z = 0.1;
                                    lines.push_back(linel);
                                    lines.push_back(liner);
                                    this_marker.points = lines;
                                    this_marker.colors = convex_color;
                                    this_marker.id = id;
                                    this_marker.lifetime = ros::Duration(0.2);
                                    vis_arr.markers.push_back(this_marker);
                                };

            {
                this_marker.ns = "extent_line";
                getline(g.convex.convex_lidx, g.convex.convex_ldist, g.qB, 
                    lines, linel, liner, this_marker, convex_color, vis_arr, g.half_scan, id++);
                getline(g.convex.convex_ridx, g.convex.convex_rdist, g.qB, 
                    lines, linel, liner, this_marker, convex_color, vis_arr, g.half_scan, id++);
                Eigen::Vector2f origin(0, 0);


                this_marker.ns = "orig_line";
                getline(g._left_idx, g._ldist, origin, 
                    lines, linel, liner, this_marker, colormap["fin_agc"], vis_arr, g.half_scan, id++);
                getline(g._right_idx, g._rdist, origin, 
                    lines, linel, liner, this_marker, colormap["fin_agc"], vis_arr, g.half_scan, id++);
            }
        }

    }

    void GapVisualizer::drawManipGaps(std::vector<quad_gap::Gap> vec) {
        if (!cfg_->gap_viz.debug_viz) return;
        visualization_msgs::MarkerArray vis_arr;
        bool circle = false;
        for (auto & gap : vec) {
            drawManipGap(vis_arr, gap, circle);
        }
        gapside_publisher.publish(vis_arr);
    }
}