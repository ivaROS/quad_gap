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
        std::vector<std_msgs::ColorRGBA> raw_swept;
        std::vector<std_msgs::ColorRGBA> fin_radial;
        std::vector<std_msgs::ColorRGBA> fin_swept;
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
        raw_swept.push_back(std_color);
        raw_swept.push_back(std_color);
        std_color.a = 1;
        std_color.r = 1;
        std_color.g = 0.9;
        std_color.b = 0.3;
        fin_radial.push_back(std_color);
        fin_radial.push_back(std_color);
        std_color.r = 0.4;
        std_color.g = 0;
        std_color.b = 0.9;
        fin_swept.push_back(std_color);
        fin_swept.push_back(std_color);
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
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("raw_radial", raw_radial));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("raw_swept", raw_swept));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("fin_radial", fin_radial));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("fin_swept", fin_swept));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("fin_extent", extent));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("fin_agc", agc));

    }

    void GapVisualizer::drawGap(visualization_msgs::MarkerArray & vis_arr, Gap g, std::string ns, std::string color) {
        // ROS_INFO_STREAM(g._right_idx << ", " << g._right_dist << ", " << g._left_idx << ", " << g._left_dist << ", " << g._frame);
        if (!cfg_->gap_viz.debug_viz) return;

        int viz_offset = 0;
        double viz_jitter = cfg_->gap_viz.viz_jitter;
        if (viz_jitter > 0 && g.setRadial()){
            viz_offset = g.isRightType() ? -2 : 2;
        }

        int num_gaps = (g._left_idx - g._right_idx) / cfg_->gap_viz.min_resoln + 1;
        float dist_step = (g._left_dist - g._right_dist) / num_gaps;
        int sub_gap_right_idx = g._right_idx + viz_offset;
        float sub_gap_right_dist = g._right_dist;

        visualization_msgs::Marker this_marker;
        this_marker.header.frame_id = g._frame;
        this_marker.header.stamp = ros::Time();
        this_marker.ns = ns;
        this_marker.type = visualization_msgs::Marker::LINE_STRIP;
        this_marker.action = visualization_msgs::Marker::ADD;

        std::string local_ns = ns;
        if (g.setRadial()) {
            local_ns.append("_radial");
        } else {
            local_ns.append("_swept");
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
            linel.x = (sub_gap_right_dist + viz_jitter) * cos(idx2theta(sub_gap_right_idx));
            linel.y = (sub_gap_right_dist + viz_jitter) * sin(idx2theta(sub_gap_right_idx));
            sub_gap_right_idx += cfg_->gap_viz.min_resoln;
            sub_gap_right_dist += dist_step;
            liner.x = (sub_gap_right_dist + viz_jitter) * cos(idx2theta(sub_gap_right_idx));
            liner.y = (sub_gap_right_dist + viz_jitter) * sin(idx2theta(sub_gap_right_idx));
            lines.push_back(linel);
            lines.push_back(liner);

            this_marker.points = lines;
            this_marker.id = id++;
            this_marker.lifetime = ros::Duration(g.life_time);
            vis_arr.markers.push_back(this_marker);
        }

        // close the last
        lines.clear();
        linel.x = (sub_gap_right_dist + viz_jitter) * cos(idx2theta(sub_gap_right_idx));
        linel.y = (sub_gap_right_dist + viz_jitter) * sin(idx2theta(sub_gap_right_idx));
        liner.x = (g._left_dist + viz_jitter) * cos(idx2theta(g._left_idx));
        liner.y = (g._left_dist + viz_jitter) * sin(idx2theta(g._left_idx));
        lines.push_back(linel);
        lines.push_back(liner);
        this_marker.points = lines;
        this_marker.id = id++;
        this_marker.lifetime = ros::Duration(g.life_time);
        vis_arr.markers.push_back(this_marker);
    }
    
    void GapVisualizer::drawGaps(std::vector<Gap> g, std::string ns, std::string color) {
        if (!cfg_->gap_viz.debug_viz) return;
        visualization_msgs::MarkerArray vis_arr;
        for (auto & gap : g) {
            drawGap(vis_arr, gap, ns);
        }
        gaparc_publisher.publish(vis_arr);
    }

    void GapVisualizer::drawManipGap(visualization_msgs::MarkerArray & vis_arr, Gap g, bool & circle) {
        // if AGC: Color is Red
        // if Convex: color is Brown, viz_jitter + 0.1
        // if RadialExtension: color is green, draw additional circle
        if (!cfg_->gap_viz.debug_viz) return;

        if (!g.mode.reduced && !g.mode.convex && !g.mode.agc) {
            return;
        }

        float viz_jitter = (float) cfg_->gap_viz.viz_jitter;
        int viz_offset = 0;
        if (viz_jitter > 0 && g.setRadial()){
            viz_offset = g.isRightType() ? -2 : 2;
        }

        std::string ns;
        if (g.mode.reduced) {
            ns = "fin_swept";
            viz_jitter += 0.1;
        }
        
        if (g.mode.convex) {
            ns = "fin_extent";
        }

        if (g.mode.agc) {
            ns = "fin_agc";
        }
        
        int num_gaps = (g.convex.convex_left_idx - g.convex.convex_right_idx) / cfg_->gap_viz.min_resoln + 1;
        float dist_step = (g.convex.convex_left_dist - g.convex.convex_right_dist) / num_gaps;
        int sub_gap_right_idx = g.convex.convex_right_idx + viz_offset;
        float sub_gap_right_dist = g.convex.convex_right_dist;

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
            linel.x = (sub_gap_right_dist + viz_jitter) * cos(idx2theta(sub_gap_right_idx));
            linel.y = (sub_gap_right_dist + viz_jitter) * sin(idx2theta(sub_gap_right_idx));
            sub_gap_right_idx += cfg_->gap_viz.min_resoln;
            sub_gap_right_dist += dist_step;
            liner.x = (sub_gap_right_dist + viz_jitter) * cos(idx2theta(sub_gap_right_idx));
            liner.y = (sub_gap_right_dist + viz_jitter) * sin(idx2theta(sub_gap_right_idx));
            lines.push_back(linel);
            lines.push_back(liner);

            this_marker.points = lines;
            this_marker.id = id++;
            vis_arr.markers.push_back(this_marker);
        }

        // close the last
        lines.clear();
        linel.x = (sub_gap_right_dist + viz_jitter) * cos(idx2theta(sub_gap_right_idx));
        linel.y = (sub_gap_right_dist + viz_jitter) * sin(idx2theta(sub_gap_right_idx));
        liner.x = (g.convex.convex_left_dist + viz_jitter) * cos(idx2theta(g.convex.convex_left_idx));
        liner.y = (g.convex.convex_left_dist + viz_jitter) * sin(idx2theta(g.convex.convex_left_idx));
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
                                int id
                                ) -> void {
                                    lines.clear();
                                    linel.x = qB(0);
                                    linel.y = qB(1);
                                    linel.z = 0.1;
                                    liner.x = dist * cos(idx2theta(idx));
                                    liner.y = dist * sin(idx2theta(idx));
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
                getline(g.convex.convex_right_idx, g.convex.convex_right_dist, g.qB, 
                    lines, linel, liner, this_marker, convex_color, vis_arr, id++);
                getline(g.convex.convex_left_idx, g.convex.convex_left_dist, g.qB, 
                    lines, linel, liner, this_marker, convex_color, vis_arr, id++);
                Eigen::Vector2f origin(0, 0);


                this_marker.ns = "orig_line";
                getline(g._right_idx, g._right_dist, origin, 
                    lines, linel, liner, this_marker, colormap["fin_agc"], vis_arr, id++);
                getline(g._left_idx, g._left_dist, origin, 
                    lines, linel, liner, this_marker, colormap["fin_agc"], vis_arr, id++);
            }
        }

    }

    void GapVisualizer::drawManipGaps(std::vector<Gap> vec) {
        if (!cfg_->gap_viz.debug_viz) return;
        visualization_msgs::MarkerArray vis_arr;
        bool circle = false;
        for (auto & gap : vec) {
            drawManipGap(vis_arr, gap, circle);
        }
        gapside_publisher.publish(vis_arr);
    }
}