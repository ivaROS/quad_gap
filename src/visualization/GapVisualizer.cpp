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
        rawGapsPublisher = nh.advertise<visualization_msgs::Marker>("raw_gaps", 10);
        simpGapsPublisher = nh.advertise<visualization_msgs::Marker>("simp_gaps", 10);
        manipGapsPublisher = nh.advertise<visualization_msgs::Marker>("manip_gaps", 10);

        std_msgs::ColorRGBA std_color;
        std_msgs::ColorRGBA raw_radial;
        std_msgs::ColorRGBA raw_swept;
        std_msgs::ColorRGBA fin_radial;
        std_msgs::ColorRGBA fin_swept;
        std_msgs::ColorRGBA extent;
        std_msgs::ColorRGBA agc;

        // Raw Therefore Alpha halved
        raw_radial.a = 0.5;
        raw_radial.r = 0.7;
        raw_radial.g = 0.1;
        raw_radial.b = 0.5;
        
        raw_swept.a = 0.5;
        raw_swept.r = 0.6;
        raw_swept.g = 0.2;
        raw_swept.b = 0.1;

        fin_radial.a = 1;
        fin_radial.r = 1;
        fin_radial.g = 0.9;
        fin_radial.b = 0.3;

        fin_swept.a = 1;
        fin_swept.r = 0.4;
        fin_swept.g = 0;
        fin_swept.b = 0.9;

        extent.a = 1;
        extent.r = 0;
        extent.g = 1;
        extent.b = 0;

        agc.a = 1;
        agc.r = 1;
        agc.g = 0;
        agc.b = 0;

        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("raw_radial", raw_radial));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("raw_swept", raw_swept));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("fin_radial", fin_radial));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("fin_swept", fin_swept));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("fin_extent", extent));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("fin_agc", agc));

    }

    void GapVisualizer::drawGap(visualization_msgs::Marker & marker, const std::vector<Gap> & gaps, 
                                const std::string & ns) // , const bool & initial)     
    {
        // ROS_INFO_STREAM("[drawGap] start");

        // visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time();
        marker.ns = ns;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;        

        float thickness = 0.05;
        marker.scale.x = thickness;     
   
        for (const Gap & gap : gaps) 
        {
            if (gap.getFrame().empty())
            {
                ROS_WARN_STREAM("[drawGap] Gap frame is empty");
                return;
            }

            std::string fullNamespace = ns;

            if (gap.isRadial()) 
            {
                fullNamespace.append("_radial");
            } else 
            {
                fullNamespace.append("_swept");
            }
    
            // std::cout << "gap category: " << g.getCategory() << std::endl;
            //ROS_INFO_STREAM("ultimate local ns: " << fullNamespace);
            auto colorIter = colorMap.find(fullNamespace);
            if (colorIter == colorMap.end()) 
            {
                ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
                return;
            }
    
            marker.header.frame_id = gap.getFrame();

            int leftIdx = gap.LIdx(); // initial ?  : gap.termLIdx(); // initial ? gap.RIdx() : gap.termRIdx(); //
            int rightIdx = gap.RIdx(); // initial ?  : gap.termRIdx(); // initial ? gap.LIdx() : gap.termLIdx(); //
            float leftRange = gap.LRange(); // initial ?  : gap.termLRange(); // initial ? gap.RRange() : gap.termRRange();
            float rightRange = gap.RRange(); // initial ?  : gap.termRRange(); // initial ? gap.LRange() : gap.termLRange();

            //ROS_INFO_STREAM("leftIdx: " << leftIdx << ", ldist: " << ldist << ", rightIdx: " << rightIdx << ", rightRange: " << rightRange);
            int gapIdxSpan = (leftIdx - rightIdx);
            if (gapIdxSpan < 0)
                gapIdxSpan += cfg_->scan.full_scan; // 2*gap.half_scan; // taking off int casting here

            int num_segments = int(invGapSpanResoln * gapIdxSpan) + 1;
            float distIncrement = (leftRange - rightRange) / num_segments;
            int midGapIdx = rightIdx; //  + viz_offset;
            float midGapDist = rightRange;

            float midGapTheta = 0.0;
            for (int i = 0; i < num_segments; i++)
            {
                geometry_msgs::Point p1;
                midGapTheta = idx2theta(midGapIdx);
                p1.x = midGapDist * cos(midGapTheta);
                p1.y = midGapDist * sin(midGapTheta);
                marker.points.push_back(p1);
                marker.colors.push_back(colorIter->second);
                
                midGapIdx = (midGapIdx + gapSpanResoln) % cfg_->scan.full_scan; // int(2*gap.half_scan);
                midGapDist += distIncrement;

                geometry_msgs::Point p2;
                midGapTheta = idx2theta(midGapIdx);
                p2.x = midGapDist * cos(midGapTheta);
                p2.y = midGapDist * sin(midGapTheta);
                marker.points.push_back(p2);
                marker.colors.push_back(colorIter->second);
            }
        }
        // ROS_INFO_STREAM("[drawGap] end");
    }
    
    void GapVisualizer::drawGaps(const std::vector<Gap> & gaps, const std::string & ns) 
    {
        // First, clearing topic.
        clearMarkerPublisher(rawGapsPublisher);
        clearMarkerPublisher(simpGapsPublisher);

        // if (!cfg_->gap_viz.debug_viz) 
        //     return;

        visualization_msgs::Marker marker;
        drawGap(marker, gaps, ns); // , true);

        // for (const Gap & gap : g) 
        // {
        //     drawGap(vis_arr, gap, ns);
        // }
        
        if (ns.find("raw") != std::string::npos) 
        {
            rawGapsPublisher.publish(marker);
        } else if (ns.find("simp") != std::string::npos) 
        {
            simpGapsPublisher.publish(marker);
        }    
    }

    void GapVisualizer::getline(const int & idx, 
                                const float & dist,
                                const Eigen::Vector2f & qB,
                                // std::vector<geometry_msgs::Point>& lines,
                                // geometry_msgs::Point& linel,
                                // geometry_msgs::Point& liner,
                                visualization_msgs::Marker& marker,
                                const std_msgs::ColorRGBA & convex_color) 
    {
        std::vector<geometry_msgs::Point> lines;                                        
        std::vector<std_msgs::ColorRGBA> colors;

        // lines.clear();
        geometry_msgs::Point linel;
        linel.x = qB(0);
        linel.y = qB(1);
        linel.z = 0.1;

        geometry_msgs::Point liner;        
        liner.x = dist * cos(idx2theta(idx));
        liner.y = dist * sin(idx2theta(idx));
        liner.z = 0.1;

        marker.points.push_back(linel);
        marker.points.push_back(liner);
        // this_marker.points = lines;

        marker.colors.push_back(convex_color);
        marker.colors.push_back(convex_color);
        // this_marker.colors = colors;

        // this_marker.id = id;
        // this_marker.lifetime = ros::Duration(0.2);
        // vis_arr.markers.push_back(this_marker);
    };

    void GapVisualizer::drawManipGap(visualization_msgs::Marker & marker, 
                                        const std::vector<Gap> & gaps, 
                                        // const std::string & ns,
                                        bool & circle)
    {
        marker.header.stamp = ros::Time();
        marker.ns = "manip_gaps";
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.01;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;        

        float thickness = 0.05;
        marker.scale.x = thickness;     

        for (const Gap & gap : gaps) 
        {
            if (gap.getFrame().empty())
            {
                ROS_WARN_STREAM("[drawManipGap] Gap frame is empty");
                return;
            }

            std::string ns;
            if (gap.mode.reduced) 
            {
                ns = "fin_swept";
                // viz_jitter += 0.1;
            }
            
            if (gap.mode.convex) 
            {
                ns = "fin_extent";
            }
    
            if (gap.mode.agc) 
            {
                ns = "fin_agc";
            }
            auto colorIter = colorMap.find(ns);
            if (colorIter == colorMap.end()) 
            {
                ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
                return;
            }

            marker.header.frame_id = gap.getFrame();

            int leftIdx = gap.manipLeftIdx();
            int rightIdx = gap.manipRightIdx();
            float leftRange = gap.manipLeftRange();
            float rightRange = gap.manipRightRange();

            // ROS_INFO_STREAM("leftIdx: " << leftIdx << ", leftRange: " << leftRange);
            // ROS_INFO_STREAM("rightIdx: " << rightIdx << ", rightRange: " << rightRange);

            int gapIdxSpan = (leftIdx - rightIdx);
            if (gapIdxSpan < 0)
                gapIdxSpan += cfg_->scan.full_scan; // 2*gap.half_scan; // taking off int casting here

            // ROS_INFO_STREAM("gapIdxSpan: " << gapIdxSpan);

            int num_segments = int(invGapSpanResoln * gapIdxSpan) + 1;
            float distIncrement = (leftRange - rightRange) / num_segments;
            int midGapIdx = rightIdx; //  + viz_offset;
            float midGapDist = rightRange;

            // ROS_INFO_STREAM("num_segments: " << num_segments);
            // ROS_INFO_STREAM("distIncrement: " << distIncrement);
            // ROS_INFO_STREAM("midGapIdx: " << midGapIdx);
            // ROS_INFO_STREAM("midGapDist: " << midGapDist);

            float midGapTheta = 0.0;
            for (int i = 0; i < num_segments; i++)
            {
                geometry_msgs::Point p1;
                midGapTheta = idx2theta(midGapIdx);
                p1.x = midGapDist * cos(midGapTheta);
                p1.y = midGapDist * sin(midGapTheta);
                marker.points.push_back(p1);
                marker.colors.push_back(colorIter->second);
                
                midGapIdx = (midGapIdx + gapSpanResoln) % cfg_->scan.full_scan; // int(2*gap.half_scan);
                midGapDist += distIncrement;

                // ROS_INFO_STREAM("midGapIdx: " << midGapIdx);
                // ROS_INFO_STREAM("midGapDist: " << midGapDist);

                geometry_msgs::Point p2;
                midGapTheta = idx2theta(midGapIdx);
                p2.x = midGapDist * cos(midGapTheta);
                p2.y = midGapDist * sin(midGapTheta);
                marker.points.push_back(p2);
                marker.colors.push_back(colorIter->second);
            }


            if (gap.mode.convex) 
            {
                float r = gap.getMinSafeDist();
                if (r < 0) {
                    ROS_WARN_STREAM("Gap min safe dist not recorded");
                }
    
                std_msgs::ColorRGBA convex_color = colorMap["fin_extent"];
    
                // this_marker.ns = "fin_extent";
    
                // The Circle
                if (!circle) 
                {
                    std::vector<geometry_msgs::Point> lines;
                    geometry_msgs::Point linel;
                    geometry_msgs::Point liner;

                    float pi_over_25 = M_PI / 25;
                    for (int i = 0; i < 50; i++) 
                    {
                        lines.clear();
                        linel.x = r * cos(pi_over_25 * float(i));
                        linel.y = r * sin(pi_over_25 * float(i));
                        linel.z = 0.1;

                        liner.x = r * cos(pi_over_25 * float(i + 1));
                        liner.y = r * sin(pi_over_25 * float(i + 1));
                        liner.z = 0.1;

                        marker.points.push_back(linel);
                        marker.points.push_back(liner);
                        // this_marker.points = lines;
                        marker.colors.push_back(convex_color);
                        marker.colors.push_back(convex_color);
                        // this_marker.colors = convex_color;
                        // this_marker.id = id++;
                        // this_marker.lifetime = ros::Duration();
                        // vis_arr.markers.push_back(this_marker);
                    }
                    circle = true;
                }
    
                // this_marker.ns = "extent_line";
                getline(gap.convex.convex_right_idx, 
                        gap.convex.convex_right_dist, 
                        gap.qB, 
                        // lines, 
                        // linel, 
                        // liner, 
                        marker, 
                        convex_color);
                
                getline(gap.convex.convex_left_idx, 
                        gap.convex.convex_left_dist, 
                        gap.qB, 
                        // lines, 
                        // linel, 
                        // liner, 
                        marker, 
                        convex_color);
                Eigen::Vector2f origin(0, 0);


                // this_marker.ns = "orig_line";
                getline(gap._right_idx, 
                        gap._right_dist, 
                        origin, 
                        // lines, 
                        // linel, 
                        // liner, 
                        marker, 
                        colorMap["fin_agc"]);

                getline(gap._left_idx, 
                        gap._left_dist, 
                        origin, 
                        // lines, 
                        // linel, 
                        // liner, 
                        marker, 
                        colorMap["fin_agc"]);
            }            
        }

    }    

    // void GapVisualizer::drawManipGap(visualization_msgs::MarkerArray & vis_arr, const Gap & g, bool & circle) 
    // {
    //     // if AGC: Color is Red
    //     // if Convex: color is Brown, viz_jitter + 0.1
    //     // if RadialExtension: color is green, draw additional circle
    //     if (!cfg_->gap_viz.debug_viz) return;

    //     if (!g.mode.reduced && !g.mode.convex && !g.mode.agc) 
    //     {
    //         return;
    //     }

    //     float viz_jitter = (float) cfg_->gap_viz.viz_jitter;
    //     int viz_offset = 0;
    //     if (viz_jitter > 0 && g.isRadial())
    //     {
    //         viz_offset = g.isRightType() ? -2 : 2;
    //     }

    //     std::string ns;
    //     if (g.mode.reduced) {
    //         ns = "fin_swept";
    //         viz_jitter += 0.1;
    //     }
        
    //     if (g.mode.convex) {
    //         ns = "fin_extent";
    //     }

    //     if (g.mode.agc) {
    //         ns = "fin_agc";
    //     }
        
    //     int num_gaps = (g.convex.convex_left_idx - g.convex.convex_right_idx) / cfg_->gap_viz.min_resoln + 1;
    //     float dist_step = (g.convex.convex_left_dist - g.convex.convex_right_dist) / num_gaps;
    //     int sub_gap_right_idx = g.convex.convex_right_idx + viz_offset;
    //     float sub_gap_right_dist = g.convex.convex_right_dist;

    //     visualization_msgs::Marker this_marker;
    //     this_marker.header.frame_id = g._frame;
    //     this_marker.header.stamp = ros::Time();
    //     this_marker.ns = ns;
    //     this_marker.type = visualization_msgs::Marker::LINE_STRIP;
    //     this_marker.action = visualization_msgs::Marker::ADD;

    //     auto color_value = colorMap.find(ns);
    //     if (color_value == colorMap.end()) 
    //     {
    //         ROS_FATAL_STREAM("[drawManipGaps] Visualization Color not found, return without drawing");
    //         return;
    //     }

    //     this_marker.colors = color_value->second;
    //     double thickness = cfg_->gap_viz.fig_gen ? 0.05 : 0.01;
    //     this_marker.scale.x = thickness;
    //     this_marker.scale.y = 0.1;
    //     this_marker.scale.z = 0.1;

    //     geometry_msgs::Point linel;
    //     geometry_msgs::Point liner;
    //     liner.z = 0.1;
    //     linel.z = 0.1;
    //     std::vector<geometry_msgs::Point> lines;

    //     int id = (int) vis_arr.markers.size();
    //     // ROS_INFO_STREAM("ID: "<< id);

    //     this_marker.lifetime = ros::Duration(0.25);

    //     for (int i = 0; i < num_gaps - 1; i++)
    //     {
    //         lines.clear();
    //         linel.x = (sub_gap_right_dist + viz_jitter) * cos(idx2theta(sub_gap_right_idx));
    //         linel.y = (sub_gap_right_dist + viz_jitter) * sin(idx2theta(sub_gap_right_idx));
    //         sub_gap_right_idx += cfg_->gap_viz.min_resoln;
    //         sub_gap_right_dist += dist_step;
    //         liner.x = (sub_gap_right_dist + viz_jitter) * cos(idx2theta(sub_gap_right_idx));
    //         liner.y = (sub_gap_right_dist + viz_jitter) * sin(idx2theta(sub_gap_right_idx));
    //         lines.push_back(linel);
    //         lines.push_back(liner);

    //         this_marker.points = lines;
    //         this_marker.id = id++;
    //         vis_arr.markers.push_back(this_marker);
    //     }

    //     // close the last
    //     lines.clear();
    //     linel.x = (sub_gap_right_dist + viz_jitter) * cos(idx2theta(sub_gap_right_idx));
    //     linel.y = (sub_gap_right_dist + viz_jitter) * sin(idx2theta(sub_gap_right_idx));
    //     liner.x = (g.convex.convex_left_dist + viz_jitter) * cos(idx2theta(g.convex.convex_left_idx));
    //     liner.y = (g.convex.convex_left_dist + viz_jitter) * sin(idx2theta(g.convex.convex_left_idx));
    //     lines.push_back(linel);
    //     lines.push_back(liner);
    //     this_marker.points = lines;
    //     this_marker.id = id++;
    //     this_marker.lifetime = ros::Duration(g.life_time);
    //     vis_arr.markers.push_back(this_marker);

    //     if (g.mode.convex) {
    //         float r = g.getMinSafeDist();
    //         if (r < 0) {
    //             ROS_WARN_STREAM("Gap min safe dist not recorded");
    //         }

    //         auto convex_color = colorMap["fin_extent"];

    //         this_marker.ns = "fin_extent";

    //         // The Circle
    //         if (!circle) 
    //         {
    //             for (int i = 0; i < 50; i++) 
    //             {
    //                 lines.clear();
    //                 linel.x = r * cos(M_PI / 25 * float(i));
    //                 linel.y = r * sin(M_PI / 25 * float(i));
    //                 liner.x = r * cos(M_PI / 25 * float(i + 1));
    //                 liner.y = r * sin(M_PI / 25 * float(i + 1));
    //                 lines.push_back(linel);
    //                 lines.push_back(liner);
    //                 this_marker.points = lines;
    //                 this_marker.colors = convex_color;
    //                 this_marker.id = id++;
    //                 this_marker.lifetime = ros::Duration();
    //                 vis_arr.markers.push_back(this_marker);
    //             }
    //             circle = true;
    //         }

    //         auto getline = [] (const int & idx, const float & dist,
    //                             const Eigen::Vector2f & qB,
    //                             std::vector<geometry_msgs::Point>& lines,
    //                             geometry_msgs::Point& linel,
    //                             geometry_msgs::Point& liner,
    //                             visualization_msgs::Marker& this_marker,
    //                             const std_msgs::ColorRGBA & convex_color,
    //                             visualization_msgs::MarkerArray& vis_arr,
    //                             const int & id
    //                             ) -> void {
    //                                 lines.clear();
    //                                 linel.x = qB(0);
    //                                 linel.y = qB(1);
    //                                 linel.z = 0.1;
    //                                 liner.x = dist * cos(idx2theta(idx));
    //                                 liner.y = dist * sin(idx2theta(idx));
    //                                 liner.z = 0.1;
    //                                 lines.push_back(linel);
    //                                 lines.push_back(liner);
    //                                 this_marker.points = lines;
    //                                 this_marker.colors = convex_color;
    //                                 this_marker.id = id;
    //                                 this_marker.lifetime = ros::Duration(0.2);
    //                                 vis_arr.markers.push_back(this_marker);
    //                             };

    //         {
    //             this_marker.ns = "extent_line";
    //             getline(g.convex.convex_right_idx, g.convex.convex_right_dist, g.qB, 
    //                 lines, linel, liner, this_marker, convex_color, vis_arr, id++);
    //             getline(g.convex.convex_left_idx, g.convex.convex_left_dist, g.qB, 
    //                 lines, linel, liner, this_marker, convex_color, vis_arr, id++);
    //             Eigen::Vector2f origin(0, 0);


    //             this_marker.ns = "orig_line";
    //             getline(g._right_idx, g._right_dist, origin, 
    //                 lines, linel, liner, this_marker, colorMap["fin_agc"], vis_arr, id++);
    //             getline(g._left_idx, g._left_dist, origin, 
    //                 lines, linel, liner, this_marker, colorMap["fin_agc"], vis_arr, id++);
    //         }
    //     }

    // }

    void GapVisualizer::drawManipGaps(const std::vector<Gap> & gaps) 
    {
        // First, clearing topic.
        clearMarkerPublisher(manipGapsPublisher);

        bool circle = false;

        visualization_msgs::Marker marker;
        drawManipGap(marker, gaps, circle); // , true);

        manipGapsPublisher.publish(marker);
    }

    // void GapVisualizer::drawManipGaps(const std::vector<Gap> & vec) 
    // {
    //     // First, clearing topic.
    //     clearMarkerArrayPublisher(manipGapsPublisher);

    //     if (!cfg_->gap_viz.debug_viz) return;
    //     visualization_msgs::MarkerArray vis_arr;
    //     bool circle = false;
        
    //     for (const Gap & gap : vec) 
    //     {
    //         drawManipGap(vis_arr, gap, circle);
    //     }
    //     manipGapsPublisher.publish(vis_arr);
    // }
}