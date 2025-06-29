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

        // std_msgs::ColorRGBA std_color;
        std_msgs::ColorRGBA raw_radial;
        std_msgs::ColorRGBA raw_swept;
        std_msgs::ColorRGBA simp_radial;
        std_msgs::ColorRGBA simp_swept;
        // std_msgs::ColorRGBA extent;
        // std_msgs::ColorRGBA agc;
        std_msgs::ColorRGBA manip;

        // Raw Therefore Alpha halved
        raw_radial.a = 1.0;
        raw_radial.r = 1.0;
        raw_radial.g = 0.3;
        raw_radial.b = 0.3;
        
        raw_swept.a = 1.0;
        raw_swept.r = 0.6;
        raw_swept.g = 0.0;
        raw_swept.b = 0.0;

        simp_radial.a = 1.0;
        simp_radial.r = 0.3;
        simp_radial.g = 0.6;
        simp_radial.b = 1.0;

        simp_swept.a = 1.0;
        simp_swept.r = 0.0;
        simp_swept.g = 0.3;
        simp_swept.b = 0.6;

        // extent.a = 1;
        // extent.r = 0;
        // extent.g = 1;
        // extent.b = 0;

        // agc.a = 1;
        // agc.r = 1;
        // agc.g = 0;
        // agc.b = 0;

        manip.a = 1.0;
        manip.r = 0.0;
        manip.g = 1.0;
        manip.b = 0.0;

        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("raw_radial", raw_radial));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("raw_swept", raw_swept));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("simp_radial", simp_radial));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("simp_swept", simp_swept));
        colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("manip", manip));
        // colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("simp_extent", extent));
        // colorMap.insert(std::pair<std::string, std_msgs::ColorRGBA>("simp_agc", agc));

    }

    void GapVisualizer::drawGap(visualization_msgs::Marker & marker, 
                                const std::vector<Gap *> & gaps, 
                                const std::string & ns) // , const bool & initial)     
    {
        // ROS_INFO_STREAM("[drawGap] start");

        // visualization_msgs::Marker marker;
        // marker.header.stamp = ros::Time();
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
   
        for (Gap * gap : gaps) 
        {
            if (gap->getFrame().empty())
            {
                ROS_WARN_STREAM("[drawGap] Gap frame is empty");
                return;
            }

            std::string fullNamespace = ns;

            if (gap->isRadial()) 
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
    
            marker.header.frame_id = gap->getFrame();
            marker.header.stamp = gap->getTimeStamp();

            int leftIdx = gap->LIdx(); // initial ?  : gap->termLIdx(); // initial ? gap->RIdx() : gap->termRIdx(); //
            int rightIdx = gap->RIdx(); // initial ?  : gap->termRIdx(); // initial ? gap->LIdx() : gap->termLIdx(); //
            float leftRange = gap->LRange(); // initial ?  : gap->termLRange(); // initial ? gap->RRange() : gap->termRRange();
            float rightRange = gap->RRange(); // initial ?  : gap->termRRange(); // initial ? gap->LRange() : gap->termLRange();

            //ROS_INFO_STREAM("leftIdx: " << leftIdx << ", ldist: " << ldist << ", rightIdx: " << rightIdx << ", rightRange: " << rightRange);
            int gapIdxSpan = (leftIdx - rightIdx);
            if (gapIdxSpan < 0)
                gapIdxSpan += cfg_->scan.full_scan; // 2*gap->half_scan; // taking off int casting here

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
                
                midGapIdx = (midGapIdx + gapSpanResoln) % cfg_->scan.full_scan; // int(2*gap->half_scan);
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
    
    void GapVisualizer::drawGaps(const std::vector<Gap *> & gaps, const std::string & ns) 
    {
        // First, clearing topic.

        visualization_msgs::Marker marker;
        drawGap(marker, gaps, ns); // , true);

        // for (const Gap & gap : g) 
        // {
        //     drawGap(vis_arr, gap, ns);
        // }
        
        if (ns.find("raw") != std::string::npos) 
        {
            clearMarkerPublisher(rawGapsPublisher);
            rawGapsPublisher.publish(marker);
        } else if (ns.find("simp") != std::string::npos) 
        {
            clearMarkerPublisher(simpGapsPublisher);
            simpGapsPublisher.publish(marker);
        } else
        {
            ROS_WARN_STREAM_NAMED("GapVisualizer", "Unknown gap namespace: " << ns);
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
                                        const std::vector<Gap *> & gaps, 
                                        // const std::string & ns,
                                        const bool & circle,
                                        const bool & sides)
    {
        // marker.header.stamp = ros::Time();
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

        std::string ns = "manip";

        auto colorIter = colorMap.find(ns);
        if (colorIter == colorMap.end()) 
        {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }

        for (Gap * gap : gaps) 
        {
            if (gap->getFrame().empty())
            {
                ROS_WARN_STREAM("[drawManipGap] Gap frame is empty");
                return;
            }

            // if (gap->isReduced()) 
            // {
            //     ns = "simp_swept";
            // }
            
            // if (gap->isExtended()) 
            // {
            //     ns = "simp_extent";
            // }
    
            // if (gap->isAGC()) 
            // {
            //     ns = "simp_agc";
            // }


            marker.header.frame_id = gap->getFrame();
            marker.header.stamp = gap->getTimeStamp();

            int leftIdx = gap->manipLeftIdx();
            int rightIdx = gap->manipRightIdx();
            float leftRange = gap->manipLeftRange();
            float rightRange = gap->manipRightRange();

            // ROS_INFO_STREAM("leftIdx: " << leftIdx << ", leftRange: " << leftRange);
            // ROS_INFO_STREAM("rightIdx: " << rightIdx << ", rightRange: " << rightRange);

            int gapIdxSpan = (leftIdx - rightIdx);
            if (gapIdxSpan < 0)
                gapIdxSpan += cfg_->scan.full_scan; // 2*gap->half_scan; // taking off int casting here

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
                
                midGapIdx = (midGapIdx + gapSpanResoln) % cfg_->scan.full_scan; // int(2*gap->half_scan);
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


            if (gap->isExtended()) 
            {
                float r = gap->getMinSafeDist();
                if (r < 0) {
                    ROS_WARN_STREAM("Gap min safe dist not recorded");
                }
    
                // std_msgs::ColorRGBA convex_color = colorMap["simp_extent"];
    
                // this_marker.ns = "simp_extent";
    
                // The Circle
                if (circle) 
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
                        marker.colors.push_back(colorIter->second);
                        marker.colors.push_back(colorIter->second);
                        // this_marker.colors = convex_color;
                        // this_marker.id = id++;
                        // this_marker.lifetime = ros::Duration();
                        // vis_arr.markers.push_back(this_marker);
                    }
                    // circle = true;
                }
    
                if (sides)
                {
                    // this_marker.ns = "extent_line";
                    getline(gap->manipRightIdx(), 
                            gap->manipRightRange(), 
                            gap->getQB(), 
                            // lines, 
                            // linel, 
                            // liner, 
                            marker, 
                            colorIter->second);
                    
                    getline(gap->manipLeftIdx(),
                            gap->manipLeftRange(), 
                            gap->getQB(), 
                            // lines, 
                            // linel, 
                            // liner, 
                            marker, 
                            colorIter->second);
                    Eigen::Vector2f origin(0, 0);


                    // this_marker.ns = "orig_line";
                    getline(gap->RIdx(), 
                            gap->RRange(), 
                            origin, 
                            // lines, 
                            // linel, 
                            // liner, 
                            marker, 
                            colorIter->second);

                    getline(gap->LIdx(), 
                            gap->LRange(), 
                            origin, 
                            // lines, 
                            // linel, 
                            // liner, 
                            marker, 
                            colorIter->second);
                }
            }            
        }
    }    

    void GapVisualizer::drawManipGaps(const std::vector<Gap *> & gaps) 
    {
        // First, clearing topic.
        clearMarkerPublisher(manipGapsPublisher);

        bool circle = false;
        bool sides = false;

        visualization_msgs::Marker marker;
        drawManipGap(marker, gaps, circle, sides); // , true);

        manipGapsPublisher.publish(marker);
    }
}