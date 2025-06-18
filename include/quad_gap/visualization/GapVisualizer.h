#pragma once

#include <quad_gap/visualization/Visualizer.h>

namespace quad_gap
{
    class GapVisualizer : public Visualizer
    {
            using Visualizer::Visualizer;
        public: 

            GapVisualizer(ros::NodeHandle& nh, const QuadGapConfig& cfg);
            void initialize(ros::NodeHandle& nh, const QuadGapConfig& cfg);
            void drawGaps(const std::vector<Gap> & g, const std::string & ns);
            void drawManipGaps(const std::vector<Gap> & vec);

        private:
            void drawGap(visualization_msgs::Marker & marker, const std::vector<Gap> & gaps, 
                                const std::string & ns);
            void drawManipGap(visualization_msgs::MarkerArray & vis_arr, const Gap & g, bool & circle);

            std::map<std::string, std_msgs::ColorRGBA> colorMap;
            ros::Publisher rawGapsPublisher;
            ros::Publisher simpGapsPublisher;

            ros::Publisher gapside_publisher;
            ros::Publisher gaqgoal_publisher;

            int gapSpanResoln = 2;
            float invGapSpanResoln = 0.5;            
    };
}