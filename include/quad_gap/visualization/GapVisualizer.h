#pragma once

#include <quad_gap/visualization/Visualizer.h>

namespace quad_gap
{
    class GapVisualizer : public Visualizer
    {
            using Visualizer::Visualizer;
        public: 

            GapVisualizer(ros::NodeHandle& nh, const quad_gap::QuadGapConfig& cfg);
            void initialize(ros::NodeHandle& nh, const quad_gap::QuadGapConfig& cfg);
            void drawGap(visualization_msgs::MarkerArray &, quad_gap::Gap g, std::string ns, std::string color = "Default");
            void drawGaps(std::vector<quad_gap::Gap> g, std::string ns, std::string color = "Default");
            void drawManipGap(visualization_msgs::MarkerArray &, quad_gap::Gap g, bool &);
            void drawManipGaps(std::vector<quad_gap::Gap> vec);

        private:
            std::map<std::string, std::vector<std_msgs::ColorRGBA>> colormap;
            ros::Publisher gaparc_publisher;
            ros::Publisher gapside_publisher;
            ros::Publisher gaqgoal_publisher;
    };
}