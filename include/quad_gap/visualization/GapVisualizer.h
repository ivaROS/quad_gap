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
            void drawGaps(const std::vector<Gap> & g, const std::string & ns, const std::string & color = "Default");
            void drawManipGaps(const std::vector<Gap> & vec);

        private:
            void drawGap(visualization_msgs::MarkerArray & vis_arr, const Gap & g, const std::string & ns, const std::string & color = "Default");
            void drawManipGap(visualization_msgs::MarkerArray & vis_arr, const Gap & g, bool & circle);

            std::map<std::string, std::vector<std_msgs::ColorRGBA>> colormap;
            ros::Publisher gaparc_publisher;
            ros::Publisher gapside_publisher;
            ros::Publisher gaqgoal_publisher;
    };
}