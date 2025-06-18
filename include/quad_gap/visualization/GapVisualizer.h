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

            void drawGaps(const std::vector<Gap> & gaps, const std::string & ns);
            void drawManipGaps(const std::vector<Gap> & gaps);

        private:
            void drawGap(visualization_msgs::Marker & marker, 
                            const std::vector<Gap> & gaps, 
                            const std::string & ns);
            void drawManipGap(visualization_msgs::Marker & marker, 
                                const std::vector<Gap> & gaps, 
                                // const std::string & ns,
                                bool & circle);

            void getline(const int & idx, 
                            const float & dist,
                            const Eigen::Vector2f & qB,
                            // std::vector<geometry_msgs::Point>& lines,
                            // geometry_msgs::Point& linel,
                            // geometry_msgs::Point& liner,
                            visualization_msgs::Marker& marker,
                            const std_msgs::ColorRGBA & convex_color);                     

            std::map<std::string, std_msgs::ColorRGBA> colorMap;

            ros::Publisher rawGapsPublisher;
            ros::Publisher simpGapsPublisher;
            ros::Publisher manipGapsPublisher;

            int gapSpanResoln = 2;
            float invGapSpanResoln = 0.5;            
    };
}