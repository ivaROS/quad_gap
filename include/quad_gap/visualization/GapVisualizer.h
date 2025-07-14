#pragma once

#include <quad_gap/visualization/Visualizer.h>

namespace quad_gap
{
    class GapVisualizer : public Visualizer
    {
            using Visualizer::Visualizer;
        public: 

            GapVisualizer(const rclcpp::Node::SharedPtr & node, const QuadGapConfig& cfg);
            void initialize(const rclcpp::Node::SharedPtr & node, const QuadGapConfig& cfg);

            void drawGaps(const std::vector<Gap *> & gaps, const std::string & ns);
            void drawManipGaps(const std::vector<Gap *> & gaps);

        private:
            void drawGap(visualization_msgs::msg::Marker & marker, 
                            const std::vector<Gap *> & gaps, 
                            const std::string & ns);
                            
            void drawManipGap(visualization_msgs::msg::Marker & marker, 
                                const std::vector<Gap *> & gaps, 
                                const bool & circle,
                                const bool & sides);

            void getline(const int & idx, 
                            const float & dist,
                            const Eigen::Vector2f & qB,
                            // std::vector<geometry_msgs::Point>& lines,
                            // geometry_msgs::Point& linel,
                            // geometry_msgs::Point& liner,
                            visualization_msgs::msg::Marker& marker,
                            const std_msgs::msg::ColorRGBA & convex_color);                     

            std::map<std::string, std_msgs::msg::ColorRGBA> colorMap;

            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rawGapsPublisher;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr simpGapsPublisher;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr manipGapsPublisher;

            int gapSpanResoln = 2;
            float invGapSpanResoln = 0.5;            
    };
}