#pragma once

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

#include <math.h>
#include <quad_gap/utils/Gap.h>
#include <quad_gap/utils/Trajectory.h>
#include <quad_gap/config/QuadGapConfig.h>
#include <vector>
#include <map>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

namespace quad_gap
{
    class Visualizer 
    {
        public: 
            Visualizer() {};
            ~Visualizer() {};

            Visualizer(const rclcpp::Node::SharedPtr & node, const QuadGapConfig& cfg);
            
            // Visualizer& operator=(Visualizer other)
            // {
            //     cfg_ = other.cfg_;
                
            //     return *this;
            // };
            
            Visualizer(const Visualizer &t) {cfg_ = t.cfg_;};

        protected:

            void clearMarkerPublisher(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & publisher)
            {
                visualization_msgs::msg::Marker clearMarker;
                clearMarker.id = 0;
                clearMarker.ns =  "clear";
                clearMarker.action = visualization_msgs::msg::Marker::DELETEALL;
                publisher->publish(clearMarker);
            }

            void clearMarkerArrayPublisher(const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & publisher)
            {
                visualization_msgs::msg::MarkerArray clearMarkerArray;
                visualization_msgs::msg::Marker clearMarker;
                clearMarker.id = 0;
                clearMarker.ns =  "clear";
                clearMarker.action = visualization_msgs::msg::Marker::DELETEALL;
                clearMarkerArray.markers.push_back(clearMarker);
                publisher->publish(clearMarkerArray);
            }

            const QuadGapConfig* cfg_;
    };
}