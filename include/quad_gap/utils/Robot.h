#pragma once

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace quad_gap
{
    enum RobotShape { circle = 0,  
                        box = 1};

    struct Robot
    {
        RobotShape shape;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr shape_pub;
        float radius = 0;
        float length = 0;
        float half_length = 0;
        float width = 0;
        float half_width = 0;
        float diagonal_length = 0;
        float avg_lin_speed = 0;
        float avg_rot_speed = 0;
        Robot(){};


        Robot(const rclcpp::Node::SharedPtr & node,
                const RobotShape & in_shape, 
                const float & robot_length, 
                const float & robot_width=0, 
                const float & robot_avg_lin_speed=0.2, 
                const float & robot_avg_rot_speed=0.5)
        {
            // ros::NodeHandle nh;
            shape_pub = node->create_publisher<visualization_msgs::msg::Marker>("robot_shape", 1);
            shape = in_shape;
            switch (shape)
            {
                case RobotShape::circle:
                    if (robot_width != 0)
                        throw std::runtime_error("Circular robot doesn't have width.");

                    radius = 0.5 * robot_length;
                    avg_lin_speed = robot_avg_lin_speed;
                    avg_rot_speed = robot_avg_rot_speed;
                    break;
                
                case RobotShape::box:
                    if (robot_length == 0 || robot_width == 0)
                        throw std::runtime_error("Box robot need length or width.");
                    
                    length = robot_length;
                    half_length = 0.5 * robot_length;
                    width = robot_width;
                    half_width = 0.5 * robot_width;
                    diagonal_length = sqrt(length * length + width * width);
                    avg_lin_speed = robot_avg_lin_speed;
                    avg_rot_speed = robot_avg_rot_speed;
                    break;
                
                default:
                    if (robot_width != 0)
                        throw std::runtime_error("Circular robot doesn't have width.");

                    radius = 0.5 * robot_length;
                    avg_lin_speed = robot_avg_lin_speed;
                    avg_rot_speed = robot_avg_rot_speed;
                    break;
            }
        }

        void drawRobotShape(const std::string & robot_frame, const rclcpp::Time & time)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = robot_frame;
            marker.header.stamp = time;
            marker.ns = "robot_shape";
            marker.id = 0;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.pose.position.x = 0.0;
            marker.pose.position.y = 0.0;
            marker.pose.position.z = 0.0;

            if (shape == RobotShape::circle)
            {
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.scale.x = radius * 2;
                marker.scale.y = radius * 2;
            }
            else if (shape == RobotShape::box)
            {
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.scale.x = length;
                marker.scale.y = width;
            }
                marker.scale.z = 0.001;

            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            shape_pub->publish(marker);
        }
    };
}