#pragma once

namespace quad_gap
{
    enum RobotShape { circle, box };

    struct Robot
    {
        RobotShape shape;
        float radius = 0, length = 0, width = 0, diagonal_length = 0;
        float avg_lin_speed, avg_rot_speed;
        Robot(){};


        Robot(const RobotShape & in_shape, 
                const float & robot_length, 
                const float & robot_width=0, 
                const float & robot_avg_lin_speed=0.2, 
                const float & robot_avg_rot_speed=0.5)
        {
            shape = in_shape;
            switch (shape)
            {
                case RobotShape::circle:
                    if (robot_width != 0)
                        throw std::runtime_error("Circular robot doesn't have width.");

                    radius = robot_length / 2;
                    avg_lin_speed = robot_avg_lin_speed;
                    avg_rot_speed = robot_avg_rot_speed;
                    break;
                
                case RobotShape::box:
                    if (robot_length == 0 || robot_width == 0)
                        throw std::runtime_error("Box robot need length or width.");
                    
                    length = robot_length;
                    width = robot_width;
                    diagonal_length = sqrt(length * length + width * width);
                    avg_lin_speed = robot_avg_lin_speed;
                    avg_rot_speed = robot_avg_rot_speed;
                    break;
                
                default:
                    if (robot_width != 0)
                        throw std::runtime_error("Circular robot doesn't have width.");

                    radius = robot_length / 2;
                    avg_lin_speed = robot_avg_lin_speed;
                    avg_rot_speed = robot_avg_rot_speed;
                    break;
            }
        }
    };
}