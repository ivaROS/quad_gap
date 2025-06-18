#pragma once

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <quad_gap/utils/Utils.h>

namespace quad_gap
{
    class Gap
    {
        public:
            Gap() {};

            Gap(const std::string & frame, 
                const int & right_idx, 
                const float & right_dist, 
                const bool & radial = false) : _frame(frame), _right_idx(right_idx), _right_dist(right_dist), _radial(radial)
            {};

            Gap(const Gap & otherGap)
            {
                _frame = otherGap._frame;
                _left_idx = otherGap._left_idx;
                _right_idx = otherGap._right_idx;
                _left_dist = otherGap._left_dist;
                _right_dist = otherGap._right_dist;
                right_type = otherGap.right_type;
                convex = otherGap.convex;
                agc_right_idx = otherGap.agc_right_idx;
                agc_left_idx = otherGap.agc_left_idx;
                agc_right_dist = otherGap.agc_right_dist;
                agc_left_dist = otherGap.agc_left_dist;
                goal_within = otherGap.goal_within;
                right_obs = otherGap.right_obs;
                left_obs = otherGap.left_obs;
                _radial = otherGap._radial;
                convex_left_idx = otherGap.convex_left_idx;
                convex_right_idx = otherGap.convex_right_idx;
                convex_left_dist = otherGap.convex_left_dist;
                convex_right_dist = otherGap.convex_right_dist;
                goal_within = otherGap.goal_within;
                goal_dir_within = otherGap.goal_dir_within;
                life_time = otherGap.life_time;
                agc = otherGap.agc;
                reduced = otherGap.reduced;
                convexified = otherGap.convexified;
                min_safe_dist = otherGap.min_safe_dist;
                qB = otherGap.qB;
                goal = otherGap.goal;
                mode = otherGap.mode;
            }

            ~Gap() {};

            void setLIdx(const int & left_idx)
            {
                _left_idx = left_idx;
            }

            void setRIdx(const int & right_idx)
            {
                _right_idx = right_idx;
            }

            // Setter and Getter for LR Distance and Index
            void setLRange(const float & left_dist)
            {
                _left_dist = left_dist;
            }

            void setRRange(const float & right_dist) 
            {
                _right_dist = right_dist;
            }

            int LIdx() const
            {
                return _left_idx;
            }

            int RIdx() const
            {
                return _right_idx;
            }            

            float LRange() const
            {
                return _left_dist;
            }

            float RRange() const
            {
                return _right_dist;
            }

            // Concluding the Gap after constructing with left information
            void addLeftInformation(const int & left_idx, const float & left_dist) 
            {
                _left_idx = left_idx;
                _left_dist = left_dist;
                right_type = _right_dist < _left_dist;

                setRadial();

                convex.convex_right_idx = _right_idx;
                convex.convex_left_idx = _left_idx;
                convex.convex_right_dist = _right_dist;
                convex.convex_left_dist = _left_dist;
            }

            // Get Right Cartesian Distance
            void getLCartesian(float &x, float &y) const
            {
                float left_theta = idx2theta(_left_idx);
                x = _left_dist * cos(left_theta);
                y = _left_dist * sin(left_theta);
            }

            Eigen::Vector2d getLCartesian() const
            {
                float left_x, left_y;
                getLCartesian(left_x, left_y);
                return Eigen::Vector2d(left_x, left_y);
            }

            // Get Left Cartesian Distance
            void getRCartesian(float &x, float &y) const
            {
                float right_theta = idx2theta(_right_idx);
                x = _right_dist * cos(right_theta);
                y = _right_dist * sin(right_theta);
            }

            Eigen::Vector2d getRCartesian() const
            {
                float right_x, right_y;
                getRCartesian(right_x, right_y);
                return Eigen::Vector2d(right_x, right_y);
            }

            void getRadialExLCartesian(float &x, float &y)
            {
                float left_theta = idx2theta(convex_left_idx);
                x = (convex_left_dist) * cos(left_theta);
                y = (convex_left_dist) * sin(left_theta);
            }

            void getRadialExRCartesian(float &x, float &y)
            {
                float right_theta = idx2theta(convex_right_idx);
                x = (convex_right_dist) * cos(right_theta);
                y = (convex_right_dist) * sin(right_theta);
            }

            void setAGCIdx(const int & right_idx, const int & left_idx) 
            {
                agc_right_idx = right_idx;
                agc_left_idx = left_idx;
                agc_right_dist = float(right_idx - _right_idx) / float(_left_idx - _right_idx) * (_left_dist - _right_dist) + _right_dist;
                agc_left_dist = float(left_idx - _right_idx) / float(_left_idx - _right_idx) * (_left_dist - _right_dist) + _right_dist;
            }

            void getAGCLCartesian(float &x, float &y)
            {
                float left_theta = idx2theta(agc_left_idx);
                x = (agc_left_dist) * cos(left_theta);
                y = (agc_left_dist) * sin(left_theta);
            }

            void getAGCRCartesian(float &x, float &y)
            {
                float right_theta = idx2theta(agc_right_idx);
                x = (agc_right_dist) * cos(right_theta);
                y = (agc_right_dist) * sin(right_theta);
            }

            void compareGoalDist(const double & goal_dist) 
            {
                goal_within = goal_dist < _right_dist && goal_dist < _left_dist;
            }

            int manipLeftIdx() const
            {
                return convex.convex_left_idx;
            }

            int manipRightIdx() const
            {
                return convex.convex_right_idx;
            }

            float manipLeftRange() const
            {
                return convex.convex_left_dist;
            }

            float manipRightRange() const
            {
                return convex.convex_right_dist;
            }

            // Getter and Setter for if side is an obstacle
            void setRightObs() 
            {
                right_obs = false;
            }

            void setLeftObs() 
            {
                left_obs = false;
            }

            bool getRightObs() const
            {
                return right_obs;
            }

            bool getLeftObs() const
            {
                return left_obs;
            }

            void setRadial()
            {
                float resoln = M_PI / half_num_scan;
                float angle1 = (_left_idx - _right_idx) * resoln;
                float short_side = right_type ? _right_dist : _left_dist;
                float opp_side = (float) sqrt(pow(_right_dist, 2) + pow(_left_dist, 2) - 2 * _right_dist * _left_dist * (float)cos(angle1));
                float small_angle = (float) asin(short_side / opp_side * (float) sin(angle1));
                // _radial = (M_PI - small_angle - angle1 > 0.75 * M_PI); 
                _radial = (M_PI - small_angle - angle1 > (2.0 / 3.0 * M_PI)); 
                // return _radial;
            }

            /**
            * \brief Getter for gap radial condition
            * \return Gap radial condition
            */
            bool isRadial() const 
            { 
                return _radial; 
            }

            bool isRightType() const
            {
                return right_type;
            }

            void resetFrame(const std::string & frame) 
            {
                _frame = frame;
            }

            void setMinSafeDist(const float & dist) 
            {
                min_safe_dist = dist;
            }

            float getMinSafeDist() const
            {
                return min_safe_dist;
            }

            std::string getFrame() const
            {
                return _frame;
            }

            float get_dist_side() const
            {
                return sqrt(pow(_right_dist, 2) + pow(_left_dist, 2) - 2 * _right_dist * _left_dist * (cos(float(_left_idx - _right_idx) / float(half_num_scan) * M_PI)));
            }

            Eigen::Vector2d get_middle_pt_vec() const
            {
                Eigen::Vector2d left_vec = getLCartesian();
                // float right_x, right_y;
                // getRCartesian(right_x, right_y);
                // Eigen::Vector2d right_vec(right_x, right_y);
                
                Eigen::Vector2d right_vec = getRCartesian();
                // float left_x, left_y;
                // getLCartesian(left_x, left_y);
                // Eigen::Vector2d left_vec(left_x, left_y);
                // Eigen::Vector2d m_vec = (right_vec + left_vec) / 2;
                Eigen::Vector2d m_vec = (right_vec + left_vec) / 2.0;
                return m_vec;
            }
            
            bool goal_within = false;
            bool goal_dir_within = false;
            float life_time = 1.0;
            bool agc = false;

            int _right_idx = 0;
            float _right_dist = 3;
            int _left_idx = 511;
            float _left_dist = 3;
            bool reduced = false;
            bool convexified = false;
            int convex_right_idx;
            int convex_left_idx;
            float convex_right_dist;
            float convex_left_dist;
            float min_safe_dist = -1;
            Eigen::Vector2f qB;
            // float half_num_scan = 256;

            int agc_right_idx;
            int agc_left_idx;
            float agc_right_dist;
            float agc_left_dist;

            std::string _frame = "";
            bool right_obs = true;
            bool left_obs = true;
            bool _radial = false;
            bool right_type = false;

            struct converted 
            {
                int convex_right_idx = 0;
                int convex_left_idx = 511;
                float convex_right_dist = 3;
                float convex_left_dist = 3;
            } convex;

            struct GapMode 
            {
                bool reduced = false;
                bool convex = false;
                bool agc = false;
            } mode;

            struct Goal 
            {
                float x, y;
                bool set = false;
                bool discard = false;
                bool goalwithin = false;
            } goal;
        // private:
    };
}