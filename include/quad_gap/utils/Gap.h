#pragma once

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace quad_gap
{
    class Gap
    {
        public:
            Gap() {};

            Gap(std::string frame, int right_flipped_idx, float right_flipped_dist, bool axial = false, float half_scan = 256) : _frame(frame), _right_flipped_idx(right_flipped_idx), _right_flipped_dist(right_flipped_dist), _axial(axial), half_scan(half_scan)
            {};

            ~Gap() {};

            void setRFlippedIdx(int right_flipped_idx)
            {
                _right_flipped_idx = right_flipped_idx;
            }

            void setLFlippedIdx(int left_flipped_idx)
            {
                _left_flipped_idx = left_flipped_idx;
            }

            // Setter and Getter for LR Distance and Index
            void setRFlippedDist(float right_flipped_dist) 
            {
                _right_flipped_dist = right_flipped_dist;
            }

            void setLFlippedDist(float left_flipped_dist)
            {
                _left_flipped_dist = left_flipped_dist;
            }

            int RFlippedIdx()
            {
                return _right_flipped_idx;
            }

            int LFlippedIdx()
            {
                return _left_flipped_idx;
            }

            float RFlippedDist()
            {
                return _right_flipped_dist;
            }

            float LFlippedDist()
            {
                return _left_flipped_dist;
            }

            // Concluding the Gap after constructing with left information
            void addLeftInformation(int left_flipped_idx, float left_flipped_dist) 
            {
                _left_flipped_idx = left_flipped_idx;
                _left_flipped_dist = left_flipped_dist;
                right_flipped_type = _right_flipped_dist < _left_flipped_dist;

                if (!_axial)
                {
                    float resoln = M_PI / half_scan;
                    float angle1 = (_left_flipped_idx - _right_flipped_idx) * resoln;
                    float short_side = right_flipped_type ? _right_flipped_dist : _left_flipped_dist;
                    float opp_side = (float) sqrt(pow(_right_flipped_dist, 2) + pow(_left_flipped_dist, 2) - 2 * _right_flipped_dist * _left_flipped_dist * (float)cos(angle1));
                    float small_angle = (float) asin(short_side / opp_side * (float) sin(angle1));
                    if (M_PI - small_angle - angle1 > 0.75 * M_PI) 
                    {
                        _axial = true;
                    }
                }

                convex.convex_right_flipped_idx = _right_flipped_idx;
                convex.convex_left_flipped_idx = _left_flipped_idx;
                convex.convex_right_flipped_dist = _right_flipped_dist;
                convex.convex_left_flipped_dist = _left_flipped_dist;
            }

            // Get Left Cartesian Distance
            void getRFlippedCartesian(float &x, float &y)
            {
                x = (_right_flipped_dist) * cos(-((float) half_scan - _right_flipped_idx) / half_scan * M_PI);
                y = (_right_flipped_dist) * sin(-((float) half_scan - _right_flipped_idx) / half_scan * M_PI);
            }

            // Get Right Cartesian Distance
            void getLFlippedCartesian(float &x, float &y)
            {
                x = (_left_flipped_dist) * cos(-((float) half_scan - _left_flipped_idx) / half_scan * M_PI);
                y = (_left_flipped_dist) * sin(-((float) half_scan - _left_flipped_idx) / half_scan * M_PI);
            }

            void getRadialExRFlippedCartesian(float &x, float &y){
                x = (convex_right_flipped_dist) * cos(-((float) half_scan - convex_right_flipped_idx) / half_scan * M_PI);
                y = (convex_right_flipped_dist) * sin(-((float) half_scan - convex_right_flipped_idx) / half_scan * M_PI);
            }

            void getRadialExLFlippedCartesian(float &x, float &y){
                x = (convex_left_flipped_dist) * cos(-((float) half_scan - convex_left_flipped_idx) / half_scan * M_PI);
                y = (convex_left_flipped_dist) * sin(-((float) half_scan - convex_left_flipped_idx) / half_scan * M_PI);
            }

            void setAGCIdx(int right_flipped_idx, int left_flipped_idx) {
                agc_right_flipped_idx = right_flipped_idx;
                agc_left_flipped_idx = left_flipped_idx;
                agc_right_flipped_dist = float(right_flipped_idx - _right_flipped_idx) / float(_left_flipped_idx - _right_flipped_idx) * (_left_flipped_dist - _right_flipped_dist) + _right_flipped_dist;
                agc_left_flipped_dist = float(left_flipped_idx - _right_flipped_idx) / float(_left_flipped_idx - _right_flipped_idx) * (_left_flipped_dist - _right_flipped_dist) + _right_flipped_dist;
            }

            void getAGCRFlippedCartesian(float &x, float &y){
                x = (agc_right_flipped_dist) * cos(-((float) half_scan - agc_right_flipped_idx) / half_scan * M_PI);
                y = (agc_right_flipped_dist) * sin(-((float) half_scan - agc_right_flipped_idx) / half_scan * M_PI);
            }

            void getAGCLFlippedCartesian(float &x, float &y){
                x = (agc_left_flipped_dist) * cos(-((float) half_scan - agc_left_flipped_idx) / half_scan * M_PI);
                y = (agc_left_flipped_dist) * sin(-((float) half_scan - agc_left_flipped_idx) / half_scan * M_PI);
            }

            void compareGoalDist(double goal_dist) {
                goal_within = goal_dist < _right_flipped_dist && goal_dist < _left_flipped_dist;
            }

            // Getter and Setter for if side is an obstacle
            void setLeftObs() {
                right_flipped_obs = false;
            }

            void setRightObs() {
                left_flipped_obs = false;
            }

            bool getRightFlippedObs() {
                return right_flipped_obs;
            }


            bool getLeftFlippedObs() {
                return left_flipped_obs;
            }

            bool setRadial()
            {
                float resoln = M_PI / half_scan;
                float angle1 = (_left_flipped_idx - _right_flipped_idx) * resoln;
                float short_side = right_flipped_type ? _right_flipped_dist : _left_flipped_dist;
                float opp_side = (float) sqrt(pow(_right_flipped_dist, 2) + pow(_left_flipped_dist, 2) - 2 * _right_flipped_dist * _left_flipped_dist * (float)cos(angle1));
                float small_angle = (float) asin(short_side / opp_side * (float) sin(angle1));
                // _axial = (M_PI - small_angle - angle1 > 0.75 * M_PI); 
                _axial = (M_PI - small_angle - angle1 > (2.0 / 3.0 * M_PI)); 
                return _axial;
            }

            bool isRightFlippedType()
            {
                return right_flipped_type;
            }

            void resetFrame(std::string frame) {
                _frame = frame;
            }

            void setMinSafeDist(float _dist) {
                min_safe_dist = _dist;
            }

            float getMinSafeDist() {
                return min_safe_dist;
            }

            std::string getFrame() {
                return _frame;
            }

            float get_dist_side() {
                return sqrt(pow(_right_flipped_dist, 2) + pow(_left_flipped_dist, 2) - 2 * _right_flipped_dist * _left_flipped_dist * (cos(float(_left_flipped_idx - _right_flipped_idx) / float(half_scan) * M_PI)));
            }

            Eigen::Vector2d get_middle_pt_vec()
            {
                float right_flipped_x, right_flipped_y;
                getRFlippedCartesian(right_flipped_x, right_flipped_y);
                Eigen::Vector2d right_flipped_vec(right_flipped_x, right_flipped_y);
                
                float left_flipped_x, left_flipped_y;
                getLFlippedCartesian(left_flipped_x, left_flipped_y);
                Eigen::Vector2d left_flipped_vec(left_flipped_x, left_flipped_y);
                Eigen::Vector2d m_vec = (right_flipped_vec + left_flipped_vec) / 2;
                return m_vec;
            }
            
            bool goal_within = false;
            bool goal_dir_within = false;
            float life_time = 1.0;
            bool agc = false;

            int _right_flipped_idx = 0;
            float _right_flipped_dist = 3;
            int _left_flipped_idx = 511;
            float _left_flipped_dist = 3;
            bool wrap = false;
            bool reduced = false;
            bool convexified = false;
            int convex_right_flipped_idx;
            int convex_left_flipped_idx;
            float convex_right_flipped_dist;
            float convex_left_flipped_dist;
            float min_safe_dist = -1;
            Eigen::Vector2f qB;
            float half_scan = 256;

            int agc_right_flipped_idx;
            int agc_left_flipped_idx;
            float agc_right_flipped_dist;
            float agc_left_flipped_dist;
            bool no_agc_coor = false;

            std::string _frame = "";
            bool right_flipped_obs = true;
            bool left_flipped_obs = true;
            bool _axial = false;
            bool right_flipped_type = false;

            struct converted {
                int convex_right_flipped_idx = 0;
                int convex_left_flipped_idx = 511;
                float convex_right_flipped_dist = 3;
                float convex_left_flipped_dist = 3;
            } convex;

            struct GapMode {
                bool reduced = false;
                bool convex = false;
                bool agc = false;
            } mode;

            struct Goal {
                float x, y;
                bool set = false;
                bool discard = false;
                bool goalwithin = false;
            } goal;
        // private:
    };
}