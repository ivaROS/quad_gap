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

            ~Gap() {};

            void setRIdx(int right_idx)
            {
                _right_idx = right_idx;
            }

            void setLIdx(int left_idx)
            {
                _left_idx = left_idx;
            }

            // Setter and Getter for LR Distance and Index
            void setRDist(float right_dist) 
            {
                _right_dist = right_dist;
            }

            void setLDist(float left_dist)
            {
                _left_dist = left_dist;
            }

            int RIdx()
            {
                return _right_idx;
            }

            int LIdx()
            {
                return _left_idx;
            }

            float RDist()
            {
                return _right_dist;
            }

            float LDist()
            {
                return _left_dist;
            }

            // Concluding the Gap after constructing with left information
            void addRightInformation(int left_idx, float left_dist) 
            {
                _left_idx = left_idx;
                _left_dist = left_dist;
                right_type = _right_dist < _left_dist;

                if (!_radial)
                {
                    float resoln = M_PI / half_num_scan;
                    float angle1 = (_left_idx - _right_idx) * resoln;
                    float short_side = right_type ? _right_dist : _left_dist;
                    float opp_side = (float) sqrt(pow(_right_dist, 2) + pow(_left_dist, 2) - 2 * _right_dist * _left_dist * (float)cos(angle1));
                    float small_angle = (float) asin(short_side / opp_side * (float) sin(angle1));
                    if (M_PI - small_angle - angle1 > 0.75 * M_PI) 
                    {
                        _radial = true;
                    }
                }

                convex.convex_right_idx = _right_idx;
                convex.convex_left_idx = _left_idx;
                convex.convex_right_dist = _right_dist;
                convex.convex_left_dist = _left_dist;
            }

            // Get Left Cartesian Distance
            void getRCartesian(float &x, float &y)
            {
                x = (_right_dist) * cos(-((float) half_num_scan - _right_idx) / half_num_scan * M_PI);
                y = (_right_dist) * sin(-((float) half_num_scan - _right_idx) / half_num_scan * M_PI);
            }

            // Get Right Cartesian Distance
            void getLCartesian(float &x, float &y)
            {
                x = (_left_dist) * cos(-((float) half_num_scan - _left_idx) / half_num_scan * M_PI);
                y = (_left_dist) * sin(-((float) half_num_scan - _left_idx) / half_num_scan * M_PI);
            }

            void getRadialExRCartesian(float &x, float &y){
                x = (convex_right_dist) * cos(-((float) half_num_scan - convex_right_idx) / half_num_scan * M_PI);
                y = (convex_right_dist) * sin(-((float) half_num_scan - convex_right_idx) / half_num_scan * M_PI);
            }

            void getRadialExLCartesian(float &x, float &y){
                x = (convex_left_dist) * cos(-((float) half_num_scan - convex_left_idx) / half_num_scan * M_PI);
                y = (convex_left_dist) * sin(-((float) half_num_scan - convex_left_idx) / half_num_scan * M_PI);
            }

            void setAGCIdx(int right_idx, int left_idx) {
                agc_right_idx = right_idx;
                agc_left_idx = left_idx;
                agc_right_dist = float(right_idx - _right_idx) / float(_left_idx - _right_idx) * (_left_dist - _right_dist) + _right_dist;
                agc_left_dist = float(left_idx - _right_idx) / float(_left_idx - _right_idx) * (_left_dist - _right_dist) + _right_dist;
            }

            void getAGCRCartesian(float &x, float &y){
                x = (agc_right_dist) * cos(-((float) half_num_scan - agc_right_idx) / half_num_scan * M_PI);
                y = (agc_right_dist) * sin(-((float) half_num_scan - agc_right_idx) / half_num_scan * M_PI);
            }

            void getAGCLCartesian(float &x, float &y){
                x = (agc_left_dist) * cos(-((float) half_num_scan - agc_left_idx) / half_num_scan * M_PI);
                y = (agc_left_dist) * sin(-((float) half_num_scan - agc_left_idx) / half_num_scan * M_PI);
            }

            void compareGoalDist(double goal_dist) {
                goal_within = goal_dist < _right_dist && goal_dist < _left_dist;
            }

            // Getter and Setter for if side is an obstacle
            void setRightObs() {
                right_obs = false;
            }

            void setLeftObs() {
                left_obs = false;
            }

            bool getRightObs() {
                return right_obs;
            }

            bool getLeftObs() {
                return left_obs;
            }

            bool setRadial()
            {
                float resoln = M_PI / half_num_scan;
                float angle1 = (_left_idx - _right_idx) * resoln;
                float short_side = right_type ? _right_dist : _left_dist;
                float opp_side = (float) sqrt(pow(_right_dist, 2) + pow(_left_dist, 2) - 2 * _right_dist * _left_dist * (float)cos(angle1));
                float small_angle = (float) asin(short_side / opp_side * (float) sin(angle1));
                // _radial = (M_PI - small_angle - angle1 > 0.75 * M_PI); 
                _radial = (M_PI - small_angle - angle1 > (2.0 / 3.0 * M_PI)); 
                return _radial;
            }

            bool isRightType()
            {
                return right_type;
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
                return sqrt(pow(_right_dist, 2) + pow(_left_dist, 2) - 2 * _right_dist * _left_dist * (cos(float(_left_idx - _right_idx) / float(half_num_scan) * M_PI)));
            }

            Eigen::Vector2d get_middle_pt_vec()
            {
                float right_x, right_y;
                getRCartesian(right_x, right_y);
                Eigen::Vector2d right_vec(right_x, right_y);
                
                float left_x, left_y;
                getLCartesian(left_x, left_y);
                Eigen::Vector2d left_vec(left_x, left_y);
                Eigen::Vector2d m_vec = (right_vec + left_vec) / 2;
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
            bool wrap = false;
            bool reduced = false;
            bool convexified = false;
            int convex_right_idx;
            int convex_left_idx;
            float convex_right_dist;
            float convex_left_dist;
            float min_safe_dist = -1;
            Eigen::Vector2f qB;
            float half_num_scan = 256;

            int agc_right_idx;
            int agc_left_idx;
            float agc_right_dist;
            float agc_left_dist;
            bool no_agc_coor = false;

            std::string _frame = "";
            bool right_obs = true;
            bool left_obs = true;
            bool _radial = false;
            bool right_type = false;

            struct converted {
                int convex_right_idx = 0;
                int convex_left_idx = 511;
                float convex_right_dist = 3;
                float convex_left_dist = 3;
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