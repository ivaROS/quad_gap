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

            Gap(std::string frame, int left_idx, float left_dist, bool axial = false, float half_scan = 256) : _frame(frame), _left_idx(left_idx), _left_dist(left_dist), _axial(axial), half_scan(half_scan)
            {};

            ~Gap() {};

            void setLIdx(int left_idx)
            {
                _left_idx = left_idx;
            }

            void setRIdx(int right_idx)
            {
                _right_idx = right_idx;
            }

            // Setter and Getter for LR Distance and Index
            void setLDist(float left_dist) 
            {
                _left_dist = left_dist;
            }

            void setRDist(float right_dist)
            {
                _right_dist = right_dist;
            }

            int LIdx()
            {
                return _left_idx;
            }

            int RIdx()
            {
                return _right_idx;
            }

            float LDist()
            {
                return _left_dist;
            }

            float RDist()
            {
                return _right_dist;
            }

            void getLRIdx(int &l, int &r)
            {
                l = _left_idx;
                r = _right_idx;
            }

            // Concluding the Gap after constructing with left information
            void addRightInformation(int right_idx, float right_dist) 
            {
                _right_idx = right_idx;
                _right_dist = right_dist;
                left_type = _left_dist < _right_dist;

                if (!_axial)
                {
                    float resoln = M_PI / half_scan;
                    float angle1 = (_right_idx - _left_idx) * resoln;
                    float short_side = left_type ? _left_dist : _right_dist;
                    float opp_side = (float) sqrt(pow(_left_dist, 2) + pow(_right_dist, 2) - 2 * _left_dist * _right_dist * (float)cos(angle1));
                    float small_angle = (float) asin(short_side / opp_side * (float) sin(angle1));
                    if (M_PI - small_angle - angle1 > 0.75 * M_PI) 
                    {
                        _axial = true;
                    }
                }

                convex.convex_left_idx = _left_idx;
                convex.convex_right_idx = _right_idx;
                convex.convex_left_dist = _left_dist;
                convex.convex_right_dist = _right_dist;
            }

            // Get Left Cartesian Distance
            void getLCartesian(float &x, float &y)
            {
                x = (_left_dist) * cos(-((float) half_scan - _left_idx) / half_scan * M_PI);
                y = (_left_dist) * sin(-((float) half_scan - _left_idx) / half_scan * M_PI);
            }

            // Get Right Cartesian Distance
            void getRCartesian(float &x, float &y)
            {
                x = (_right_dist) * cos(-((float) half_scan - _right_idx) / half_scan * M_PI);
                y = (_right_dist) * sin(-((float) half_scan - _right_idx) / half_scan * M_PI);
            }

            void getRadialExLCartesian(float &x, float &y){
                x = (convex_left_dist) * cos(-((float) half_scan - convex_left_idx) / half_scan * M_PI);
                y = (convex_left_dist) * sin(-((float) half_scan - convex_left_idx) / half_scan * M_PI);
            }

            void getRadialExRCartesian(float &x, float &y){
                x = (convex_right_dist) * cos(-((float) half_scan - convex_right_idx) / half_scan * M_PI);
                y = (convex_right_dist) * sin(-((float) half_scan - convex_right_idx) / half_scan * M_PI);
            }

            void setAGCIdx(int left_idx, int right_idx) {
                agc_left_idx = left_idx;
                agc_right_idx = right_idx;
                agc_left_dist = float(left_idx - _left_idx) / float(_right_idx - _left_idx) * (_right_dist - _left_dist) + _left_dist;
                agc_right_dist = float(right_idx - _left_idx) / float(_right_idx - _left_idx) * (_right_dist - _left_dist) + _left_dist;
            }

            void getAGCLCartesian(float &x, float &y){
                x = (agc_left_dist) * cos(-((float) half_scan - agc_left_idx) / half_scan * M_PI);
                y = (agc_left_dist) * sin(-((float) half_scan - agc_left_idx) / half_scan * M_PI);
            }

            void getAGCRCartesian(float &x, float &y){
                x = (agc_right_dist) * cos(-((float) half_scan - agc_right_idx) / half_scan * M_PI);
                y = (agc_right_dist) * sin(-((float) half_scan - agc_right_idx) / half_scan * M_PI);
            }

            // Decimate Gap 
            void segmentGap2Vec(std::vector<Gap>& gap, int min_resoln)
            {
                int num_gaps = (_right_idx - _left_idx) / min_resoln + 1;
                int idx_step = (_right_idx - _left_idx) / num_gaps;
                float dist_step = (_right_dist - _left_dist) / num_gaps;
                int sub_gap_left_idx = _left_idx;
                float sub_gap_left_dist = _left_dist;
                int sub_gap_right_idx = _left_idx;

                if (num_gaps < 3) {
                    gap.push_back(*this);
                    return;
                }
                
                for (int i = 0; i < num_gaps; i++) {
                    Gap detected_gap(_frame, sub_gap_left_idx, sub_gap_left_dist);
                    // ROS_DEBUG_STREAM("left_idx: " << sub_gap_left_idx << "left_dist: " << sub_gap_left_dist);
                    if (i != 0) {
                        detected_gap.setLeftObs();
                    }

                    if (i != num_gaps - 1) {
                        detected_gap.setRightObs();
                    }

                    sub_gap_left_idx += idx_step;
                    sub_gap_left_dist += dist_step;
                    // ROS_DEBUG_STREAM("right_idx: " << sub_gap_left_idx << "right_dist: " << sub_gap_left_dist);
                    if (i == num_gaps - 1)
                    {
                        detected_gap.addRightInformation(_right_idx, _right_dist);
                    } else {
                        detected_gap.addRightInformation(sub_gap_left_idx - 1, sub_gap_left_dist);
                    }
                    gap.push_back(detected_gap);
                }
            }

            void compareGoalDist(double goal_dist) {
                goal_within = goal_dist < _left_dist && goal_dist < _right_dist;
            }

            // Getter and Setter for if side is an obstacle
            void setLeftObs() {
                left_obs = false;
            }

            void setRightObs() {
                right_obs = false;
            }

            bool getLeftObs() {
                return left_obs;
            }


            bool getRightObs() {
                return right_obs;
            }

            bool isAxial()
            {
                float resoln = M_PI / half_scan;
                float angle1 = (_right_idx - _left_idx) * resoln;
                float short_side = left_type ? _left_dist : _right_dist;
                float opp_side = (float) sqrt(pow(_left_dist, 2) + pow(_right_dist, 2) - 2 * _left_dist * _right_dist * (float)cos(angle1));
                float small_angle = (float) asin(short_side / opp_side * (float) sin(angle1));
                // _axial = (M_PI - small_angle - angle1 > 0.75 * M_PI); 
                _axial = (M_PI - small_angle - angle1 > (2.0 / 3.0 * M_PI)); 
                return _axial;
            }

            void setRadial()
            {
                _axial = false;
            }

            bool isLeftType()
            {
                return left_type;
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
                return sqrt(pow(_left_dist, 2) + pow(_right_dist, 2) - 2 * _left_dist * _right_dist * (cos(float(_right_idx - _left_idx) / float(half_scan) * M_PI)));
            }

            Eigen::Vector2d get_middle_pt_vec()
            {
                float lx, ly;
                getLCartesian(lx, ly);
                Eigen::Vector2d l_vec(lx, ly);
                float rx, ry;
                getRCartesian(rx, ry);
                Eigen::Vector2d r_vec(rx, ry);
                Eigen::Vector2d m_vec = (l_vec + r_vec) / 2;
                return m_vec;
            }
            
            bool goal_within = false;
            bool goal_dir_within = false;
            float life_time = 1.0;
            bool agc = false;

            int _left_idx = 0;
            float _left_dist = 3;
            int _right_idx = 511;
            float _right_dist = 3;
            bool wrap = false;
            bool reduced = false;
            bool convexified = false;
            int convex_left_idx;
            int convex_right_idx;
            float convex_left_dist;
            float convex_right_dist;
            float min_safe_dist = -1;
            Eigen::Vector2f qB;
            float half_scan = 256;

            int agc_left_idx;
            int agc_right_idx;
            float agc_left_dist;
            float agc_right_dist;
            bool no_agc_coor = false;

            std::string _frame = "";
            bool left_obs = true;
            bool right_obs = true;
            bool _axial = false;
            bool left_type = false;

            struct converted {
                int convex_left_idx = 0;
                int convex_right_idx = 511;
                float convex_left_dist = 3;
                float convex_right_dist = 3;
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