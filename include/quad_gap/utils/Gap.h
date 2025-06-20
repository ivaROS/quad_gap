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
                const int & rightIdx, 
                const float & rightRange, 
                const bool & radial = false) : _frame(frame), rightIdx_(rightIdx), rightRange_(rightRange), _radial(radial)
            {};

            Gap(const Gap & otherGap)
            {
                _frame = otherGap._frame;
                leftIdx_ = otherGap.leftIdx_;
                rightIdx_ = otherGap.rightIdx_;
                leftRange_ = otherGap.leftRange_;
                rightRange_ = otherGap.rightRange_;
                right_type = otherGap.right_type;
                convex = otherGap.convex;
                agcrightIdx_ = otherGap.agcrightIdx_;
                agcleftIdx_ = otherGap.agcleftIdx_;
                agcrightRange_ = otherGap.agcrightRange_;
                agcleftRange_ = otherGap.agcleftRange_;
                goal_within = otherGap.goal_within;
                right_obs = otherGap.right_obs;
                left_obs = otherGap.left_obs;
                _radial = otherGap._radial;
                convexLeftIdx_ = otherGap.convexLeftIdx_;
                convexRightIdx_ = otherGap.convexRightIdx_;
                convexLeftDist_ = otherGap.convexLeftDist_;
                convexRightDist_ = otherGap.convexRightDist_;
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

            void setLIdx(const int & leftIdx)
            {
                leftIdx_ = leftIdx;
            }

            void setRIdx(const int & rightIdx)
            {
                rightIdx_ = rightIdx;
            }

            // Setter and Getter for LR Distance and Index
            void setLRange(const float & leftRange)
            {
                leftRange_ = leftRange;
            }

            void setRRange(const float & rightRange) 
            {
                rightRange_ = rightRange;
            }

            int LIdx() const
            {
                return leftIdx_;
            }

            int RIdx() const
            {
                return rightIdx_;
            }            

            float LRange() const
            {
                return leftRange_;
            }

            float RRange() const
            {
                return rightRange_;
            }

            // Concluding the Gap after constructing with left information
            void addLeftInformation(const int & leftIdx, const float & leftRange) 
            {
                leftIdx_ = leftIdx;
                leftRange_ = leftRange;
                right_type = rightRange_ < leftRange_;

                setRadial();

                convex.convexLeftIdx_ = leftIdx_;
                convex.convexLeftDist_ = leftRange_;

                convex.convexRightIdx_ = rightIdx_;
                convex.convexRightDist_ = rightRange_;
            }

            // Get Right Cartesian Distance
            void getLCartesian(float &x, float &y) const
            {
                float leftTheta = idx2theta(leftIdx_);
                x = leftRange_ * cos(leftTheta);
                y = leftRange_ * sin(leftTheta);
            }

            Eigen::Vector2f getLCartesian() const
            {
                float left_x, left_y;
                getLCartesian(left_x, left_y);
                return Eigen::Vector2f(left_x, left_y);
            }

            // Get Left Cartesian Distance
            void getRCartesian(float &x, float &y) const
            {
                float right_theta = idx2theta(rightIdx_);
                x = rightRange_ * cos(right_theta);
                y = rightRange_ * sin(right_theta);
            }

            Eigen::Vector2f getRCartesian() const
            {
                float right_x, right_y;
                getRCartesian(right_x, right_y);
                return Eigen::Vector2f(right_x, right_y);
            }

            void getRadialExLCartesian(float &x, float &y)
            {
                float leftTheta = idx2theta(convexLeftIdx_);
                x = (convexLeftDist_) * cos(leftTheta);
                y = (convexLeftDist_) * sin(leftTheta);
            }

            void getRadialExRCartesian(float &x, float &y)
            {
                float right_theta = idx2theta(convexRightIdx_);
                x = (convexRightDist_) * cos(right_theta);
                y = (convexRightDist_) * sin(right_theta);
            }

            void setAGCIdx(const int & rightIdx, const int & leftIdx) 
            {
                agcrightIdx_ = rightIdx;
                agcleftIdx_ = leftIdx;
                agcrightRange_ = float(rightIdx - rightIdx_) / float(leftIdx_ - rightIdx_) * (leftRange_ - rightRange_) + rightRange_;
                agcleftRange_ = float(leftIdx - rightIdx_) / float(leftIdx_ - rightIdx_) * (leftRange_ - rightRange_) + rightRange_;
            }

            void getAGCLCartesian(float &x, float &y)
            {
                float leftTheta = idx2theta(agcleftIdx_);
                x = (agcleftRange_) * cos(leftTheta);
                y = (agcleftRange_) * sin(leftTheta);
            }

            void getAGCRCartesian(float &x, float &y)
            {
                float right_theta = idx2theta(agcrightIdx_);
                x = (agcrightRange_) * cos(right_theta);
                y = (agcrightRange_) * sin(right_theta);
            }

            void compareGoalDist(const float & goal_dist) 
            {
                goal_within = goal_dist < rightRange_ && goal_dist < leftRange_;
            }

            int manipLeftIdx() const
            {
                return convex.convexLeftIdx_;
            }

            int manipRightIdx() const
            {
                return convex.convexRightIdx_;
            }

            float manipLeftRange() const
            {
                return convex.convexLeftDist_;
            }

            float manipRightRange() const
            {
                return convex.convexRightDist_;
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
                float angle1 = (leftIdx_ - rightIdx_) * resoln;
                float short_side = right_type ? rightRange_ : leftRange_;
                float opp_side = (float) sqrt(pow(rightRange_, 2) + pow(leftRange_, 2) - 2 * rightRange_ * leftRange_ * (float)cos(angle1));
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
                return sqrt(pow(rightRange_, 2) + pow(leftRange_, 2) - 2 * rightRange_ * leftRange_ * (cos(float(leftIdx_ - rightIdx_) / float(half_num_scan) * M_PI)));
            }

            Eigen::Vector2f get_middle_pt_vec() const
            {
                Eigen::Vector2f left_vec = getLCartesian();
                // float right_x, right_y;
                // getRCartesian(right_x, right_y);
                // Eigen::Vector2f right_vec(right_x, right_y);
                
                Eigen::Vector2f right_vec = getRCartesian();
                // float left_x, left_y;
                // getLCartesian(left_x, left_y);
                // Eigen::Vector2f left_vec(left_x, left_y);
                // Eigen::Vector2f m_vec = (right_vec + left_vec) / 2;
                Eigen::Vector2f m_vec = (right_vec + left_vec) / 2.0;
                return m_vec;
            }
            
            bool goal_within = false;
            bool goal_dir_within = false;
            float life_time = 1.0;
            bool agc = false;


            bool reduced = false;
            bool convexified = false;
            int convexRightIdx_;
            int convexLeftIdx_;
            float convexRightDist_;
            float convexLeftDist_;
            float min_safe_dist = -1;
            Eigen::Vector2f qB;
            // float half_num_scan = 256;

            int agcrightIdx_;
            int agcleftIdx_;
            float agcrightRange_;
            float agcleftRange_;

            std::string _frame = "";
            bool right_obs = true;
            bool left_obs = true;
            bool _radial = false;
            bool right_type = false;

            struct converted 
            {
                int convexRightIdx_ = 0;
                int convexLeftIdx_ = 511;
                float convexRightDist_ = 3;
                float convexLeftDist_ = 3;
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

        private:

        int leftIdx_ = 511;
        float leftRange_ = 3;
        
        int rightIdx_ = 0;
        float rightRange_ = 3;        
    };
}