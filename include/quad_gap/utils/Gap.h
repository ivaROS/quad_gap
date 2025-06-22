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
                right_obs = otherGap.right_obs;
                left_obs = otherGap.left_obs;
                _radial = otherGap._radial;
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

                convex.leftIdx_ = leftIdx_;
                convex.leftRange_ = leftRange_;

                convex.rightIdx_ = rightIdx_;
                convex.rightRange_ = rightRange_;
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

            int manipLeftIdx() const
            {
                return convex.leftIdx_;
            }

            int manipRightIdx() const
            {
                return convex.rightIdx_;
            }

            float manipLeftRange() const
            {
                return convex.leftRange_;
            }

            float manipRightRange() const
            {
                return convex.rightRange_;
            }

            void setManipPoints(const int & leftIdx, const float & leftRange, const int & rightIdx, const float & rightRange)
            {
                convex.leftIdx_ = leftIdx;
                convex.leftRange_ = leftRange;
                convex.rightIdx_ = rightIdx;
                convex.rightRange_ = rightRange;
            }

            void getManipLCartesian(float &x, float &y) const
            {
                float leftTheta = idx2theta(convex.leftIdx_);
                x = convex.leftRange_ * cos(leftTheta);
                y = convex.leftRange_ * sin(leftTheta);
            }

            Eigen::Vector2f getManipLCartesian() const
            {
                float left_x, left_y;
                getManipLCartesian(left_x, left_y);
                return Eigen::Vector2f(left_x, left_y);
            }

            void getManipRCartesian(float &x, float &y) const
            {
                float rightTheta = idx2theta(convex.rightIdx_);
                x = convex.rightRange_ * cos(rightTheta);
                y = convex.rightRange_ * sin(rightTheta);
            }

            Eigen::Vector2f getManipRCartesian() const
            {
                float right_x, right_y;
                getManipRCartesian(right_x, right_y);
                return Eigen::Vector2f(right_x, right_y);
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

            void setAGC()
            {
                mode.agc = true;
            }

            bool isAGC() const
            {
                return mode.agc;
            }

            void setReduced()
            {
                mode.reduced = true;
            }

            bool isReduced() const
            {
                return mode.reduced;
            }

            void setExtended()
            {
                mode.extended = true;
            }

            bool isExtended() const
            {
                return mode.extended;
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

            void setGoalPos(const float & x, const float & y) 
            {
                goal.x = x;
                goal.y = y;
                goal.set = true;
            }

            void setGoalDiscard() 
            {
                goal.discard = true;
            }

            bool isGoalSet() const 
            {
                return goal.set;
            }

            void setGoalWithin() 
            {
                goal.goalwithin = true;
            }    
            
            bool isGoalWithin() const 
            {
                return goal.goalwithin;
            }

            float getGoalX() const 
            {
                return goal.x;
            }

            float getGoalY() const 
            {
                return goal.y;
            }

            void setQB(const Eigen::Vector2f & qB)
            {
                this->qB = qB;
            }

            Eigen::Vector2f getQB() const
            {
                return qB;
            }

        private:

            float min_safe_dist = -1;
            Eigen::Vector2f qB;
            // float half_num_scan = 256;

            std::string _frame = "";
            bool right_obs = true;
            bool left_obs = true;
            bool _radial = false;
            bool right_type = false;

            struct GapMode 
            {
                bool reduced = false;
                bool extended = false;
                bool agc = false;
            } mode;

            struct Goal 
            {
                float x = -1;
                float y = -1;
                bool set = false;
                bool discard = false;
                bool goalwithin = false;
            } goal;        

            struct Convex 
            {
                int leftIdx_ = 511;
                int rightIdx_ = 0;

                float leftRange_ = 3;
                float rightRange_ = 3;
            } convex;


            int leftIdx_ = 511;
            float leftRange_ = 3;
            
            int rightIdx_ = 0;
            float rightRange_ = 3;        
    };
}