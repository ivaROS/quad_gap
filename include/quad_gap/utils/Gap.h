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
#include <quad_gap/utils/GapPoint.h>

namespace quad_gap
{
    class Gap
    {
        public:
            Gap() {};

            // Only called during gap detection
            Gap(const std::string & frame,
                const ros::Time & timeStamp, 
                const int & leftIdx,
                const float & leftRange,
                const int & rightIdx, 
                const float & rightRange,
                const float & minSafeDist, 
                const bool & radial) 
            {
                frame_ = frame;
                radial_ = radial;

                if (! checkPtIdx(rightIdx))
                {
                    ROS_WARN_STREAM_NAMED("Gap", "[Gap constructor 1]: Gap right index is not valid: " << rightIdx);
                    // ROS_INFO_STREAM_NAMED("Gap", "[Gap constructor 1]: Gap right index is not valid: " << rightIdx);
                    // rightIdx = 0;
                }

                if (! checkPtRange(rightRange))
                {
                    ROS_WARN_STREAM_NAMED("Gap", "[Gap constructor 1]: Gap right range is not valid: " << rightRange);
                    // ROS_INFO_STREAM_NAMED("Gap", "[Gap constructor 1]: Gap right range is not valid: " << rightRange);
                    // rightRange = 0.0;
                }                

                timeStamp_ = timeStamp;
                leftIdx_ = leftIdx;
                leftRange_ = leftRange;
                rightIdx_ = rightIdx;
                rightRange_ = rightRange;
                minSafeDist_ = minSafeDist;
            };

            Gap(const Gap & otherGap)
            {
                frame_ = otherGap.frame_;
                timeStamp_ = otherGap.timeStamp_;

                leftIdx_ = otherGap.leftIdx_;
                rightIdx_ = otherGap.rightIdx_;
                leftRange_ = otherGap.leftRange_;
                rightRange_ = otherGap.rightRange_;

                radial_ = otherGap.radial_;
                rightType_ = otherGap.rightType_;
                
                minSafeDist_ = otherGap.minSafeDist_;
                qB_ = otherGap.qB_;

                convex = otherGap.convex;
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
                rightType_ = rightRange_ < leftRange_;

                setRadial();

                // convex.leftIdx_ = leftIdx_;
                // convex.leftRange_ = leftRange_;

                // convex.rightIdx_ = rightIdx_;
                // convex.rightRange_ = rightRange_;
                setManipPoints(leftIdx_, leftRange_, rightIdx_, rightRange_);
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

            // bool checkPoints()
            // {
            //     return (leftGapPt_->checkOrigPoint() && rightGapPt_->checkOrigPoint() && 
            //             leftGapPt_->checkManipPoint() && rightGapPt_->checkManipPoint());
            // }            

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
                // float resoln = M_PI / half_num_scan;
                float angle1 = (leftIdx_ - rightIdx_) * angle_increment;

                if (angle1 < 0.0)
                {
                    angle1 += 2 * M_PI; // Ensure angle is positive
                }


                float short_side = rightType_ ? rightRange_ : leftRange_;
                float opp_side = (float) sqrt(pow(rightRange_, 2) + pow(leftRange_, 2) - 2 * rightRange_ * leftRange_ * (float)cos(angle1));
                float small_angle = (float) asin(short_side / opp_side * (float) sin(angle1));
                // radial_ = (M_PI - small_angle - angle1 > 0.75 * M_PI); 
                radial_ = (M_PI - small_angle - angle1 > (2.0 / 3.0 * M_PI)); 
                // return radial_;
            }

            /**
            * \brief Getter for gap radial condition
            * \return Gap radial condition
            */
            bool isRadial() const 
            { 
                return radial_; 
            }

            bool isRightType() const
            {
                return rightType_;
            }

            void resetFrame(const std::string & frame) 
            {
                frame_ = frame;
            }

            void setMinSafeDist(const float & dist) 
            {
                minSafeDist_ = dist;
            }

            float getMinSafeDist() const
            {
                return minSafeDist_;
            }

            std::string getFrame() const
            {
                return frame_;
            }

            ros::Time getTimeStamp() const
            {
                return timeStamp_;
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
                this->qB_ = qB;
            }

            Eigen::Vector2f getQB() const
            {
                return qB_;
            }

        private:

            float minSafeDist_ = -1;
            Eigen::Vector2f qB_;

            std::string frame_ = "";
            ros::Time timeStamp_ = ros::Time(0);

            bool radial_ = false;
            bool rightType_ = false;

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