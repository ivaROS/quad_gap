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
            // Gap() {};

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
                timeStamp_ = timeStamp;

                if (! checkPtIdx(rightIdx))
                {
                    ROS_INFO_STREAM_NAMED("Gap", "[Gap constructor 1]: Gap right index is not valid: " << rightIdx);
                    ROS_WARN_STREAM_NAMED("Gap", "[Gap constructor 1]: Gap right index is not valid: " << rightIdx);
                    // rightIdx = 0;
                }

                if (! checkPtRange(rightRange))
                {
                    ROS_INFO_STREAM_NAMED("Gap", "[Gap constructor 1]: Gap right range is not valid: " << rightRange);
                    ROS_WARN_STREAM_NAMED("Gap", "[Gap constructor 1]: Gap right range is not valid: " << rightRange);
                    // rightRange = 0.0;
                }                

                leftGapPt_ = new GapPoint(leftIdx, leftRange); // temp values
                rightGapPt_ = new GapPoint(rightIdx, rightRange);

                minSafeDist_ = minSafeDist;

                // initializing convex polar gap coordinates to raw ones
                leftGapPt_->initManipPoint();
                rightGapPt_->initManipPoint();
                
                setRadial();
                setRightType();

                if (frame_.empty())
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Gap frame is empty");
                } else
                {
                    ROS_INFO_STREAM_NAMED("Gap", "Gap frame is: " << frame_);
                }
            };

            Gap(const Gap & otherGap)
            {
                frame_ = otherGap.frame_;
                radial_ = otherGap.radial_;
                rightType_ = otherGap.rightType_;
                timeStamp_ = otherGap.timeStamp_;

                // deep copy for new points
                leftGapPt_ = new GapPoint(*otherGap.leftGapPt_);
                rightGapPt_ = new GapPoint(*otherGap.rightGapPt_);
                
                minSafeDist_ = otherGap.minSafeDist_;
                qB_ = otherGap.qB_;

                // convex = otherGap.convex;
                goal = otherGap.goal;
                mode = otherGap.mode;
            }

            ~Gap() 
            {
                if (leftGapPt_ != nullptr)
                {
                    delete leftGapPt_;
                    leftGapPt_ = nullptr;
                }
                if (rightGapPt_ != nullptr)
                {
                    delete rightGapPt_;
                    rightGapPt_ = nullptr;
                }
            };

            /**
            * \brief Getter for initial left gap point index
            * \return initial left gap point index
            */
            int LIdx() const { return leftGapPt_->getOrigIdx(); }

            /**
            * \brief Setter for initial left gap point index
            * \param lidx initial left gap point index
            */
            void setLIdx(const int & lidx) { leftGapPt_->setOrigIdx(lidx); }

            /**
            * \brief Getter for initial right gap point index
            * \return initial right gap point index
            */
            int RIdx() const { return rightGapPt_->getOrigIdx(); }

            /**
            * \brief Setter for initial right gap point index
            * \param ridx initial right gap point index
            */            
            void setRIdx(const int & ridx) { rightGapPt_->setOrigIdx(ridx); }

            /**
            * \brief Getter for initial left gap point range
            * \return initial left gap point range
            */
            float LRange() const { return leftGapPt_->getOrigRange(); }

            /**
            * \brief Setter for initial left gap point range
            * \param lrange initial left gap point range
            */
            void setLRange(const float & lrange) { leftGapPt_->setOrigRange(lrange); }

            /**
            * \brief Getter for initial right gap point range
            * \param initial right gap point range
            */
            float RRange() const { return rightGapPt_->getOrigRange(); }

            /**
            * \brief Setter for initial right gap point range
            * \param rrange initial right gap point range
            */            
            void setRRange(const float & rrange) { rightGapPt_->setOrigRange(rrange); }

            void setManipPoints(const int & newLeftIdx, const float & newLeftRange, 
                                const int & newRightIdx, const float & newRightRange)
            {
                leftGapPt_->setManipIdx(newLeftIdx);
                leftGapPt_->setManipRange(newLeftRange);
                rightGapPt_->setManipIdx(newRightIdx);
                rightGapPt_->setManipRange(newRightRange);
            }

            bool checkPoints()
            {
                return (leftGapPt_->checkOrigPoint() && rightGapPt_->checkOrigPoint() && 
                        leftGapPt_->checkManipPoint() && rightGapPt_->checkManipPoint());
            }

            /**
            * \brief Getter for initial manipulated left gap point index
            * \return initial manipulated left gap point index
            */
            int manipLeftIdx() const { return leftGapPt_->getManipIdx(); }

            /**
            * \brief Getter for initial manipulated right gap point index
            * \return initial manipulated right gap point index
            */
            int manipRightIdx() const { return rightGapPt_->getManipIdx(); }

            /**
            * \brief Getter for initial manipulated left gap point distance
            * \return manipulated left gap point distance
            */
            float manipLeftRange() const { return leftGapPt_->getManipRange(); }

            /**
            * \brief Getter for initial manipulated right gap point distance
            * \return manipulated right gap point distance
            */
            float manipRightRange() const { return rightGapPt_->getManipRange(); }

            /**
            * \brief Conclude gap construction by populating gap's initial left side information 
            * and remaining characteristics
            * \param leftIdx initial left gap point index
            * \param leftRange initial left gap point range
            */
            void addLeftInformation(const int & leftIdx, const float & leftRange) 
            {
                leftGapPt_->setOrigIdx(leftIdx); // leftIdx_ = leftIdx;
                leftGapPt_->setOrigRange(leftRange); // leftRange_ = leftRange;

                // initializing convex polar gap coordinates to raw ones
                leftGapPt_->initManipPoint();
                rightGapPt_->initManipPoint();

                setRadial();
                setRightType();
            }

            /**
            * \brief Getter for initial left gap point in Cartesian frame
            * \param x x-position for left gap point
            * \param y y-position for left gap point
            */
            void getLCartesian(float &x, float &y) const { leftGapPt_->getOrigCartesian(x, y); }

            /**
            * \brief Getter for right gap point in Cartesian frame
            * \param x x-position for right gap point
            * \param y y-position for right gap point
            */
            void getRCartesian(float &x, float &y) const { rightGapPt_->getOrigCartesian(x, y); }

            /**
            * \brief Getter for initial manipulated left gap point in Cartesian frame
            * \param x x-position for left gap point
            * \param y y-position for left gap point
            */
            void getManipLCartesian(float &x, float &y) const { leftGapPt_->getManipCartesian(x, y); }

            /**
            * \brief Getter for initial manipulated right gap point in Cartesian frame
            * \param x x-position for right gap point
            * \param y y-position for right gap point
            */
            void getManipRCartesian(float &x, float &y) const { rightGapPt_->getManipCartesian(x, y); }

            Eigen::Vector2f getLPosition() const { return leftGapPt_->getOrigCartesian(); }

            Eigen::Vector2f getRPosition() const { return rightGapPt_->getOrigCartesian(); }

            /**
            * \brief Getter for initial manipulated left gap point in Cartesian frame
            * \return Initial manipulated left gap point in Cartesian frame
            */
            Eigen::Vector2f getManipLPosition() const { return leftGapPt_->getManipCartesian(); }

            /**
            * \brief Getter for initial manipulated right gap point in Cartesian frame
            * \return Initial manipulated right gap point in Cartesian frame
            */
            Eigen::Vector2f getManipRPosition() const { return rightGapPt_->getManipCartesian(); }

            void setRightType() { rightType_ = rightGapPt_->getOrigRange() < leftGapPt_->getOrigRange(); }

            /**
            * \brief Determine if gap is radial
            *
            *   far pt _____
            *          \ A  `___          
            *           \       `___      
            *            \          `___  
            *             \           B ` near pt
            *              \            / 
            *               \          /     A - far side angle 
            *                \        /      B - near side angle
            *                 \      /       C - gap angle
            *                  \    /
            *                   \ C/
            *                    \/
            *                 gap origin
            */
            void setRadial()
            {
                // ROS_INFO_STREAM_NAMED("Gap", "setRadial:");
                int checkLeftIdx = leftGapPt_->getOrigIdx(); // leftIdx_;
                int checkRightIdx = rightGapPt_->getOrigIdx(); // rightIdx_;

                float checkLeftRange = leftGapPt_->getOrigRange(); // leftRange_;
                float checkRightRange = rightGapPt_->getOrigRange(); // rightRange_;

                // ROS_INFO_STREAM_NAMED("Gap", "   checkLeftIdx: " << checkLeftIdx);
                // ROS_INFO_STREAM_NAMED("Gap", "   checkLeftRange: " << checkLeftRange);
                // ROS_INFO_STREAM_NAMED("Gap", "   checkRightIdx: " << checkRightIdx);
                // ROS_INFO_STREAM_NAMED("Gap", "   checkRightRange: " << checkRightRange);

                float gapAngle = (checkLeftIdx - checkRightIdx) * angle_increment;
                if (gapAngle < 0)
                    gapAngle += TWO_M_PI;

                // ROS_INFO_STREAM_NAMED("Gap", "   gapAngle: " << gapAngle);
                float nearRange = rightType_ ? checkRightRange : checkLeftRange;

                // law of cosines
                float pt1 = pow(checkRightRange, 2) + pow(checkLeftRange, 2);
                float pt2 = 2 * checkRightRange * checkLeftRange * cos(gapAngle);
                float leftPtToRightPtDist = sqrt(pt1 - pt2);
                // ROS_INFO_STREAM_NAMED("Gap", "   pt1: " << pt1);
                // ROS_INFO_STREAM_NAMED("Gap", "   pt2: " << pt2);
                // ROS_INFO_STREAM_NAMED("Gap", "   cos(gapAngle): " << cos(gapAngle));
                // ROS_INFO_STREAM_NAMED("Gap", "   leftPtToRightPtDist: " << leftPtToRightPtDist);
                
                // law of sines
                float farSideAngle = asin(epsilonDivide(nearRange, leftPtToRightPtDist) * sin(gapAngle));
                
                // ROS_INFO_STREAM_NAMED("Gap", "nearRange: " << nearRange);
                // ROS_INFO_STREAM_NAMED("Gap", "leftPtToRightPtDist: " << leftPtToRightPtDist);
                // ROS_INFO_STREAM_NAMED("Gap", "small angle: " << farSideAngle);

                // ROS_INFO_STREAM_NAMED("Gap", "   farSideAngle: " << farSideAngle);
                // ROS_INFO_STREAM_NAMED("Gap", "   gapAngle: " << gapAngle);
                float nearSideAngle = (M_PI - farSideAngle - gapAngle);
                // ROS_INFO_STREAM_NAMED("Gap", "   nearSideAngle: " << nearSideAngle);

                radial_ = nearSideAngle > (0.6667 * M_PI);
            }

            GapPoint * getLeftGapPt() const { return leftGapPt_; }
            GapPoint * getRightGapPt() const { return rightGapPt_; }

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

            std::string frame_ = "";
            bool radial_ = false;
            bool rightType_ = false;
            ros::Time timeStamp_ = ros::Time(0);

            GapPoint * leftGapPt_ = NULL; /**< Left gap point */
            GapPoint * rightGapPt_ = NULL; /**< Right gap point */

            float minSafeDist_ = -1;
            Eigen::Vector2f qB_;

            struct Goal 
            {
                float x = -1;
                float y = -1;
                bool set = false;
                bool discard = false;
                bool goalwithin = false;
            } goal;        

            struct GapMode 
            {
                bool reduced = false;
                bool extended = false;
                bool agc = false;
            } mode;

            // struct Convex 
            // {
            //     int leftIdx_ = 511;
            //     int rightIdx_ = 0;

            //     float leftRange_ = 3;
            //     float rightRange_ = 3;
            // } convex;


            // int leftIdx_ = 511;
            // float leftRange_ = 3;
            
            // int rightIdx_ = 0;
            // float rightRange_ = 3;        
    };
}