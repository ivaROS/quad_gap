#pragma once

#include <geometry_msgs/PoseArray.h>
#include <vector>

namespace dynamic_gap
{
    /**
    * \brief Wrapper class for candidate local trajectories that planner produces
    */
    class Trajectory
    {
        public:
            Trajectory()
            {
                pathRbtFrame_ = geometry_msgs::PoseArray();
                pathOdomFrame_ = geometry_msgs::PoseArray();
            }

            Trajectory(const geometry_msgs::PoseArray & pathRbtFrame, 
                       const std::vector<float> & pathTiming)
            {
                pathRbtFrame_ = pathRbtFrame;
                pathTiming_ = pathTiming;

                if (pathRbtFrame.poses.empty())
                {
                    ROS_WARN_STREAM("Trajectory path in robot frame is empty");
                }

                if (pathTiming.empty())
                {
                    ROS_WARN_STREAM("Trajectory path timing is empty");
                }

                if (pathRbtFrame.poses.size() != pathTiming.size())
                {
                    ROS_WARN_STREAM("Trajectory path and timing size mismatch");
                }

                if (pathRbtFrame_.header.frame_id.empty())
                {
                    ROS_WARN_STREAM("Trajectory path frame id is empty");
                }
            }

            /**
            * \brief Setter for trajectory path in robot frame
            * \param pathRbtFrame trajectory path in robot frame
            */
            void setPathRbtFrame(const geometry_msgs::PoseArray & pathRbtFrame) { pathRbtFrame_ = pathRbtFrame; }
            
            /**
            * \brief Getter for trajectory path in robot frame
            * \return trajectory path in robot frame
            */
            geometry_msgs::PoseArray getPathRbtFrame() const { return pathRbtFrame_; }

            /**
            * \brief Setter for trajectory path in odom frame
            * \param pathOdomFrame trajectory path in odom frame
            */
            void setPathOdomFrame(const geometry_msgs::PoseArray & pathOdomFrame) { pathOdomFrame_ = pathOdomFrame; }
            
            /**
            * \brief Getter for trajectory path in odom frame
            * \return trajectory path in odom frame
            */            
            geometry_msgs::PoseArray getPathOdomFrame() const { return pathOdomFrame_; }

            /**
            * \brief Setter for trajectory path timing
            * \param pathTiming trajectory path timing
            */
            void setPathPosewiseCosts(const std::vector<float> & posewiseCosts) { posewiseCosts_ = posewiseCosts; }

            /**
            * \brief Getter for trajectory path timing
            * \return trajectory path timing
            */
            std::vector<float> getPathPosewiseCosts() const { return posewiseCosts_; }

            /**
            * \brief Setter for trajectory terminal pose cost
            * \param terminalPoseCost trajectory terminal pose cost
            */
            void setTerminalPoseCost(float terminalPoseCost) { terminalPoseCost_ = terminalPoseCost; }

            /**
            * \brief Getter for trajectory terminal pose cost
            * \return trajectory terminal pose cost
            */
            float getTerminalPoseCost() const { return terminalPoseCost_; }

            int size() const
            {
                return pathRbtFrame_.poses.size();
            }

        private:
            geometry_msgs::PoseArray pathRbtFrame_; /**< trajectory path in robot frame */
            geometry_msgs::PoseArray pathOdomFrame_; /**< trajectory path in odom frame */
            geometry_msgs::PoseArray orientedPathRbtFrame_; /**< trajectory path in robot frame with orientation decay */
            std::vector<float> posewiseCosts_; /**< trajectory path costs */
            float terminalPoseCost_; /**< trajectory terminal pose cost */
    };
}