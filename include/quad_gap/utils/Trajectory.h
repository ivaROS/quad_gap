#pragma once

#include <geometry_msgs/msg/pose_array.h>
#include <vector>
#include <numeric>

namespace quad_gap
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

            Trajectory(const geometry_msgs::PoseArray & pathRbtFrame)
            {
                pathRbtFrame_ = pathRbtFrame;
                // pathTiming_ = pathTiming;

                // if (pathRbtFrame.poses.empty())
                // {
                //     ROS_WARN_STREAM_NAMED("Trajectory", "Trajectory path in robot frame is empty");
                // }

                // if (pathTiming.empty())
                // {
                //     ROS_WARN_STREAM_NAMED("Trajectory", "Trajectory path timing is empty");
                // }

                // if (pathRbtFrame.poses.size() != pathTiming.size())
                // {
                //     ROS_WARN_STREAM_NAMED("Trajectory", "Trajectory path and timing size mismatch");
                // }

                if (pathRbtFrame_.header.frame_id.empty())
                {
                    ROS_WARN_STREAM_NAMED("Trajectory", "Trajectory path frame id is empty");
                }
            }

            void setRbtFrameDefaultHeader(const std_msgs::Header & header)
            {
                pathRbtFrame_.header = header;
                orientedPathRbtFrame_.header = header;
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

            void setOrientedPathRbtFrame(const geometry_msgs::PoseArray & orientedPathRbtFrame) { orientedPathRbtFrame_ = orientedPathRbtFrame; }

            geometry_msgs::PoseArray getOrientedPathRbtFrame() const { return orientedPathRbtFrame_; }

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

            void setOrientedPathOdomFrame(const geometry_msgs::PoseArray & orientedPathOdomFrame) { orientedPathOdomFrame_ = orientedPathOdomFrame; }

            geometry_msgs::PoseArray getOrientedPathOdomFrame() const { return orientedPathOdomFrame_; }

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

            float getAveragePosewiseCost() const
            {
                if (posewiseCosts_.empty())
                {
                    return 0.0f;
                }
                float sum = std::accumulate(posewiseCosts_.begin(), posewiseCosts_.end(), 0.0f);
                return sum / posewiseCosts_.size();
            }

            /**
            * \brief Setter for trajectory terminal pose cost
            * \param terminalPoseCost trajectory terminal pose cost
            */
            void setTerminalPoseCost(const float & terminalPoseCost) { terminalPoseCost_ = terminalPoseCost; }

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
            geometry_msgs::PoseArray orientedPathOdomFrame_; /**< trajectory path in odom frame with orientation decay */
            std::vector<float> posewiseCosts_; /**< trajectory path costs */
            float terminalPoseCost_; /**< trajectory terminal pose cost */
    };
}