#pragma once

// NON-ROS
#include <vector>
#include <boost/shared_ptr.hpp>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

// QUADGAP
#include <quad_gap/utils/Gap.h>
#include <quad_gap/config/QuadGapConfig.h>

#include <quad_gap/utils/RobotGeometryProcessor.h>

namespace quad_gap 
{
    class GapDetector 
    {
        public: 
            GapDetector(const QuadGapConfig& cfg, RobotGeometryProcessor& robot_geo_proc);

            GapDetector& operator=(GapDetector other) 
            {
                cfg_ = other.cfg_;
                robot_geo_proc_ = other.robot_geo_proc_;

                return *this;
            };

            GapDetector(const GapDetector &t) 
            {
                cfg_ = t.cfg_;
                robot_geo_proc_ = t.robot_geo_proc_;
            };

            /**
            * \brief Preprocess incoming laser scan to remove NaN/Inf values
            *
            * \param scan pointer to incoming laser scan
            */
            void preprocessScan(boost::shared_ptr<sensor_msgs::LaserScan> scan);

            /**
            * \brief Detect raw set of gaps from incoming laser scan.
            * 
            * \param scanPtr pointer to incoming laser scan
            * \return raw set of gaps
            */            
            std::vector<Gap *> gapDetection(boost::shared_ptr<sensor_msgs::LaserScan const> scanPtr);

            /**
            * \brief Condense raw set of gaps into a smaller set of simplified gaps more amenable for navigation.
            * 
            * \param rawGaps set of raw gaps
            * \return set of simplified gaps
            */              
            std::vector<Gap *> gapSimplification(const std::vector<Gap *> & rawGaps);

        private:

            /**
            * \brief Check if a raw swept gap should be merged into a simplified swept gap
            * or if should exist on its own.
            * 
            * \param rawGap queried raw swept gap
            * \param simplifiedGaps existing set of simplified gaps
            * \return boolean for if raw gap should be merged or not
            */
            bool mergeSweptGapCondition(const Gap * rawGap, 
                                        const std::vector<Gap *> & simplifiedGaps);

            bool equivalentCheck(const Gap * detected_gap);

            bool bridgeCondition(const std::vector<Gap *> & rawGaps);

            /**
            * \brief Check if scan range registers an object (finite range value)
            * 
            * \param range incoming scan range value
            * \return boolean if range is finite or not
            */        
            bool isFinite(const float & range);

            /**
            * \brief Determining if swept gap has 
            * either started (finite scan --> infinite scan)
            * or ended (infinite scan --> finite scan)
            * 
            * \param currRange current scan range value
            * \param prevRange previous scan range value
            * \return boolean if a swept gap has either started or ended at current scan index
            */      
            bool sweptGapStartedOrEnded(const float & currRange, 
                                        const float & prevRange);

            /**
            * \brief Iterate backwards through simplified gaps to see if/where
            * a raw radial gap should be merged 
            *
            * \param rawGap queried raw radial gap
            * \param simplifiedGaps existing set of simplified gaps
            * \return index within simplified gaps that should be merged
            */
            int checkSimplifiedGapsMergeability(const Gap * rawGap, 
                                                const std::vector<Gap *> & simpGaps);

            /**
            * \brief Checking if gap should be classified as radial 
            
            * Checking if robot can fit between 
            * consecutive scan points (precondition to radial gap)
            * \param currRange current scan range value
            * \param prevRange previous scan range value
            * \param gapAngle angle between consecutive scan points
            * \return boolean if gap should be classified as radial 
            */
            bool radialGapSizeCheck(const float & currRange, 
                                    const float & prevRange, 
                                    const float & gapAngle);

            sensor_msgs::LaserScan scan_; /**< Current laser scan */
            const QuadGapConfig * cfg_ = NULL; /**< Planner hyperparameter config list */
            RobotGeometryProcessor robot_geo_proc_;
            float minScanDist_ = 0.0; /**< Minimum distance within current laser scan */
            float maxScanDist_ = 0.0; /**< Maximum distance within current laser scan */
            float halfScanRayCount_ = 0.0; /**< Half of number of rays within scan (float) */
            int fullScanRayCount_ = 0; /**< Number of rays within scan (int) */
        };
}