#include <quad_gap/Planner.h>
// #include "tf/transform_datatypes.h"
// #include <tf/LinearMath/Matrix3x3.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <Eigen/Core>
// #include <Eigen/Geometry>

namespace quad_gap
{
    Planner::~Planner()
    {
        for (Gap * rawGap : currRawGaps_)
            delete rawGap;
        currRawGaps_.clear();

        for (Gap * simplifiedGap : currSimpGaps_)
            delete simplifiedGap;
        currSimpGaps_.clear();   
        
        if (gapDetector_)
            delete gapDetector_;

        if (gapManipulator_)
            delete gapManipulator_;

        if (trajEvaluator_)
            delete trajEvaluator_;    

        if (trajController_)
            delete trajController_;

        if (globalPlanManager_)
            delete globalPlanManager_;

        if (gapVisualizer_)
            delete gapVisualizer_;

        if (goalVisualizer_)
            delete goalVisualizer_;

        if (trajVisualizer_)
            delete trajVisualizer_;

        if (gapTrajGenerator_)
            delete gapTrajGenerator_;

        if (gapGoalPlacer_)
            delete gapGoalPlacer_;

        // if (robotGeoProc_)
        //     delete robotGeoProc_;

        // if (robotGeoStorage_)
        //     delete robotGeoStorage_;

        if (timeKeeper_)
            delete timeKeeper_;

        // if (tfListener_)
        //     delete tfListener_;
    }

    bool Planner::initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
                             const std::string & name)
    {
        node_ = node;

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "bool Planner::initialize started");

        if (initialized_)
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), "PotentialGap Planner already initalized");
            return true;
        }

        // ros::NodeHandle unh("~/" + name);

        // pnh = unh;
        // pnh = ros::NodeHandle(unh.getNamespace() + "/cc");

        // Config Setup
        cfg_.loadRosParamFromNodeHandle(node, name);

        // Load precomputed robot geo
        // std::string file_name = ros::package::getPath("quad_gap") + "/config/box_1_geometry.yaml";
        // unh.getParam("file_name", file_name);
        // unh.setParam("file_name", file_name);

        RobotShape robot_shape = static_cast<RobotShape>(cfg_.rbt.shape_id);
        if (robot_shape == RobotShape::circle)
            cfg_.rbt.width = 0;

        robot_ = Robot(node_, robot_shape, 
                        cfg_.rbt.length, cfg_.rbt.width, 
                        cfg_.rbt.avg_lin_speed, cfg_.rbt.avg_rot_speed);

        // if (cfg_.rbt.use_geo_storage)
        //     robotGeoStorage_ = RobotGeometryStorage(file_name);
        // else
        robotGeoProc_ = RobotGeometryProcessor(robot_, cfg_.planning.decay_factor);
        
        // Visualization Setup
        transformed_laser_pub = node_->create_publisher<sensor_msgs::msg::LaserScan>("transformed_laserscan", 5);

        // TF Lookup setup
        tfBuffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
        initialized_ = true;

        // tfSub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, &Planner::tfCB, this);
        laserSub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(cfg_.scan_topic, 100, std::bind(&Planner::laserScanCB, this, std::placeholders::_1));
        poseSub_ = node_->create_subscription<nav_msgs::msg::Odometry>(cfg_.odom_topic, 10, std::bind(&Planner::poseCB, this, std::placeholders::_1));

        gapDetector_ = new GapDetector(cfg_, robotGeoProc_);
        globalPlanManager_ = new GlobalPlanManager(cfg_); // , robotGeoProc_
        trajEvaluator_ = new TrajectoryEvaluator(cfg_, robotGeoProc_);
        gapTrajGenerator_ = new GapTrajGenerator(cfg_, robotGeoProc_);
        gapGoalPlacer_ = new GapGoalPlacer(cfg_, robotGeoProc_);
        gapVisualizer_ = new GapVisualizer(node, cfg_);
        trajVisualizer_ = new TrajectoryVisualizer(node, cfg_);
        goalVisualizer_ = new GoalVisualizer(node_, cfg_);
        gapManipulator_ = new GapManipulator(cfg_, robotGeoProc_);
        trajController_ = new TrajectoryController(node_, cfg_);
        timeKeeper_ = new TimeKeeper();

        map2rbt_.transform.rotation.w = 1;
        rbt2map_.transform.rotation.w = 1;
        odom2rbt_.transform.rotation.w = 1;
        rbt2odom_.transform.rotation.w = 1;
        rbtPoseInRbtFrame_.pose.orientation.w = 1;
        rbtPoseInRbtFrame_.header.frame_id = cfg_.robot_frame_id;

        // Set collision checker
        if(!cfg_.collision_checker.collision_checker_enable)
        {
            RCLCPP_WARN_STREAM(node_->get_logger(),  "Collision checking is disabled.");
            // return;
        }

        // if (cfg_.collision_checker.cc_type == CC_DEPTH)
        // {
        //     // RCLCPP_INFO_STREAM(node_->get_logger(),  "New cc type = depth");
        //     cc_wrapper_ = std::make_shared<pips_trajectory_testing::DepthImageCCWrapper>(nh, pnh, tf2_utils::TransformManager(tfBuffer, tfListener));
        // } else if(cfg_.collision_checker.cc_type == CC_DEPTH_EGO)
        // {
        //     // RCLCPP_INFO_STREAM(node_->get_logger(),  "New cc type = depth ego");
        //     cc_wrapper_ = std::make_shared<pips_egocylindrical::EgocylindricalRangeImageCCWrapper>(nh, pnh, tf2_utils::TransformManager(tfBuffer, tfListener));
        // } else if(cfg_.collision_checker.cc_type == CC_EGOCIRCLE)
        // {
        //     // RCLCPP_INFO_STREAM(node_->get_logger(),  "New cc type = egocircle");
        //     cc_wrapper_ = std::make_shared<pips_egocircle::EgoCircleCCWrapper>(nh, pnh, tf2_utils::TransformManager(tfBuffer, tfListener));
        // }

        // traj_tester_ = std::make_shared<TurtlebotGenAndTest>(nh, pnh);
        
        // cc_wrapper_->init();
        // cc_wrapper_->autoUpdate();

        // traj_tester_->init();
        // traj_tester_->setCollisionChecker(cc_wrapper_->getCC());
        
        // cc_type_ = cfg_.collision_checker.cc_type;

        int bufferSize = 5;
        cmdVelBuffer.set_capacity(bufferSize);

        currScanTime_ = node_->get_clock()->now();
        lastScanTime_ = currScanTime_;

        currPlanTime_ = node_->get_clock()->now();
        lastPlanTime_ = currPlanTime_;

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "bool Planner::initialize ended");

        return true;
    }

    bool Planner::isGoalReached()
    {

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "bool Planner::isGoalReached started");


        // Linear distance
        float globalGoalXDiff = globalGoalOdomFrame_.pose.position.x - rbtPoseInOdomFrame_.pose.position.x;
        float globalGoalYDiff = globalGoalOdomFrame_.pose.position.y - rbtPoseInOdomFrame_.pose.position.y;
        float globalGoalLinDist = sqrt(pow(globalGoalXDiff, 2) + pow(globalGoalYDiff, 2));

        // Angular distance
        float globalGoalOrientation = quaternionToYaw(globalGoalOdomFrame_.pose.orientation);
        float rbtPoseOrientation = quaternionToYaw(globalGoalOdomFrame_.pose.orientation);
        float globalGoalAngDist = normalize_theta(globalGoalOrientation - rbtPoseOrientation);
        
        reachedGlobalGoal_ = globalGoalLinDist < cfg_.goal.xy_global_goal_tolerance &&
                             globalGoalAngDist < cfg_.goal.yaw_global_goal_tolerance;
        
        // if (reachedGlobalGoal_)
            // RCLCPP_INFO_STREAM(node_->get_logger(),  "[Reset] Goal Reached");
        // else
        //     // RCLCPP_INFO_STREAM(node_->get_logger(),  "Distance from goal: " << globalGoalDist << 
        //                                      ", Goal tolerance: " << cfg_.goal.xy_global_goal_tolerance);

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "bool Planner::isGoalReached ended");

        return reachedGlobalGoal_;
    }    

    sensor_msgs::msg::LaserScan::ConstSharedPtr Planner::transformLaserToRbt(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scanSensorFrame)
    {

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "sensor_msgs::msg::LaserScan::ConstSharedPtr Planner::transformLaserToRbt started");

        // // RCLCPP_INFO_STREAM(node_->get_logger(), "[transformLaserToRbt()]");

        sensor_msgs::msg::LaserScan scanRbtFrame = *scanSensorFrame;
        scanRbtFrame.header = scanSensorFrame->header;
        scanRbtFrame.header.frame_id = cfg_.robot_frame_id;
        scanRbtFrame.angle_min = scanSensorFrame->angle_min;
        scanRbtFrame.angle_max = scanSensorFrame->angle_max;
        scanRbtFrame.angle_increment = scanSensorFrame->angle_increment;
        scanRbtFrame.time_increment = scanSensorFrame->time_increment;
        scanRbtFrame.scan_time = scanSensorFrame->scan_time;
        scanRbtFrame.range_min = scanSensorFrame->range_min;
        scanRbtFrame.range_max = scanSensorFrame->range_max;
        scanRbtFrame.intensities = scanSensorFrame->intensities;
        // scanRbtFrame.ranges = scanSensorFrame->ranges;

        // Initialize ranges to negative values to indicate no data, will use at second pass
        scanRbtFrame.ranges = std::vector<float>(scanSensorFrame->ranges.size(), -scanSensorFrame->range_max);

        float origRange = 0.0;
        float origAng = 0.0;
        geometry_msgs::msg::PointStamped scanPtSensorFrame;
        geometry_msgs::msg::PointStamped scanPtRbtFrame;
        geometry_msgs::msg::TransformStamped trans = tfBuffer->lookupTransform(cfg_.robot_frame_id, cfg_.sensor_frame_id, rclcpp::Time(0));
        float transRange = 0.0;
        float transTheta = 0.0;
        int transIdx = 0;

        for (size_t i = 0; i < scanSensorFrame->ranges.size(); i++)
        {
            // // RCLCPP_INFO_STREAM(node_->get_logger(), "  scan point: " << i);

            origRange = scanSensorFrame->ranges[i];
            origAng = idx2theta(i);

            scanPtSensorFrame.header = scanSensorFrame->header;
            scanPtSensorFrame.point.x = origRange * cos(origAng);
            scanPtSensorFrame.point.y = origRange * sin(origAng);

            // // RCLCPP_INFO_STREAM(node_->get_logger(), "  Original scan point (pol): " << 
            //     " (" << origAng << ", " << origRange << ")");

            // // RCLCPP_INFO_STREAM(node_->get_logger(), "  Original scan point (cart): " 
            //     << scanPtSensorFrame.point.x << ", " << scanPtSensorFrame.point.y);
            
            tf2::doTransform(scanPtSensorFrame, scanPtRbtFrame, trans);

            // // RCLCPP_INFO_STREAM(node_->get_logger(), "  Transformed scan point (cart): " 
            //     << scanPtRbtFrame.point.x << ", " << scanPtRbtFrame.point.y);
            // // RCLCPP_INFO_STREAM(node_->get_logger(),  cfg_.sensor_frame_id << " " << scanPtSensorFrame.header.frame_id << " " << scanPtRbtFrame.header.frame_id);

            transRange = sqrt(pow(scanPtRbtFrame.point.x, 2) + pow(scanPtRbtFrame.point.y, 2));
            transTheta = std::atan2(scanPtRbtFrame.point.y, scanPtRbtFrame.point.x);
            transIdx = theta2idx(transTheta);

            // what if original index is missed?
            // // RCLCPP_INFO_STREAM(node_->get_logger(), "  Transformed scan point (pol): " << 
            //     " (" << transTheta << ", " << transRange << ")");

            // if (transRange < scanRbtFrame.ranges[transIdx])
            scanRbtFrame.ranges[transIdx] = std::min(scanSensorFrame->range_max, transRange);

            // // RCLCPP_INFO_STREAM(node_->get_logger(), "  Transformed scan point index: " << transIdx << 
            //     ", range: " << scanRbtFrame.ranges[transIdx]);
        }

        // TODO: second pass through scan to interpolate missing values

        // // RCLCPP_INFO_STREAM(node_->get_logger(), "  Transformed scan second pass...");
        // second pass through scan to interpolate missing values
        for (size_t i = 0; i < scanRbtFrame.ranges.size(); i++)
        {
            // // RCLCPP_INFO_STREAM(node_->get_logger(), "  (pre) scan point: " << i << ", range: " << scanRbtFrame.ranges[i]);
            if (scanRbtFrame.ranges[i] < 0.0)
            {
                // find closest non-negative value
                size_t leftIdx = (i + 1) % cfg_.scan.full_scan;
                while (scanRbtFrame.ranges[leftIdx] < 0.0 && leftIdx != i) // prevent infinite loop
                {
                    leftIdx = (leftIdx + 1) % cfg_.scan.full_scan;
                }

                size_t rightIdx = subtractAndWrapScanIndices(i - 1, cfg_.scan.full_scan);
                while (scanRbtFrame.ranges[rightIdx] < 0.0 && rightIdx != i) // prevent infinite loop
                {
                    rightIdx = subtractAndWrapScanIndices(rightIdx - 1, cfg_.scan.full_scan);
                }

                scanRbtFrame.ranges[i] = std::min(scanRbtFrame.ranges[leftIdx], scanRbtFrame.ranges[rightIdx]);

                // // RCLCPP_INFO_STREAM(node_->get_logger(), "  scan point: " << i << " is negative, interpolating");
                // interpolate missing values
                // float leftRange = (i > 0) ? scanRbtFrame.ranges[i - 1] : scanRbtFrame.range_max;
                // float rightRange = (i < scanRbtFrame.ranges.size() - 1) ? scanRbtFrame.ranges[i + 1] : scanRbtFrame.range_max;
                // scanRbtFrame.ranges[i] = std::min(leftRange, rightRange);
            }
            // // RCLCPP_INFO_STREAM(node_->get_logger(), "  (post) scan point: " << i << ", range: " << scanRbtFrame.ranges[i]);
        }

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "sensor_msgs::msg::LaserScan::ConstSharedPtr Planner::transformLaserToRbt ended");

        return std::make_shared<sensor_msgs::msg::LaserScan>(scanRbtFrame);
    }

    void Planner::laserScanCB(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scanSensorFrame)
    {

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "void Planner::laserScanCB started");

        boost::mutex::scoped_lock gapset(gapMutex_);

        // // RCLCPP_INFO_STREAM(node_->get_logger(),  "[laserScanCB()]");

        timeKeeper_->startTimer(SCAN);

        /////////////////////////////////////
        ///////// UPDATE TRANSFORMS /////////
        /////////////////////////////////////
        updateTFs();

        if (!haveTFs_)
        {
            RCLCPP_WARN_STREAM(node_->get_logger(),  "Transforms not available yet, skipping scan processing");
            timeKeeper_->stopTimer(SCAN);
            return;
        }

        cfg_.updateParamFromScan(scanSensorFrame);

        /////////////////////////////////////
        //////// SCAN PRE-PROCESSING ////////
        /////////////////////////////////////

        currScanTime_ = scanSensorFrame->header.stamp;
        // // RCLCPP_INFO_STREAM(node_->get_logger(),  "     Current scan time: " << currScanTime_.seconds() << "." << currScanTime_.nanoseconds());
        rclcpp::Duration scanDuration = currScanTime_ - lastScanTime_;
        // // RCLCPP_INFO_STREAM(node_->get_logger(),  "     Time since last scan: " << scanDuration.seconds() << "." << scanDuration.nanoseconds() << " seconds");

        float minScanRaw = *std::min_element(scanSensorFrame->ranges.begin(), scanSensorFrame->ranges.end());

        sensor_msgs::msg::LaserScan::ConstSharedPtr preprocessed_scan = 
            gapDetector_->preprocessScan(scanSensorFrame);

        float minScanPostPreProcess = *std::min_element(preprocessed_scan->ranges.begin(), preprocessed_scan->ranges.end());

        scanRbtFrame_ = transformLaserToRbt(preprocessed_scan);

        assert(scanRbtFrame_->header.frame_id == cfg_.robot_frame_id);

        transformed_laser_pub->publish(*scanRbtFrame_);

        float minScanDist = *std::min_element(scanRbtFrame_->ranges.begin(), scanRbtFrame_->ranges.end());

        // // RCLCPP_INFO_STREAM(node_->get_logger(),  "     Min scan distance: " << minScanDist << 
        //                                         ", preprocessed: " << minScanPostPreProcess << 
        //                                         ", raw: " << minScanRaw);

        if (minScanDist < cfg_.rbt.r_inscr)
        {
            // RCLCPP_INFO_STREAM(node_->get_logger(),  "       in collision!");
            colliding_ = true;
            return;
        } else
        {
            colliding_ = false;
        }

        ///////////////////////////////
        //////// GAP DETECTION ////////
        ///////////////////////////////

        timeKeeper_->startTimer(GAP_DET);
        currRawGaps_ = gapDetector_->gapDetection(scanRbtFrame_);
        timeKeeper_->stopTimer(GAP_DET);

        for (Gap * rawGap : currRawGaps_)
        {
            assert(rawGap->getFrame() == cfg_.robot_frame_id);
            // // RCLCPP_INFO_STREAM(node_->get_logger(),  "raw gap: " << rawGap->getLeftX() << ", " << rawGap->getLeftY() << " | " << rawGap->getRightX() << ", " << rawGap->getRightY());
        }

        gapVisualizer_->drawGaps(currRawGaps_, std::string("raw"));

        ////////////////////////////////////
        //////// GAP SIMPLIFICATION ////////
        ////////////////////////////////////

        timeKeeper_->startTimer(GAP_SIMP);
        currSimpGaps_ = gapDetector_->gapSimplification(currRawGaps_);
        timeKeeper_->stopTimer(GAP_SIMP);

        for (Gap * simplifiedGap : currSimpGaps_)
        {
            assert(simplifiedGap->getFrame() == cfg_.robot_frame_id);
            // // RCLCPP_INFO_STREAM(node_->get_logger(),  "simp gap: " << simplifiedGap->getLeftX() << ", " << simplifiedGap->getLeftY() << " | " << simplifiedGap->getRightX() << ", " << simplifiedGap->getRightY());
        }

        gapVisualizer_->drawGaps(currSimpGaps_, std::string("simp"));

        // // RCLCPP_INFO_STREAM(node_->get_logger(),  "currSimpGaps_ count:" << currSimpGaps_.size());

        hasLaserScan_ = true;

        // update current scan for proper classes
        updateEgoCircle();

        if (hasGlobalGoal_)
        {
            // update global path local waypoint according to new scan
            globalPlanManager_->generateGlobalPathLocalWaypoint(map2rbt_);
            geometry_msgs::msg::PoseStamped globalPathLocalWaypointOdomFrame = globalPlanManager_->getGlobalPathLocalWaypointOdomFrame(rbt2odom_);
            goalVisualizer_->drawGlobalPathLocalWaypoint(globalPathLocalWaypointOdomFrame);
            goalVisualizer_->drawGlobalGoal(globalGoalOdomFrame_);
            trajEvaluator_->transformGlobalPathLocalWaypointToRbtFrame(globalPathLocalWaypointOdomFrame, odom2rbt_);
        }  

        for (Gap * rawGap : prevRawGaps_)
            delete rawGap;
        prevRawGaps_.clear();

        for (Gap * simplifiedGap : prevSimpGaps_)
            delete simplifiedGap;
        prevSimpGaps_.clear();

        // store current gaps as previous gaps
        prevRawGaps_ = currRawGaps_;
        prevSimpGaps_ = currSimpGaps_;

        lastScanTime_ = currScanTime_;

        timeKeeper_->stopTimer(SCAN);

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "void Planner::laserScanCB ended");
    }

    void Planner::updateEgoCircle()
    {
        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "void Planner::updateEgoCircle started");

        // If no global plan, the local goal finding won't execute.
        globalPlanManager_->updateEgoCircle(scanRbtFrame_);
        trajEvaluator_->updateEgoCircle(scanRbtFrame_);

        gapManipulator_->updateEgoCircle(scanRbtFrame_);
        gapGoalPlacer_->updateEgoCircle(scanRbtFrame_);
        trajController_->updateEgoCircle(scanRbtFrame_);

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "void Planner::updateEgoCircle ended");
    }

    void Planner::poseCB(const nav_msgs::msg::Odometry::ConstSharedPtr& rbtOdomMsg)
    {
        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "void Planner::poseCB started");

        // // RCLCPP_INFO_STREAM(node_->get_logger(),  "[poseCB()]");
        // // RCLCPP_INFO_STREAM(node_->get_logger(),  "[poseCB()]");

        if (!haveTFs_)
            return;

        //////////////////////////
        //         POSE         //
        //////////////////////////       

        // Transform the msg to odom frame
        if (rbtOdomMsg->header.frame_id != cfg_.odom_frame_id)
        {
            RCLCPP_WARN_STREAM(node_->get_logger(),  "Odom msg header frame (for ego-robot pose) " << rbtOdomMsg->header.frame_id << " not same as cfg_ odom frame:" << cfg_.odom_frame_id);

            geometry_msgs::msg::TransformStamped origFrame2OdomFrame = tfBuffer->lookupTransform(cfg_.odom_frame_id, 
                                                                                            rbtOdomMsg->header.frame_id, 
                                                                                            rclcpp::Time(0));

            geometry_msgs::msg::PoseStamped poseIn, poseOut;
            poseIn.header = rbtOdomMsg->header;
            poseIn.pose = rbtOdomMsg->pose.pose;

            tf2::doTransform(poseIn, poseOut, origFrame2OdomFrame);
            rbtPoseInOdomFrame_ = poseOut;
        } else
        {
            rbtPoseInOdomFrame_.header = rbtOdomMsg->header;
            rbtPoseInOdomFrame_.pose = rbtOdomMsg->pose.pose;
        }

        ///////////////////////////
        //       VELOCITY        //
        ///////////////////////////

        // if (rbtOdomMsg->child_frame_id != cfg_.robot_frame_id)
        // {

        // RCLCPP_WARN_STREAM(node_->get_logger(),  "Odom msg child frame (for ego-robot velocity) " << rbtOdomMsg->child_frame_id << " not same as cfg_ rbt frame: " << cfg_.robot_frame_id);

        geometry_msgs::msg::Vector3Stamped velIn, velOut;
        velIn.header = rbtOdomMsg->header; // TODO: make sure this is correct frame
        velIn.vector = rbtOdomMsg->twist.twist.linear;

        geometry_msgs::msg::TransformStamped origFrame2RbtFrame = tfBuffer->lookupTransform(cfg_.robot_frame_id,
                                                                                        // rbtOdomMsg->child_frame_id, 
                                                                                        rbtOdomMsg->header.frame_id, 
                                                                                        rclcpp::Time(0)); 
                                                                                    
        tf2::doTransform(velIn, velOut, origFrame2RbtFrame);

        rbtVelRbtFrame_.header = velOut.header;
        rbtVelRbtFrame_.twist.linear = velOut.vector;
        rbtVelRbtFrame_.twist.angular = rbtOdomMsg->twist.twist.angular; // z is same between frames

        // } else
        // {

        // }
        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "void Planner::poseCB ended");
    }

    void Planner::setPlan(const nav_msgs::msg::Path & incomingGlobalPlan)
    {
        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "void Planner::setPlan started");

        // RCLCPP_INFO_STREAM(node_->get_logger(),  "[setPlan()]");

        if (incomingGlobalPlan.poses.size() == 0) 
            return;

        if (!haveTFs_)
            return;        

        ////////////////////////////////////
        // CHECK IF GLOBAL PLAN MAP FRAME //
        ////////////////////////////////////

        // plan should be in global frame
        nav_msgs::msg::Path globalPlanMapFrame;
        if (incomingGlobalPlan.header.frame_id != cfg_.map_frame_id)
        {
            RCLCPP_WARN_STREAM(node_->get_logger(),  "Global plan header frame " << incomingGlobalPlan.header.frame_id << 
                                                    " not same as cfg_ map frame: " << cfg_.map_frame_id);
            geometry_msgs::msg::TransformStamped other_to_global_trans = tfBuffer->lookupTransform(cfg_.map_frame_id, incomingGlobalPlan.header.frame_id, rclcpp::Time(0));
            for (size_t i = 0; i < incomingGlobalPlan.poses.size(); i++)
            {
                geometry_msgs::msg::PoseStamped plan_pose = incomingGlobalPlan.poses[i];
                geometry_msgs::msg::PoseStamped out_pose;
                tf2::doTransform(plan_pose, out_pose, other_to_global_trans);
                out_pose.header.stamp = plan_pose.header.stamp;
                out_pose.header.frame_id = cfg_.map_frame_id;
                globalPlanMapFrame.poses.push_back(out_pose);
            }
        } else
        {
            globalPlanMapFrame = incomingGlobalPlan;
        }
        
        geometry_msgs::msg::PoseStamped globalGoalMapFrame = *std::prev(globalPlanMapFrame.poses.end());
        // geometry_msgs::msg::PoseStamped globalGoalOdomFrame;
        // Transform global goal to odom frame
        tf2::doTransform(globalGoalMapFrame, globalGoalOdomFrame_, map2odom_);

        // Store New Global Plan to Goal Selector
        globalPlanManager_->updateGlobalPathMapFrame(globalPlanMapFrame);
        
        trajVisualizer_->drawGlobalPlan(globalPlanManager_->getGlobalPathOdomFrame());

        // Find Local Goal
        globalPlanManager_->generateGlobalPathLocalWaypoint(map2rbt_);
        // return local goal (odom) frame
        geometry_msgs::msg::PoseStamped newglobalPathLocalWaypointOdomFrame = globalPlanManager_->getGlobalPathLocalWaypointOdomFrame(rbt2odom_);

        // Plan New
        float diffX = globalPathLocalWaypointOdomFrame_.pose.position.x - newglobalPathLocalWaypointOdomFrame.pose.position.x;
        float diffY = globalPathLocalWaypointOdomFrame_.pose.position.y - newglobalPathLocalWaypointOdomFrame.pose.position.y;
        
        if (sqrt(pow(diffX, 2) + pow(diffY, 2)) > cfg_.goal.xy_waypoint_tolerance)
            globalPathLocalWaypointOdomFrame_ = newglobalPathLocalWaypointOdomFrame;

        // Set new local goal to trajectory arbiter
        trajEvaluator_->transformGlobalPathLocalWaypointToRbtFrame(globalPathLocalWaypointOdomFrame_, odom2rbt_);

        std::vector<geometry_msgs::msg::PoseStamped> visibleGlobalPlanSnippetRobotFrame = globalPlanManager_->getVisibleGlobalPlanSnippetRobotFrame(map2rbt_);
        trajVisualizer_->drawRelevantGlobalPlanSnippet(visibleGlobalPlanSnippetRobotFrame);

        hasGlobalGoal_ = true;
        setReachedGlobalGoal(false);

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "void Planner::setPlan ended");
        return;
    }

    // void Planner::tfCB(const tf2_msgs::msg::TFMessage& msg)
    // {
        // boost::mutex::scoped_lock tfset(tfMutex_);
        // // RCLCPP_INFO_STREAM(node_->get_logger(),  "[tfCB()]");
    void Planner::updateTFs()
    {
        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "void Planner::updateTFs started");

        try
        {
            map2rbt_  = tfBuffer->lookupTransform(cfg_.robot_frame_id, cfg_.map_frame_id, rclcpp::Time(0));
            rbt2map_  = tfBuffer->lookupTransform(cfg_.map_frame_id, cfg_.robot_frame_id, rclcpp::Time(0));
            odom2rbt_ = tfBuffer->lookupTransform(cfg_.robot_frame_id, cfg_.odom_frame_id, rclcpp::Time(0));
            rbt2odom_ = tfBuffer->lookupTransform(cfg_.odom_frame_id, cfg_.robot_frame_id, rclcpp::Time(0));
            cam2odom_ = tfBuffer->lookupTransform(cfg_.odom_frame_id, cfg_.sensor_frame_id, rclcpp::Time(0));
            odom2cam_ = tfBuffer->lookupTransform(cfg_.sensor_frame_id, cfg_.odom_frame_id, rclcpp::Time(0));
            map2odom_ = tfBuffer->lookupTransform(cfg_.odom_frame_id, cfg_.map_frame_id, rclcpp::Time(0));
            rbt2cam_ = tfBuffer->lookupTransform(cfg_.sensor_frame_id, cfg_.robot_frame_id, rclcpp::Time(0));
            cam2rbt_ = tfBuffer->lookupTransform(cfg_.robot_frame_id, cfg_.sensor_frame_id, rclcpp::Time(0));

            tf2::doTransform(rbtPoseInRbtFrame_, rbtPoseInSensorFrame_, rbt2cam_);

            robot_.drawRobotShape(cfg_.robot_frame_id, node_->get_clock()->now());
        
            haveTFs_ = true;
        } catch (tf2::TransformException &ex) 
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), ex.what());
            // rclcpp::Duration::from_seconds(0.1).sleep();
            return;
        }

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "void Planner::updateTFs ended");
    }

    std::vector<Gap *> Planner::gapManipulate(const std::vector<Gap *> & planningGaps) 
    {

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "std::vector<Gap *> Planner::gapManipulate started");

        // RCLCPP_INFO_STREAM(node_->get_logger(), "[manipulateGaps()]");

        boost::mutex::scoped_lock gapset(gapMutex_);
        std::vector<Gap *> manipGaps;

        // geometry_msgs::msg::PoseStamped local_goal_sensor_frame;
        // tf2::doTransform(globalPlanManager_->rbtFrameLocalGoal(), local_goal_sensor_frame, rbt2cam_);
        geometry_msgs::msg::PoseStamped globalPathLocalWaypointRobotFrame = globalPlanManager_->getGlobalPathLocalWaypointRobotFrame();
        try 
        {
            for (size_t i = 0; i < planningGaps.size(); i++)
            {
                gapManipulator_->reduceGap(planningGaps.at(i), globalPathLocalWaypointRobotFrame);
                gapManipulator_->convertRadialGap(planningGaps.at(i));
                gapManipulator_->inflateGapSides(planningGaps.at(i));
                gapManipulator_->radialExtendGap(planningGaps.at(i));

                bool valid = planningGaps.at(i)->checkPoints();

                if (!valid)
                {
                    RCLCPP_WARN_STREAM(node_->get_logger(), "    invalid gap after manipulation " << i);
                    continue;
                }
                manipGaps.push_back(planningGaps.at(i)); // shallow copy
            }
        } catch(...) 
        {
            RCLCPP_FATAL_STREAM(node_->get_logger(), "gapManipulate");
        }

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "std::vector<Gap *> Planner::gapManipulate ended");
        return manipGaps;
    }

    void Planner::gapGoalPlace(const std::vector<Gap *> & planningGaps) 
    {
        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "void Planner::gapGoalPlace started");

        boost::mutex::scoped_lock gapset(gapMutex_);

        geometry_msgs::msg::PoseStamped globalPathLocalWaypointRobotFrame = globalPlanManager_->getGlobalPathLocalWaypointRobotFrame();

        try 
        {
            for (size_t i = 0; i < planningGaps.size(); i++)
            {
                gapGoalPlacer_->setGapWaypoint(planningGaps.at(i), globalPathLocalWaypointRobotFrame);
            }
        } catch(...) 
        {
            RCLCPP_FATAL_STREAM(node_->get_logger(), "gapGoalPlace");
        }            

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "void Planner::gapGoalPlace ended");
    }

    // std::vector<geometry_msgs::msg::PoseArray>& gapPaths, 
    // std::vector<geometry_msgs::msg::PoseArray>& virtualGapPaths,
    // std::vector<std::vector<float>> & pathPoseCosts,
    // std::vector<float> & pathTerminalPoseCosts    
    void Planner::generateGapTrajectories(const std::vector<Gap *> & gaps, 
                                            std::vector<Trajectory> & gapTrajs) 
    {
        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "void Planner::generateGapTrajectories started");

        // // RCLCPP_INFO_STREAM(node_->get_logger(), "[generateGapTrajectories()]");

        boost::mutex::scoped_lock gapset(gapMutex_);

        gapTrajs = std::vector<Trajectory>(gaps.size());
        // gapPaths = std::vector<geometry_msgs::msg::PoseArray> (gaps.size());
        // virtualGapPaths = std::vector<geometry_msgs::msg::PoseArray> (gaps.size());
        // pathPoseCosts = std::vector<std::vector<float>>(gaps.size());
        // pathTerminalPoseCosts = std::vector<float>(gaps.size());

        geometry_msgs::msg::PoseStamped rbt_local_pose;
        rbt_local_pose.header.frame_id = cfg_.robot_frame_id;
        rbt_local_pose.header.stamp = rbt2odom_.header.stamp;
        rbt_local_pose.pose.orientation.w = 1;

        try 
        {
            for (size_t i = 0; i < gaps.size(); i++) 
            {
                // // RCLCPP_INFO_STREAM(node_->get_logger(),  "   Generating trajectory for gap " << i);

                Trajectory gapTraj;

                // std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();        

                // Generate trajectory in robot frame.
                if (cfg_.planning.use_bezier)
                {
                    gapTraj = gapTrajGenerator_->generateBezierTrajectory(gaps.at(i), rbtVelRbtFrame_);
                } else
                {
                    gapTraj = gapTrajGenerator_->generateTrajectory(gaps.at(i), rbt_local_pose);
                }

                // std::chrono::steady_clock::time_point gen_traj_time = std::chrono::steady_clock::now();
                
                // // RCLCPP_INFO_STREAM(node_->get_logger(),  "   Trajectory size: " << gapTraj.size());
                // // RCLCPP_INFO_STREAM(node_->get_logger(),  "   Trajectory:");
                // for (const auto & pose : gapTraj.getPathRbtFrame().poses)
                // {
                //     // RCLCPP_INFO_STREAM(node_->get_logger(),  "      " << pose.position.x << ", " << pose.position.y);
                // }

                gapTraj = gapTrajGenerator_->processTrajectory(gapTraj);

                // std::chrono::steady_clock::time_point proc_traj_time = std::chrono::steady_clock::now();

                // RCLCPP_INFO_STREAM(node_->get_logger(),  "   orient decayed path 1");
                gapTrajGenerator_->getOrientDecayedPath(gapTraj);
                // virtualGapPaths.at(i) = orientedGapTraj;

                // std::chrono::steady_clock::time_point orient_traj_time = std::chrono::steady_clock::now();

                // std::vector<float> pathPoseCost;
                // float pathTerminalCost;                
                trajEvaluator_->evaluateTrajectory(gapTraj); // orientedGapTraj, pathPoseCost, pathTerminalCost

                // pathPoseCosts.at(i) = pathPoseCost;
                // pathTerminalPoseCosts.at(i) = pathTerminalCost;

                float averagedPoseCost = gapTraj.getAveragePosewiseCost();
                float pathTerminalCost = gapTraj.getTerminalPoseCost();
                float pathCost = pathTerminalCost + averagedPoseCost;
                // // RCLCPP_INFO_STREAM(node_->get_logger(),  "   Trajectory averaged pose cost: " << averagedPoseCost);
                // // RCLCPP_INFO_STREAM(node_->get_logger(),  "   Trajectory terminal pose cost: " << pathTerminalCost);
                // // RCLCPP_INFO_STREAM(node_->get_logger(),  "   Trajectory total cost: " << pathCost);

                // std::chrono::steady_clock::time_point score_traj_time = std::chrono::steady_clock::now();

                gapTraj.setPathOdomFrame(gapTrajGenerator_->transformPath(gapTraj.getPathRbtFrame(), rbt2odom_));
            
                // std::chrono::steady_clock::time_point transform_traj_time = std::chrono::steady_clock::now();

                // Log the time taken for each step
                // auto gen_traj_duration = std::chrono::duration_cast<std::chrono::milliseconds>(gen_traj_time - start_time).count();
                // auto proc_traj_duration = std::chrono::duration_cast<std::chrono::milliseconds>(proc_traj_time - gen_traj_time).count();
                // auto orient_traj_duration = std::chrono::duration_cast<std::chrono::milliseconds>(orient_traj_time - proc_traj_time).count();
                // auto score_traj_duration = std::chrono::duration_cast<std::chrono::milliseconds>(score_traj_time - orient_traj_time).count();
                // auto transform_traj_duration = std::chrono::duration_cast<std::chrono::milliseconds>(transform_traj_time - score_traj_time).count();    

                // auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(transform_traj_time - start_time).count();

                // // RCLCPP_INFO_STREAM(node_->get_logger(), "   Gap Trajectory " << i << " generated");
                // // RCLCPP_INFO_STREAM(node_->get_logger(), "   Time taken for trajectory generation: " << gen_traj_duration << " ms");
                // // RCLCPP_INFO_STREAM(node_->get_logger(), "   Time taken for trajectory processing: " << proc_traj_duration << " ms");
                // // RCLCPP_INFO_STREAM(node_->get_logger(), "   Time taken for trajectory orientation decay: " << orient_traj_duration << " ms");
                // // RCLCPP_INFO_STREAM(node_->get_logger(), "   Time taken for trajectory scoring: " << score_traj_duration << " ms");
                // // RCLCPP_INFO_STREAM(node_->get_logger(), "   Time taken for trajectory transformation: " << transform_traj_duration << " ms");
                // // RCLCPP_INFO_STREAM(node_->get_logger(), "   Total time taken for trajectory generation: " << total_duration << " ms");
            
                gapTrajs.at(i) = gapTraj;
            }
        } catch (...) 
        {
            RCLCPP_FATAL_STREAM(node_->get_logger(), "generateGapTrajectories");
        }
        
        // trajVisualizer_->pubAllScore(ret_traj, ret_traj_scores);
        trajVisualizer_->drawGapTrajectories(gapTrajs);

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "void Planner::generateGapTrajectories ended");
        return;
    }

    // const std::vector<geometry_msgs::msg::PoseArray> & gapPaths, 
    // const std::vector<geometry_msgs::msg::PoseArray> & virtualGapPaths, 
    // const std::vector<std::vector<float>> & pathPoseCosts, 
    // const std::vector<float> & pathTerminalPoseCosts, 
    // geometry_msgs::msg::PoseArray& bestGapPath,
    // geometry_msgs::msg::PoseArray& bestVirtualGapPath    
    int Planner::pickTraj(const std::vector<Trajectory> & gapTrajs) 
    {
        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "int Planner::pickTraj started");

        boost::mutex::scoped_lock gapset(gapMutex_);

        // // RCLCPP_INFO_STREAM(node_->get_logger(),  "gapTrajs size:, " << gapTrajs.size());

        if (gapTrajs.size() == 0) 
        {
            RCLCPP_WARN_STREAM(node_->get_logger(),  "No traj synthesized");
            // bestGapPath = geometry_msgs::msg::PoseArray();
            // bestVirtualGapPath = geometry_msgs::msg::PoseArray();
            return -1;
        }

        // if (gapTrajs.size() != pathPoseCosts.size() ||
        //     gapTrajs.size() != pathTerminalPoseCosts.size())
        // {
        //     RCLCPP_FATAL_STREAM(node_->get_logger(), "pickTraj size mismatch: gapTrajs = " << gapTrajs.size() << " != pathPoseCosts =" << pathPoseCosts.size() << 
        //                      " != pathTerminalPoseCosts = " << pathTerminalPoseCosts.size());
        //     bestGapPath = geometry_msgs::msg::PoseArray();
        //     bestVirtualGapPath = geometry_msgs::msg::PoseArray();
        //     return;
        // }

        std::vector<float> gapTrajCosts(gapTrajs.size());
        
        try 
        {
            // if (omp_get_dynamic()) 
            //     omp_set_dynamic(0);
            
            for (size_t i = 0; i < gapTrajCosts.size(); i++) 
            {
                gapTrajCosts.at(i) = gapTrajs.at(i).getTerminalPoseCost() + gapTrajs.at(i).getAveragePosewiseCost();
                // float averagedPoseCost = std::accumulate(pathPoseCosts.at(i).begin(), 
                //                                             pathPoseCosts.at(i).end(), float(0)) / (pathPoseCosts.at(i).size() + eps);
                // gapTrajCosts.at(i) = pathTerminalPoseCosts.at(i) + averagedPoseCost;
                // gapTrajCosts.at(i) = gapTrajs.at(i).poses.size() == 0 ? -std::numeric_limits<float>::infinity() : gapTrajCosts.at(i);
                // RCLCPP_DEBUG_STREAM(node_->get_logger(), "Cost: " << gapTrajCosts.at(i));
            }
        } catch (...) 
        {
            RCLCPP_FATAL_STREAM(node_->get_logger(), "pickTraj");
        }

        auto lowestCostTrajIter = std::min_element(gapTrajCosts.begin(), gapTrajCosts.end());
        int candidateLowestCostTrajIdx = std::distance(gapTrajCosts.begin(), lowestCostTrajIter);

        if (gapTrajCosts.at(candidateLowestCostTrajIdx) == std::numeric_limits<float>::infinity()) 
        {
            RCLCPP_WARN_STREAM(node_->get_logger(),  "No executable trajectory, values: ");
            for (const float & gapTrajCost : gapTrajCosts) 
            {
                // RCLCPP_INFO_STREAM(node_->get_logger(),  "Cost: " << gapTrajCost);
            }
            // RCLCPP_INFO_STREAM(node_->get_logger(),  "------------------");
        }

        // bestGapPath = gapTrajs.at(idx);
        // bestVirtualGapPath = virtualGapPaths.at(idx);
        // RCLCPP_INFO_STREAM(node_->get_logger(),  "Picked [" << candidateLowestCostTrajIdx << "] traj" );

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "int Planner::pickTraj ended");

        return candidateLowestCostTrajIdx;
    }

    Trajectory Planner::changeTrajectoryHelper(Trajectory & incomingTraj,
                                                const bool & switchToIncoming)
    {
        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "Trajectory Planner::changeTrajectoryHelper started");

        trajectoryChangeCount_++;

        if (switchToIncoming)
        {
            incomingTraj.setOrientedPathOdomFrame(gapTrajGenerator_->transformPath(incomingTraj.getOrientedPathRbtFrame(), rbt2odom_));
            trajVisualizer_->drawCurrentTrajectory(incomingTraj);
            trajVisualizer_->drawTrajectorySwitchCount(trajectoryChangeCount_, incomingTraj);
            setCurrentTraj(incomingTraj);         
            
            return incomingTraj;
        } else
        {
            Trajectory emptyTraj = Trajectory();
            emptyTraj.setRbtFrameDefaultHeader(incomingTraj.getPathRbtFrame().header);
            trajVisualizer_->drawCurrentTrajectory(emptyTraj);
            trajVisualizer_->drawTrajectorySwitchCount(trajectoryChangeCount_, emptyTraj);
            setCurrentTraj(emptyTraj);

            return emptyTraj;
        }

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "Trajectory Planner::changeTrajectoryHelper ended");
    }

    // , 
    // geometry_msgs::msg::PoseArray& virtual_currTraj
    Trajectory Planner::compareToCurrentTraj(Trajectory & incomingTraj) 
    {
        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "Trajectory Planner::compareToCurrentTraj started");

        boost::mutex::scoped_lock gapset(gapMutex_);

        try 
        {
            Trajectory chosenTraj = Trajectory();

            Trajectory currTraj = getCurrentTraj();

            //////////////////////////////////////////////////////
            // Transform incoming traj into current robot frame //
            //        to score against the current scan         //
            //////////////////////////////////////////////////////
            // RCLCPP_INFO_STREAM(node_->get_logger(),  "    evaluating incoming trajectory");

            // geometry_msgs::msg::PoseArray orientedIncomingPathRbtFrame = 
            // RCLCPP_INFO_STREAM(node_->get_logger(),  "   orient decayed path 2");
            // gapTrajGenerator_->getOrientDecayedPath(incomingTraj);
            // std::vector<float> incomingPathPoseCosts;
            // float incomingPathTerminalCost;
            trajEvaluator_->evaluateTrajectory(incomingTraj); // orientedIncomingPathRbtFrame, incomingPathPoseCosts, incomingPathTerminalCost);
            
            // RCLCPP_INFO_STREAM(node_->get_logger(),  "    length of incoming path: " << incomingTraj.size());

            // float averagedIncomingPoseCost = std::accumulate(incomingPathPoseCosts.begin(), 
            //                                                     incomingPathPoseCosts.end(), float(0)) / (incomingPathPoseCosts.size() + eps);
            float incomingTrajCost = incomingTraj.getTerminalPoseCost() + incomingTraj.getAveragePosewiseCost(); // incomingPathTerminalCost + averagedIncomingPoseCost;

            ///////////////////////////////////////////////////////////////////////
            //  Evaluate the incoming path to determine if we can switch onto it //
            ///////////////////////////////////////////////////////////////////////
            std::string incomingTrajStatus = "incoming path is safe to switch onto";
            bool ableToSwitchToIncomingPath = true;

            if (incomingTraj.size() == 0)
            {
                incomingTrajStatus = "incoming path is empty";
                ableToSwitchToIncomingPath = false;
            } else if (incomingTrajCost == std::numeric_limits<float>::infinity()) 
            {
                incomingTrajStatus = "incoming path is not feasible";
                ableToSwitchToIncomingPath = false;
            }
     
            ///////////////////////////////////////////////////////////////////////////////////
            //  Enact a trajectory switch if the currently executing path is empty (size: 0) //
            ///////////////////////////////////////////////////////////////////////////////////

            if (currTraj.size() == 0) 
            {
                // RCLCPP_INFO_STREAM(node_->get_logger(),  "        trajectory change " << trajectoryChangeCount_ <<  
                                                    // ": current path is of length zero, " << incomingTrajStatus);   

                return changeTrajectoryHelper(incomingTraj, ableToSwitchToIncomingPath);               
            }

            // Transform current traj into most recent robot frame to score against the current scan

            // Update the current trajectory
            geometry_msgs::msg::PoseArray updatedCurrentPathRobotFrame = gapTrajGenerator_->transformPath(currTraj.getPathOdomFrame(), odom2rbt_);
            Trajectory updatedCurrentTraj(updatedCurrentPathRobotFrame);
            updatedCurrentTraj.setPathOdomFrame(currTraj.getPathOdomFrame());
            gapTrajGenerator_->getOrientDecayedPath(updatedCurrentTraj);

            int updatedCurrentPathPoseIdx = getClosestTrajectoryPoseIdx(updatedCurrentPathRobotFrame); // updatedCurrentPathRobotFrame
            geometry_msgs::msg::PoseArray reducedCurrentPathRobotFrame = updatedCurrentPathRobotFrame;
            reducedCurrentPathRobotFrame.poses = std::vector<geometry_msgs::msg::Pose>(updatedCurrentPathRobotFrame.poses.begin() + updatedCurrentPathPoseIdx, updatedCurrentPathRobotFrame.poses.end());
            
            Trajectory reducedCurrentTraj(reducedCurrentPathRobotFrame);

            if (reducedCurrentTraj.size() < 2) 
            {
                // RCLCPP_INFO_STREAM(node_->get_logger(),  "        trajectory change " << trajectoryChangeCount_ <<  
                                                                // ": old path length less than 2, " << incomingTrajStatus);

                return changeTrajectoryHelper(incomingTraj, ableToSwitchToIncomingPath);
            }
            
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //  Compare the costs of the incoming trajectory with the cost of the current trajectory to see if we need to switch //
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            // geometry_msgs::msg::PoseArray virtual_curr_score_path = getOrientDecayedPath(reducedCurrentPathRobotFrame);
            // RCLCPP_INFO_STREAM(node_->get_logger(),  "   orient decayed path 3");            
            gapTrajGenerator_->getOrientDecayedPath(reducedCurrentTraj);

            trajEvaluator_->evaluateTrajectory(reducedCurrentTraj); // reducedCurrentPathRobotFrame, reducedCurrentPathPoseCosts, reducedCurrentPathTerminalCost);

            float reducedCurrTrajCost = reducedCurrentTraj.getTerminalPoseCost() + reducedCurrentTraj.getAveragePosewiseCost(); // reducedCurrentPathTerminalCost + currAveragedPoseCost;

            // RCLCPP_INFO_STREAM(node_->get_logger(),  "Reduced curr score: " << reducedCurrTrajCost << ", incom Cost:" << incomingTrajCost);

            if (reducedCurrTrajCost == std::numeric_limits<float>::infinity())
            {
                // RCLCPP_INFO_STREAM(node_->get_logger(),  "        trajectory change " << trajectoryChangeCount_ << 
                                                            // ": current trajectory is of cost infinity," << incomingTrajStatus);

                return changeTrajectoryHelper(incomingTraj, ableToSwitchToIncomingPath);               
            }

            if (incomingTrajCost < reducedCurrTrajCost)  
            {
                // RCLCPP_INFO_STREAM(node_->get_logger(),  "        trajectory change " << trajectoryChangeCount_ << 
                                                            // ": incoming trajectory is lower score");
                return changeTrajectoryHelper(incomingTraj, ableToSwitchToIncomingPath);
            }

            // RCLCPP_INFO_STREAM(node_->get_logger(),  "   orient decayed path 4");
            // gapTrajGenerator_->getOrientDecayedPath(currTraj);
            // currTraj.setOrientedPathOdomFrame(gapTrajGenerator_->transformPath(currTraj.getOrientedPathRbtFrame(), rbt2odom_));

            // trajectory_pub.publish(currTraj);
            trajVisualizer_->drawCurrentTrajectory(reducedCurrentTraj);

            return updatedCurrentTraj;
        } catch (...) 
        {
            RCLCPP_FATAL_STREAM(node_->get_logger(), "compareToCurrentTraj");
        }

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "Trajectory Planner::compareToCurrentTraj ended");

        return Trajectory();
    }

    // CollisionResults Planner::checkCollision(const Trajectory & trajectory)
    // {
    //     // Convert the trajectory from odom to base frame
    //     geometry_msgs::msg::PoseArray orientedPathRbtFrame = gapTrajGenerator_->transformPath(trajectory.getOrientedPathOdomFrame(), odom2rbt_);
    //     geometry_msgs::msg::Pose curr_pose;
    //     curr_pose.orientation.w = 1;

    //     // TrajPlan orig_ref = trajController_->trajGen(orientedPathRbtFrame);
    //     orientedPathRbtFrame.header.frame_id = cfg_.robot_frame_id;
    //     int targetTrajectoryPoseIdx_ = trajController_->extractTargetPoseIdx(curr_pose, orientedPathRbtFrame);

    //     pips_trajectory_msgs::trajectory_points local_traj;
    //     local_traj.header.frame_id = cfg_.robot_frame_id;
    //     for (int i = targetTrajectoryPoseIdx_; i < orientedPathRbtFrame.poses.size(); i++)
    //     {
    //         pips_trajectory_msgs::trajectory_point pt;
    //         pt.x = orientedPathRbtFrame.poses[i].position.x;
    //         pt.y = orientedPathRbtFrame.poses[i].position.y;

    //         // // RCLCPP_INFO_STREAM(node_->get_logger(),  pt.x << " " << pt.y);

    //         tf2::Quaternion quat_tf;
    //         tf2::convert(orientedPathRbtFrame.poses[i].orientation, quat_tf);
    //         // tf2::Matrix3x3 m(quat_tf);
    //         // float roll, pitch, yaw;
    //         // m.getRPY(roll, pitch, yaw);
    //         float yaw = quaternionToYaw(quat_tf);
    //         pt.theta = yaw;

    //         local_traj.points.push_back(pt);
    //     }
        

    //     int collision_ind = traj_tester_->evaluateTrajectory(local_traj);

    //     CollisionResults cc_results(collision_ind, local_traj);

    //     return cc_results;
    // }

    int Planner::getClosestTrajectoryPoseIdx(const geometry_msgs::msg::PoseArray & currTrajRbtFrame) 
    {

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "int Planner::getClosestTrajectoryPoseIdx started");

        std::vector<float> pathPoseNorms(currTrajRbtFrame.poses.size());
        // // RCLCPP_INFO_STREAM(node_->get_logger(),  "Ref_pose length: " << ref_pose.poses.size());
        for (size_t i = 0; i < pathPoseNorms.size(); i++) // i will always be positive, so this is fine
        {
            pathPoseNorms.at(i) = sqrt(pow(currTrajRbtFrame.poses.at(i).position.x, 2) + 
                                    pow(currTrajRbtFrame.poses.at(i).position.y, 2));
        }

        auto minPoseNormIter = std::min_element(pathPoseNorms.begin(), pathPoseNorms.end());
        int minPoseNormIdx = std::distance(pathPoseNorms.begin(), minPoseNormIter) + 1;

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "int Planner::getClosestTrajectoryPoseIdx ended");

        return std::min(minPoseNormIdx, int(currTrajRbtFrame.poses.size() - 1));
    }

    void Planner::setCurrentTraj(const Trajectory & currTraj) 
    {
        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "void Planner::setCurrentTraj started");

        currTraj_ = currTraj;

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "void Planner::setCurrentTraj ended");

        return;
    }

    Trajectory Planner::getCurrentTraj() 
    {
        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "Trajectory Planner::getCurrentTraj started");

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "Trajectory Planner::getCurrentTraj ended");

        return currTraj_;
    }

    void Planner::reset()
    {

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "void Planner::reset started");

        // currSimpGaps_.clear();
        setCurrentTraj(Trajectory());

        // RCLCPP_INFO_STREAM(node_->get_logger(),  "cmdVelBuffer size: " << cmdVelBuffer.size());
        cmdVelBuffer.clear();
        // RCLCPP_INFO_STREAM(node_->get_logger(),  "cmdVelBuffer size after clear: " << cmdVelBuffer.size() << ", is full: " << cmdVelBuffer.capacity());
        
        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "void Planner::reset ended");

        return;
    }

    geometry_msgs::msg::Twist Planner::ctrlGeneration(const Trajectory & traj) 
    {

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "geometry_msgs::msg::Twist Planner::ctrlGeneration started");

        // RCLCPP_INFO_STREAM(node_->get_logger(), "[ctrlGeneration()]");

        timeKeeper_->startTimer(CONTROL);

        geometry_msgs::msg::Twist rawCmdVel = geometry_msgs::msg::Twist();
        geometry_msgs::msg::Twist cmdVel = rawCmdVel;

        try
        {        
            if (!haveTFs_)
                return cmdVel;

            geometry_msgs::msg::PoseArray pathOdomFrame = traj.getPathOdomFrame();

            if (pathOdomFrame.poses.size() < 1)
            {
                RCLCPP_WARN_STREAM(node_->get_logger(),  "Available Execution Traj length: " << pathOdomFrame.poses.size() << " < 1");
                if (cfg_.planning.holonomic)
                {
                    rawCmdVel = trajController_->obstacleAvoidanceControlLaw();
                } else
                {
                    rawCmdVel = trajController_->obstacleAvoidanceControlLawNonHolonomic();
                }

                return rawCmdVel;
            }
    
            // Current Pose (Robot frame)
            geometry_msgs::msg::PoseStamped currPoseStRobotFrame;
            currPoseStRobotFrame.header.frame_id = cfg_.robot_frame_id;
            currPoseStRobotFrame.pose.orientation.w = 1;

            // Current Pose (Odom frame)
            geometry_msgs::msg::PoseStamped currPoseStampedOdomFrame;
            currPoseStampedOdomFrame.header.frame_id = cfg_.odom_frame_id;
            tf2::doTransform(currPoseStRobotFrame, currPoseStampedOdomFrame, rbt2odom_);
            geometry_msgs::msg::Pose currPoseOdomFrame = currPoseStampedOdomFrame.pose;

            // TrajPlan orig_ref = trajController_->trajGen(pathOdomFrame);
            int targetTrajectoryPoseIdx_ = trajController_->extractTargetPoseIdx(currPoseOdomFrame, pathOdomFrame);

            geometry_msgs::msg::Pose targetTrajectoryPoseOdomFrame = pathOdomFrame.poses.at(targetTrajectoryPoseIdx_);

            // sensor_msgs::msg::LaserScan stored_scan_msgs = *scan_.get();

            // timeKeeper_->startTimer(FEEDBACK);
            // geometry_msgs::msg::Twist cmd_vel = trajController_->controlLaw(currPoseOdomFrame, targetTrajectoryPoseOdomFrame, stored_scan_msgs, currPoseStRobotFrame);
            // timeKeeper_->stopTimer(FEEDBACK);


            if (cfg_.planning.holonomic)
            {
                timeKeeper_->startTimer(FEEDBACK);
                rawCmdVel = trajController_->controlLawHolonomic(currPoseOdomFrame, targetTrajectoryPoseOdomFrame);
                timeKeeper_->stopTimer(FEEDBACK);

                timeKeeper_->startTimer(PO);
                cmdVel = trajController_->processCmdVelHolonomic(rawCmdVel, rbtPoseInSensorFrame_); 
                timeKeeper_->stopTimer(PO);
            } else
            {
                timeKeeper_->startTimer(FEEDBACK);
                rawCmdVel = trajController_->controlLawNonholonomic(currPoseOdomFrame, targetTrajectoryPoseOdomFrame);
                timeKeeper_->stopTimer(FEEDBACK);

                timeKeeper_->startTimer(PO);
                cmdVel = trajController_->processCmdVelNonholonomic(currPoseOdomFrame,
                                                                    targetTrajectoryPoseOdomFrame,
                                                                    rawCmdVel,
                                                                    rbtPoseInSensorFrame_); 
                timeKeeper_->stopTimer(FEEDBACK);
            }

        } catch (const std::exception &e)
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(), "Exception in ctrlGeneration: " << e.what());
        } catch (...)
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(), "Unknown exception in ctrlGeneration");
        }

        timeKeeper_->stopTimer(CONTROL);

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "geometry_msgs::msg::Twist Planner::ctrlGeneration ended");

        return cmdVel;        
    }

    // void Planner::rcfgCallback(qgConfig &config, uint32_t level)
    // {
    //     cfg_.reconfigure(config);
        
    //     // set_capacity destroys everything if different from original size, 
    //     // resize only if the new size is greater
    //     cmdVelBuffer.clear();
    //     cmdVelBuffer.set_capacity(cfg_.planning.halt_size);
    // }


    std::vector<Gap *> Planner::deepCopyCurrentSimplifiedGaps()
    {
        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "std::vector<Gap *> Planner::deepCopyCurrentSimplifiedGaps started");

        boost::mutex::scoped_lock gapset(gapMutex_);

        std::vector<Gap *> planningGaps;

        for (Gap * gap : currSimpGaps_)
        {
            assert(gap->getFrame() == cfg_.robot_frame_id);
            planningGaps.push_back(new Gap(*gap));
        }

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "std::vector<Gap *> Planner::deepCopyCurrentSimplifiedGaps ended");

        return planningGaps;
    }

    Trajectory Planner::runPlanningLoop() 
    {
        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "Trajectory Planner::runPlanningLoop started");

        currPlanTime_ = node_->get_clock()->now();
        // RCLCPP_INFO_STREAM(node_->get_logger(), "Current planning time: " << currPlanTime_.seconds() << "." << currPlanTime_.nanoseconds() << " seconds");
        rclcpp::Duration lastPlanTime = currPlanTime_ - lastPlanTime_;
        // RCLCPP_INFO_STREAM(node_->get_logger(), "Time since last plan: " << lastPlanTime.seconds() << "." << lastPlanTime.nanoseconds() << " seconds");

        if (!haveTFs_)
        {
            RCLCPP_WARN_STREAM(node_->get_logger(),  "Waiting for TFs to be available");
            // chosenTraj = Trajectory();
            return Trajectory();
        }

        if (!initialized_ || !hasLaserScan_ || !hasGlobalGoal_)
        {
            RCLCPP_WARN_STREAM(node_->get_logger(),  "Not ready to plan, initialized: " << initialized_ << ", laser scan: " << hasLaserScan_ << ", global goal: " << hasGlobalGoal_);
            // chosenTraj = Trajectory();
            return Trajectory();            
        }

        if (colliding_)
        {
            RCLCPP_WARN_STREAM(node_->get_logger(),  "In collision");
            // chosenTraj = Trajectory();
            return Trajectory();
        }

        timeKeeper_->startTimer(PLAN);

        trajVisualizer_->drawPlanningLoopIdx(timeKeeper_->getPlanningLoopCalls());

        ///////////////////////////////////////////////////////////////////////////////////////
        //                           IS GLOBAL GOAL REACHED?                                 //
        ///////////////////////////////////////////////////////////////////////////////////////

        isGoalReached();

        ///////////////////////////////////////////////////////////////////////////////////////
        //                              GRAB CURRENT GAPS                                    //
        ///////////////////////////////////////////////////////////////////////////////////////

        std::vector<Gap *> planningGaps = deepCopyCurrentSimplifiedGaps();

        int gapCount = planningGaps.size();
        if (gapCount == 0)
        {
            RCLCPP_WARN_STREAM(node_->get_logger(),  "No gaps found, planning loop will not continue.");
            // chosenTraj = Trajectory();
            return Trajectory();
        }

        // RCLCPP_INFO_STREAM(node_->get_logger(),  "Planning gaps:");
        for (int i = 0; i < gapCount; i++)
        {
            assert(planningGaps.at(i)->getFrame() == cfg_.robot_frame_id);

            Gap * gap = planningGaps.at(i);
            float leftX, leftY, rightX, rightY;
            gap->getLCartesian(leftX, leftY);
            gap->getRCartesian(rightX, rightY);
            // RCLCPP_INFO_STREAM(node_->get_logger(),  "Gap " << i);
            // RCLCPP_INFO_STREAM(node_->get_logger(),  "      Left polar: (" << gap->LIdx() << ", " << gap->LRange() << "), Right polar: (" << gap->RIdx() << ", " << gap->RRange() << ")");
            // RCLCPP_INFO_STREAM(node_->get_logger(),  "      Left cartesian: (" << leftX << ", " << leftY << "), Right cartesian: (" << rightX << ", " << rightY << ")");
        }

        //////////////////////////////////////////////////////////////////////////////////////
        //                              GAP MANIPULATION                                    //
        //////////////////////////////////////////////////////////////////////////////////////

        timeKeeper_->startTimer(GAP_MANIP);
        std::vector<Gap *> manipGaps = gapManipulate(planningGaps);
        timeKeeper_->stopTimer(GAP_MANIP);


        // RCLCPP_INFO_STREAM(node_->get_logger(),  "Manipulated gaps:");
        for (int i = 0; i < manipGaps.size(); i++)
        {
            assert(manipGaps.at(i)->getFrame() == cfg_.robot_frame_id);

            Gap * gap = manipGaps.at(i);
            float leftX, leftY, rightX, rightY;
            gap->getManipLCartesian(leftX, leftY);
            gap->getManipRCartesian(rightX, rightY);
            // RCLCPP_INFO_STREAM(node_->get_logger(),  "Gap " << i);
            // RCLCPP_INFO_STREAM(node_->get_logger(),  "      Left polar: (" << gap->manipLeftIdx() << ", " << gap->manipLeftRange() << "), Right polar: (" << gap->manipRightIdx() << ", " << gap->manipRightRange() << ")");
            // RCLCPP_INFO_STREAM(node_->get_logger(),  "      Left cartesian: (" << leftX << ", " << leftY << "), Right cartesian: (" << rightX << ", " << rightY << ")");
        }


        //////////////////////////////////////////////////////////////////////////////////////
        //                             GAP GOAL PLACEMENT                                   //
        //////////////////////////////////////////////////////////////////////////////////////

        timeKeeper_->startTimer(GAP_MANIP);
        gapGoalPlace(manipGaps);
        timeKeeper_->stopTimer(GAP_MANIP);

        goalVisualizer_->drawGapGoals(manipGaps);
        gapVisualizer_->drawManipGaps(manipGaps);

        //////////////////////////////////////////////////////////////////////////////////////
        //                          GAP TRAJECTORY GENERATION                               //
        //////////////////////////////////////////////////////////////////////////////////////

        timeKeeper_->startTimer(GAP_TRAJ_GEN);
        // std::vector<geometry_msgs::msg::PoseArray> gapPaths;
        // std::vector<geometry_msgs::msg::PoseArray> virtualGapPaths;
        // std::vector<std::vector<float>> pathPoseCosts; 
        // std::vector<float> pathTerminalPoseCosts;    
        std::vector<Trajectory> gapTrajs;    
        generateGapTrajectories(manipGaps, gapTrajs); // gapPaths, virtualGapPaths, pathPoseCosts, pathTerminalPoseCosts
        timeKeeper_->stopTimer(GAP_TRAJ_GEN);

        //////////////////////////////////////////////////////////////////////////////////////
        //                                PICK TRAJECTORY                                   //
        //////////////////////////////////////////////////////////////////////////////////////

        timeKeeper_->startTimer(TRAJ_PICK);
        // geometry_msgs::msg::PoseArray bestGapPath;
        // geometry_msgs::msg::PoseArray bestVirtualGapPath;
        // gapPaths, virtualGapPaths, 
        // pathPoseCosts, pathTerminalPoseCosts, 
        // bestGapPath, bestVirtualGapPath
        int trajIdx = pickTraj(gapTrajs);
        // virtual_orient_traj_pub.publish(bestVirtualGapPath);
        timeKeeper_->stopTimer(TRAJ_PICK);

        //////////////////////////////////////////////////////////////////////////////////////
        //                              GAP TRAJECTORY COMPARISON                           //
        //////////////////////////////////////////////////////////////////////////////////////

        timeKeeper_->startTimer(TRAJ_COMP);
        // geometry_msgs::msg::PoseArray chosenVirtualGapPath;
        Trajectory chosenTraj = compareToCurrentTraj(gapTrajs.at(trajIdx));
        timeKeeper_->stopTimer(TRAJ_COMP);

        /////////////////////////////////////////////////////////////////////////////////////
        //                                 COLLISION CHECKING                              //
        /////////////////////////////////////////////////////////////////////////////////////

        // timeKeeper_->startTimer(COLL_CHECK);
        // CollisionResults cc_results;
        // if (cfg_.collision_checker.collision_checker_enable)
        // {
        //     ros::WallTime start = ros::WallTime::now();

        //     // cc_results = checkCollision(chosenGapPath);
        //     // cc_results = checkCollision(chosenTraj); // chosenVirtualGapPath

        //     // RCLCPP_INFO_STREAM(node_->get_logger(),  "Current trajectory collision checked in " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
        //     int cc_ite_min = 10;
        //     float cc_itc_ratio = 0.2;

        //     if(cc_results.collision_idx_ >= 0 && float(cc_results.collision_idx_) / cc_results.local_traj_.points.size() <= cc_itc_ratio)
        //     {
        //         RCLCPP_WARN_STREAM(node_->get_logger(),  "Current trajectory collides! " << cc_results.collision_idx_ << " " << cc_results.local_traj_.points.size());
        //         setCurrentTraj(Trajectory()); // geometry_msgs::msg::PoseArray()
        //     }
        // }
        // timeKeeper_->stopTimer(COLL_CHECK);

        // delete set of planning gaps
        for (Gap * planningGap : planningGaps)
            delete planningGap;

        timeKeeper_->stopTimer(PLAN);
        timeKeeper_->computeAverageNumberGaps(gapCount);      
        
        lastPlanTime_ = currPlanTime_;

        // function ends

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "Trajectory Planner::runPlanningLoop started");

        return chosenTraj;
    }

    bool Planner::recordAndCheckVel(const geometry_msgs::msg::TwistStamped & cmd_vel) 
    {
        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "bool Planner::recordAndCheckVel started");

        float val = std::abs(cmd_vel.twist.linear.x) + std::abs(cmd_vel.twist.linear.y) + std::abs(cmd_vel.twist.angular.z);
        cmdVelBuffer.push_back(val);
        float cum_vel_sum = std::accumulate(cmdVelBuffer.begin(), cmdVelBuffer.end(), float(0));
        bool ret_val = cum_vel_sum > 1.0 || !cmdVelBuffer.full();
        if (!ret_val && !cfg_.man.man_ctrl) {
            RCLCPP_FATAL_STREAM(node_->get_logger(), "--------------------------Planning Failed--------------------------");
            reset();
        }

        // print initalize message
        RCLCPP_INFO_STREAM(node_->get_logger(),  "bool Planner::recordAndCheckVel ended");

        return ret_val || cfg_.man.man_ctrl;
    }

}