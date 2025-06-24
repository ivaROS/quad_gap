#include <quad_gap/Planner.h>
#include "tf/transform_datatypes.h"
#include <tf/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

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

        // if (robot_geo_proc_)
        //     delete robot_geo_proc_;

        // if (robot_geo_storage_)
        //     delete robot_geo_storage_;

        if (timeKeeper_)
            delete timeKeeper_;

        // if (tfListener_)
        //     delete tfListener_;
    }

    bool Planner::initialize(const std::string & name)
    {
        if (initialized_)
        {
            ROS_WARN("PotentialGap Planner already initalized");
            return true;
        }

        ros::NodeHandle unh("~/" + name);

        // pnh = unh;
        pnh = ros::NodeHandle(unh.getNamespace() + "/cc");

        // Config Setup
        cfg_.loadRosParamFromNodeHandle(name);

        // Load precomputed robot geo
        std::string file_name = ros::package::getPath("quad_gap") + "/config/box_1_geometry.yaml";
        unh.getParam("file_name", file_name);
        unh.setParam("file_name", file_name);

        RobotShape robot_shape = static_cast<RobotShape>(cfg_.rbt.shape_id);
        if (robot_shape == RobotShape::circle)
            cfg_.rbt.width = 0;

        Robot robot(robot_shape, cfg_.rbt.length, cfg_.rbt.width, cfg_.rbt.avg_lin_speed, cfg_.rbt.avg_rot_speed);

        if (cfg_.rbt.use_geo_storage)
            robot_geo_storage_ = RobotGeometryStorage(file_name);
        else
            robot_geo_proc_ = RobotGeometryProcessor(robot, cfg_.planning.decay_factor);
        
        // Visualization Setup
        trajectory_pub = nh.advertise<geometry_msgs::PoseArray>("qg_traj", 10);

        transformed_laser_pub = nh.advertise<sensor_msgs::LaserScan>("transformed_laserscan", 5);
        // virtual_orient_traj_pub = nh.advertise<geometry_msgs::PoseArray>("picked_virtual_traj", 10);

        // TF Lookup setup
        tfBuffer = std::make_shared<tf2_ros::Buffer>();
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
        initialized_ = true;

        tfSub_ = nh.subscribe("/tf", 10, &Planner::tfCB, this);
        laserSub_ = nh.subscribe(cfg_.scan_topic, 100, &Planner::laserScanCB, this);
        poseSub_ = nh.subscribe(cfg_.odom_topic, 10, &Planner::poseCB, this);        

        gapDetector_ = new GapDetector(cfg_, robot_geo_proc_);
        globalPlanManager_ = new GlobalPlanManager(cfg_); // , robot_geo_proc_
        trajEvaluator_ = new TrajectoryEvaluator(cfg_, robot_geo_proc_);
        gapTrajGenerator_ = new GapTrajGenerator(cfg_, robot_geo_proc_);
        gapGoalPlacer_ = new GapGoalPlacer(cfg_, robot_geo_proc_);
        gapVisualizer_ = new GapVisualizer(nh, cfg_);
        trajVisualizer_ = new TrajectoryVisualizer(nh, cfg_);
        goalVisualizer_ = new GoalVisualizer(nh, cfg_);
        gapManipulator_ = new GapManipulator(cfg_, robot_geo_proc_);
        trajController_ = new TrajectoryController(nh, cfg_);
        timeKeeper_ = new TimeKeeper();

        map2rbt_.transform.rotation.w = 1;
        rbt2map_.transform.rotation.w = 1;
        odom2rbt_.transform.rotation.w = 1;
        rbt2odom_.transform.rotation.w = 1;
        rbtPoseRbtFrame_.pose.orientation.w = 1;
        rbtPoseRbtFrame_.header.frame_id = cfg_.robot_frame_id;

        // Set collision checker
        if(!cfg_.collision_checker.collision_checker_enable)
        {
            ROS_WARN_STREAM("Collision checking is disabled.");
            // return;
        }

        if (cfg_.collision_checker.cc_type == CC_DEPTH)
        {
            ROS_INFO_STREAM_NAMED("Planner", "New cc type = depth");
            cc_wrapper_ = std::make_shared<pips_trajectory_testing::DepthImageCCWrapper>(nh, pnh, tf2_utils::TransformManager(tfBuffer, tfListener));
        } else if(cfg_.collision_checker.cc_type == CC_DEPTH_EGO)
        {
            ROS_INFO_STREAM_NAMED("Planner", "New cc type = depth ego");
            cc_wrapper_ = std::make_shared<pips_egocylindrical::EgocylindricalRangeImageCCWrapper>(nh, pnh, tf2_utils::TransformManager(tfBuffer, tfListener));
        } else if(cfg_.collision_checker.cc_type == CC_EGOCIRCLE)
        {
            ROS_INFO_STREAM_NAMED("Planner", "New cc type = egocircle");
            cc_wrapper_ = std::make_shared<pips_egocircle::EgoCircleCCWrapper>(nh, pnh, tf2_utils::TransformManager(tfBuffer, tfListener));
        }

        traj_tester_ = std::make_shared<TurtlebotGenAndTest>(nh, pnh);
        
        cc_wrapper_->init();
        cc_wrapper_->autoUpdate();

        traj_tester_->init();
        traj_tester_->setCollisionChecker(cc_wrapper_->getCC());
        
        // cc_type_ = cfg_.collision_checker.cc_type;

        cmdVelBuffer.set_capacity(cfg_.planning.halt_size);
        return true;
    }

    bool Planner::isGoalReached()
    {
        // Linear distance
        float globalGoalXDiff = globalGoalOdomFrame_.pose.position.x - rbtPoseOdomFrame_.pose.position.x;
        float globalGoalYDiff = globalGoalOdomFrame_.pose.position.y - rbtPoseOdomFrame_.pose.position.y;
        float globalGoalLinDist = sqrt(pow(globalGoalXDiff, 2) + pow(globalGoalYDiff, 2));

        // Angular distance
        float globalGoalOrientation = quaternionToYaw(globalGoalOdomFrame_.pose.orientation);
        float rbtPoseOrientation = quaternionToYaw(globalGoalOdomFrame_.pose.orientation);
        float globalGoalAngDist = normalize_theta(globalGoalOrientation - rbtPoseOrientation);
        
        reachedGlobalGoal_ = globalGoalLinDist < cfg_.goal.xy_global_goal_tolerance &&
                             globalGoalAngDist < cfg_.goal.yaw_global_goal_tolerance;
        
        if (reachedGlobalGoal_)
            ROS_INFO_STREAM_NAMED("Planner", "[Reset] Goal Reached");
        // else
        //     ROS_INFO_STREAM_NAMED("Planner", "Distance from goal: " << globalGoalDist << 
        //                                      ", Goal tolerance: " << cfg_.goal.xy_global_goal_tolerance);

        return reachedGlobalGoal_;
    }    

    boost::shared_ptr<sensor_msgs::LaserScan const> Planner::transformLaserToRbt(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        sensor_msgs::LaserScan transformed_laser = *msg;
        // transformed_laser.header = msg->header;
        // transformed_laser.header.frame_id = cfg_.robot_frame_id;
        // transformed_laser.angle_min = msg->angle_min;
        // transformed_laser.angle_max = msg->angle_max;
        // transformed_laser.angle_increment = msg->angle_increment;
        // transformed_laser.time_increment = msg->time_increment;
        // transformed_laser.scan_time = msg->scan_time;
        // transformed_laser.range_min = msg->range_min;
        // transformed_laser.range_max = msg->range_max;
        // transformed_laser.intensities = msg->intensities;

        std::vector<float> ranges(msg->ranges.size(), msg->range_max);
        transformed_laser.ranges = ranges;

        for (size_t i = 0; i < msg->ranges.size(); i++)
        {
            float orig_range = msg->ranges[i];
            float orig_ang = i * msg->angle_increment + msg->angle_min;
            orig_ang = orig_ang <= msg->angle_max ? orig_ang : msg->angle_max;

            geometry_msgs::PointStamped orig_pt, transformed_pt;
            orig_pt.header = msg->header;
            orig_pt.point.x = orig_range * cos(orig_ang);
            orig_pt.point.y = orig_range * sin(orig_ang);
            
            geometry_msgs::TransformStamped trans = tfBuffer->lookupTransform(cfg_.robot_frame_id, cfg_.sensor_frame_id, ros::Time(0));
            tf2::doTransform(orig_pt, transformed_pt, trans);
            // ROS_INFO_STREAM(cfg_.sensor_frame_id << " " << orig_pt.header.frame_id << " " << transformed_pt.header.frame_id);

            float transformed_range = sqrt(pow(transformed_pt.point.x, 2) + pow(transformed_pt.point.y, 2));
            float transformed_ang = std::atan2(transformed_pt.point.y, transformed_pt.point.x);
            int idx = (int) round((transformed_ang - msg->angle_min) / msg->angle_increment);
            idx = idx < msg->ranges.size() ? idx : (msg->ranges.size() - 1);
            idx = idx >= 0 ? idx : 0;

            if (transformed_range < transformed_laser.ranges[idx])
                transformed_laser.ranges[idx] = transformed_range;
        }

        return boost::make_shared<sensor_msgs::LaserScan const>(transformed_laser);
    }

    void Planner::laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan> scan)
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        ROS_INFO_STREAM_NAMED("Scan", "[laserScanCB()]");

        timeKeeper_->startTimer(SCAN);

        /////////////////////////////////////
        //////// SCAN PRE-PROCESSING ////////
        /////////////////////////////////////

        gapDetector_->preprocessScan(scan);

        scan_ = transformLaserToRbt(scan);
        transformed_laser_pub.publish(scan_);

        float minScanDist = *std::min_element(scan_->ranges.begin(), scan_->ranges.end());

        if (minScanDist < cfg_.rbt.r_inscr)
        {
            ROS_INFO_STREAM_NAMED("Scan", "       in collision!");
            colliding_ = true;
            return;
        } else
        {
            colliding_ = false;
        }

        cfg_.updateParamFromScan(scan_);

        ///////////////////////////////
        //////// GAP DETECTION ////////
        ///////////////////////////////

        timeKeeper_->startTimer(GAP_DET);
        currRawGaps_ = gapDetector_->gapDetection(scan);
        timeKeeper_->stopTimer(GAP_DET);

        gapVisualizer_->drawGaps(currRawGaps_, std::string("raw"));

        ////////////////////////////////////
        //////// GAP SIMPLIFICATION ////////
        ////////////////////////////////////

        timeKeeper_->startTimer(GAP_SIMP);
        currSimpGaps_ = gapDetector_->gapSimplification(currRawGaps_);
        timeKeeper_->stopTimer(GAP_SIMP);

        gapVisualizer_->drawGaps(currSimpGaps_, std::string("simp"));

        // ROS_INFO_STREAM("currSimpGaps_ count:" << currSimpGaps_.size());

        hasLaserScan_ = true;

        // update current scan for proper classes
        updateEgoCircle();

        if (hasGlobalGoal_)
        {
            // update global path local waypoint according to new scan
            globalPlanManager_->generateGlobalPathLocalWaypoint(map2rbt_);
            geometry_msgs::PoseStamped globalPathLocalWaypointOdomFrame = globalPlanManager_->getGlobalPathLocalWaypointOdomFrame(rbt2odom_);
            goalVisualizer_->drawGlobalPathLocalWaypoint(globalPathLocalWaypointOdomFrame);
            goalVisualizer_->drawGlobalGoal(globalGoalOdomFrame_);
            trajEvaluator_->transformGlobalPathLocalWaypointToRbtFrame(globalPathLocalWaypointOdomFrame, odom2rbt_);
        }  

        timeKeeper_->stopTimer(SCAN);
    }

    void Planner::updateEgoCircle()
    {
        // If no global plan, the local goal finding won't execute.
        globalPlanManager_->updateEgoCircle(scan_);
        trajEvaluator_->updateEgoCircle(scan_);

        gapManipulator_->updateEgoCircle(scan_);
        gapGoalPlacer_->updateEgoCircle(scan_);
        trajController_->updateEgoCircle(scan_);
    }

    void Planner::poseCB(const nav_msgs::Odometry::ConstPtr& rbtOdomMsg)
    {
        // ROS_INFO_STREAM_NAMED("Planner", "[poseCB()]");
        // ROS_INFO_STREAM("[poseCB()]");

        if (!haveTFs_)
            return;

        //////////////////////////
        //         POSE         //
        //////////////////////////       

        // Transform the msg to odom frame
        if (rbtOdomMsg->header.frame_id != cfg_.odom_frame_id)
        {
            ROS_WARN_STREAM("Odom msg header frame (for ego-robot pose) " << rbtOdomMsg->header.frame_id << " not same as cfg_ odom frame:" << cfg_.odom_frame_id);

            geometry_msgs::TransformStamped origFrame2OdomFrame = tfBuffer->lookupTransform(cfg_.odom_frame_id, 
                                                                                            rbtOdomMsg->header.frame_id, 
                                                                                            ros::Time(0));

            geometry_msgs::PoseStamped poseIn, poseOut;
            poseIn.header = rbtOdomMsg->header;
            poseIn.pose = rbtOdomMsg->pose.pose;

            tf2::doTransform(poseIn, poseOut, origFrame2OdomFrame);
            rbtPoseOdomFrame_ = poseOut;
        } else
        {
            rbtPoseOdomFrame_.header = rbtOdomMsg->header;
            rbtPoseOdomFrame_.pose = rbtOdomMsg->pose.pose;
        }

        ///////////////////////////
        //       VELOCITY        //
        ///////////////////////////

        // if (rbtOdomMsg->child_frame_id != cfg_.robot_frame_id)
        // {

        ROS_WARN_STREAM("Odom msg child frame (for ego-robot velocity) " << rbtOdomMsg->child_frame_id << " not same as cfg_ rbt frame:" << cfg_.robot_frame_id);

        geometry_msgs::Vector3Stamped velIn, velOut;
        velIn.header = rbtOdomMsg->header; // TODO: make sure this is correct frame
        velIn.vector = rbtOdomMsg->twist.twist.linear;

        geometry_msgs::TransformStamped origFrame2RbtFrame = tfBuffer->lookupTransform(cfg_.robot_frame_id,
                                                                                        // rbtOdomMsg->child_frame_id, 
                                                                                        rbtOdomMsg->header.frame_id, 
                                                                                        ros::Time(0)); 
                                                                                    
        tf2::doTransform(velIn, velOut, origFrame2RbtFrame);

        rbtVelRbtFrame_.header = velOut.header;
        rbtVelRbtFrame_.twist.linear = velOut.vector;
        rbtVelRbtFrame_.twist.angular = rbtOdomMsg->twist.twist.angular; // z is same between frames

        // } else
        // {

        // }
    }

    bool Planner::setPlan(const std::vector<geometry_msgs::PoseStamped> &incomingGlobalPlan)
    {
        ROS_INFO_STREAM_NAMED("Planner", "[setPlan()]");

        if (incomingGlobalPlan.size() == 0) 
            return true;

        if (!haveTFs_)
            return false;        

        ////////////////////////////////////
        // CHECK IF GLOBAL PLAN MAP FRAME //
        ////////////////////////////////////

        // plan should be in global frame
        std::vector<geometry_msgs::PoseStamped> globalPlanMapFrame;
        if(incomingGlobalPlan[0].header.frame_id != cfg_.map_frame_id)
        {
            geometry_msgs::TransformStamped other_to_global_trans = tfBuffer->lookupTransform(cfg_.map_frame_id, incomingGlobalPlan[0].header.frame_id, ros::Time(0));
            for (size_t i = 0; i < incomingGlobalPlan.size(); i++)
            {
                geometry_msgs::PoseStamped plan_pose = incomingGlobalPlan[i];
                geometry_msgs::PoseStamped out_pose;
                tf2::doTransform(plan_pose, out_pose, other_to_global_trans);
                out_pose.header.stamp = plan_pose.header.stamp;
                out_pose.header.frame_id = cfg_.map_frame_id;
                globalPlanMapFrame.push_back(out_pose);
            }
        }
        else
        {
            globalPlanMapFrame = incomingGlobalPlan;
        }
        
        geometry_msgs::PoseStamped globalGoalMapFrame = *std::prev(globalPlanMapFrame.end());
        // geometry_msgs::PoseStamped globalGoalOdomFrame;
        // Transform global goal to odom frame
        tf2::doTransform(globalGoalMapFrame, globalGoalOdomFrame_, map2odom_);

        // Store New Global Plan to Goal Selector
        globalPlanManager_->updateGlobalPathMapFrame(globalPlanMapFrame);
        
        trajVisualizer_->drawGlobalPlan(globalPlanManager_->getGlobalPathOdomFrame());

        // Find Local Goal
        globalPlanManager_->generateGlobalPathLocalWaypoint(map2rbt_);
        // return local goal (odom) frame
        geometry_msgs::PoseStamped newglobalPathLocalWaypointOdomFrame = globalPlanManager_->getGlobalPathLocalWaypointOdomFrame(rbt2odom_);

        // Plan New
        float diffX = globalPathLocalWaypointOdomFrame_.pose.position.x - newglobalPathLocalWaypointOdomFrame.pose.position.x;
        float diffY = globalPathLocalWaypointOdomFrame_.pose.position.y - newglobalPathLocalWaypointOdomFrame.pose.position.y;
        
        if (sqrt(pow(diffX, 2) + pow(diffY, 2)) > cfg_.goal.xy_waypoint_tolerance)
            globalPathLocalWaypointOdomFrame_ = newglobalPathLocalWaypointOdomFrame;

        // Set new local goal to trajectory arbiter
        trajEvaluator_->transformGlobalPathLocalWaypointToRbtFrame(globalPathLocalWaypointOdomFrame_, odom2rbt_);

        std::vector<geometry_msgs::PoseStamped> visibleGlobalPlanSnippetRobotFrame = globalPlanManager_->getVisibleGlobalPlanSnippetRobotFrame(map2rbt_);
        trajVisualizer_->drawRelevantGlobalPlanSnippet(visibleGlobalPlanSnippetRobotFrame);

        hasGlobalGoal_ = true;
        setReachedGlobalGoal(false);

        return true;
    }

    void Planner::tfCB(const tf2_msgs::TFMessage& msg)
    {
        // ROS_INFO_STREAM_NAMED("Planner", "[tfCB()]");

        try 
        {
            map2rbt_  = tfBuffer->lookupTransform(cfg_.robot_frame_id, cfg_.map_frame_id, ros::Time(0));
            rbt2map_  = tfBuffer->lookupTransform(cfg_.map_frame_id, cfg_.robot_frame_id, ros::Time(0));
            odom2rbt_ = tfBuffer->lookupTransform(cfg_.robot_frame_id, cfg_.odom_frame_id, ros::Time(0));
            rbt2odom_ = tfBuffer->lookupTransform(cfg_.odom_frame_id, cfg_.robot_frame_id, ros::Time(0));
            cam2odom_ = tfBuffer->lookupTransform(cfg_.odom_frame_id, cfg_.sensor_frame_id, ros::Time(0));
            odom2cam_ = tfBuffer->lookupTransform(cfg_.sensor_frame_id, cfg_.odom_frame_id, ros::Time(0));
            map2odom_ = tfBuffer->lookupTransform(cfg_.odom_frame_id, cfg_.map_frame_id, ros::Time(0));
            rbt2cam_ = tfBuffer->lookupTransform(cfg_.sensor_frame_id, cfg_.robot_frame_id, ros::Time(0));
            cam2rbt_ = tfBuffer->lookupTransform(cfg_.robot_frame_id, cfg_.sensor_frame_id, ros::Time(0));

            tf2::doTransform(rbtPoseRbtFrame_, rbtPoseCamFrame_, rbt2cam_);
        
            haveTFs_ = true;
        } catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.1).sleep();
            return;
        }
    }

    // [[deprecated("Use Proper trajectory scoring instead")]]
    // void Planner::vectorSelectGap(Gap & selected_gap)
    // {
    //     Gap result = trajEvaluator_->returnAndScoreGaps();
    //     selected_gap = result;
    //     return;
    // }

    std::vector<Gap *> Planner::gapManipulate(const std::vector<Gap *> & planningGaps) 
    {
        ROS_INFO_STREAM_NAMED("GapManipulator", "[manipulateGaps()]");

        boost::mutex::scoped_lock gapset(gapMutex_);
        std::vector<Gap *> manipGaps = planningGaps;

        // geometry_msgs::PoseStamped local_goal_sensor_frame;
        // tf2::doTransform(globalPlanManager_->rbtFrameLocalGoal(), local_goal_sensor_frame, rbt2cam_);
        geometry_msgs::PoseStamped local_goal_rbt_frame = globalPlanManager_->getGlobalPathLocalWaypointRobotFrame();
        try 
        {
            for (size_t i = 0; i < manipGaps.size(); i++)
            {
                gapManipulator_->reduceGap(manipGaps.at(i), local_goal_rbt_frame);
                gapManipulator_->convertRadialGap(manipGaps.at(i));
                gapManipulator_->radialExtendGap(manipGaps.at(i));
            }
        } catch(...) 
        {
            ROS_FATAL_STREAM("gapManipulate");
        }

        goalVisualizer_->drawGapGoals(manipGaps);
        gapVisualizer_->drawManipGaps(manipGaps);
        return manipGaps;
    }

    void Planner::gapGoalPlace(const std::vector<Gap *> & planningGaps) 
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        geometry_msgs::PoseStamped local_goal_rbt_frame = globalPlanManager_->getGlobalPathLocalWaypointRobotFrame();
        try 
        {
            for (size_t i = 0; i < planningGaps.size(); i++)
            {
                gapGoalPlacer_->setGapWaypoint(planningGaps.at(i), local_goal_rbt_frame);
            }
        } catch(...) 
        {
            ROS_FATAL_STREAM("gapGoalPlace");
        }            

    }

    void Planner::generateGapTrajectories(const std::vector<Gap *> & gaps, 
                                            std::vector<geometry_msgs::PoseArray>& gapPaths, 
                                            std::vector<geometry_msgs::PoseArray>& virtualGapPaths,
                                            std::vector<std::vector<float>> & pathPoseCosts,
                                            std::vector<float> & pathTerminalPoseCosts) 
    {
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "[generateGapTrajectories()]");

        boost::mutex::scoped_lock gapset(gapMutex_);

        gapPaths = std::vector<geometry_msgs::PoseArray> (gaps.size());
        virtualGapPaths = std::vector<geometry_msgs::PoseArray> (gaps.size());
        pathPoseCosts = std::vector<std::vector<float>>(gaps.size());
        pathTerminalPoseCosts = std::vector<float>(gaps.size());

        geometry_msgs::PoseStamped rbt_local_pose;
        rbt_local_pose.header.frame_id = cfg_.robot_frame_id;
        rbt_local_pose.header.stamp = rbt2odom_.header.stamp;
        rbt_local_pose.pose.orientation.w = 1;

        try 
        {
            for (size_t i = 0; i < gaps.size(); i++) 
            {
                ROS_INFO_STREAM_NAMED("Planner", "   Generating trajectory for gap " << i);

                // std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();        

                // Generate trajectory in robot frame.
                geometry_msgs::PoseArray tmp;
                if (cfg_.planning.use_bezier)
                {
                    tmp = gapTrajGenerator_->generateBezierTrajectory(gaps.at(i), rbtVelRbtFrame_, odom2rbt_);
                } else
                {
                    tmp = gapTrajGenerator_->generateTrajectory(gaps.at(i), rbt_local_pose);
                }

                // std::chrono::steady_clock::time_point gen_traj_time = std::chrono::steady_clock::now();
                
                ROS_INFO_STREAM_NAMED("Planner", "   Trajectory size: " << tmp.poses.size());
                ROS_INFO_STREAM_NAMED("Planner", "   Trajectory:");
                for (const auto & pose : tmp.poses)
                {
                    ROS_INFO_STREAM_NAMED("Planner", "      " << pose.position.x << ", " << pose.position.y);
                }

                tmp = gapTrajGenerator_->processTrajectory(tmp);

                // std::chrono::steady_clock::time_point proc_traj_time = std::chrono::steady_clock::now();

                geometry_msgs::PoseArray virtual_score_path = getOrientDecayedPath(tmp);
                virtualGapPaths.at(i) = virtual_score_path;

                // std::chrono::steady_clock::time_point orient_traj_time = std::chrono::steady_clock::now();

                std::vector<float> pathPoseCost;
                float pathTerminalCost;                
                trajEvaluator_->scoreTrajectory(virtual_score_path, pathPoseCost, pathTerminalCost);
                pathPoseCosts.at(i) = pathPoseCost;
                pathTerminalPoseCosts.at(i) = pathTerminalCost;

                float pathCost = pathTerminalCost + std::accumulate(pathPoseCost.begin(), pathPoseCost.end(), 0.0f) / pathPoseCost.size();
                ROS_INFO_STREAM_NAMED("Planner", "   Trajectory score: " << pathCost);

                // std::chrono::steady_clock::time_point score_traj_time = std::chrono::steady_clock::now();

                gapPaths.at(i) = gapTrajGenerator_->transformPath(tmp, rbt2odom_);
            
                // std::chrono::steady_clock::time_point transform_traj_time = std::chrono::steady_clock::now();

                // Log the time taken for each step
                // auto gen_traj_duration = std::chrono::duration_cast<std::chrono::milliseconds>(gen_traj_time - start_time).count();
                // auto proc_traj_duration = std::chrono::duration_cast<std::chrono::milliseconds>(proc_traj_time - gen_traj_time).count();
                // auto orient_traj_duration = std::chrono::duration_cast<std::chrono::milliseconds>(orient_traj_time - proc_traj_time).count();
                // auto score_traj_duration = std::chrono::duration_cast<std::chrono::milliseconds>(score_traj_time - orient_traj_time).count();
                // auto transform_traj_duration = std::chrono::duration_cast<std::chrono::milliseconds>(transform_traj_time - score_traj_time).count();    

                // auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(transform_traj_time - start_time).count();

                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   Time taken for trajectory generation: " << gen_traj_duration << " ms");
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   Time taken for trajectory processing: " << proc_traj_duration << " ms");
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   Time taken for trajectory orientation decay: " << orient_traj_duration << " ms");
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   Time taken for trajectory scoring: " << score_traj_duration << " ms");
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   Time taken for trajectory transformation: " << transform_traj_duration << " ms");
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   Total time taken for trajectory generation: " << total_duration << " ms");
            }
        } catch (...) 
        {
            ROS_FATAL_STREAM("generateGapTrajectories");
        }
        
        // trajVisualizer_->pubAllScore(ret_traj, ret_traj_scores);
        trajVisualizer_->drawGapTrajectories(gapPaths);
        return;
    }

    geometry_msgs::PoseArray Planner::getOrientDecayedPath(const geometry_msgs::PoseArray & orig_path)
    {
        // The original path should be in robot frame
        assert(orig_path.header.frame_id == cfg_.robot_frame_id);
        geometry_msgs::PoseArray decayed_path;

        if (orig_path.poses.size() <= 1)
        {
            ROS_WARN_STREAM("[getOrientDecayedPath] Original path is too short with size [ " << orig_path.poses.size() << " ].");
            decayed_path = orig_path;
            return decayed_path;
        }
        
        if (robot_geo_proc_.robot_.shape == RobotShape::circle || !cfg_.planning.virtual_path_decay_enable)
        {
            decayed_path = orig_path;
        } else if (robot_geo_proc_.robot_.shape == RobotShape::box)
        {
            decayed_path.header = orig_path.header;
            geometry_msgs::Pose first_pose = orig_path.poses[0];
            geometry_msgs::Quaternion init_quat;
            init_quat.w = 1;
            first_pose.orientation = init_quat;
            decayed_path.poses.push_back(first_pose);
            float length = 0;
            for (size_t i = 1; i < orig_path.poses.size(); i++)
            {
                if (!cfg_.planning.robot_path_orient_linear_decay)
                {
                    geometry_msgs::Pose curr_pose = orig_path.poses[i];
                    curr_pose.orientation = init_quat;
                    decayed_path.poses.push_back(curr_pose);
                } else
                {
                    geometry_msgs::Pose curr_pose = orig_path.poses[i];
                    geometry_msgs::Pose prev_pose = orig_path.poses[i-1];
                    float x_diff = curr_pose.position.x - prev_pose.position.x;
                    float y_diff = curr_pose.position.y - prev_pose.position.y;
                    float dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
                    length += dist;

                    float avg_speed = 0.2;
                    float t = length / avg_speed;
                    float avg_ang = cfg_.control.ang_absmax / cfg_.control.speed_factor;

                    Eigen::Quaternionf q(curr_pose.orientation.w, curr_pose.orientation.x, curr_pose.orientation.y, curr_pose.orientation.z);
                    Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
                    float ang_diff = std::abs(euler[2]);
                    float decayed_ang = avg_ang * t;
                    decayed_ang = decayed_ang <= ang_diff ? decayed_ang : ang_diff;
                    if (euler[2] <= 0)
                        decayed_ang = -decayed_ang;
                    
                    float roll = 0, pitch = 0;    
                    Eigen::Quaternionf e;
                    e = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
                        * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
                        * Eigen::AngleAxisf(decayed_ang, Eigen::Vector3f::UnitZ());
                    
                    curr_pose.orientation.w = e.w();
                    curr_pose.orientation.x = e.x();
                    curr_pose.orientation.y = e.y();
                    curr_pose.orientation.z = e.z();

                    decayed_path.poses.push_back(curr_pose);
                }
                
            }
        }
        else
        {
            ROS_WARN("Doesn't support robot shape, use original path.");
            decayed_path = orig_path;
        }

        return decayed_path;
    }

    void Planner::pickTraj(const std::vector<geometry_msgs::PoseArray> & gapPaths, 
                            const std::vector<geometry_msgs::PoseArray> & virtualGapPaths, 
                            const std::vector<std::vector<float>> & pathPoseCosts, 
                            const std::vector<float> & pathTerminalPoseCosts, 
                            geometry_msgs::PoseArray& bestGapPath,
                            geometry_msgs::PoseArray& bestVirtualGapPath) 
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        ROS_INFO_STREAM_NAMED("qg_trajCount", "qg_trajCount, " << gapPaths.size());
        if (gapPaths.size() == 0) 
        {
            ROS_WARN_STREAM("No traj synthesized");
            bestGapPath = geometry_msgs::PoseArray();
            bestVirtualGapPath = geometry_msgs::PoseArray();
            return;
        }

        if (gapPaths.size() != pathPoseCosts.size() ||
            gapPaths.size() != pathTerminalPoseCosts.size())
        {
            ROS_FATAL_STREAM("pickTraj size mismatch: gapPaths = " << gapPaths.size() << " != pathPoseCosts =" << pathPoseCosts.size() << 
                             " != pathTerminalPoseCosts = " << pathTerminalPoseCosts.size());
            bestGapPath = geometry_msgs::PoseArray();
            bestVirtualGapPath = geometry_msgs::PoseArray();
            return;
        }

        std::vector<float> result_score(gapPaths.size());
        
        try 
        {
            if (omp_get_dynamic()) 
                omp_set_dynamic(0);
            
            for (size_t i = 0; i < result_score.size(); i++) 
            {
                result_score.at(i) = pathTerminalPoseCosts.at(i) + std::accumulate(pathPoseCosts.at(i).begin(), 
                                                                                    pathPoseCosts.at(i).end(), float(0)) / float(pathPoseCosts.at(i).size());
                result_score.at(i) = gapPaths.at(i).poses.size() == 0 ? -std::numeric_limits<float>::infinity() : result_score.at(i);
                ROS_DEBUG_STREAM("Score: " << result_score.at(i));
            }
        } catch (...) 
        {
            ROS_FATAL_STREAM("pickTraj");
        }

        auto iter = std::max_element(result_score.begin(), result_score.end());
        int idx = std::distance(result_score.begin(), iter);

        if (result_score.at(idx) == -std::numeric_limits<float>::infinity()) 
        {
            ROS_WARN_STREAM("No executable trajectory, values: ");
            for (const float & val : result_score) 
            {
                ROS_INFO_STREAM_NAMED("Planner", "Score: " << val);
            }
            ROS_INFO_STREAM_NAMED("Planner", "------------------");
        }

        bestGapPath = gapPaths.at(idx);
        bestVirtualGapPath = virtualGapPaths.at(idx);
        ROS_INFO_STREAM_NAMED("Planner", "Picked [" << idx << "] traj" );
    }

    geometry_msgs::PoseArray Planner::compareToCurrentTraj(const geometry_msgs::PoseArray & incomingPath, 
                                                            geometry_msgs::PoseArray& virtual_currTraj) 
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        geometry_msgs::PoseArray  currTraj = getCurrentTraj();

        try 
        {
            //////////////////////////////////////////////////////////////////////////////
            // Transform into the current robot frame to score against the current scan //
            //////////////////////////////////////////////////////////////////////////////

            // Both Args are in Odom frame
            geometry_msgs::PoseArray incomingPathRbtFrame = gapTrajGenerator_->transformPath(incomingPath, odom2rbt_);
            incomingPathRbtFrame.header.frame_id = cfg_.robot_frame_id;

            ROS_INFO_STREAM_NAMED("Planner", "    evaluating incoming trajectory");

            geometry_msgs::PoseArray orientedIncomingPathRbtFrame = getOrientDecayedPath(incomingPathRbtFrame);
            std::vector<float> incomingPathPoseCosts;
            float incomingPathTerminalCost;
            trajEvaluator_->scoreTrajectory(orientedIncomingPathRbtFrame, incomingPathPoseCosts, incomingPathTerminalCost);
            
            ROS_INFO_STREAM_NAMED("Planner", "    length of incoming path: " << incomingPathRbtFrame.poses.size());

            float incom_subscore = incomingPathTerminalCost + std::accumulate(incomingPathPoseCosts.begin(), 
                                                                                incomingPathPoseCosts.end(), float(0)) / float(incomingPathPoseCosts.size());

            ///////////////////////////////////////////////////////////////////////
            //  Evaluate the incoming path to determine if we can switch onto it //
            ///////////////////////////////////////////////////////////////////////
            std::string incomingPathStatus = "incoming path is safe to switch onto";
            bool ableToSwitchToIncomingPath = true;

            if (incomingPath.poses.size() == 0)
            {
                incomingPathStatus = "incoming path is empty";
                ableToSwitchToIncomingPath = false;
            } else if (incom_subscore == -std::numeric_limits<float>::infinity()) 
            {
                incomingPathStatus = "incoming path is not feasible";
                ableToSwitchToIncomingPath = false;
            }
     
            ///////////////////////////////////////////////////////////////////////////////////
            //  Enact a trajectory switch if the currently executing path is empty (size: 0) //
            ///////////////////////////////////////////////////////////////////////////////////

            if (currTraj.poses.size() == 0) 
            {
                if (!ableToSwitchToIncomingPath)
                {
                    geometry_msgs::PoseArray empty_traj = geometry_msgs::PoseArray();
                    setCurrentTraj(empty_traj);
                    virtual_currTraj = empty_traj;
                    ROS_WARN_STREAM_NAMED("Planner", "Old Traj length 0, curr traj score -inf.");
                    return empty_traj;
                } else
                {
                    setCurrentTraj(incomingPath);
                    virtual_currTraj = gapTrajGenerator_->transformPath(orientedIncomingPathRbtFrame, rbt2odom_);
                    // trajectory_pub.publish(incomingPath);
                    trajVisualizer_->drawCurrentTrajectory(incomingPathRbtFrame);
                    ROS_WARN_STREAM_NAMED("Planner", "Old Traj length 0");
                    return incomingPath;                    
                }
            }

            //     if (incom_subscore == -std::numeric_limits<float>::infinity()) 
            //     {
            //         geometry_msgs::PoseArray empty_traj = geometry_msgs::PoseArray();
            //         setCurrentTraj(empty_traj);
            //         virtual_currTraj = empty_traj;
            //         ROS_WARN_STREAM("Old Traj length 0, curr traj score -inf.");
            //         return empty_traj;
            //     } else 
            //     {
            //         setCurrentTraj(incomingPath);
            //         virtual_currTraj = gapTrajGenerator_->transformPath(orientedIncomingPathRbtFrame, rbt2odom_);
            //         trajectory_pub.publish(incomingPath);
            //         ROS_WARN_STREAM("Old Traj length 0");
            //         return incomingPath;
            //     }
            // } 

            // Update the current trajectory
            geometry_msgs::PoseArray updatedCurrentPathRobotFrame = gapTrajGenerator_->transformPath(currTraj, odom2rbt_);
            updatedCurrentPathRobotFrame.header.frame_id = cfg_.robot_frame_id;

            int updatedCurrentPathPoseIdx = getClosestTrajectoryPoseIdx(updatedCurrentPathRobotFrame);
            geometry_msgs::PoseArray reducedCurrentPathRobotFrame = updatedCurrentPathRobotFrame;
            reducedCurrentPathRobotFrame.poses = std::vector<geometry_msgs::Pose>(updatedCurrentPathRobotFrame.poses.begin() + updatedCurrentPathPoseIdx, updatedCurrentPathRobotFrame.poses.end());
            if (reducedCurrentPathRobotFrame.poses.size() < 2) 
            {
                ROS_WARN_STREAM_NAMED("Planner", "Old Traj short");
                setCurrentTraj(incomingPath);
                virtual_currTraj = gapTrajGenerator_->transformPath(orientedIncomingPathRbtFrame, rbt2odom_);
                return incomingPath;
            }
            
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //  Compare the costs of the incoming trajectory with the cost of the current trajectory to see if we need to switch //
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            geometry_msgs::PoseArray virtual_curr_score_path = getOrientDecayedPath(reducedCurrentPathRobotFrame);

            std::vector<float> reducedCurrentPathPoseCosts;
            float reducedCurrentPathTerminalCost;                       
            trajEvaluator_->scoreTrajectory(virtual_curr_score_path, reducedCurrentPathPoseCosts, reducedCurrentPathTerminalCost);

            float curr_subscore = reducedCurrentPathTerminalCost + std::accumulate(reducedCurrentPathPoseCosts.begin(), 
                                                                                    reducedCurrentPathPoseCosts.end(), float(0)) / float(reducedCurrentPathPoseCosts.size());
            
            // incom_subscore = std::accumulate(incomingPathPoseCosts.begin(), incomingPathPoseCosts.begin() + counts, float(0));

            std::vector<std::vector<float>> ret_traj_scores(2);
            ret_traj_scores.at(0) = incomingPathPoseCosts;
            ret_traj_scores.at(1) = reducedCurrentPathPoseCosts;
            std::vector<geometry_msgs::PoseArray> viz_traj(2);
            viz_traj.at(0) = incomingPathRbtFrame;
            viz_traj.at(1) = reducedCurrentPathRobotFrame;
            // trajVisualizer_->pubAllScore(viz_traj, ret_traj_scores);

            ROS_INFO_STREAM_NAMED("Planner", "Curr Score: " << curr_subscore << ", incom Score:" << incom_subscore);

            if (curr_subscore == -std::numeric_limits<float>::infinity())
            {
                ROS_WARN_STREAM("current score infinity, switching to incoming path: " << incom_subscore);
                setCurrentTraj(incomingPath);
                virtual_currTraj = gapTrajGenerator_->transformPath(orientedIncomingPathRbtFrame, rbt2odom_);
                // trajectory_pub.publish(incomingPath);
                trajVisualizer_->drawCurrentTrajectory(incomingPathRbtFrame);
                return incomingPath;                
            }

            // if (curr_subscore == -std::numeric_limits<float>::infinity() && incom_subscore == -std::numeric_limits<float>::infinity()) 
            // {
            //     ROS_WARN_STREAM("Both Failed");
            //     geometry_msgs::PoseArray empty_traj = geometry_msgs::PoseArray();
            //     setCurrentTraj(empty_traj);
            //     virtual_currTraj = empty_traj;
            //     return empty_traj;
            // }

            if (incom_subscore > curr_subscore) 
            {
                ROS_WARN_STREAM("Swap to new for better score: " << incom_subscore << " > " << curr_subscore);
                setCurrentTraj(incomingPath);
                virtual_currTraj = gapTrajGenerator_->transformPath(orientedIncomingPathRbtFrame, rbt2odom_);
                // trajectory_pub.publish(incomingPath);
                trajVisualizer_->drawCurrentTrajectory(incomingPathRbtFrame);
                return incomingPath;
            }

            geometry_msgs::PoseArray virtual_score_path_curr = getOrientDecayedPath(updatedCurrentPathRobotFrame);
            virtual_currTraj = gapTrajGenerator_->transformPath(virtual_score_path_curr, rbt2odom_);
            // trajectory_pub.publish(currTraj);
            trajVisualizer_->drawCurrentTrajectory(updatedCurrentPathRobotFrame);
        } catch (...) 
        {
            ROS_FATAL_STREAM("compareToCurrentTraj");
        }
        return currTraj;
    }

    CollisionResults Planner::checkCollision(const geometry_msgs::PoseArray & pathOdomFrame)
    {
        // Convert the trajectory from odom to base frame
        geometry_msgs::PoseArray pathRbtFrame = gapTrajGenerator_->transformPath(pathOdomFrame, odom2rbt_);
        geometry_msgs::Pose curr_pose;
        curr_pose.orientation.w = 1;

        // TrajPlan orig_ref = trajController_->trajGen(pathRbtFrame);
        pathRbtFrame.header.frame_id = cfg_.robot_frame_id;
        ctrl_idx = trajController_->targetPoseIdx(curr_pose, pathRbtFrame);

        pips_trajectory_msgs::trajectory_points local_traj;
        local_traj.header.frame_id = cfg_.robot_frame_id;
        for (int i = ctrl_idx; i < pathRbtFrame.poses.size(); i++)
        {
            pips_trajectory_msgs::trajectory_point pt;
            pt.x = pathRbtFrame.poses[i].position.x;
            pt.y = pathRbtFrame.poses[i].position.y;

            // ROS_INFO_STREAM(pt.x << " " << pt.y);

            tf2::Quaternion quat_tf;
            tf2::convert(pathRbtFrame.poses[i].orientation, quat_tf);
            // tf2::Matrix3x3 m(quat_tf);
            // float roll, pitch, yaw;
            // m.getRPY(roll, pitch, yaw);
            float yaw = quaternionToYaw(quat_tf);
            pt.theta = yaw;

            local_traj.points.push_back(pt);
        }
        

        int collision_ind = traj_tester_->evaluateTrajectory(local_traj);

        CollisionResults cc_results(collision_ind, local_traj);

        return cc_results;
    }

    int Planner::getClosestTrajectoryPoseIdx(const geometry_msgs::PoseArray & currTrajRbtFrame) 
    {
        std::vector<float> pathPoseNorms(currTrajRbtFrame.poses.size());
        // ROS_INFO_STREAM("Ref_pose length: " << ref_pose.poses.size());
        for (size_t i = 0; i < pathPoseNorms.size(); i++) // i will always be positive, so this is fine
        {
            pathPoseNorms.at(i) = sqrt(pow(currTrajRbtFrame.poses.at(i).position.x, 2) + 
                                    pow(currTrajRbtFrame.poses.at(i).position.y, 2));
        }

        auto minPoseNormIter = std::min_element(pathPoseNorms.begin(), pathPoseNorms.end());
        int minPoseNormIdx = std::distance(pathPoseNorms.begin(), minPoseNormIter) + 1;
        return std::min(minPoseNormIdx, int(currTrajRbtFrame.poses.size() - 1));
    }

    void Planner::setCurrentTraj(const geometry_msgs::PoseArray & currTraj) 
    {
        curr_executing_traj = currTraj;
        return;
    }

    geometry_msgs::PoseArray Planner::getCurrentTraj() 
    {
        return curr_executing_traj;
    }

    void Planner::reset()
    {
        // currSimpGaps_.clear();
        setCurrentTraj(geometry_msgs::PoseArray());

        ROS_INFO_STREAM_NAMED("Planner", "cmdVelBuffer size: " << cmdVelBuffer.size());
        cmdVelBuffer.clear();
        ROS_INFO_STREAM_NAMED("Planner", "cmdVelBuffer size after clear: " << cmdVelBuffer.size() << ", is full: " << cmdVelBuffer.capacity());
        return;
    }

    geometry_msgs::Twist Planner::ctrlGeneration(const geometry_msgs::PoseArray & pathOdomFrame) 
    {
        
        if (!haveTFs_)
            return geometry_msgs::Twist();

        if (pathOdomFrame.poses.size() < 1)
        {
            ROS_WARN_STREAM("Available Execution Traj length: " << pathOdomFrame.poses.size() << " < 1");
            return geometry_msgs::Twist();
        }

        timeKeeper_->startTimer(CONTROL);

        // Know Current Pose
        geometry_msgs::PoseStamped currPoseStRobotFrame;
        currPoseStRobotFrame.header.frame_id = cfg_.robot_frame_id;
        currPoseStRobotFrame.pose.orientation.w = 1;

        geometry_msgs::PoseStamped currPoseStampedOdomFrame;
        currPoseStampedOdomFrame.header.frame_id = cfg_.odom_frame_id;
        currPoseStampedOdomFrame.pose.orientation.w = 1;

        tf2::doTransform(currPoseStRobotFrame, currPoseStampedOdomFrame, rbt2odom_);
        geometry_msgs::Pose currPoseOdomFrame = currPoseStampedOdomFrame.pose;

        // TrajPlan orig_ref = trajController_->trajGen(pathOdomFrame);
        ctrl_idx = trajController_->targetPoseIdx(currPoseOdomFrame, pathOdomFrame);

        geometry_msgs::Pose ctrl_target_pose_odom = pathOdomFrame.poses.at(ctrl_idx);

        // nav_msgs::Odometry ctrl_target_pose;
        // ctrl_target_pose.header = pathOdomFrame.header;
        // ctrl_target_pose.pose.pose = pathOdomFrame.poses.at(ctrl_idx);
        // ctrl_target_pose.twist.twist = orig_ref.twist.at(ctrl_idx);

        sensor_msgs::LaserScan stored_scan_msgs = *scan_.get();

        timeKeeper_->startTimer(FEEBDACK);
        geometry_msgs::Twist cmd_vel = trajController_->controlLaw(currPoseOdomFrame, ctrl_target_pose_odom, stored_scan_msgs, currPoseStRobotFrame);
        timeKeeper_->stopTimer(FEEBDACK);

        timeKeeper_->stopTimer(CONTROL);
        return cmd_vel;
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
        boost::mutex::scoped_lock gapset(gapMutex_);

        std::vector<Gap *> planningGaps;

        for (Gap * gap : currSimpGaps_)
            planningGaps.push_back(new Gap(*gap));

        return planningGaps;
    }

    geometry_msgs::PoseArray Planner::runPlanningLoop() 
    {
        ROS_INFO_STREAM_NAMED("Planner", "[runPlanningLoop()]: count " << timeKeeper_->getPlanningLoopCalls());

        if (!initialized_ || !hasLaserScan_ || !hasGlobalGoal_)
        {
            ROS_WARN_STREAM_NAMED("Planner", "Not ready to plan, initialized: " << initialized_ << ", laser scan: " << hasLaserScan_ << ", global goal: " << hasGlobalGoal_);
            // chosenTraj = Trajectory();
            return geometry_msgs::PoseArray();            
        }

        if (colliding_)
        {
            ROS_WARN_STREAM_NAMED("Planner", "In collision");
            // chosenTraj = Trajectory();
            return geometry_msgs::PoseArray();
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
            ROS_WARN_STREAM_NAMED("Planner", "No gaps found, planning loop will not continue.");
            // chosenTraj = Trajectory();
            return geometry_msgs::PoseArray();
        }

        ROS_INFO_STREAM_NAMED("Planner", "Planning gaps:");
        for (int i = 0; i < gapCount; i++)
        {
            Gap * gap = planningGaps.at(i);
            float leftX, leftY, rightX, rightY;
            gap->getLCartesian(leftX, leftY);
            gap->getRCartesian(rightX, rightY);
            ROS_INFO_STREAM_NAMED("Planner", "Gap " << i);
            ROS_INFO_STREAM_NAMED("Planner", "      Left polar: (" << gap->LIdx() << ", " << gap->LRange() << "), Right polar: (" << gap->RIdx() << ", " << gap->RRange() << ")");
            ROS_INFO_STREAM_NAMED("Planner", "      Left cartesian: (" << leftX << ", " << leftY << "), Right cartesian: (" << rightX << ", " << rightY << ")");
        }

        //////////////////////////////////////////////////////////////////////////////////////
        //                              GAP MANIPULATION                                    //
        //////////////////////////////////////////////////////////////////////////////////////

        timeKeeper_->startTimer(GAP_MANIP);
        std::vector<Gap *> manipGaps = gapManipulate(planningGaps);
        timeKeeper_->stopTimer(GAP_MANIP);

        //////////////////////////////////////////////////////////////////////////////////////
        //                             GAP GOAL PLACEMENT                                   //
        //////////////////////////////////////////////////////////////////////////////////////

        timeKeeper_->startTimer(GAP_MANIP);
        gapGoalPlace(manipGaps);
        timeKeeper_->stopTimer(GAP_MANIP);

        //////////////////////////////////////////////////////////////////////////////////////
        //                          GAP TRAJECTORY GENERATION                               //
        //////////////////////////////////////////////////////////////////////////////////////

        timeKeeper_->startTimer(GAP_TRAJ_GEN);
        std::vector<geometry_msgs::PoseArray> gapPaths;
        std::vector<geometry_msgs::PoseArray> virtualGapPaths;
        std::vector<std::vector<float>> pathPoseCosts; 
        std::vector<float> pathTerminalPoseCosts;        
        generateGapTrajectories(manipGaps, gapPaths, virtualGapPaths, pathPoseCosts, pathTerminalPoseCosts);
        timeKeeper_->stopTimer(GAP_TRAJ_GEN);

        geometry_msgs::PoseArray chosenGapPath;
        // //////////////////////////////////////////////////////////////////////////////////////
        // //                                PICK TRAJECTORY                                   //
        // //////////////////////////////////////////////////////////////////////////////////////

        // timeKeeper_->startTimer(TRAJ_PICK);
        // geometry_msgs::PoseArray bestGapPath;
        // geometry_msgs::PoseArray bestVirtualGapPath;
        // pickTraj(gapPaths, virtualGapPaths, 
        //          pathPoseCosts, pathTerminalPoseCosts, 
        //          bestGapPath, bestVirtualGapPath);
        // // virtual_orient_traj_pub.publish(bestVirtualGapPath);
        // timeKeeper_->stopTimer(TRAJ_PICK);

        // //////////////////////////////////////////////////////////////////////////////////////
        // //                              GAP TRAJECTORY COMPARISON                           //
        // //////////////////////////////////////////////////////////////////////////////////////

        // timeKeeper_->startTimer(TRAJ_COMP);
        // geometry_msgs::PoseArray chosenVirtualGapPath;
        // geometry_msgs::PoseArray chosenGapPath = compareToCurrentTraj(bestGapPath, chosenVirtualGapPath);
        // timeKeeper_->stopTimer(TRAJ_COMP);

        // /////////////////////////////////////////////////////////////////////////////////////
        // //                                 COLLISION CHECKING                              //
        // /////////////////////////////////////////////////////////////////////////////////////

        // timeKeeper_->startTimer(COLL_CHECK);
        // CollisionResults cc_results;
        // if (cfg_.collision_checker.collision_checker_enable)
        // {
        //     ros::WallTime start = ros::WallTime::now();

        //     // cc_results = checkCollision(chosenGapPath);
        //     cc_results = checkCollision(chosenVirtualGapPath);

        //     ROS_INFO_STREAM_NAMED("Planner", "Current trajectory collision checked in " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
        //     int cc_ite_min = 10;
        //     float cc_itc_ratio = 0.2;

        //     if(cc_results.collision_idx_ >= 0 && float(cc_results.collision_idx_) / cc_results.local_traj_.points.size() <= cc_itc_ratio)
        //     {
        //         ROS_WARN_STREAM("Current trajectory collides! " << cc_results.collision_idx_ << " " << cc_results.local_traj_.points.size());
        //         setCurrentTraj(geometry_msgs::PoseArray());
        //     }
        // }
        // timeKeeper_->stopTimer(COLL_CHECK);

        // // delete set of planning gaps
        // for (Gap * planningGap : planningGaps)
        //     delete planningGap;

        // timeKeeper_->stopTimer(PLAN);
        // timeKeeper_->computeAverageNumberGaps(gapCount);        

        return chosenGapPath;
    }

    bool Planner::recordAndCheckVel(const geometry_msgs::Twist & cmd_vel) 
    {
        float val = std::abs(cmd_vel.linear.x) + std::abs(cmd_vel.linear.y) + std::abs(cmd_vel.angular.z);
        cmdVelBuffer.push_back(val);
        float cum_vel_sum = std::accumulate(cmdVelBuffer.begin(), cmdVelBuffer.end(), float(0));
        bool ret_val = cum_vel_sum > 1.0 || !cmdVelBuffer.full();
        if (!ret_val && !cfg_.man.man_ctrl) {
            ROS_FATAL_STREAM("--------------------------Planning Failed--------------------------");
            reset();
        }
        return ret_val || cfg_.man.man_ctrl;
    }

}