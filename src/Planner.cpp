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

        int shape_id = 1;
        unh.getParam("shape_id", shape_id);
        unh.setParam("shape_id", shape_id);

        double length = 0.7, width = 0.3, decay_factor = 0, avg_lin_speed = 0.2, avg_rot_speed = 0.5;
        unh.getParam("length", length);
        unh.getParam("width", width);
        unh.getParam("decay_factor", decay_factor);
        unh.getParam("avg_lin_speed", avg_lin_speed);
        unh.getParam("avg_rot_speed", avg_rot_speed);
        unh.setParam("length", length);
        unh.setParam("width", width);
        unh.setParam("decay_factor", decay_factor);
        unh.setParam("avg_lin_speed", avg_lin_speed);
        unh.setParam("avg_rot_speed", avg_rot_speed);

        RobotShape robot_shape = static_cast<RobotShape>(shape_id);
        if (robot_shape == RobotShape::circle)
            width = 0;

        Robot robot(robot_shape, length, width, avg_lin_speed, avg_rot_speed);

        use_geo_storage_ = false;
        unh.getParam("use_geo_storage", use_geo_storage_);
        unh.setParam("use_geo_storage", use_geo_storage_);
        
        if (use_geo_storage_)
            robot_geo_storage_ = RobotGeometryStorage(file_name);
        else
            robot_geo_proc_ = RobotGeometryProcessor(robot, decay_factor);

        robot_path_orient_linear_decay_ = true;
        virtual_path_decay_enable_ = true;
        unh.getParam("robot_path_orient_linear_decay", robot_path_orient_linear_decay_);
        unh.getParam("virtual_path_decay_enable", virtual_path_decay_enable_);
        unh.setParam("robot_path_orient_linear_decay", robot_path_orient_linear_decay_);
        unh.setParam("virtual_path_decay_enable", virtual_path_decay_enable_);
        speed_factor_ = 2;
        unh.getParam("speed_factor", speed_factor_);
        unh.setParam("speed_factor", speed_factor_);

        // Bezier curve
        use_bezier_ = true;
        unh.getParam("use_bezier", use_bezier_);
        unh.setParam("use_bezier", use_bezier_);
        
        // Debug robot geometry storage and process
        // robot_geo_storage_ = RobotGeometryStorage(file_name);
        // robot_geo_proc_ = RobotGeometryProcessor(robot);

        // Eigen::Vector2d orientation_vec(1, 0);
        // Eigen::Vector2d pt_vec(1,0);
        // Eigen::Vector2d motion_vec = pt_vec;
        // double vec_dot_pro = orientation_vec.dot(pt_vec);
        
        // ros::WallTime interp_start = ros::WallTime::now();
        // double interp_er = robot_geo_storage_.getInterpEquivR(vec_dot_pro);
        // double interp_epl = robot_geo_storage_.getInterpEquivPL(vec_dot_pro);
        // ros::WallDuration interp_time = ros::WallTime::now() - interp_start;
        
        // ros::WallTime comp_start = ros::WallTime::now();
        // double er = robot_geo_proc_.getEquivalentR(orientation_vec, pt_vec);
        // double epl = robot_geo_proc_.getEquivalentPL(orientation_vec, motion_vec);
        // ros::WallDuration comp_time = ros::WallTime::now() - comp_start;

        // ROS_INFO_STREAM("Interp er: " << interp_er << ", Interp epl: " << interp_epl << ", time: " << (double)interp_time.toNSec() << " ns");
        // ROS_INFO_STREAM("Comp er: " << er << ", Comp epl: " << epl << ", time: " << (double)comp_time.toNSec() << " ns");
        // throw;
        
        // Debug robot_geo_processor
        // Eigen::Vector2d orientation_vec(1, 0);
        // Eigen::Vector2d p1(-0.35,0);
        // Eigen::Vector2d p2(-0.35,-0.15);
        // Eigen::Vector2d p3(0,-0.15);
        // Eigen::Vector2d p4(0.35,-0.15);
        // Eigen::Vector2d p5(0.35,0);
        // Eigen::Vector2d p6(0.35,0.15);
        // Eigen::Vector2d p7(0,0.15);
        // Eigen::Vector2d p8(-0.35,0.15);
        // Eigen::Vector2d p9(0.1, 0.1);
        // Eigen::Vector2d p10(1, 1);
        // double vec_length = 3; // 3
        // std::vector<Eigen::Vector2d> pt_list{vec_length*p1/p1.norm(), vec_length*p2/p2.norm(), vec_length*p3/p3.norm(), vec_length*p4/p4.norm(), vec_length*p5/p5.norm(), vec_length*p6/p6.norm(), vec_length*p7/p7.norm(), vec_length*p8/p8.norm(), p9, p10};

        // // True 
        // std::vector<double> r{p1.norm(), p2.norm(), p3.norm(), p4.norm(), p5.norm(), p6.norm(), p7.norm(), p8.norm(), sqrt(0.15*0.15*2), sqrt(0.15*0.15*2)};
        // double alpha = 2 * atan2(0.15, 0.35);
        // std::vector<double> el{0.3, 2*p2.norm()*sin(alpha), 0.7, 2*p2.norm()*sin(alpha), 0.3, 2*p2.norm()*sin(alpha), 0.7, 2*p2.norm()*sin(alpha), 2*p2.norm()*cos(M_PI / 4 - alpha/2), 2*p2.norm()*cos(M_PI / 4 - alpha/2)};
        
        
        // double p1_min_dist = vec_length - 0.35;
        // double p1_max_dist = sqrt(pow(vec_length + 0.35, 2) + 0.15 * 0.15);
        // double dia = 2*p2.norm();
        // double p3_min_dist = vec_length - 0.15;
        // double p3_max_dist = sqrt(pow(vec_length + 0.15, 2) + 0.35 * 0.35);
        // std::vector<double> er{p1_max_dist - p1_min_dist, dia, p3_max_dist - p3_min_dist, dia, p1_max_dist - p1_min_dist, dia, p3_max_dist - p3_min_dist, dia, sqrt(pow(0.45, 2) + pow(0.25, 2)), (p10+p6).norm() - (p10-p6).norm()};

        // std::vector<double> n_dist{vec_length - 0.35, vec_length - p2.norm(), vec_length - 0.15, vec_length - p2.norm(), vec_length - 0.35, vec_length - p2.norm(), vec_length - 0.15, vec_length - p2.norm(), -1, (p10-p6).norm()};

        // double rot_ang = M_PI / 2;
        // Eigen::Matrix2d rot;
        //         rot << cos(rot_ang), -sin(rot_ang), sin(rot_ang), cos(rot_ang);
        // for (size_t i = 0; i < pt_list.size(); i++)
        // {   
        //     Eigen::Vector2d p = pt_list[i];
        //     Eigen::Vector2d p_tmp = p;
        //     // True values
        //     double r_dist = r[i];
        //     double er_p = er[i];
        //     double el_p = el[i];
        //     double nd = n_dist[i];

        //     // Calculated values
        //     Eigen::Vector2d o_vec = rot * orientation_vec;
        //     Eigen::Vector2d p_vec = rot * p;
        //     Eigen::Vector2d p_tmp_vec = rot * p_tmp;
        //     double r_c_dist = robot_geo_proc_.getEquivalentR(o_vec, p_vec);
        //     ros::WallTime start_time = ros::WallTime::now();
        //     double er_c_p = robot_geo_proc_.getEquivalentRL(o_vec, p_tmp_vec);
        //     ros::WallDuration d = ros::WallTime::now() - start_time;
        //     ROS_INFO_STREAM("RL time: " << (float) d.toNSec() / 1000 << "mu sec");
        //     double el_c_p = robot_geo_proc_.getEquivalentPL(o_vec, p_vec);
        //     ros::WallTime start_n_time = ros::WallTime::now();
        //     double n_dist_c = robot_geo_proc_.getNearestDistance(o_vec, p_tmp_vec);
        //     ros::WallDuration dn = ros::WallTime::now() - start_n_time;
        //     ROS_INFO_STREAM("Nearest time: " << (float) dn.toNSec() / 1000 << "mu sec");

        //     ROS_INFO_STREAM("True values: " << r_dist << " " << er_p << " " << el_p << " " << nd << "; Calculated values: " << r_c_dist << " " << er_c_p << " " << el_c_p << " " << n_dist_c);
        //     ROS_INFO_STREAM("Equals: " << (r_dist - r_c_dist) << " " << (er_p - er_c_p) << " " << (el_p - el_c_p) << " " << (nd - n_dist_c));
        // }
        // throw;

        // Visualization Setup
        // Fix this later
        local_traj_pub = nh.advertise<geometry_msgs::PoseArray>("relevant_traj", 500);
        trajectory_pub = nh.advertise<geometry_msgs::PoseArray>("qg_traj", 10);

        transformed_laser_pub = nh.advertise<sensor_msgs::LaserScan>("transformed_laserscan", 5);
        virtual_orient_traj_pub = nh.advertise<geometry_msgs::PoseArray>("picked_virtual_traj", 10);

        // TF Lookup setup
        tfBuffer = std::make_shared<tf2_ros::Buffer>();
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
        initialized_ = true;

        tfSub_ = nh.subscribe("/tf", 10, &Planner::tfCB, this);
        laserSub_ = nh.subscribe(cfg_.scan_topic, 100, &Planner::laserScanCB, this);
        poseSub_ = nh.subscribe(cfg_.odom_topic, 10, &Planner::poseCB, this);        

        gapDetector_ = new GapDetector(cfg_, robot_geo_proc_);
        gapVisualizer_ = new GapVisualizer(nh, cfg_);
        globalPlanManager_ = new GlobalPlanManager(cfg_, robot_geo_proc_);
        trajVisualizer_ = new TrajectoryVisualizer(nh, cfg_);
        trajEvaluator_ = new TrajectoryEvaluator(nh, cfg_, robot_geo_proc_);
        gapTrajGenerator_ = new GapTrajGenerator(cfg_, robot_geo_proc_);
        goalVisualizer_ = new GoalVisualizer(nh, cfg_);
        gapManipulator_ = new GapManipulator(nh, cfg_, robot_geo_proc_);
        trajController_ = new TrajectoryController(nh, cfg_);
        timeKeeper_ = new TimeKeeper();

        map2rbt_.transform.rotation.w = 1;
        rbt2map_.transform.rotation.w = 1;
        odom2rbt_.transform.rotation.w = 1;
        rbt2odom_.transform.rotation.w = 1;
        rbtPoseRbtFrame_.pose.orientation.w = 1;
        rbtPoseRbtFrame_.header.frame_id = cfg_.robot_frame_id;

        // reconfigure_server_ = std::make_shared<ReconfigureServer>(pnh);
        // reconfigure_server_->setCallback(boost::bind(&Planner::configCB, this, _1, _2));

        // Set collision checker
        collision_checker_enable_ = cfg_.collision_checker.collision_checker_enable;
        if(!collision_checker_enable_)
        {
            ROS_WARN_STREAM("Collision checking is disabled.");
            // return;
        }

        if(cfg_.collision_checker.cc_type == CollisionChecker_depth)
        {
            ROS_INFO_STREAM("New cc type = depth");
            cc_wrapper_ = std::make_shared<pips_trajectory_testing::DepthImageCCWrapper>(nh, pnh, tf2_utils::TransformManager(tfBuffer, tfListener));
        }
        else if(cfg_.collision_checker.cc_type == CollisionChecker_depth_ego)
        {
            ROS_INFO_STREAM("New cc type = depth ego");
            cc_wrapper_ = std::make_shared<pips_egocylindrical::EgocylindricalRangeImageCCWrapper>(nh, pnh, tf2_utils::TransformManager(tfBuffer, tfListener));
        }
        else if(cfg_.collision_checker.cc_type == CollisionChecker_egocircle)
        {
            ROS_INFO_STREAM("New cc type = egocircle");
            cc_wrapper_ = std::make_shared<pips_egocircle::EgoCircleCCWrapper>(nh, pnh, tf2_utils::TransformManager(tfBuffer, tfListener));
        }

        traj_tester_ = std::make_shared<TurtlebotGenAndTest>(nh, pnh);
        
        cc_wrapper_->init();
        cc_wrapper_->autoUpdate();

        traj_tester_->init();
        traj_tester_->setCollisionChecker(cc_wrapper_->getCC());
        
        cc_type_ = cfg_.collision_checker.cc_type;

        cmdVelBuffer.set_capacity(cfg_.planning.halt_size);
        return true;
    }

    // void Planner::configCB(CollisionCheckerConfig &config, uint32_t level)
    // {
    //     ROS_INFO_STREAM("CC Reconfigure Request: "); // TODO: print out the cc type and other parameter values

    //     Lock lock(connect_mutex_);

    //     collision_checker_enable_ = config.cc_enable;
    //     if(!collision_checker_enable_)
    //     {
    //         ROS_WARN_STREAM("Collision checking is disabled.");
    //         return;
    //     }

    //     if(config.cc_type != cc_type_)
    //     {
    //         if(config.cc_type == CollisionChecker_depth)
    //         {
    //             ROS_INFO_STREAM("New cc type = depth");
    //             cc_wrapper_ = std::make_shared<pips_trajectory_testing::DepthImageCCWrapper>(nh, pnh, tf2_utils::TransformManager(tfBuffer, tfListener));
    //         }
    //         else if(config.cc_type == CollisionChecker_depth_ego)
    //         {
    //             ROS_INFO_STREAM("New cc type = depth ego");
    //             cc_wrapper_ = std::make_shared<pips_egocylindrical::EgocylindricalRangeImageCCWrapper>(nh, pnh, tf2_utils::TransformManager(tfBuffer, tfListener));
    //         }
    //         else if(config.cc_type == CollisionChecker_egocircle)
    //         {
    //             ROS_INFO_STREAM("New cc type = egocircle");
    //             cc_wrapper_ = std::make_shared<pips_egocircle::EgoCircleCCWrapper>(nh, pnh, tf2_utils::TransformManager(tfBuffer, tfListener));
    //         }

    //         traj_tester_ = std::make_shared<TurtlebotGenAndTest>(nh, pnh);
            
    //         cc_wrapper_->init();
    //         cc_wrapper_->autoUpdate();

    //         traj_tester_->init();
    //         traj_tester_->setCollisionChecker(cc_wrapper_->getCC());
            
    //         cc_type_ = config.cc_type;
    //     }
    // }

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
        
        reachedGlobalGoal_ = globalGoalLinDist < cfg_.goal.lin_goal_tolerance &&
                             globalGoalAngDist < cfg_.goal.yaw_goal_tolerance;
        
        if (reachedGlobalGoal_)
            ROS_INFO_STREAM_NAMED("Planner", "[Reset] Goal Reached");
        // else
        //     ROS_INFO_STREAM_NAMED("Planner", "Distance from goal: " << globalGoalDist << 
        //                                      ", Goal tolerance: " << cfg_.goal.lin_goal_tolerance);

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
            double orig_range = msg->ranges[i];
            double orig_ang = i * msg->angle_increment + msg->angle_min;
            orig_ang = orig_ang <= msg->angle_max ? orig_ang : msg->angle_max;

            geometry_msgs::PointStamped orig_pt, transformed_pt;
            orig_pt.header = msg->header;
            orig_pt.point.x = orig_range * cos(orig_ang);
            orig_pt.point.y = orig_range * sin(orig_ang);
            
            geometry_msgs::TransformStamped trans = tfBuffer->lookupTransform(cfg_.robot_frame_id, cfg_.sensor_frame_id, ros::Time(0));
            tf2::doTransform(orig_pt, transformed_pt, trans);
            // ROS_INFO_STREAM(cfg_.sensor_frame_id << " " << orig_pt.header.frame_id << " " << transformed_pt.header.frame_id);

            double transformed_range = sqrt(pow(transformed_pt.point.x, 2) + pow(transformed_pt.point.y, 2));
            double transformed_ang = std::atan2(transformed_pt.point.y, transformed_pt.point.x);
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

        ROS_INFO_STREAM_NAMED("Planner", "[laserScanCB()]");

        timeKeeper_->startTimer(SCAN);

        // boost::shared_ptr<sensor_msgs::LaserScan const> tmp_msg = scan_;

        // ROS_INFO_STREAM(msg.get()->ranges.size());

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
            // THEIRS
            // geometry_msgs::PoseStamped local_goal;

            // if (goal_set)
            // {
                // globalPlanManager_->generateGlobalPathLocalWaypoint(map2rbt_);
                // local_goal = globalPlanManager_->getCurrentLocalGoal(rbt2odom_);
                // goalVisualizer_->localGoal(local_goal);
            
                // trajEvaluator_->updateLocalGoal(local_goal, odom2rbt_);
            // }


            // OURS
            // // update global path local waypoint according to new scan
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
        
        if (sqrt(pow(diffX, 2) + pow(diffY, 2)) > cfg_.goal.waypoint_tolerance)
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

    std::vector<Gap *> Planner::gapManipulate(const std::vector<Gap *> & planning_gaps) 
    {
        boost::mutex::scoped_lock gapset(gapMutex_);
        std::vector<Gap *> manip_set = planning_gaps;

        // geometry_msgs::PoseStamped local_goal_sensor_frame;
        // tf2::doTransform(globalPlanManager_->rbtFrameLocalGoal(), local_goal_sensor_frame, rbt2cam_);
        geometry_msgs::PoseStamped local_goal_rbt_frame = globalPlanManager_->getGlobalPathLocalWaypointRobotFrame();
        try 
        {
            for (size_t i = 0; i < manip_set.size(); i++)
            {
                gapManipulator_->reduceGap(manip_set.at(i), local_goal_rbt_frame);
                gapManipulator_->convertAxialGap(manip_set.at(i));
                gapManipulator_->radialExtendGap(manip_set.at(i));
                gapManipulator_->setGapWaypoint(manip_set.at(i), local_goal_rbt_frame);
            }
        } catch(...) 
        {
            ROS_FATAL_STREAM("gapManipulate");
        }

        goalVisualizer_->drawGapGoals(manip_set);
        gapVisualizer_->drawManipGaps(manip_set);
        return manip_set;
    }

    // std::vector<geometry_msgs::PoseArray> 
    std::vector<std::vector<double>> Planner::initialTrajGen(const std::vector<Gap *> & gaps, 
                                                                std::vector<geometry_msgs::PoseArray>& res, 
                                                                std::vector<geometry_msgs::PoseArray>& virtual_decayed) 
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        std::vector<geometry_msgs::PoseArray> ret_traj(gaps.size());
        std::vector<geometry_msgs::PoseArray> virtual_traj(gaps.size());
        std::vector<std::vector<double>> ret_traj_scores(gaps.size());

        geometry_msgs::PoseStamped rbt_local_pose;
        rbt_local_pose.header.frame_id = cfg_.robot_frame_id;
        rbt_local_pose.header.stamp = rbt2odom_.header.stamp;
        rbt_local_pose.pose.orientation.w = 1;
        ROS_INFO_STREAM("Gap number: " << gaps.size());
        try 
        {
            for (size_t i = 0; i < gaps.size(); i++) 
            {
                // Generate trajectory in robot frame.
                geometry_msgs::PoseArray tmp;
                if (use_bezier_)
                {
                    tmp = gapTrajGenerator_->generateBezierTrajectory(gaps.at(i), rbtVelRbtFrame_, odom2rbt_);
                } else
                {
                    tmp = gapTrajGenerator_->generateTrajectory(gaps.at(i), rbt_local_pose);
                }
                
                tmp = gapTrajGenerator_->forwardPassTrajectory(tmp);

                geometry_msgs::PoseArray virtual_score_path = getOrientDecayedPath(tmp);
                virtual_traj.at(i) = virtual_score_path;
                ret_traj_scores.at(i) = trajEvaluator_->scoreTrajectory(virtual_score_path);
                ret_traj.at(i) = gapTrajGenerator_->transformPath(tmp, rbt2odom_);
            }
        } catch (...) 
        {
            ROS_FATAL_STREAM("initialTrajGen");
        }
        
        // trajVisualizer_->pubAllScore(ret_traj, ret_traj_scores);
        trajVisualizer_->drawGapTrajectories(ret_traj);
        res = ret_traj;
        virtual_decayed = virtual_traj;
        return ret_traj_scores;
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
        
        if (robot_geo_proc_.robot_.shape == RobotShape::circle || !virtual_path_decay_enable_)
        {
            decayed_path = orig_path;
        }
        else if (robot_geo_proc_.robot_.shape == RobotShape::box)
        {
            decayed_path.header = orig_path.header;
            geometry_msgs::Pose first_pose = orig_path.poses[0];
            geometry_msgs::Quaternion init_quat;
            init_quat.w = 1;
            first_pose.orientation = init_quat;
            decayed_path.poses.push_back(first_pose);
            double length = 0;
            for (size_t i = 1; i < orig_path.poses.size(); i++)
            {
                if (!robot_path_orient_linear_decay_)
                {
                    geometry_msgs::Pose curr_pose = orig_path.poses[i];
                    curr_pose.orientation = init_quat;
                    decayed_path.poses.push_back(curr_pose);
                } else
                {
                    geometry_msgs::Pose curr_pose = orig_path.poses[i];
                    geometry_msgs::Pose prev_pose = orig_path.poses[i-1];
                    double x_diff = curr_pose.position.x - prev_pose.position.x;
                    double y_diff = curr_pose.position.y - prev_pose.position.y;
                    double dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
                    length += dist;
                    // double avg_speed = sqrt(pow(cfg_.control.vx_absmax, 2) + pow(cfg_.control.vy_absmax, 2)) / speed_factor_;
                    double avg_speed = 0.2;
                    double t = length / avg_speed;
                    double avg_ang = cfg_.control.ang_absmax / speed_factor_;

                    Eigen::Quaterniond q(curr_pose.orientation.w, curr_pose.orientation.x, curr_pose.orientation.y, curr_pose.orientation.z);
                    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
                    double ang_diff = std::abs(euler[2]);
                    double decayed_ang = avg_ang * t;
                    decayed_ang = decayed_ang <= ang_diff ? decayed_ang : ang_diff;
                    if (euler[2] <= 0)
                        decayed_ang = -decayed_ang;
                    
                    double roll = 0, pitch = 0;    
                    Eigen::Quaterniond e;
                    e = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(decayed_ang, Eigen::Vector3d::UnitZ());
                    
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

    geometry_msgs::PoseArray Planner::pickTraj(const std::vector<geometry_msgs::PoseArray> & prr, 
                                                const std::vector<std::vector<double>> & score, 
                                                const std::vector<geometry_msgs::PoseArray> & virtual_path, 
                                                geometry_msgs::PoseArray& chosen_virtual_path) 
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        ROS_INFO_STREAM_NAMED("qg_trajCount", "qg_trajCount, " << prr.size());
        if (prr.size() == 0) {
            ROS_WARN_STREAM("No traj synthesized");
            return geometry_msgs::PoseArray();
        }

        if (prr.size() != score.size()) 
        {
            ROS_FATAL_STREAM("pickTraj size mismatch: prr = " << prr.size() << " != score =" << score.size());
            return geometry_msgs::PoseArray();
        }

        std::vector<double> result_score(prr.size());
        
        try 
        {
            if (omp_get_dynamic()) 
                omp_set_dynamic(0);
            
            for (size_t i = 0; i < result_score.size(); i++) 
            {
                int counts = std::min(cfg_.planning.num_feasi_check, int(score.at(i).size()));
                result_score.at(i) = std::accumulate(score.at(i).begin(), score.at(i).begin() + counts, double(0));
                result_score.at(i) = prr.at(i).poses.size() == 0 ? -std::numeric_limits<double>::infinity() : result_score.at(i);
                ROS_DEBUG_STREAM("Score: " << result_score.at(i));
            }
        } catch (...) 
        {
            ROS_FATAL_STREAM("pickTraj");
        }

        auto iter = std::max_element(result_score.begin(), result_score.end());
        int idx = std::distance(result_score.begin(), iter);

        if (result_score.at(idx) == -std::numeric_limits<double>::infinity()) 
        {
            ROS_WARN_STREAM("No executable trajectory, values: ");
            for (const double & val : result_score) 
            {
                ROS_INFO_STREAM("Score: " << val);
            }
            ROS_INFO_STREAM("------------------");
        }

        chosen_virtual_path = virtual_path.at(idx);
        ROS_INFO_STREAM("Picked [" << idx << "] traj" );
        return prr.at(idx);
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

            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    evaluating incoming trajectory");

            geometry_msgs::PoseArray orientedIncomingPathRbtFrame = getOrientDecayedPath(incomingPathRbtFrame);
            std::vector<double> incomingPathPoseCosts = trajEvaluator_->scoreTrajectory(orientedIncomingPathRbtFrame);
            // int counts = std::min(cfg_.planning.num_feasi_check, (int) std::min(incomingPathPoseCosts.size(), curr_score.size()));
            
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    length of incoming path: " << incomingPathRbtFrame.poses.size());

            // int counts = std::min(cfg_.planning.num_feasi_check, (int) incomingPathPoseCosts.size());
            double incom_subscore = std::accumulate(incomingPathPoseCosts.begin(), incomingPathPoseCosts.end(), double(0)) / double(incomingPathPoseCosts.size());

            ///////////////////////////////////////////////////////////////////////
            //  Evaluate the incoming path to determine if we can switch onto it //
            ///////////////////////////////////////////////////////////////////////
            std::string incomingPathStatus = "incoming path is safe to switch onto";
            bool ableToSwitchToIncomingPath = true;

            if (incomingPath.poses.size() == 0)
            {
                incomingPathStatus = "incoming path is empty";
                ableToSwitchToIncomingPath = false;
            } else if (incom_subscore == -std::numeric_limits<double>::infinity()) 
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
                    ROS_WARN_STREAM("Old Traj length 0, curr traj score -inf.");
                    return empty_traj;
                } else
                {
                    setCurrentTraj(incomingPath);
                    virtual_currTraj = gapTrajGenerator_->transformPath(orientedIncomingPathRbtFrame, rbt2odom_);
                    trajectory_pub.publish(incomingPath);
                    ROS_WARN_STREAM("Old Traj length 0");
                    return incomingPath;                    
                }
            }

            //     if (incom_subscore == -std::numeric_limits<double>::infinity()) 
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
                ROS_WARN_STREAM("Old Traj short");
                setCurrentTraj(incomingPath);
                virtual_currTraj = gapTrajGenerator_->transformPath(orientedIncomingPathRbtFrame, rbt2odom_);
                return incomingPath;
            }
            
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //  Compare the costs of the incoming trajectory with the cost of the current trajectory to see if we need to switch //
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            geometry_msgs::PoseArray virtual_curr_score_path = getOrientDecayedPath(reducedCurrentPathRobotFrame);
            std::vector<double> reducedCurrentPathPoseCosts = trajEvaluator_->scoreTrajectory(virtual_curr_score_path);
            // counts = std::min(cfg_.planning.num_feasi_check, (int) std::min(incomingPathPoseCosts.size(), curr_score.size()));

            double curr_subscore = std::accumulate(reducedCurrentPathPoseCosts.begin(), reducedCurrentPathPoseCosts.end(), double(0)) / double(reducedCurrentPathPoseCosts.size());
            
            // incom_subscore = std::accumulate(incomingPathPoseCosts.begin(), incomingPathPoseCosts.begin() + counts, double(0));

            std::vector<std::vector<double>> ret_traj_scores(2);
            ret_traj_scores.at(0) = incomingPathPoseCosts;
            ret_traj_scores.at(1) = reducedCurrentPathPoseCosts;
            std::vector<geometry_msgs::PoseArray> viz_traj(2);
            viz_traj.at(0) = incomingPathRbtFrame;
            viz_traj.at(1) = reducedCurrentPathRobotFrame;
            // trajVisualizer_->pubAllScore(viz_traj, ret_traj_scores);

            ROS_INFO_STREAM("Curr Score: " << curr_subscore << ", incom Score:" << incom_subscore);

            if (curr_subscore == -std::numeric_limits<double>::infinity())
            {
                ROS_WARN_STREAM("current score infinity, switching to incoming path: " << incom_subscore);
                setCurrentTraj(incomingPath);
                virtual_currTraj = gapTrajGenerator_->transformPath(orientedIncomingPathRbtFrame, rbt2odom_);
                trajectory_pub.publish(incomingPath);
                return incomingPath;                
            }

            // if (curr_subscore == -std::numeric_limits<double>::infinity() && incom_subscore == -std::numeric_limits<double>::infinity()) 
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
                trajectory_pub.publish(incomingPath);
                return incomingPath;
            }

            geometry_msgs::PoseArray virtual_score_path_curr = getOrientDecayedPath(updatedCurrentPathRobotFrame);
            virtual_currTraj = gapTrajGenerator_->transformPath(virtual_score_path_curr, rbt2odom_);
            trajectory_pub.publish(currTraj);
        } catch (...) 
        {
            ROS_FATAL_STREAM("compareToCurrentTraj");
        }
        return currTraj;
    }

    CollisionResults Planner::checkCollision(const geometry_msgs::PoseArray & path)
    {
        // Convert the trajectory from odom to base frame
        geometry_msgs::PoseArray path_rbt = gapTrajGenerator_->transformPath(path, odom2rbt_);
        geometry_msgs::Pose curr_pose;
        curr_pose.orientation.w = 1;
        TrajPlan orig_ref = trajController_->trajGen(path_rbt);
        orig_ref.header.frame_id = cfg_.robot_frame_id;
        ctrl_idx = trajController_->targetPoseIdx(curr_pose, orig_ref);

        pips_trajectory_msgs::trajectory_points local_traj;
        local_traj.header.frame_id = cfg_.robot_frame_id;
        for (int i = ctrl_idx; i < orig_ref.poses.size(); i++)
        {
            pips_trajectory_msgs::trajectory_point pt;
            pt.x = orig_ref.poses[i].position.x;
            pt.y = orig_ref.poses[i].position.y;

            // ROS_INFO_STREAM(pt.x << " " << pt.y);

            tf2::Quaternion quat_tf;
            tf2::convert(orig_ref.poses[i].orientation, quat_tf);
            tf2::Matrix3x3 m(quat_tf);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
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

        ROS_INFO_STREAM("cmdVelBuffer size: " << cmdVelBuffer.size());
        cmdVelBuffer.clear();
        ROS_INFO_STREAM("cmdVelBuffer size after clear: " << cmdVelBuffer.size() << ", is full: " << cmdVelBuffer.capacity());
        return;
    }

    geometry_msgs::Twist Planner::ctrlGeneration(const geometry_msgs::PoseArray & traj) 
    {
        
        if (!haveTFs_)
            return geometry_msgs::Twist();

        if (traj.poses.size() < 1)
        {
            ROS_WARN_STREAM("Available Execution Traj length: " << traj.poses.size() << " < 1");
            return geometry_msgs::Twist();
        }

        timeKeeper_->startTimer(CONTROL);

        // Know Current Pose
        geometry_msgs::PoseStamped currPoseStRobotFrame;
        currPoseStRobotFrame.header.frame_id = cfg_.robot_frame_id;
        currPoseStRobotFrame.pose.orientation.w = 1;
        geometry_msgs::PoseStamped currPoseStampedOdomFrame;
        currPoseStampedOdomFrame.header.frame_id = cfg_.odom_frame_id;
        tf2::doTransform(currPoseStRobotFrame, currPoseStampedOdomFrame, rbt2odom_);
        geometry_msgs::Pose currPoseOdomFrame = currPoseStampedOdomFrame.pose;

        TrajPlan orig_ref = trajController_->trajGen(traj);
        ctrl_idx = trajController_->targetPoseIdx(currPoseOdomFrame, orig_ref);
        nav_msgs::Odometry ctrl_target_pose;
        ctrl_target_pose.header = orig_ref.header;
        ctrl_target_pose.pose.pose = orig_ref.poses.at(ctrl_idx);
        ctrl_target_pose.twist.twist = orig_ref.twist.at(ctrl_idx);

        sensor_msgs::LaserScan stored_scan_msgs = *scan_.get();

        timeKeeper_->startTimer(FEEBDACK);
        geometry_msgs::Twist cmd_vel = trajController_->controlLaw(currPoseOdomFrame, ctrl_target_pose, stored_scan_msgs, currPoseStRobotFrame);
        timeKeeper_->stopTimer(FEEBDACK);

        timeKeeper_->stopTimer(CONTROL);
        return cmd_vel;
    }

    void Planner::rcfgCallback(qgConfig &config, uint32_t level)
    {
        cfg_.reconfigure(config);
        
        // set_capacity destroys everything if different from original size, 
        // resize only if the new size is greater
        cmdVelBuffer.clear();
        cmdVelBuffer.set_capacity(cfg_.planning.halt_size);
    }


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

        isGoalReached();

        std::vector<Gap *> planningGaps = deepCopyCurrentSimplifiedGaps();

        int gapCount = planningGaps.size();
        if (gapCount == 0)
        {
            ROS_WARN_STREAM_NAMED("Planner", "No gaps found, planning loop will not continue.");
            // chosenTraj = Trajectory();
            return geometry_msgs::PoseArray();
        }

        timeKeeper_->startTimer(GAP_MANIP);
        std::vector<Gap *> gap_set = gapManipulate(planningGaps);
        timeKeeper_->stopTimer(GAP_MANIP);

        timeKeeper_->startTimer(GAP_TRAJ_GEN);
        std::vector<geometry_msgs::PoseArray> traj_set, virtual_traj_set;
        
        std::vector<std::vector<double>> score_set = initialTrajGen(gap_set, traj_set, virtual_traj_set);
        timeKeeper_->stopTimer(GAP_TRAJ_GEN);

        timeKeeper_->startTimer(TRAJ_PICK);
        geometry_msgs::PoseArray chosen_virtual_traj_set;
        geometry_msgs::PoseArray picked_traj = pickTraj(traj_set, score_set, virtual_traj_set, chosen_virtual_traj_set);
        virtual_orient_traj_pub.publish(chosen_virtual_traj_set);
        timeKeeper_->stopTimer(TRAJ_PICK);

        timeKeeper_->startTimer(TRAJ_COMP);
        geometry_msgs::PoseArray chosen_final_virtual_traj_set;
        geometry_msgs::PoseArray final_traj = compareToCurrentTraj(picked_traj, chosen_final_virtual_traj_set);
        timeKeeper_->stopTimer(TRAJ_COMP);

        timeKeeper_->startTimer(COLL_CHECK);
        CollisionResults cc_results;
        if (collision_checker_enable_)
        {
            ros::WallTime start = ros::WallTime::now();

            // cc_results = checkCollision(final_traj);
            cc_results = checkCollision(chosen_final_virtual_traj_set);

            ROS_INFO_STREAM("Current trajectory collision checked in " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
            int cc_ite_min = 10;
            double cc_itc_ratio = 0.2;

            if(cc_results.collision_idx_ >= 0 && double(cc_results.collision_idx_) / cc_results.local_traj_.points.size() <= cc_itc_ratio)
            {
                ROS_WARN_STREAM("Current trajectory collides! " << cc_results.collision_idx_ << " " << cc_results.local_traj_.points.size());
                setCurrentTraj(geometry_msgs::PoseArray());
            }
        }
        timeKeeper_->stopTimer(COLL_CHECK);

        // delete set of planning gaps
        for (Gap * planningGap : planningGaps)
            delete planningGap;

        timeKeeper_->stopTimer(PLAN);
        timeKeeper_->computeAverageNumberGaps(gapCount);        

        return final_traj;
    }

    bool Planner::recordAndCheckVel(const geometry_msgs::Twist & cmd_vel) 
    {
        double val = std::abs(cmd_vel.linear.x) + std::abs(cmd_vel.linear.y) + std::abs(cmd_vel.angular.z);
        cmdVelBuffer.push_back(val);
        double cum_vel_sum = std::accumulate(cmdVelBuffer.begin(), cmdVelBuffer.end(), double(0));
        bool ret_val = cum_vel_sum > 1.0 || !cmdVelBuffer.full();
        if (!ret_val && !cfg_.man.man_ctrl) {
            ROS_FATAL_STREAM("--------------------------Planning Failed--------------------------");
            reset();
        }
        return ret_val || cfg_.man.man_ctrl;
    }

}