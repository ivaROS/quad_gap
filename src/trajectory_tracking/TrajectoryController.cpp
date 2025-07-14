#include <quad_gap/trajectory_tracking/TrajectoryController.h>

namespace quad_gap
{
    TrajectoryController::TrajectoryController(const rclcpp::Node::SharedPtr & node, const QuadGapConfig& cfg) 
    {
        projOpPublisher_ = node->create_publisher<visualization_msgs::msg::Marker>("po_dir", 10);
        cfg_ = & cfg;

        // thres = 0.1;
        // last_time = rclcpp::Time::now();

        l_ = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio; // error.norm();
    }

    void TrajectoryController::updateEgoCircle(boost::shared_ptr<sensor_msgs::msg::LaserScan const> scan)
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        scan_ = scan;
    }

    // [[deprecated("Not Used, Deemed Unnecessary")]]
    // std::vector<geometry_msgs::Point> TrajectoryController::findLocalLine(const int & idx) 
    // {
    //     sensor_msgs::msg::LaserScan egocircle = *scan_.get();
    //     std::vector<float> dist(egocircle.ranges.size());

    //     if (!scan_) {
    //         return std::vector<geometry_msgs::Point>(0);
    //     }
        
    //     if (egocircle.ranges.size() < 500) {
    //         ROS_FATAL_STREAM_NAMED("TrajectoryController", "Scan range incorrect findLocalLine");
    //     }



    //     for (int i = 1; i < dist.size(); i++) {
    //         float l1 = egocircle.ranges.at(i);
    //         float t1 = float(i) * egocircle.angle_increment + egocircle.angle_min;
    //         float l2 = egocircle.ranges.at(i - 1);
    //         float t2 = float(i - 1) * egocircle.angle_increment + egocircle.angle_min;
    //         if (l1 > 2.9) {
    //             dist.at(i) = 10;
    //         } else {
    //             dist.at(i) = polDist(l1, t1, l2, t2);
    //         } 
    //     }

    //     dist.at(0) = polDist(egocircle.ranges.at(0), egocircle.angle_min, egocircle.ranges.at(511), float(511) * egocircle.angle_increment + egocircle.angle_min);

    //     auto result_fwd = std::find_if(dist.begin() + idx, dist.end(), 
    //         std::bind1st(std::mem_fun(&TrajectoryController::geqThres), this));

    //     auto res_rev = std::find_if(dist.rbegin() + (dist.size() - idx), dist.rend(),
    //         std::bind1st(std::mem_fun(&TrajectoryController::geqThres), this));
        

    //     if (res_rev == dist.rend()) {
    //         return std::vector<geometry_msgs::Point>(0);
    //     }

    //     int idx_fwd = std::distance(dist.begin(), std::prev(result_fwd));
    //     int idx_rev = std::distance(res_rev, dist.rend());

    //     int min_idx_range = 0;
    //     int max_idx_range = int(egocircle.ranges.size() - 1);
    //     if (idx_fwd < min_idx_range || idx_fwd > max_idx_range || idx_rev < min_idx_range || idx_rev > max_idx_range) {
    //         return std::vector<geometry_msgs::Point>(0);
    //     }
        
    //     float dist_fwd = egocircle.ranges.at(idx_fwd);
    //     float dist_rev = egocircle.ranges.at(idx_rev);
    //     float dist_cent = egocircle.ranges.at(idx);

    //     float angle_fwd = float(idx_fwd) * egocircle.angle_increment + egocircle.angle_min;
    //     float angle_rev = float(idx_rev) * egocircle.angle_increment + egocircle.angle_min;
        
    //     if (idx_fwd < idx || idx_rev > idx) {
    //         return std::vector<geometry_msgs::Point>(0);
    //     }

    //     Eigen::Vector2f fwd_pol(dist_fwd, angle_fwd);
    //     Eigen::Vector2f rev_pol(dist_rev, angle_rev);
    //     Eigen::Vector2f cent_pol(dist_cent, float(idx) * egocircle.angle_increment + egocircle.angle_min);
    //     Eigen::Vector2f fwd_car = pol2car(fwd_pol);
    //     Eigen::Vector2f rev_car = pol2car(rev_pol);
    //     Eigen::Vector2f cent_car = pol2car(cent_pol);

    //     Eigen::Vector2f pf;
    //     Eigen::Vector2f pr;

    //     if (dist_cent < dist_fwd && dist_cent < dist_rev) {
    //         // ROS_INFO_STREAM_NAMED("TrajectoryController", "Non line");
    //         Eigen::Vector2f a = cent_car - fwd_car;
    //         Eigen::Vector2f b = rev_car - fwd_car;
    //         Eigen::Vector2f a1 = (a.dot(b / b.norm())) * (b / b.norm());
    //         Eigen::Vector2f a2 = a - a1;
    //         pf = fwd_car + a2;
    //         pr = rev_car + a2;
    //     } else {
    //         pf = fwd_car;
    //         pr = rev_car;
    //     }

    //     geometry_msgs::Point lower_point;
    //     lower_point.x = pf(0);
    //     lower_point.y = pf(1);
    //     lower_point.z = 3;
    //     geometry_msgs::Point upper_point;
    //     upper_point.x = pr(0);
    //     upper_point.y = pr(1);
    //     upper_point.z = 3;
    //     // if form convex hull
    //     std::vector<geometry_msgs::Point> retArr(0);
    //     retArr.push_back(lower_point);
    //     retArr.push_back(upper_point);
    //     return retArr;
    // }

    // bool TrajectoryController::geqThres(const float dist)
    // {
    //     return dist >= thres;
    // }

    // float TrajectoryController::polDist(const float & l1, const float & t1, const float & l2, const float & t2) 
    // {
    //     return abs(float(pow(l1, 2) + pow(l2, 2) - 2 * l1 * l2 * std::cos(t1 - t2)));
    // }

    // ,
    // const sensor_msgs::msg::LaserScan & inflated_egocircle, 
    // const geometry_msgs::msg::PoseStamped & init_pose    
    geometry_msgs::msg::Twist TrajectoryController::controlLawHolonomic(const geometry_msgs::msg::Pose & currentPoseOdomFrame, 
                                                                    const geometry_msgs::msg::Pose & desiredPoseOdomFrame) 
    {
        // Setup Vars
        boost::mutex::scoped_lock lock(scanMutex_);
        // bool holonomic = cfg_->planning.holonomic;
        // bool projection_operator = cfg_->planning.projection_operator;
        // float Kpz_ = cfg_->control.Kpz;
        // if (holonomic) Kpz_ = 0.8;
        // float Kpx_ = cfg_->control.Kpx;
        // float Kpy_ = cfg_->control.Kpy;
        // float k_po_ = cfg_->projection.k_po;
        // float v_ang_const = cfg_->control.v_ang_const;
        // float v_lin_x_const = cfg_->control.v_lin_x_const;
        // float v_lin_y_const = cfg_->control.v_lin_y_const;
        // float r_min = cfg_->projection.r_min;
        // float r_norm = cfg_->projection.r_norm;
        // float r_norm_offset = cfg_->projection.r_norm_offset; 
        // float k_po_turn_ = cfg_->projection.k_po_turn;

        // auto inflated_egocircle = *scan_.get();
        geometry_msgs::msg::Twist cmdVel = geometry_msgs::msg::Twist();

        // obtain roll, pitch, and yaw of current orientation (I think we're only using yaw)
        geometry_msgs::Quaternion currOrient = currentPoseOdomFrame.orientation;
        tf::Quaternion currQuat(currOrient.x, currOrient.y, currOrient.z, currOrient.w);
        float currYaw = quaternionToYaw(currQuat); 

        // get current x,y,theta
        geometry_msgs::Point currPosn = currentPoseOdomFrame.position;
        Eigen::Matrix2cf currRbtTransform = getComplexMatrix(currPosn.x, currPosn.y, currYaw);

        // obtaining RPY of desired orientation
        geometry_msgs::Point desPosn = desiredPoseOdomFrame.position;
        geometry_msgs::Quaternion desOrient = desiredPoseOdomFrame.orientation;
        tf::Quaternion desQuat(desOrient.x, desOrient.y, desOrient.z, desOrient.w);

        float desYaw = quaternionToYaw(desQuat);

        // get desired x,y,theta
        Eigen::Matrix2cf desRbtTransform = getComplexMatrix(desPosn.x, desPosn.y, desYaw);

        // get x,y,theta error
        Eigen::Matrix2cf errorMat = currRbtTransform.inverse() * desRbtTransform;
        float errorX = errorMat.real()(0, 1);
        float errorY = errorMat.imag()(0, 1);
        float errorTheta = std::arg(errorMat(0, 0));

        float v_lin_x_fb = errorX * cfg_->control.Kpx;
        float v_lin_y_fb = errorY * cfg_->control.Kpy;
        float v_ang_fb = cfg_->planning.heading * errorTheta * cfg_->control.Kpz;

        cmdVel.linear.x = v_lin_x_fb;
        cmdVel.linear.y = v_lin_y_fb;
        cmdVel.angular.z = v_ang_fb;

        // // ROS_INFO_STREAM_NAMED("TrajectoryController", init_pose.pose);
        
        // Eigen::Vector3f comp;
        // float prod_mul;
        // Eigen::Vector2f si_der;
        // Eigen::Vector2f v_err(v_lin_x_fb, v_lin_y_fb);


        // if (inflated_egocircle.ranges.size() < 500) 
        // {
        //     ROS_FATAL_STREAM_NAMED("TrajectoryController", "Scan range incorrect controlLaw");
        // }

        // if (holonomic)
        // {
        //     v_ang_fb = v_ang_fb + v_ang_const;
        //     v_lin_x_fb = abs(theta_error) > M_PI / 3 ? 0 : v_lin_x_fb + v_lin_x_const + k_po_ * u_add_x;
        //     v_lin_y_fb = abs(theta_error) > M_PI / 3 ? 0 : v_lin_y_fb + v_lin_y_const + k_po_ * u_add_y;

        //     if (v_lin_x_fb < 0)
        //         v_lin_x_fb = 0;
        // }
        // else
        // {
        //     v_ang_fb = v_ang_fb + v_lin_y_fb + k_po_turn_ * u_add_y + v_ang_const;
        //     v_lin_x_fb = v_lin_x_fb + v_lin_x_const + k_po_ * u_add_x;

        //     if (projection_operator && min_dist_ang > - M_PI / 4 && min_dist_ang < M_PI / 4 && min_dist < cfg_->rbt.r_inscr)
        //     {
        //         v_lin_x_fb = 0;
        //         v_ang_fb *= 2;
        //     }

        //     v_lin_y_fb = 0;

        //     if(v_lin_x_fb < 0)
        //         v_lin_x_fb = 0;
        // }

        // cmdVel.linear.x = std::max(-cfg_->rbt.vx_absmax, std::min(cfg_->rbt.vx_absmax, v_lin_x_fb));
        // cmdVel.linear.y = std::max(-cfg_->rbt.vy_absmax, std::min(cfg_->rbt.vy_absmax, v_lin_y_fb));
        // cmdVel.angular.z = std::max(-cfg_->rbt.vang_absmax, std::min(cfg_->rbt.vang_absmax, v_ang_fb));

        return cmdVel;
    }


    geometry_msgs::msg::Twist TrajectoryController::controlLawNonholonomic(const geometry_msgs::msg::Pose & currentPoseOdomFrame, 
                                                                        const geometry_msgs::msg::Pose & desiredPoseOdomFrame) 
    { 
        ROS_INFO_STREAM_NAMED("Controller", "    [constantVelocityControlLawNonHolonomicLookahead()]");
        // Setup Vars
        // boost::mutex::scoped_lock lock(scanMutex_);

        geometry_msgs::msg::Twist cmdVel = geometry_msgs::msg::Twist();

        // obtain roll, pitch, and yaw of current orientation (I think we're only using yaw)
        geometry_msgs::Quaternion currOrient = currentPoseOdomFrame.orientation;
        tf::Quaternion currQuat(currOrient.x, currOrient.y, currOrient.z, currOrient.w);
        float currYaw = quaternionToYaw(currQuat); 

        // get current x,y,theta
        geometry_msgs::Point currPosn = currentPoseOdomFrame.position;
        Eigen::Matrix2cf currRbtTransform = getComplexMatrix(currPosn.x, currPosn.y, currYaw);

        ROS_INFO_STREAM_NAMED("Controller", "        current pose x: " << currPosn.x << ", y: " << currPosn.y << ", yaw: " << currYaw);

        // obtaining RPY of desired orientation
        geometry_msgs::Point desPosn = desiredPoseOdomFrame.position;
        geometry_msgs::Quaternion desOrient = desiredPoseOdomFrame.orientation;
        tf::Quaternion desQuat(desOrient.x, desOrient.y, desOrient.z, desOrient.w);
        float desYaw = quaternionToYaw(desQuat);

        ROS_INFO_STREAM_NAMED("Controller", "        desired pose x: " << desiredPoseOdomFrame.position.x << ", y: " << desiredPoseOdomFrame.position.y << ", yaw: "<< desYaw);

        // get desired x,y,theta
        Eigen::Matrix2cf desRbtTransform = getComplexMatrix(desPosn.x, desPosn.y, desYaw);

        // get x,y,theta error
        Eigen::Matrix2cf errorMat = currRbtTransform.inverse() * desRbtTransform;
        float errorX = desPosn.x - currPosn.x; // errorMat.real()(0, 1);
        float errorY = desPosn.y - currPosn.y; // errorMat.imag()(0, 1);
        // float errorTheta = std::arg(errorMat(0, 0));

        float v_lin_x_fb = errorX * cfg_->control.Kpx;
        float v_lin_y_fb = errorY * cfg_->control.Kpy;
        // float v_ang_fb = cfg_->planning.heading * errorTheta * cfg_->control.Kpz;

        // Eigen::Vector2f errorDir = epsilonDivide(error, error.norm());

        Eigen::Vector2f constantVelocityCommand(v_lin_x_fb, v_lin_y_fb); 

        // lookahead distance

        // float l_adj = l; // 0.5 * l;

        ROS_INFO_STREAM_NAMED("Controller", "        error: (" << errorX << ", " << errorY << "), l: " << l_); //  << ", l_adj: " << l_adj

        Eigen::Matrix2f negRotMat = getRotMat(-currYaw);

        Eigen::Matrix2f nidMat = Eigen::Matrix2f::Identity();
        nidMat(1, 1) = (1.0 / l_);

        Eigen::Vector2f nonholoVelocityCommand = nidMat * negRotMat * constantVelocityCommand;

        float velLinXFeedback = nonholoVelocityCommand[0];
        float velLinYFeedback = 0.0;
        float velAngFeedback = nonholoVelocityCommand[1];

        // Just storing desired displacement for now
        cmdVel.linear.x = velLinXFeedback;
        cmdVel.linear.y = velLinYFeedback;
        cmdVel.angular.z = velAngFeedback;

        return cmdVel; 
    }


    /*
    Taken from the code provided in stdr_simulator repo. Probably does not perform too well.
    */
    geometry_msgs::msg::Twist TrajectoryController::obstacleAvoidanceControlLaw() 
    {
        ROS_INFO_STREAM_NAMED("Controller", "obstacle avoidance control");
        float safeDirX = 0;
        float safeDirY = 0;                                   
        
        float scanRange = 0.0, scanTheta = 0.0;
        for (int i = 0; i < scan_->ranges.size(); i++) 
        {
            scanRange = scan_->ranges.at(i);
            scanTheta =  idx2theta(i);

            safeDirX += epsilonDivide(-1.0 * std::cos(scanTheta), pow(scanRange, 2));
            safeDirY += epsilonDivide(-1.0 * std::sin(scanTheta), pow(scanRange, 2));
        }

        safeDirX /= scan_->ranges.size();
        safeDirY /= scan_->ranges.size();

        float cmdVelX = safeDirX;
        float cmdVelY = safeDirY;
        float cmdVelTheta = 0.0;


        ROS_INFO_STREAM_NAMED("Controller", "raw safe vels: " << cmdVelX << ", " << cmdVelY);
        ROS_INFO_STREAM_NAMED("Controller", "weighted safe vels: " << cmdVelX << ", " << cmdVelY);

        clipRobotVelocity(cmdVelX, cmdVelY, cmdVelTheta);

        ROS_INFO_STREAM_NAMED("Controller", "final safe vels: " << cmdVelX << ", " << cmdVelY);

        geometry_msgs::msg::Twist cmdVel = geometry_msgs::msg::Twist();
        cmdVel.linear.x = cmdVelX;
        cmdVel.linear.y = cmdVelY; 
        cmdVel.angular.z = cmdVelTheta;    

        return cmdVel;
    }

    /*
    Taken from the code provided in stdr_simulator repo. Probably does not perform too well.
    */
    geometry_msgs::msg::Twist TrajectoryController::obstacleAvoidanceControlLawNonHolonomic() 
    {
        ROS_INFO_STREAM_NAMED("Controller", "obstacle avoidance control");
        float safeDirX = 0;
        float safeDirZ = 0;                                   
        
        float scanRange = 0.0, scanTheta = 0.0;
        for (int i = 0; i < scan_->ranges.size(); i++) 
        {
            scanRange = scan_->ranges.at(i);
            scanTheta =  idx2theta(i);

            safeDirX += epsilonDivide(-1.0 * std::cos(scanTheta), pow(scanRange, 2));
            safeDirZ += epsilonDivide(-1.0 * std::sin(scanTheta), pow(scanRange, 2));
        }

        safeDirX /= scan_->ranges.size();
        safeDirZ /= scan_->ranges.size();

        ROS_INFO_STREAM_NAMED("Controller", "raw safe vels: x: " << safeDirX << ", z: " << safeDirZ);

        // clipRobotVelocity(cmdVelX, cmdVelY, cmdVelTheta);

        geometry_msgs::msg::Twist cmdVel = geometry_msgs::msg::Twist();

        float clippedCmdVelX = 0.0;
        if (std::abs(safeDirX) < cfg_->rbt.vx_absmax)
        {
            clippedCmdVelX = safeDirX;
        } else
        {
            clippedCmdVelX = cfg_->rbt.vx_absmax * epsilonDivide(safeDirX, std::abs(safeDirX));
        }

        cmdVel.linear.x = clippedCmdVelX;
        cmdVel.linear.y = 0.0;
        cmdVel.angular.z = std::max(-cfg_->rbt.vang_absmax, std::min(cfg_->rbt.vang_absmax, safeDirZ));

        ROS_INFO_STREAM_NAMED("Controller", "final safe vels: " << cmdVel.linear.x << ", " << cmdVel.angular.z);

        return cmdVel;
    }

    geometry_msgs::msg::Twist TrajectoryController::processCmdVelHolonomic(const geometry_msgs::msg::Twist & rawCmdVel,
                                                                        const geometry_msgs::msg::PoseStamped & rbtPoseInSensorFrame) 
    {
        ROS_INFO_STREAM_NAMED("Controller", "    [processCmdVel()]");

        geometry_msgs::msg::Twist cmdVel = geometry_msgs::msg::Twist();

        float velLinXFeedback = rawCmdVel.linear.x;
        float velLinYFeedback = rawCmdVel.linear.y;
        float velAngFeedback = rawCmdVel.angular.z;

        // ROS_INFO_STREAM_NAMED("Controller", rbtPoseInSensorFrame.pose);
        float minRangeTheta = 0;
        float minRange = 0;

        ROS_INFO_STREAM_NAMED("Controller", "        feedback command velocities: " << velLinXFeedback << ", " << velLinYFeedback);

        // applies PO
        float velLinXSafe = 0.;
        float velLinYSafe = 0.;
        
        if (cfg_->planning.projection_operator)
        {
            ROS_INFO_STREAM_NAMED("Controller", "        running projection operator");
            
            Eigen::Vector2f cmdVelFeedback(rawCmdVel.linear.x, rawCmdVel.linear.y);

            runProjectionOperator(rbtPoseInSensorFrame,
                                    cmdVelFeedback, velLinXSafe, velLinYSafe,
                                    minRangeTheta, minRange);
            
        } else 
        {
            ROS_DEBUG_STREAM_THROTTLE_NAMED(10, "Controller", "Projection operator off");
        }
        
        float weightedVelLinXSafe = cfg_->projection.k_po_x * velLinXSafe;
        float weightedVelLinYSafe = cfg_->projection.k_po_x * velLinYSafe;

        ROS_INFO_STREAM_NAMED("Controller", "        safe command velocity, v_x:" << weightedVelLinXSafe << ", v_y: " << weightedVelLinYSafe);

        // cmdVel_safe
        visualizeProjectionOperator(weightedVelLinXSafe, weightedVelLinYSafe, minRangeTheta, minRange);

        velLinXFeedback += weightedVelLinXSafe;
        velLinYFeedback += weightedVelLinYSafe; 

        ROS_INFO_STREAM_NAMED("Controller", "        summed command velocity, v_x:" << velLinXFeedback << ", v_y: " << velLinYFeedback << ", v_ang: " << velAngFeedback);
        clipRobotVelocity(velLinXFeedback, velLinYFeedback, velAngFeedback);
        ROS_INFO_STREAM_NAMED("Controller", "        clipped command velocity, v_x:" << velLinXFeedback << ", v_y: " << velLinYFeedback << ", v_ang: " << velAngFeedback);

        cmdVel.linear.x = velLinXFeedback;
        cmdVel.linear.y = velLinYFeedback;
        cmdVel.angular.z = velAngFeedback;

        // ROS_INFO_STREAM_NAMED("Controller", "ultimate command velocity: " << cmdVel.linear.x << ", " << cmdVel.linear.y << ", " << cmdVel.angular.z);

        return cmdVel;       
    }


    geometry_msgs::msg::Twist TrajectoryController::processCmdVelNonholonomic(const geometry_msgs::msg::Pose & currentPoseOdomFrame,
                                                                            const geometry_msgs::msg::Pose & desiredPoseOdomFrame,
                                                                            const geometry_msgs::msg::Twist & nonholoCmdVel,
                                                                            const geometry_msgs::msg::PoseStamped & rbtPoseInSensorFrame) 
    {
        ROS_INFO_STREAM_NAMED("Controller", "    [processCmdVelNonHolonomic()]");

        // obtain roll, pitch, and yaw of current orientation (I think we're only using yaw)
        geometry_msgs::Quaternion currOrient = currentPoseOdomFrame.orientation;
        tf::Quaternion currQuat(currOrient.x, currOrient.y, currOrient.z, currOrient.w);
        float currYaw = quaternionToYaw(currQuat); 

        // get current x,y,theta
        geometry_msgs::Point currPosn = currentPoseOdomFrame.position;
        Eigen::Matrix2cf currRbtTransform = getComplexMatrix(currPosn.x, currPosn.y, currYaw);

        ROS_INFO_STREAM_NAMED("Controller", "        current pose x: " << currPosn.x << ", y: " << currPosn.y << ", yaw: " << currYaw);

        // // obtaining RPY of desired orientation
        // geometry_msgs::Point desPosn = desiredPoseOdomFrame.position;
        // geometry_msgs::Quaternion desOrient = desiredPoseOdomFrame.orientation;
        // tf::Quaternion desQuat(desOrient.x, desOrient.y, desOrient.z, desOrient.w);
        // float desYaw = quaternionToYaw(desQuat);

        // ROS_INFO_STREAM_NAMED("Controller", "        desired pose x: " << desiredPoseOdomFrame.position.x << ", y: " << desiredPoseOdomFrame.position.y << ", yaw: "<< desYaw);

        // // get desired x,y,theta
        // Eigen::Matrix2cf desRbtTransform = getComplexMatrix(desPosn.x, desPosn.y, desYaw);

        // // get x,y,theta error
        // Eigen::Matrix2cf errorMat = currRbtTransform.inverse() * desRbtTransform;
        // float errorX = desPosn.x - currPosn.x; // errorMat.real()(0, 1);
        // float errorY = desPosn.y - currPosn.y; // errorMat.imag()(0, 1);
        // float errorTheta = std::arg(errorMat(0, 0));

        // Eigen::Vector2f error(errorX, errorY);

        // float l = error.norm();

        // float l_adj = l; // 0.5 * l;

        // Map nonholonomic command velocities to holonomic command velocities
        geometry_msgs::msg::Twist holoCmdVel = geometry_msgs::msg::Twist();
        holoCmdVel.linear.x = nonholoCmdVel.linear.x;
        holoCmdVel.linear.y = l_ * nonholoCmdVel.angular.z;
        holoCmdVel.angular.z = 0.0;



        // float errorX = rawCmdVel.linear.x;
        // float errorY = rawCmdVel.linear.y;
        // float errorTheta = rawCmdVel.angular.z;

        // ROS_INFO_STREAM_NAMED("Controller", rbtPoseInSensorFrame.pose);
        float minRangeTheta = 0;
        float minRange = 0;

        // ROS_INFO_STREAM_NAMED("Controller", "        feedback errors: x: " << errorX << ", y: " << errorY << ", theta: " << errorTheta);

        // applies PO
        float velLinXSafe = 0.;
        float velLinYSafe = 0.;
        
        if (cfg_->planning.projection_operator)
        {
            ROS_INFO_STREAM_NAMED("Controller", "        running projection operator");

            Eigen::Vector2f holoCmdVelVector(holoCmdVel.linear.x, holoCmdVel.linear.y);

            runProjectionOperator(rbtPoseInSensorFrame,
                                    holoCmdVelVector, velLinXSafe, velLinYSafe,
                                    minRangeTheta, minRange);
            
        } else 
        {
            ROS_DEBUG_STREAM_THROTTLE_NAMED(10, "Controller", "Projection operator off");
        }
        
        float weightedVelLinXSafe = cfg_->projection.k_po_x * velLinXSafe;
        float weightedVelLinYSafe = cfg_->projection.k_po_x * velLinYSafe;

        // cmdVel_safe
        visualizeProjectionOperator(weightedVelLinXSafe, weightedVelLinYSafe, minRangeTheta, minRange);

        ROS_INFO_STREAM_NAMED("Controller", "        safe desired direction, v_x:" << weightedVelLinXSafe << ", v_y: " << weightedVelLinYSafe);

        float safeErrorX = holoCmdVel.linear.x + weightedVelLinXSafe;
        float safeErrorY = holoCmdVel.linear.y + weightedVelLinYSafe; 

        ROS_INFO_STREAM_NAMED("Controller", "        summed desired direction, v_x:" << safeErrorX << ", v_y: " << safeErrorY);
        Eigen::Vector2f safeError(safeErrorX, safeErrorY);

        // Eigen::Matrix2f negRotMat = getRotMat(-currYaw);

        Eigen::Matrix2f nidMat = Eigen::Matrix2f::Identity();
        nidMat(1, 1) = (1.0 / l_);

        Eigen::Vector2f nonholoVelocityCommand = nidMat * safeError; // negRotMat * 

        float velLinXFeedback = nonholoVelocityCommand[0]; // nonholoCmdVel.linear.x; // 
        float velLinYFeedback = 0.0;
        float velAngFeedback = nonholoVelocityCommand[1]; // nonholoCmdVel.angular.z; //  

        ROS_INFO_STREAM_NAMED("Controller", "        generating nonholonomic control signal");            
        ROS_INFO_STREAM_NAMED("Controller", "        Feedback command velocities, v_x: " << velLinXFeedback << ", v_ang: " << velAngFeedback);

        float clippedVelLinXFeedback = 0.0;
        if (std::abs(velLinXFeedback) < cfg_->rbt.vx_absmax)
        {
            clippedVelLinXFeedback = velLinXFeedback;
        } else
        {
            clippedVelLinXFeedback = cfg_->rbt.vx_absmax * epsilonDivide(velLinXFeedback, std::abs(velLinXFeedback));
        }

        geometry_msgs::msg::Twist cmdVel = geometry_msgs::msg::Twist();
        cmdVel.linear.x = clippedVelLinXFeedback;
        cmdVel.linear.y = 0.0;
        cmdVel.angular.z = std::max(-cfg_->rbt.vang_absmax, std::min(cfg_->rbt.vang_absmax, velAngFeedback));

        // clipRobotVelocity(velLinXFeedback, velLinYFeedback, velAngFeedback);
        ROS_INFO_STREAM_NAMED("Controller", "        clipped nonholonomic command velocity, v_x:" << cmdVel.linear.x << ", v_ang: " << cmdVel.angular.z);

        return cmdVel;
    }

    void TrajectoryController::visualizeProjectionOperator(const float & weightedVelLinXSafe, 
                                                           const float & weightedVelLinYSafe,
                                                           const float & minRangeTheta, 
                                                           const float & minRange) 
    {
        ROS_INFO_STREAM_NAMED("Controller", "[visualizeProjectionOperator()]");

        if (cfg_->robot_frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Controller", "robot_frame_id not set, cannot visualize projection operator");
            return;
        }

        visualization_msgs::msg::Marker projOpMarker;
        projOpMarker.header.frame_id = cfg_->robot_frame_id;
        projOpMarker.header.stamp = rclcpp::Time();
        projOpMarker.id = 0;

        projOpMarker.type = visualization_msgs::msg::Marker::ARROW;
        projOpMarker.action = visualization_msgs::msg::Marker::ADD;
        projOpMarker.pose.position.x = minRange * std::cos(minRangeTheta);
        projOpMarker.pose.position.y = minRange * std::sin(minRangeTheta);
        projOpMarker.pose.position.z = 0.01;
        float dir = std::atan2(weightedVelLinYSafe, weightedVelLinXSafe);
        tf2::Quaternion projOpQuat;
        projOpQuat.setRPY(0, 0, dir);
        projOpMarker.pose.orientation = tf2::toMsg(projOpQuat);

        projOpMarker.scale.x = sqrt(pow(weightedVelLinXSafe, 2) + pow(weightedVelLinYSafe, 2)) + 0.00001;
        projOpMarker.scale.y = 0.1;
        projOpMarker.scale.z = 0.000001;
        

        
        projOpMarker.color.a = 1;
        projOpMarker.color.r = 0.0;
        projOpMarker.color.g = 0.0;
        projOpMarker.color.b = 0.0;
        projOpMarker.lifetime = ros::Duration(0);

        projOpPublisher_.publish(projOpMarker);
    }

    void TrajectoryController::clipRobotVelocity(float & velLinXFeedback, float & velLinYFeedback, float & velAngFeedback) 
    {
        float speedLinXFeedback = std::abs(velLinXFeedback);
        float speedLinYFeedback = std::abs(velLinYFeedback);
        
        if (speedLinXFeedback <= cfg_->rbt.vx_absmax && speedLinYFeedback <= cfg_->rbt.vy_absmax) 
        {
            // std::cout << "not clipping" << std::endl;
        } else 
        {
            velLinXFeedback *= epsilonDivide(cfg_->rbt.vx_absmax, std::max(speedLinXFeedback, speedLinYFeedback));
            velLinYFeedback *= epsilonDivide(cfg_->rbt.vy_absmax, std::max(speedLinXFeedback, speedLinYFeedback));
        }

        std::max(-cfg_->rbt.vang_absmax, std::min(cfg_->rbt.vang_absmax, velAngFeedback));
        return;
    }

    void TrajectoryController::runProjectionOperator(const geometry_msgs::msg::PoseStamped & rbtPoseInSensorFrame,
                                                     Eigen::Vector2f & cmdVelFeedback,
                                                     float & velLinXSafe, float & velLinYSafe,
                                                     float & minRangeTheta, float & minRange) 
    {
        ROS_INFO_STREAM_NAMED("Controller", "        [runProjectionOperator()]");
        float Psi = 0.0;
        Eigen::Vector2f dPsiDx(0.0, 0.0);

        // iterates through current egocircle and finds the minimum distance to the robot's pose
        // ROS_INFO_STREAM_NAMED("Controller", "rbtPoseInSensorFrame pose: " << rbtPoseInSensorFrame.pose.position.x << ", " << rbtPoseInSensorFrame.pose.position.y);
        std::vector<float> minScanDists(scan_->ranges.size());
        float theta = 0.0, dist = 0.0;
        for (int i = 0; i < minScanDists.size(); i++) 
        {
            theta = idx2theta(i);
            dist = scan_->ranges.at(i);
            minScanDists.at(i) = dist2Pose(theta, dist, rbtPoseInSensorFrame.pose);
        }
        auto minDistScanIter = std::min_element(minScanDists.begin(), minScanDists.end());
        int minDistScanIdx = std::distance(minScanDists.begin(), minDistScanIter);
        minRangeTheta = idx2theta(minDistScanIdx);

        minRange = minScanDists.at(minDistScanIdx);

        ROS_INFO_STREAM_NAMED("Controller", "           minDistScanIdx: " << minDistScanIdx << ", minRangeTheta: "<< minRangeTheta << ", minRange: " << minRange);
        // ROS_INFO_STREAM_NAMED("Controller", "min_x: " << min_x << ", min_y: " << min_y);
              
        Eigen::Vector2f closestScanPtToRobot(-minRange * std::cos(minRangeTheta), -minRange * std::sin(minRangeTheta));

        // float psi = 0.0;
        // Eigen::Vector2f dPsiDx(0.0, 0.0);
        // Eigen::Vector3f PsiDerAndPsi = 
        calculateProjectionOperator(closestScanPtToRobot, Psi, dPsiDx); // return Psi, and dPsiDx
        // dPsiDx = Eigen::Vector2f(PsiDerAndPsi(0), PsiDerAndPsi(1));

        Eigen::Vector2f normDPsiDx = dPsiDx.normalized();

        float projOpDotProd = cmdVelFeedback.dot(normDPsiDx);

        // Psi = PsiDerAndPsi(2);

        ROS_INFO_STREAM_NAMED("Controller", "           Psi: " << Psi);
        ROS_INFO_STREAM_NAMED("Controller", "           dPsiDx: " << dPsiDx[0] << ", " << dPsiDx[1]);
        ROS_INFO_STREAM_NAMED("Controller", "           Dot product check: " << projOpDotProd);

        if (Psi >= 0 && projOpDotProd >= 0)
        {
            velLinXSafe = - Psi * projOpDotProd * normDPsiDx(0);
            velLinYSafe = - Psi * projOpDotProd * normDPsiDx(1);
        }
        ROS_INFO_STREAM_NAMED("Controller", "           cmdVel_safe: " << velLinXSafe << ", " << velLinYSafe);
    }

    void TrajectoryController::calculateProjectionOperator(const Eigen::Vector2f & closestScanPtToRobot,
                                                            float & Psi, Eigen::Vector2f & dPsiDx)

    {
        float rUnity = cfg_->projection.r_unity;
        float rZero = cfg_->projection.r_zero;

        float minRange = closestScanPtToRobot.norm(); // sqrt(pow(min_diff_x, 2) + pow(min_diff_y, 2)); // (closest_pt - rbt)
        
        // Psi
        Psi = (rUnity / minRange - rUnity / rZero) / (1.0 - rUnity / rZero);
        
        // dPsiDx
        float derivativeDenominator = pow(minRange, 3) * (rUnity - rZero);
        float derivatorNominatorTerm = rUnity * rZero;
        float PsiDerivativeXTerm = epsilonDivide(derivatorNominatorTerm * closestScanPtToRobot[0], derivativeDenominator);
        float PsiDerivativeYTerm = epsilonDivide(derivatorNominatorTerm * closestScanPtToRobot[1], derivativeDenominator);

        // float dPsiDxNorm = sqrt(pow(PsiDerivativeXTerm, 2) + pow(PsiDerivativeYTerm, 2));
        // float normPsiDerivativeXTerm = epsilonDivide(PsiDerivativeXTerm, dPsiDxNorm);
        // float normPsiDerivativeYTerm = epsilonDivide(PsiDerivativeYTerm, dPsiDxNorm);
        
        dPsiDx[0] = PsiDerivativeXTerm;
        dPsiDx[1] = PsiDerivativeYTerm;
        
        // return Eigen::Vector3f(normPsiDerivativeXTerm, normPsiDerivativeYTerm, Psi);
    }

    Eigen::Matrix2cf TrajectoryController::getComplexMatrix(const float & x, const float & y, const float & quat_w, const float & quat_z)
    {
        std::complex<float> phase(quat_w, quat_z);
        phase = phase * phase;

        Eigen::Matrix2cf g(2, 2);
        //g.real()(0,0) = phase.real();
        g.real()(0, 1) = x;
        g.real()(1, 0) = 0;
        g.real()(1, 1) = 1;

        //g.imag()(0,0) = phase.imag();
        g.imag()(0, 1) = y;
        g.imag()(1, 0) = 0;
        g.imag()(1, 1) = 0;

        g(0, 0) = phase;

        return g;
    }

    Eigen::Matrix2cf TrajectoryController::getComplexMatrix(const float & x, const float & y, const float & theta)
    {
        std::complex<float> phase(std::cos(theta), std::sin(theta));

        Eigen::Matrix2cf g(2, 2);
        //g.real()(0,0) = phase.real();
        g.real()(0, 1) = x;
        g.real()(1, 0) = 0;
        g.real()(1, 1) = 1;

        //g.imag()(0,0) = phase.imag();
        g.imag()(0, 1) = y;
        g.imag()(1, 0) = 0;
        g.imag()(1, 1) = 0;

        g(0, 0) = phase;

        return g;
    }

    int TrajectoryController::extractTargetPoseIdx(const geometry_msgs::msg::Pose & currPose, const geometry_msgs::msg::PoseArray & localTrajectory) 
    {
        // Find pose right ahead
        std::vector<float> localTrajectoryDeviations(localTrajectory.poses.size());
        ROS_INFO_STREAM_NAMED("Controller", "[extractTargetPoseIdx()]");

        // obtain distance from entire ref traj and current pose
        for (int i = 0; i < localTrajectoryDeviations.size(); i++) // i will always be positive, so this is fine
        {
            tf::Quaternion currQuatInv(currPose.orientation.x, currPose.orientation.y, currPose.orientation.z, -currPose.orientation.w); // -w for inverse
  
            tf::Quaternion desQuat(localTrajectory.poses[i].orientation.x, localTrajectory.poses[i].orientation.y, 
                                   localTrajectory.poses[i].orientation.z, localTrajectory.poses[i].orientation.w);
            
            tf::Quaternion deviationQuat = desQuat * currQuatInv;
            float deviationYaw = quaternionToYaw(deviationQuat);

            // ROS_INFO_STREAM_NAMED("Controller", "   pose" << i << ", yaw_curr: " << yaw_curr << ", yaw_des: " << yaw_des << ", deviationYaw: " << deviationYaw);

            localTrajectoryDeviations.at(i) = sqrt(pow(currPose.position.x - localTrajectory.poses[i].position.x, 2) + 
                                                pow(currPose.position.y - localTrajectory.poses[i].position.y, 2)) + 
                                                0.5 * std::abs(deviationYaw);
        }

        // find pose in ref traj with smallest difference
        auto minimumDeviationIter = std::min_element(localTrajectoryDeviations.begin(), localTrajectoryDeviations.end());
        
        // go n steps ahead of pose with smallest difference
        int targetPose = std::distance(localTrajectoryDeviations.begin(), minimumDeviationIter) + cfg_->control.ctrl_ahead_pose;

        // make sure pose does note exceed trajectory size
        return std::min(targetPose, int(localTrajectory.poses.size() - 1));
    }

    // float TrajectoryController::dist2Pose(const float & theta, const float & dist, const geometry_msgs::msg::Pose & pose) 
    // {
    //     float x = dist * std::cos(theta);
    //     float y = dist * std::sin(theta);
    //     return sqrt(pow(pose.position.x - x, 2) + pow(pose.position.y - y, 2));
    // }

    // Eigen::Vector2f TrajectoryController::car2pol(const Eigen::Vector2f & a) 
    // {
    //     return Eigen::Vector2f(a.norm(), float(std::atan2(a(1), a(0))));
    // }

    // Eigen::Vector2f TrajectoryController::pol2car(const Eigen::Vector2f & a) 
    // {
    //     return Eigen::Vector2f(cos(a(1)) * a(0), sin(a(1)) * a(0));
    // }


}
