#pragma once


/////////////
// QUADGAP //
/////////////
#include <quad_gap/utils/Gap.h>
#include <quad_gap/utils/Trajectory.h>
// #include "quad_gap/TrajPlan.h"
#include <quad_gap/utils/Gap.h>
#include <quad_gap/utils/Utils.h>
#include <quad_gap/gap_detection/GapDetector.h>
#include <quad_gap/config/QuadGapConfig.h>
#include <quad_gap/visualization/TrajectoryVisualizer.h>
#include <quad_gap/visualization/GoalVisualizer.h>
#include <quad_gap/visualization/GapVisualizer.h>
#include <quad_gap/global_plan_management/GlobalPlanManager.h>
#include <quad_gap/trajectory_evaluation/TrajectoryEvaluator.h>
#include <quad_gap/trajectory_generation/GapManipulator.h>
#include <quad_gap/trajectory_generation/GapGoalPlacer.h>
#include <quad_gap/trajectory_tracking/TrajectoryController.h>
#include <quad_gap/TimeKeeper.h>

// #include <quad_gap/CollisionCheckerConfig.h>

#include <quad_gap/utils/RobotGeometryStorage.h>
#include <quad_gap/utils/RobotGeometryProcessor.h>

/////////
// ROS //
/////////

// #include <ros/ros.h>
// #include <ros/package.h>
#include "rclcpp/rclcpp.hpp"

//////////////
// ROS MSGS //
//////////////
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

//////////
// MISC //
//////////

#include <boost/numeric/odeint.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <math.h>
#include <chrono>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/header.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include <tf2_utils/transform_manager.h>

#include <omp.h>

// #include <dynamic_reconfigure/server.h>
// #include <quad_gap/qgConfig.h>

#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>

// #include <turtlebot_trajectory_testing/turtlebot_trajectory_tester.h>
// // #include <pips_trajectory_testing/pips_trajectory_tester.h>
// #include <pips_trajectory_msgs/trajectory_points.h>
// #include <pips_trajectory_testing/pips_cc_wrapper.h>
// #include <pips_trajectory_testing/depth_image_cc_wrapper.h>
// #include <pips_egocylindrical/egocylindrical_image_cc_wrapper.h>
// #include <pips_egocircle/egocircle_cc_wrapper.h>

namespace quad_gap
{
    // struct CollisionResults
    // {
    //     int collision_idx_ = -1;
    //     pips_trajectory_msgs::trajectory_points local_traj_;
        
    //     CollisionResults()
    //     {
    //         collision_idx_ = -1;
    //     }

    //     CollisionResults(int collision_idx, pips_trajectory_msgs::trajectory_points local_traj)
    //     {
    //         collision_idx_ = collision_idx;
    //         local_traj_ = local_traj;
    //     }
    // };

    class Planner
    {
        public:
            ~Planner();

            /**
            * \brief initialize Planner class
            * 
            * \param name planner name (used for ROS namespaces) 
            * \return initialization success / failure
            */
            bool initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & node);

            /**
            * \brief Indicator for if planner has been initialized
            * \return boolean for if planner has been initialized 
            */
            int initialized() { return initialized_; } 

            /**
            * Check if reached goal using euclidean dist
            * @param None, internally stored goal location and robot position
            * @return bool reached
            */
            bool isGoalReached();

            /**
            * call back function to laserscan, externally linked
            * @param msg laser scan msg
            * @return None, laser scan msg stored locally
            */
            void laserScanCB(boost::shared_ptr<sensor_msgs::msg::LaserScan> msg);

            /**
            * call back function to pose, pose information obtained here only used when a new goal is used
            * @param msg pose msg
            * @return None
            */
            void poseCB(const nav_msgs::msg::Odometry::ConstPtr& msg);

            /**
            * \brief Function for updating all tf transform at the beginning of every planning cycle
            * \param msg incoming agent odometry message
            */
            void tfCB(const tf2_msgs::msg::TFMessage& msg);

            /**
            * Interface function for receiving global plan
            * @param plan, vector of PoseStamped
            * @return boolean type on whether successfully registered goal
            */
            void setPlan(const std::vector<geometry_msgs::msg::PoseStamped> &plan);

            // /**
            // * update all tf transform at the beginning of every planning cycle
            // * @param None, all tf received via TF
            // * @return None, all registered via internal variables in TransformStamped
            // */
            // void updateTF();

            // /**
            //  * select the gap to pass through based on where the goal is
            //  * TODO: make this polymorphism so more than one strategy can be adopted
            //  * @param selected_gap that will be returned by the same variable
            //  * @return selected_gap via the passed in variable
            //  */
            // void vectorSelectGap(Gap & selected_gap);

            /**
            * Generate ctrl command to a target pose
            * TODO: fix vector pop and get rid of pose_counter
            * @param pose_arr_odom
            * @return cmd_vel by assigning to pass by reference
            */
            geometry_msgs::msg::Twist ctrlGeneration(const Trajectory & traj);
            
            /**
            * Take current observed gaps and perform gap conversion
            * @param None, directly taken from private variable space
            * @return manipGaps, simplfied radial prioritized gaps
            */
            std::vector<Gap *> gapManipulate(const std::vector<Gap *> & planning_gaps);

            /**
            * Place goal in the gap, this is done by the GapGoalPlacer class
            * @param planningGaps, vector of Gap pointers
            * @return None, goal is placed in the Gap object
            */
            void gapGoalPlace(const std::vector<Gap *> & planningGaps);

            void generateGapTrajectories(const std::vector<Gap *> & vec, 
                                            std::vector<Trajectory> & gapTrajs);

            // /**
            // * Callback function to config object
            // * @param incoming config
            // * @param level Level of incoming config
            // */
            // void rcfgCallback(qgConfig &config, uint32_t level);

            /**
            * Pick the best trajectory from the current set
            * @param Vector of PoseArray
            * @param Vector of corresponding trajectory scores
            * @return the best trajectory
            */         
            int pickTraj(const std::vector<Trajectory> & gapTrajs);

            /**
            * Compare to the old trajectory and pick the best one
            * @param incoming trajectory
            * @return the best trajectory  
            */
            Trajectory compareToCurrentTraj(Trajectory & incomingTraj);

            // geometry_msgs::msg::PoseArray getOrientDecayedPath(const geometry_msgs::msg::PoseArray & orig_path);

            // CollisionResults checkCollision(const Trajectory & traj);

            /**
            * \brief Function for getting index of closest pose in trajectory
            * \param currTrajRbtFrame current trajectory in robot frame
            * \return index of closest pose in trajectory
            */
            int getClosestTrajectoryPoseIdx(const geometry_msgs::msg::PoseArray & currTrajRbtFrame);

            /**
            * Setter and Getter of Current Trajectory, this is performed in the compareToCurrentTraj function
            */
            void setCurrentTraj(const Trajectory & currTraj);   

            Trajectory getCurrentTraj();

            /**
            * Conglomeration of getting a plan Trajectory
            * @return the trajectory
            */
            Trajectory runPlanningLoop();    

            /**
            * Reset Planner, clears current observedSet
            */
            void reset();

            /**
            * Check if the robot has been stuck
            * @param command velocity
            * @return False if robot has been stuck for the past cfg.planning.halt_size iterations
            */
            bool recordAndCheckVel(const geometry_msgs::msg::TwistStamped & cmd_vel);
            
            // void setCCWrapper(const std::shared_ptr<pips_trajectory_testing::PipsCCWrapper>& cc_wrapper)
            // {
            //     cc_wrapper_ = cc_wrapper;
            // }

            // std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> getCCWrapper()
            // {
            //     return cc_wrapper_;
            // }

            // bool ccEnabled()
            // {
            //     return collision_checker_enable_;
            // }

            // typedef TurtlebotGenAndTest::trajectory_ptr trajectory_ptr;
            // typedef TurtlebotGenAndTest::traj_func_type traj_func_type;
            // typedef TurtlebotGenAndTest::traj_func_ptr traj_func_ptr;
            // typedef TurtlebotGenAndTest::trajectory_points trajectory_points;
            // typedef TurtlebotGenAndTest::TrajBridge TrajBridge;
            // typedef std::shared_ptr<TurtlebotGenAndTest> GenAndTest_ptr;

            // std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cc_wrapper_;
            // GenAndTest_ptr traj_tester_;

            // bool collision_checker_enable_ = false;
            // int cc_type_ = -1;

            // typedef dynamic_reconfigure::Server<CollisionCheckerConfig> ReconfigureServer;
            // std::shared_ptr<ReconfigureServer> reconfigure_server_;

            // void configCB(CollisionCheckerConfig &config, uint32_t level);

            /**
            * \brief Function to check if global goal has been reached
            * \param status whether or not global goal has been reached 
            */
            void setReachedGlobalGoal(const bool & status) { reachedGlobalGoal_ = status; }

            void updateEgoCircle();

        private:

            Trajectory changeTrajectoryHelper(Trajectory & incomingTraj,
                                              const bool & switchToIncoming);

            std::vector<Gap *> deepCopyCurrentSimplifiedGaps();

            boost::shared_ptr<sensor_msgs::msg::LaserScan const> transformLaserToRbt(boost::shared_ptr<sensor_msgs::msg::LaserScan const> msg);

            // Transforms
            geometry_msgs::msg::TransformStamped map2rbt_;
            geometry_msgs::msg::TransformStamped rbt2map_;
            geometry_msgs::msg::TransformStamped odom2rbt_;
            geometry_msgs::msg::TransformStamped rbt2odom_;
            geometry_msgs::msg::TransformStamped map2odom_;
            geometry_msgs::msg::TransformStamped cam2odom_;
            geometry_msgs::msg::TransformStamped odom2cam_;
            geometry_msgs::msg::TransformStamped rbt2cam_;
            geometry_msgs::msg::TransformStamped cam2rbt_;

            // Robot poses
            geometry_msgs::msg::PoseStamped rbtPoseInRbtFrame_;
            geometry_msgs::msg::PoseStamped rbtPoseInSensorFrame_;
            geometry_msgs::msg::PoseStamped rbtPoseInOdomFrame_;
            
            std::shared_ptr<tf2_ros::Buffer> tfBuffer;
            std::shared_ptr<tf2_ros::TransformListener> tfListener;

            // ros::NodeHandle nh, pnh;
            // ros::Publisher trajectory_pub;

            rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr transformed_laser_pub;

            bool reachedGlobalGoal_ = false; /**< Flag for if global goal has been reached */
            bool hasLaserScan_ = false;
            bool initialized_ = false;

            // Goals and stuff
            geometry_msgs::msg::PoseStamped globalGoalOdomFrame_; /**< Global goal in odometry frame */
            geometry_msgs::msg::PoseStamped globalGoalRobotFrame_; /**< Global goal in robot frame */
            geometry_msgs::msg::PoseStamped globalPathLocalWaypointOdomFrame_; /**< Global path local waypoint in odometry frame */

            // geometry_msgs::msg::PoseStamped local_waypoint_odom; // local_waypoint, 
            // geometry_msgs::msg::PoseStamped final_goal_odom;

            // Gaps:
            std::vector<Gap *> currRawGaps_;
            std::vector<Gap *> currSimpGaps_;
            std::vector<Gap *> prevRawGaps_; /**< Previous set of raw gaps */
            std::vector<Gap *> prevSimpGaps_; /**< Previous set of simplified gaps */

            // std::vector<Gap *> currSimpGaps__ptr;

            // Helper modules
            GapDetector * gapDetector_  = NULL; 
            GapVisualizer * gapVisualizer_  = NULL; 
            GlobalPlanManager * globalPlanManager_  = NULL; 
            TrajectoryVisualizer * trajVisualizer_  = NULL; 
            GoalVisualizer * goalVisualizer_  = NULL; 
            TrajectoryEvaluator * trajEvaluator_  = NULL; 
            GapGoalPlacer * gapGoalPlacer_ = NULL; /**< Gap goal placer, used to place goal in gap */
            GapTrajGenerator * gapTrajGenerator_  = NULL; 
            GapManipulator * gapManipulator_  = NULL; 
            TrajectoryController * trajController_  = NULL;
            TimeKeeper * timeKeeper_ = NULL; /**< Time keeper */

            // Status
            bool hasGlobalGoal_ = false;

            bool colliding_ = false;

            // geometry_msgs::msg::PoseArray pose_arr;
            // geometry_msgs::msg::PoseArray pose_arr_odom;

            // std::vector<turtlebot_trajectory_generator::ni_state> ctrl;
            // int ctrl_idx = 0;

            geometry_msgs::msg::TwistStamped rbtVelRbtFrame_;

            boost::shared_ptr<sensor_msgs::msg::LaserScan const> scanRbtFrame_;

            int trajectoryChangeCount_ = 0; /**< Counter for how many times the trajectory has been changed */

            // ros::WallTime last_time;
            // TrajPlan ni_ref, orig_ref;

            // Dynamic Reconfigure
            // boost::shared_ptr<dynamic_reconfigure::Server<qgConfig> > dynamic_recfg_server;
            // dynamic_reconfigure::Server<qgConfig>::CallbackType f;

            // bool replan = true;

            rclcpp::Time lastScanTime_;
            rclcpp::Time currScanTime_;
            rclcpp::Time lastPlanTime_;
            rclcpp::Time currPlanTime_;

            rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
            
            QuadGapConfig cfg_;

            boost::mutex gapMutex_;
            boost::mutex tfMutex_;

            Trajectory currTraj_;

            boost::circular_buffer<float> cmdVelBuffer;

            rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tfSub_; /**< Subscriber to TF tree */
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_; /**< Subscriber to robot laser */
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr poseSub_; /**< Subscriber to robot pose */
            // ros::Subscriber accSub_; /**< Subscriber to robot acceleration */

            bool haveTFs_ = false; /**< Flag to indicate if TFs have been received */

            // Box modification
            // bool use_geo_storage_;
            RobotGeometryStorage robotGeoStorage_;
            RobotGeometryProcessor robotGeoProc_;
            Robot robot_;
            // float speed_factor_;

            // Bezier curve
            // bool use_bezier_;        
    };
}