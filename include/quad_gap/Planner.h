#pragma once

#include <ros/ros.h>
#include <ros/package.h>

#include <quad_gap/utils/Gap.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/numeric/odeint.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <math.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Header.h>
#include "nav_msgs/Odometry.h"
#include "quad_gap/TrajPlan.h"
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
#include <quad_gap/trajectory_tracking/TrajectoryController.h>
#include <quad_gap/TimeKeeper.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include <tf2_utils/transform_manager.h>

#include <omp.h>

#include <dynamic_reconfigure/server.h>
#include <quad_gap/qgConfig.h>

#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>

#include <turtlebot_trajectory_testing/turtlebot_trajectory_tester.h>
// #include <pips_trajectory_testing/pips_trajectory_tester.h>
#include <pips_trajectory_msgs/trajectory_points.h>
#include <pips_trajectory_testing/pips_cc_wrapper.h>
#include <pips_trajectory_testing/depth_image_cc_wrapper.h>
#include <pips_egocylindrical/egocylindrical_image_cc_wrapper.h>
#include <pips_egocircle/egocircle_cc_wrapper.h>

#include <quad_gap/CollisionCheckerConfig.h>

#include <quad_gap/utils/RobotGeometryStorage.h>
#include <quad_gap/utils/RobotGeometryProcessor.h>

namespace quad_gap
{
    struct CollisionResults
    {
        int collision_idx_ = -1;
        pips_trajectory_msgs::trajectory_points local_traj_;
        
        CollisionResults()
        {
            collision_idx_ = -1;
        }

        CollisionResults(int collision_idx, pips_trajectory_msgs::trajectory_points local_traj)
        {
            collision_idx_ = collision_idx;
            local_traj_ = local_traj;
        }
    };

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
            bool initialize(const std::string & name);

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
            void laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan> msg);

            /**
            * call back function to pose, pose information obtained here only used when a new goal is used
            * @param msg pose msg
            * @return None
            */
            void poseCB(const nav_msgs::Odometry::ConstPtr& msg);

            /**
            * \brief Function for updating all tf transform at the beginning of every planning cycle
            * \param msg incoming agent odometry message
            */
            void tfCB(const tf2_msgs::TFMessage& msg);

            /**
            * Interface function for receiving global plan
            * @param plan, vector of PoseStamped
            * @return boolean type on whether successfully registered goal
            */
            bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);

            /**
            * update all tf transform at the beginning of every planning cycle
            * @param None, all tf received via TF
            * @return None, all registered via internal variables in TransformStamped
            */
            void updateTF();

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
            geometry_msgs::Twist ctrlGeneration(const geometry_msgs::PoseArray & traj);
            
            /**
            * Take current observed gaps and perform gap conversion
            * @param None, directly taken from private variable space
            * @return gap_set, simplfied radial prioritized gaps
            */
            std::vector<Gap *> gapManipulate(const std::vector<Gap *> & planning_gaps);

            /**
            * 
            *
            */
            std::vector<std::vector<float>> initialTrajGen(const std::vector<Gap *> & vec, 
                                                            std::vector<geometry_msgs::PoseArray>& res, 
                                                            std::vector<geometry_msgs::PoseArray>& virtual_decayed);

            /**
            * Callback function to config object
            * @param incoming config
            * @param level Level of incoming config
            */
            void rcfgCallback(qgConfig &config, uint32_t level);

            /**
            * Pick the best trajectory from the current set
            * @param Vector of PoseArray
            * @param Vector of corresponding trajectory scores
            * @return the best trajectory
            */
            geometry_msgs::PoseArray pickTraj(const std::vector<geometry_msgs::PoseArray> & prr, 
                                                const std::vector<std::vector<float>> & score, 
                                                const std::vector<geometry_msgs::PoseArray> & virtual_path, 
                                                geometry_msgs::PoseArray& chosen_virtual_path);

            /**
            * Compare to the old trajectory and pick the best one
            * @param incoming trajectory
            * @return the best trajectory  
            */
            geometry_msgs::PoseArray compareToCurrentTraj(const geometry_msgs::PoseArray & incoming, 
                                                        geometry_msgs::PoseArray& virtual_curr_traj);

            geometry_msgs::PoseArray getOrientDecayedPath(const geometry_msgs::PoseArray & orig_path);

            CollisionResults checkCollision(const geometry_msgs::PoseArray & path);

            /**
            * \brief Function for getting index of closest pose in trajectory
            * \param currTrajRbtFrame current trajectory in robot frame
            * \return index of closest pose in trajectory
            */
            int getClosestTrajectoryPoseIdx(const geometry_msgs::PoseArray & currTrajRbtFrame);

            /**
            * Setter and Getter of Current Trajectory, this is performed in the compareToCurrentTraj function
            */
            void setCurrentTraj(const geometry_msgs::PoseArray & curr_traj);   

            geometry_msgs::PoseArray getCurrentTraj();

            /**
            * Conglomeration of getting a plan Trajectory
            * @return the trajectory
            */
            geometry_msgs::PoseArray runPlanningLoop();    

            /**
            * Reset Planner, clears current observedSet
            */
            void reset();

            /**
            * Check if the robot has been stuck
            * @param command velocity
            * @return False if robot has been stuck for the past cfg.planning.halt_size iterations
            */
            bool recordAndCheckVel(const geometry_msgs::Twist & cmd_vel);
            
            void setCCWrapper(const std::shared_ptr<pips_trajectory_testing::PipsCCWrapper>& cc_wrapper)
            {
                cc_wrapper_ = cc_wrapper;
            }

            std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> getCCWrapper()
            {
                return cc_wrapper_;
            }

            bool ccEnabled()
            {
                return collision_checker_enable_;
            }

            // typedef TurtlebotGenAndTest::trajectory_ptr trajectory_ptr;
            // typedef TurtlebotGenAndTest::traj_func_type traj_func_type;
            // typedef TurtlebotGenAndTest::traj_func_ptr traj_func_ptr;
            // typedef TurtlebotGenAndTest::trajectory_points trajectory_points;
            // typedef TurtlebotGenAndTest::TrajBridge TrajBridge;
            typedef std::shared_ptr<TurtlebotGenAndTest> GenAndTest_ptr;

            std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cc_wrapper_;
            GenAndTest_ptr traj_tester_;

            bool collision_checker_enable_ = false;
            int cc_type_ = -1;

            using Mutex = boost::mutex;
            using Lock = Mutex::scoped_lock;
            Mutex connect_mutex_;

            typedef dynamic_reconfigure::Server<CollisionCheckerConfig> ReconfigureServer;
            std::shared_ptr<ReconfigureServer> reconfigure_server_;

            void configCB(CollisionCheckerConfig &config, uint32_t level);

            /**
            * \brief Function to check if global goal has been reached
            * \param status whether or not global goal has been reached 
            */
            void setReachedGlobalGoal(const bool & status) { reachedGlobalGoal_ = status; }

            void updateEgoCircle();

        private:

            std::vector<Gap *> deepCopyCurrentSimplifiedGaps();

            boost::shared_ptr<sensor_msgs::LaserScan const> transformLaserToRbt(boost::shared_ptr<sensor_msgs::LaserScan const> msg);

            // Transforms
            geometry_msgs::TransformStamped map2rbt_;
            geometry_msgs::TransformStamped rbt2map_;
            geometry_msgs::TransformStamped odom2rbt_;
            geometry_msgs::TransformStamped rbt2odom_;
            geometry_msgs::TransformStamped map2odom_;
            geometry_msgs::TransformStamped cam2odom_;
            geometry_msgs::TransformStamped odom2cam_;
            geometry_msgs::TransformStamped rbt2cam_;
            geometry_msgs::TransformStamped cam2rbt_;

            // Robot poses
            geometry_msgs::PoseStamped rbtPoseRbtFrame_;
            geometry_msgs::PoseStamped rbtPoseCamFrame_;
            geometry_msgs::PoseStamped rbtPoseOdomFrame_;
            
            std::shared_ptr<tf2_ros::Buffer> tfBuffer;
            std::shared_ptr<tf2_ros::TransformListener> tfListener;

            ros::NodeHandle nh, pnh;
            ros::Publisher local_traj_pub;
            ros::Publisher trajectory_pub;

            ros::Publisher transformed_laser_pub;
            ros::Publisher virtual_orient_traj_pub;

            bool reachedGlobalGoal_ = false; /**< Flag for if global goal has been reached */
            bool hasLaserScan_ = false;
            bool initialized_ = false;

            // Goals and stuff
            geometry_msgs::PoseStamped globalGoalOdomFrame_; /**< Global goal in odometry frame */
            geometry_msgs::PoseStamped globalGoalRobotFrame_; /**< Global goal in robot frame */
            geometry_msgs::PoseStamped globalPathLocalWaypointOdomFrame_; /**< Global path local waypoint in odometry frame */

            geometry_msgs::PoseStamped local_waypoint_odom; // local_waypoint, 
            geometry_msgs::PoseStamped final_goal_odom;

            // Gaps:
            std::vector<Gap *> currRawGaps_;
            std::vector<Gap *> currSimpGaps_;
            // std::vector<Gap *> currSimpGaps__ptr;

            // Helper modules
            GapDetector * gapDetector_  = NULL; 
            GapVisualizer * gapVisualizer_  = NULL; 
            GlobalPlanManager * globalPlanManager_  = NULL; 
            TrajectoryVisualizer * trajVisualizer_  = NULL; 
            GoalVisualizer * goalVisualizer_  = NULL; 
            TrajectoryEvaluator * trajEvaluator_  = NULL; 
            GapTrajGenerator * gapTrajGenerator_  = NULL; 
            GapManipulator * gapManipulator_  = NULL; 
            TrajectoryController * trajController_  = NULL;
            TimeKeeper * timeKeeper_ = NULL; /**< Time keeper */


            // Status
            bool hasGlobalGoal_ = false;

            bool colliding_ = false;

            geometry_msgs::PoseArray pose_arr;
            geometry_msgs::PoseArray pose_arr_odom;

            // std::vector<turtlebot_trajectory_generator::ni_state> ctrl;
            int ctrl_idx = 0;

            geometry_msgs::TwistStamped rbtVelRbtFrame_;

            boost::shared_ptr<sensor_msgs::LaserScan const> scan_;

            ros::WallTime last_time;
            TrajPlan ni_ref, orig_ref;

            // Dynamic Reconfigure
            // boost::shared_ptr<dynamic_reconfigure::Server<qgConfig> > dynamic_recfg_server;
            // dynamic_reconfigure::Server<qgConfig>::CallbackType f;

            bool replan = true;
            
            QuadGapConfig cfg_;

            boost::mutex gapMutex_;

            geometry_msgs::PoseArray curr_executing_traj;

            boost::circular_buffer<float> cmdVelBuffer;

            ros::Subscriber tfSub_; /**< Subscriber to TF tree */
            ros::Subscriber laserSub_; /**< Subscriber to robot laser */
            ros::Subscriber poseSub_; /**< Subscriber to robot pose */
            ros::Subscriber accSub_; /**< Subscriber to robot acceleration */

            bool haveTFs_ = false; /**< Flag to indicate if TFs have been received */

            // Box modification
            // bool use_geo_storage_;
            RobotGeometryStorage robot_geo_storage_;
            RobotGeometryProcessor robot_geo_proc_;
            // bool robot_path_orient_linear_decay_, virtual_path_decay_enable_;
            // float speed_factor_;

            // Bezier curve
            // bool use_bezier_;        
    };
}