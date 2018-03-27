#ifndef OMG_LOCAL_PLANNER_OMG_PLANNER_ROS_H
#define OMG_LOCAL_PLANNER_OMG_PLANNER_ROS_H

#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

#include <string>
#include <vector>

namespace omg_local_planner {
/**
 * @class OMGPlannerROS
 * @brief ROS Wrapper for the omg-tools planner that adheres to the
 * BaseLocalPlanner interface and can be used as a plugin for move_base.
 */
class OMGPlannerROS : public nav_core::BaseLocalPlanner {
 public:
  /**
   * @brief Constructor for the OMGPlannerROS wrapper.
   */
  OMGPlannerROS();

  /**
   * @brief Initialize Initialises the wrapper.
   * @param name The name of the trajectory planner instance.
   * @param tf A pointer to the transform listener.
   * @param costmap_ros The cost map to use for assigning costs to trajectories.
   */
  void initialize(std::string name, tf::TransformListener* tf,
                  costmap_2d::Costmap2DROS* costmap_ros);
  /**
   * @brief Destructor of the wrapper.
   */
  ~OMGPlannerROS();

  /**
   * @brief Given the current position, orientation, and velocity of the robot,
   * compute velocity commands to send to the base.
   * @param cmd_vel The velocity command to be passed to the robot base.
   * @return True if a valid trajectory was found, false otherwise.
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
   * @brief setPlan Set the plan that the planner is following.
   * @param orig_global_plan The plan to pass to the planner.
   * @return True if the plan was updated successfully, false otherwise.
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
   * @brief Check if the goal pose has been achieved.
   * @return True if achieved, false otherwise.
   */
  bool isGoalReached();

  /**
   * @brief Check if the local planner is initialized.
   * @return True if initialized, false otherwise.
   */
  bool isInitialized();

 private:
  /**
   * @brief Publish the local plan to be followed.
   * @param path A set of poses composing the plan.
   */
  void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

  /**
   * @brief Publish the global plan to be followed.
   * @param path A set of poses composing the plan.
   */
  void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

  bool initialized_;  ///< @brief Holds the planner initialization status.

  /**
   * @brief odom_helper_ Class used to provide odometry information.
   */
  base_local_planner::OdometryHelperRos odom_helper_;

  tf::TransformListener* tf_;  ///< @brief Used for transforming point clouds.

  costmap_2d::Costmap2DROS* costmap_ros_;  ///< @brief The global costmap.
  tf::Stamped<tf::Pose>
      current_pose_;  ///< @brief The current pose of the robot.
  ros::Publisher g_plan_pub_; ///< @brief Global plan publisher.
  ros::Publisher l_plan_pub_; ///< @brief Local plan publisher.


  /**
   * @brief goal_reached_client_ Client for calling the goal_reached service.
   */
  ros::ServiceClient goal_reached_client_;

  /**
   * @brief goal_reached_client_ Client for calling the set_plan service.
   */
  ros::ServiceClient set_plan_client_;

  /**
   * @brief goal_reached_client_ Client for calling the initialize service.
   */
  ros::ServiceClient initialize_client_;

  /**
   * @brief goal_reached_client_ Client for calling the compute_velocity service.
   */
  ros::ServiceClient compute_velocity_client_;

  /**
   * @brief goal_reached_srv_ Service name.
   */
  const std::string kGoalReachedSrv_ = "goal_reached";

  /**
   * @brief goal_reached_srv_ Service name.
   */
  const std::string kSetPlanSrv_ = "config_planner";

  /**
   * @brief goal_reached_srv_ Service name.
   */
  const std::string kInitializeSrv_ = "init_planner";

  /**
   * @brief goal_reached_srv_ Service name.
   */
  const std::string kComputeVelocitySrv_ = "compute_vel_cmd";
};
}

#endif  // OMG_LOCAL_PLANNER_OMG_PLANNER_ROS_H
