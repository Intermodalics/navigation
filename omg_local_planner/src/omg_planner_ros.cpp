#include <pluginlib/class_list_macros.h>

#include <omg_local_planner/omg_planner_ros.h>

#include <omg_ros_nav_bridge/ComputeVelCmd.h>
#include <omg_ros_nav_bridge/ConfigPlanner.h>
#include <omg_ros_nav_bridge/GoalReached.h>
#include <omg_ros_nav_bridge/InitPlanner.h>

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(omg_local_planner::OMGPlannerROS,
                       nav_core::BaseLocalPlanner)

namespace omg_local_planner {
OMGPlannerROS::OMGPlannerROS() : initialized_(false), odom_helper_("odom") {}

void OMGPlannerROS::initialize(std::string name,
                               tf::TransformListener *tf,
                               costmap_2d::Costmap2DROS *costmap_ros) {
  if (!initialized_) {
    ros::NodeHandle private_nh("~/" + name);
    g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
    l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ros_->getRobotPose(current_pose_);

    // Initialise the service clients.
    ros::NodeHandle public_nh;
    ros::service::waitForService(kGoalReachedSrv_, -1);
    goal_reached_client_ =
        public_nh.serviceClient<omg_ros_nav_bridge::GoalReached>(
            kGoalReachedSrv_, true);
    set_plan_client_ =
        public_nh.serviceClient<omg_ros_nav_bridge::ConfigPlanner>(kSetPlanSrv_,
                                                                   true);
    initialize_client_ =
        public_nh.serviceClient<omg_ros_nav_bridge::InitPlanner>(
            kInitializeSrv_, true);
    compute_velocity_client_ =
        public_nh.serviceClient<omg_ros_nav_bridge::ComputeVelCmd>(
            kComputeVelocitySrv_, true);

    omg_ros_nav_bridge::InitPlanner srv;
    srv.request.width = 10.0;
    srv.request.height = 5.0;
    omg_ros_nav_bridge::static_obst obs;
    obs.single_obstacle = {-2.0, -2.3, 1.5, 4.0, 0.1};
    srv.request.obstacles.push_back(obs);
    initialize_client_.call(srv);

    initialized_ = true;
  }
}

OMGPlannerROS::~OMGPlannerROS() {}

bool OMGPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  if (!costmap_ros_->getRobotPose(current_pose_)) {
    ROS_ERROR("Could not get robot pose");
    return false;
  }

  if (compute_velocity_client_) {
    omg_ros_nav_bridge::ComputeVelCmd srv;
    srv.request.position.x = current_pose_.getOrigin().getX();
    srv.request.position.y = current_pose_.getOrigin().getY();
    srv.request.position.theta = tf::getYaw(current_pose_.getRotation());

    tf::Stamped<tf::Pose> robot_vel;
    odom_helper_.getRobotVel(robot_vel);
    srv.request.vel_in.linear.x = robot_vel.getOrigin().getX();
    srv.request.vel_in.linear.y = robot_vel.getOrigin().getY();
    srv.request.vel_in.angular.z = tf::getYaw(robot_vel.getRotation());

    compute_velocity_client_.call(srv);

    if (!srv.response.computed) {
      ROS_ERROR("Could not calculate the velocity");
      return false;
    }
    cmd_vel = srv.response.cmd_vel;
  } else {
    ROS_ERROR_STREAM("compute_velocity_client_ is not connected.");
    return false;
  }
  return true;
}

bool OMGPlannerROS::setPlan(
    const std::vector<geometry_msgs::PoseStamped> &orig_global_plan) {
  if (!isInitialized()) {
    ROS_ERROR(
        "This planner has not been initialized, please call initialize() "
        "before using this planner");
    return false;
  }

  ROS_INFO("Got new plan");
  if (set_plan_client_) {
    omg_ros_nav_bridge::ConfigPlanner srv;
    srv.request.waypoint_lst.reserve(orig_global_plan.size());
    for (geometry_msgs::PoseStamped p : orig_global_plan) {
      geometry_msgs::Pose2D pose;
      pose.x = p.pose.position.x;
      pose.y = p.pose.position.y;
      pose.theta = tf::getYaw(p.pose.orientation);
      srv.request.waypoint_lst.push_back(pose);
    }
    set_plan_client_.call(srv);
    if (!srv.response.planned) {
      ROS_ERROR("Could not generate OMG plan!");
      return false;
    }
  } else {
    ROS_ERROR_STREAM("set_plan_client_ is not connected.");
    return false;
  }
  return true;
}

bool OMGPlannerROS::isGoalReached() {
  if (!isInitialized()) {
    ROS_ERROR(
        "This planner has not been initialized, please call initialize() "
        "before using this planner");
    return false;
  }

  if (!costmap_ros_->getRobotPose(current_pose_)) {
    ROS_ERROR("Could not get robot pose");
    return false;
  }

  if (goal_reached_client_) {
    omg_ros_nav_bridge::GoalReached srv;
    if (goal_reached_client_.call(srv)) {
      return srv.response.reached;
    } else {
      ROS_ERROR("Failed to call service goal_reached");
      return 1;
    }
  } else {
    ROS_ERROR_STREAM("goal_reached_client_ is not connected.");
  }
}

bool OMGPlannerROS::isInitialized() { return initialized_; }

void OMGPlannerROS::publishLocalPlan(
    std::vector<geometry_msgs::PoseStamped> &path) {
  base_local_planner::publishPlan(path, l_plan_pub_);
}

void OMGPlannerROS::publishGlobalPlan(
    std::vector<geometry_msgs::PoseStamped> &path) {
  base_local_planner::publishPlan(path, g_plan_pub_);
}
}
