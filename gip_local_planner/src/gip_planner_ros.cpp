#include <pluginlib/class_list_macros.h>

#include <gip_local_planner/gip_planner_ros.h>

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(gip_local_planner::GIPPlannerROS,
                       nav_core::BaseLocalPlanner)

namespace gip_local_planner {
GIPPlannerROS::GIPPlannerROS() : initialized_(false), odom_helper_("odom") {}

void GIPPlannerROS::initialize(std::string name,
                               tf::TransformListener *tf,
                               costmap_2d::Costmap2DROS *costmap_ros) {
  if (!initialized_) {
    ros::NodeHandle private_nh("~/" + name);
    g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
    l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ros_->getRobotPose(current_pose_);
    initialized_ = true;
  }
}

GIPPlannerROS::~GIPPlannerROS() {}

bool GIPPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  if (!costmap_ros_->getRobotPose(current_pose_)) {
    ROS_ERROR("Could not get robot pose");
    return false;
  }
  return false;
}

bool GIPPlannerROS::setPlan(
    const std::vector<geometry_msgs::PoseStamped> &orig_global_plan) {
return false;
}

bool GIPPlannerROS::isGoalReached() {
  return false;
}

bool GIPPlannerROS::isInitialized() { return initialized_; }

void GIPPlannerROS::publishLocalPlan(
    const std::vector<geometry_msgs::PoseStamped> &path) {
  base_local_planner::publishPlan(path, l_plan_pub_);
}

void GIPPlannerROS::publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped> &path) {
  base_local_planner::publishPlan(path, g_plan_pub_);
}
}
