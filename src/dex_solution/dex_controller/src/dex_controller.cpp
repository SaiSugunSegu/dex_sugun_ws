// Copyright 2023 Dexory


#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "dex_controller/dex_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;  //NOLINT

namespace dex_controller
{

void DexController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  double transform_tolerance = 0.1;
  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(plugin_name_ + ".use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_);
  node->get_parameter( plugin_name_ + ".use_interpolation", use_interpolation_);
  node->get_parameter( plugin_name_ + ".max_robot_pose_search_dist", max_robot_pose_search_dist_);
  node->get_parameter("control_duration", control_duration_);

  
}

void DexController::cleanup() 
{
  RCLCPP_INFO( logger_, "Cleaning up controller: %s of type dex_controller::DexController", plugin_name_.c_str());
}

void DexController::activate() 
{
  RCLCPP_INFO( logger_, "Activating controller: %s of type dex_controller::DexController", plugin_name_.c_str());
}

void DexController::deactivate() 
{
  RCLCPP_INFO( logger_, "Deactivating controller: %s of type dex_controller::DexController", plugin_name_.c_str());
}


bool DexController::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

double DexController::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters = std::max(
    costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}

nav_msgs::msg::Path DexController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  double max_costmap_extent = getCostmapMaxExtent();

  auto closest_pose_upper_bound =
    nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto transformation_begin =
    nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // Find points up to max_transform_dist so we only transform them.
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) {
      return euclidean_distance(pose, robot_pose) > max_costmap_extent;
    });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
      transformed_pose.pose.position.z = 0.0;
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

double DexController::getLookAheadDistance(
  const geometry_msgs::msg::Twist & speed)
{
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance
  double lookahead_dist = lookahead_dist_;
  if (use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = fabs(speed.linear.x) * lookahead_time_;
    lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  }

  return lookahead_dist;
}

geometry_msgs::msg::Point DexController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2,
  double r)
{
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double D = x1 * y2 - x2 * y1;

  // Augmentation to only return point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  geometry_msgs::msg::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - D * D);
  p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;

  return p;
}

geometry_msgs::msg::PoseStamped DexController::getLookAheadPoint(
  const double & lookahead_dist,
  const nav_msgs::msg::Path & transformed_plan)
{
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) 
  {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  } 
  else if (use_interpolation_ && goal_pose_it != transformed_plan.poses.begin()) 
  {
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = circleSegmentIntersection(
      prev_pose_it->pose.position,
      goal_pose_it->pose.position, lookahead_dist);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    return pose;
  }

  return *goal_pose_it;
}

bool DexController::shouldRotateToPath(
  const geometry_msgs::msg::PoseStamped & goal_pose, double & angle_to_path)
{
  // Whether we should rotate robot to rough path heading
  angle_to_path = atan2(goal_pose.pose.position.y, goal_pose.pose.position.x);
  return use_rotate_to_heading_ && fabs(angle_to_path) > rotate_to_heading_min_angle_;
}

bool DexController::shouldRotateToGoalHeading(
  const geometry_msgs::msg::PoseStamped & goal_pose)
{
  // Whether we should rotate robot to goal heading
  double dist_to_goal = std::hypot(goal_pose.pose.position.x, goal_pose.pose.position.y);
  return use_rotate_to_heading_ && dist_to_goal < goal_dist_tol_;
}

void DexController::rotateToHeading(
  double & linear_vel, double & angular_vel,
  const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed)
{
  // Rotate in place using max angular velocity / acceleration possible
  linear_vel = 0.0;
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  angular_vel = sign * rotate_to_heading_angular_vel_;

  const double & dt = control_duration_;
  const double min_feasible_angular_speed = curr_speed.angular.z - max_angular_accel_ * dt;
  const double max_feasible_angular_speed = curr_speed.angular.z + max_angular_accel_ * dt;
  angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
}


double DexController::costAtPose(const double & x, const double & y)
{
  unsigned int mx, my;

  if (!costmap_->worldToMap(x, y, mx, my)) {
    RCLCPP_FATAL(
      logger_,
      "The dimensions of the costmap is too small to fully include your robot's footprint, "
      "thusly the robot cannot proceed further");
    throw nav2_core::PlannerException(
            "DexController: Dimensions of the costmap are too small "
            "to encapsulate the robot footprint at current speeds!");
  }

  unsigned char cost = costmap_->getCost(mx, my);
  return static_cast<double>(cost);
}

geometry_msgs::msg::TwistStamped DexController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{

  // Step 1: Transform path to robot base frame
  auto transformed_plan = transformGlobalPlan(pose);

  // Step 2: Find look ahead distance and point on path
  double lookahead_dist = getLookAheadDistance(velocity);

  // Find look ahead Point (goal point)
  auto goal_pose = getLookAheadPoint(lookahead_dist, transformed_plan);


  // Step 3 : Calculating Goal distance assuming Robot as (0,0)
  const double goal_dist2 =
    (goal_pose.pose.position.x * goal_pose.pose.position.x) +
    (goal_pose.pose.position.y * goal_pose.pose.position.y);

  // Find curvature of circle (k = 1 / R)
  double curvature = 0.0;
  if (goal_dist2 > 0.001) {
    curvature = 2.0 * goal_pose.pose.position.y / goal_dist2;
  }

  // Step 4 : apply control 

  double linear_vel, angular_vel;

  linear_vel = desired_linear_vel_;

  // Make sure we're in compliance with basic constraints
  double angle_to_heading;
  if (shouldRotateToGoalHeading(goal_pose)) {
    double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    rotateToHeading(linear_vel, angular_vel, angle_to_goal, velocity);
  } else if (shouldRotateToPath(goal_pose, angle_to_heading)) {
    rotateToHeading(linear_vel, angular_vel, angle_to_heading, velocity);
  } else {
    // Apply curvature to angular velocity
    angular_vel = linear_vel * curvature;
  }

  RCLCPP_INFO( logger_, "Goal dist: %f, Linear Velocity: %f, Angular Velocity: %f" , goal_dist2, linear_vel, angular_vel);

  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
  return cmd_vel;
}

void DexController::setPlan(const nav_msgs::msg::Path & path) 
{
  global_plan_ = path;
}

void DexController::setSpeedLimit(const double & speed_limit, const bool & percentage) {}

}  // namespace dex_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(dex_controller::DexController, nav2_core::Controller)
