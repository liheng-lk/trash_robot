#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

#include "roarm_moveit/srv/move_point_cmd.hpp"
#include "roarm_moveit_cmd/ik.h"

namespace
{

struct PlannerConfig
{
  std::string pipeline_id;
  std::string planner_id;
  double velocity_scale;
  double acceleration_scale;
  double planning_time;
  bool invert_y_for_ik;
};

template<typename T>
T get_or_declare_parameter(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const T & default_value)
{
  if (!node->has_parameter(name)) {
    return node->declare_parameter<T>(name, default_value);
  }
  T value{};
  node->get_parameter(name, value);
  return value;
}

PlannerConfig read_planner_config(const rclcpp::Node::SharedPtr & node)
{
  PlannerConfig config;
  config.pipeline_id = get_or_declare_parameter<std::string>(
    node, "planning_pipeline_id", "pilz_industrial_motion_planner");
  config.planner_id = get_or_declare_parameter<std::string>(node, "planner_id", "PTP");
  config.velocity_scale = get_or_declare_parameter<double>(node, "velocity_scale", 0.18);
  config.acceleration_scale = get_or_declare_parameter<double>(node, "acceleration_scale", 0.18);
  config.planning_time = get_or_declare_parameter<double>(node, "planning_time", 1.0);
  // Waveshare/RoArm SDK convention maps service +Y to IK -Y. Keep that as the
  // default and expose a parameter only for field verification.
  config.invert_y_for_ik = get_or_declare_parameter<bool>(node, "invert_y_for_ik", true);
  return config;
}

void apply_planner_config(
  const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & move_group,
  const PlannerConfig & config)
{
  move_group->setPlanningPipelineId(config.pipeline_id);
  move_group->setPlannerId(config.planner_id);
  move_group->setPlanningTime(config.planning_time);
  move_group->setStartStateToCurrentState();
  move_group->setMaxVelocityScalingFactor(config.velocity_scale);
  move_group->setMaxAccelerationScalingFactor(config.acceleration_scale);
}

bool plan_and_execute(
  const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & move_group,
  const std::vector<double> & target,
  const rclcpp::Logger & logger,
  std::string & message)
{
  move_group->stop();
  move_group->clearPoseTargets();
  move_group->setStartStateToCurrentState();
  move_group->setJointValueTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool planned =
    move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  if (!planned) {
    message = "Planning failed";
    RCLCPP_ERROR(logger, "%s", message.c_str());
    return false;
  }

  const auto exec_result = move_group->execute(plan);
  move_group->stop();
  move_group->clearPoseTargets();
  if (exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    message = "Execution failed";
    RCLCPP_ERROR(logger, "%s", message.c_str());
    return false;
  }

  message = "MovePointCmd executed successfully";
  RCLCPP_INFO(logger, "%s", message.c_str());
  return true;
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "move_point_cmd_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  const PlannerConfig primary_config = read_planner_config(node);
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "hand");
  apply_planner_config(move_group, primary_config);
  RCLCPP_INFO(
    logger,
    "MovePointCmd service uses persistent MoveGroup: pipeline=%s planner=%s velocity=%.2f acceleration=%.2f invert_y_for_ik=%s",
    primary_config.pipeline_id.c_str(), primary_config.planner_id.c_str(),
    primary_config.velocity_scale, primary_config.acceleration_scale,
    primary_config.invert_y_for_ik ? "true" : "false");

  std::mutex move_mutex;
  auto server = node->create_service<roarm_moveit::srv::MovePointCmd>(
    "move_point_cmd",
    [node, move_group, logger, primary_config, &move_mutex](
      const std::shared_ptr<roarm_moveit::srv::MovePointCmd::Request> request,
      std::shared_ptr<roarm_moveit::srv::MovePointCmd::Response> response) {
      std::lock_guard<std::mutex> lock(move_mutex);

      const double ik_x_mm = 1000.0 * request->x;
      const double ik_y_mm =
        1000.0 * (primary_config.invert_y_for_ik ? -request->y : request->y);
      cartesian_to_polar(ik_x_mm, ik_y_mm, &base_r, &BASE_point_RAD);
      simpleLinkageIkRad(l2, l3, base_r, 1000.0 * request->z);

      RCLCPP_INFO(
        logger,
        "MovePointCmd request xyz=%.3f,%.3f,%.3f ik_mm=%.1f,%.1f,%.1f -> joints base=%.3f shoulder=%.3f elbow=%.3f",
        request->x, request->y, request->z,
        ik_x_mm, ik_y_mm, 1000.0 * request->z,
        BASE_point_RAD, -SHOULDER_point_RAD, ELBOW_point_RAD);

      if (nanIK || !std::isfinite(BASE_point_RAD) || !std::isfinite(SHOULDER_point_RAD) ||
        !std::isfinite(ELBOW_point_RAD))
      {
        response->success = false;
        response->message = "IK failed";
        RCLCPP_ERROR(
          logger, "IK failed for xyz=%.3f,%.3f,%.3f", request->x, request->y, request->z);
        return;
      }

      const std::vector<double> target = {BASE_point_RAD, -SHOULDER_point_RAD, ELBOW_point_RAD};
      apply_planner_config(move_group, primary_config);
      std::string message;
      bool ok = plan_and_execute(move_group, target, logger, message);

      if (!ok && primary_config.pipeline_id != "ompl") {
        RCLCPP_WARN(logger, "Primary planner failed, falling back to OMPL/hand once");
        PlannerConfig fallback = primary_config;
        fallback.pipeline_id = "ompl";
        fallback.planner_id = "hand";
        fallback.planning_time = 2.0;
        apply_planner_config(move_group, fallback);
        ok = plan_and_execute(move_group, target, logger, message);
      }

      response->success = ok;
      response->message = message;
    });

  RCLCPP_INFO(logger, "MovePointCmd service is ready.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
