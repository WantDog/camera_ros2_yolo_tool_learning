#include <memory>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("hello_moveit");
  auto const logger = rclcpp::get_logger("hello_moveit");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // ⚙️ 修改为你机械臂的规划组名称
  static const std::string PLANNING_GROUP = "manipulator";

  moveit::planning_interface::MoveGroupInterface move_group_interface(node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  RCLCPP_INFO(logger, "参考坐标系: %s", move_group_interface.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "默认末端执行器 link: %s", move_group_interface.getEndEffectorLink().c_str());

  // ⚙️ 让 MoveIt 只控制 link5（而不是末端吸盘）
  move_group_interface.setEndEffectorLink("sucker_link");

  RCLCPP_INFO(logger, "修改后的控制目标 link: %s", move_group_interface.getEndEffectorLink().c_str());

  move_group_interface.setPlanningTime(10.0);
  move_group_interface.setNumPlanningAttempts(100);
  move_group_interface.setMaxVelocityScalingFactor(0.3);
  move_group_interface.setMaxAccelerationScalingFactor(0.3);

  RCLCPP_INFO(logger, "等待 MoveGroup 准备...");
  executor.spin_some();

  // ============= 固定目标点（根据机械臂工作空间修改） =============
  std::vector<geometry_msgs::msg::Pose> candidate_poses;

  geometry_msgs::msg::Pose pose1;
  pose1.position.x = 0.47;
  pose1.position.y = 0.048;
  pose1.position.z = -0.0319;
  pose1.orientation.x = 0.17;
  pose1.orientation.y = -0.518;
  pose1.orientation.z = -0.59543;
  pose1.orientation.w = 0.588;
  candidate_poses.push_back(pose1);

  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 0.73;
  pose2.position.y = 0.048;
  pose2.position.z = 0.18;
  pose2.orientation = pose1.orientation;
  candidate_poses.push_back(pose2);

  geometry_msgs::msg::Pose pose3;
  pose3.position.x = 0.47;
  pose3.position.y = 0.048;
  pose3.position.z = -0.03;
  pose3.orientation = pose1.orientation;
  candidate_poses.push_back(pose3);

  geometry_msgs::msg::Pose pose4;
  pose4.position.x = 0.05;
  pose4.position.y = 0.00;
  pose4.position.z = 0.80;
  pose4.orientation = pose1.orientation;
  candidate_poses.push_back(pose4);
  // ================================================================

  bool success = false;

  for (size_t i = 0; i < candidate_poses.size(); ++i)
  {
    const auto &target_pose = candidate_poses[i];

    RCLCPP_INFO(logger, "尝试第 %zu 个目标点: [x=%.3f, y=%.3f, z=%.3f]",
                i + 1, target_pose.position.x, target_pose.position.y, target_pose.position.z);

    move_group_interface.clearPoseTargets();
    // 🎯 用 link5 的 pose 作为目标
    move_group_interface.setPoseTarget(target_pose, "link5");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    RCLCPP_INFO(logger, "开始规划...");
    auto error_code = move_group_interface.plan(plan);

    if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(logger, "✅ 规划成功！轨迹点数: %zu", plan.trajectory_.joint_trajectory.points.size());
      RCLCPP_INFO(logger, "执行轨迹...");
      auto exec_code = move_group_interface.execute(plan);
      if (exec_code == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(logger, "✅ 执行成功完成");
        success = true;
        break;
      }
      else
      {
        RCLCPP_WARN(logger, "⚠️ 执行失败 (错误码: %d)", exec_code.val);
      }
    }
    else
    {
      RCLCPP_WARN(logger, "❌ 规划失败 (错误码: %d)", error_code.val);
    }

    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  if (success)
  {
    RCLCPP_INFO(logger, "✨ 任务完成！");
  }
  else
  {
    RCLCPP_ERROR(logger, "💥 所有目标点都失败！");
    RCLCPP_INFO(logger, "建议:");
    RCLCPP_INFO(logger, "1. 检查目标点是否在 link5 可达范围内");
    RCLCPP_INFO(logger, "2. 检查关节角限制");
    RCLCPP_INFO(logger, "3. 尝试关节空间规划");
  }

  rclcpp::shutdown();
  return success ? 0 : -1;
}
