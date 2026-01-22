#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.7;
  pose.position.y = -0.25;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  // 创建MTC任务对象
mtc::Task task;
// 设置任务名称
task.stages()->setName("demo task");
// 从ROS节点加载机器人模型
task.loadRobotModel(node_);

// 定义机械臂运动组的名称
const auto& arm_group_name = "panda_arm";
// 定义手爪运动组的名称
const auto& hand_group_name = "hand";
// 定义手爪坐标系名称
const auto& hand_frame = "panda_hand";

// 设置任务属性：机械臂运动组
task.setProperty("group", arm_group_name);
// 设置任务属性：末端执行器组
task.setProperty("eef", hand_group_name);
// 设置任务属性：逆运动学参考坐标系
task.setProperty("ik_frame", hand_frame);

// 声明指向当前状态阶段的指针，用于传递给抓取姿态生成器
mtc::Stage* current_state_ptr = nullptr;
// 声明指向附加物体阶段的指针，用于传递给放置姿态生成器
mtc::Stage* attach_object_stage = nullptr;

// 创建当前状态阶段，获取机器人当前状态
auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
// 保存当前状态阶段的指针
current_state_ptr = stage_state_current.get();
// 将当前状态阶段添加到任务中
task.add(std::move(stage_state_current));

// 创建基于采样的规划器，用于关节空间规划
auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
// 创建关节插值规划器，用于简单运动
auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

// 创建笛卡尔路径规划器，用于直线运动
auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
// 设置笛卡尔规划器的最大速度缩放因子
cartesian_planner->setMaxVelocityScalingFactor(1.0);
// 设置笛卡尔规划器的最大加速度缩放因子
cartesian_planner->setMaxAccelerationScalingFactor(1.0);
// 设置笛卡尔规划器的步长
cartesian_planner->setStepSize(.01);

// 创建打开手爪阶段
auto stage_open_hand =
    std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
// 设置手爪运动组
stage_open_hand->setGroup(hand_group_name);
// 设置目标为"open"姿态
stage_open_hand->setGoal("open");
// 将打开手爪阶段添加到任务中
task.add(std::move(stage_open_hand));

// 创建移动到抓取位置的连接阶段
auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "move to pick", // 阶段名称
    // 为机械臂组配置采样规划器
    mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
// 设置连接阶段的超时时间
stage_move_to_pick->setTimeout(5.0);
// 从父级继承属性
stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
// 将移动到抓取位置阶段添加到任务中
task.add(std::move(stage_move_to_pick));

// 开始抓取容器（序列容器）
{
  // 创建抓取序列容器
  auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
  // 将任务属性暴露给抓取容器
  task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
  // 从父级继承属性
  grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                        { "eef", "group", "ik_frame" });

  // 接近物体阶段
  {
    // 创建相对移动阶段，用于接近物体
    auto stage =
        std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
    // 设置标记命名空间，用于可视化
    stage->properties().set("marker_ns", "approach_object");
    // 设置移动参考链接
    stage->properties().set("link", hand_frame);
    // 从父级继承组属性
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    // 设置最小和最大移动距离
    stage->setMinMaxDistance(0.1, 0.15);

    // 设置手爪前进方向
    geometry_msgs::msg::Vector3Stamped vec;
    // 设置方向参考坐标系为手爪坐标系
    vec.header.frame_id = hand_frame;
    // 设置Z轴方向（手爪前方）
    vec.vector.z = 1.0;
    // 设置移动方向
    stage->setDirection(vec);
    // 将接近物体阶段添加到抓取容器中
    grasp->insert(std::move(stage));
  }

  // 生成抓取姿态阶段
  {
    // 创建抓取姿态生成阶段
    auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
    // 从父级继承属性，确保使用正确的运动组和末端执行器配置
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    // 设置标记命名空间，用于可视化
    stage->properties().set("marker_ns", "grasp_pose");
    // 设置预抓取姿态
    stage->setPreGraspPose("open");
    // 设置目标物体
    stage->setObject("object");
    // 设置角度采样增量
    stage->setAngleDelta(M_PI / 12);
    // 设置监控阶段为当前状态阶段
    stage->setMonitoredStage(current_state_ptr);


    //定义了一个从手爪坐标系到抓取坐标系的变换,这个变换定义了手爪相对于目标物体的抓取姿态
    // 计算抓取帧变换
    Eigen::Isometry3d grasp_frame_transform;
    // 创建旋转四元数（绕X轴旋转90度，再绕Y轴旋转90度，再绕Z轴旋转90度）
    Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
    // 设置旋转部分
    grasp_frame_transform.linear() = q.matrix();
    // 设置平移部分（Z方向偏移0.1米）
    grasp_frame_transform.translation().z() = 0.1;

    // 创建逆运动学计算阶段，包装抓取姿态生成阶段
    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
    // 设置最大逆运动学解数量
    wrapper->setMaxIKSolutions(4);
    // 设置最小解距离
    wrapper->setMinSolutionDistance(1.0);
    // 设置逆运动学参考帧和变换
    wrapper->setIKFrame(grasp_frame_transform, hand_frame);
    // 从父级继承末端执行器和组属性
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    // 从接口继承目标姿态属性
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    // 将逆运动学计算阶段添加到抓取容器中
    grasp->insert(std::move(wrapper));
  }

  // 允许碰撞阶段（手爪和物体）
  {
    // 创建规划场景修改阶段
    auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
    // 允许手爪和物体之间的碰撞
    stage->allowCollisions("object", // 物体名称
                          // 获取手爪组的所有碰撞几何体链接名称
                          task.getRobotModel()
                              ->getJointModelGroup(hand_group_name)
                              ->getLinkModelNamesWithCollisionGeometry(),
                          true); // 允许碰撞
    // 将允许碰撞阶段添加到抓取容器中
    grasp->insert(std::move(stage));
  }

  // 闭合手爪阶段
  {
    // 创建移动到目标姿态阶段
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    // 设置手爪运动组
    stage->setGroup(hand_group_name);
    // 设置目标为"close"姿态
    stage->setGoal("close");
    // 将闭合手爪阶段添加到抓取容器中
    grasp->insert(std::move(stage));
  }

  // 附加物体阶段
  {
    // 创建规划场景修改阶段
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    // 将物体附加到手爪上
    stage->attachObject("object", hand_frame);
    // 保存附加物体阶段的指针
    attach_object_stage = stage.get();
    // 将附加物体阶段添加到抓取容器中
    grasp->insert(std::move(stage));
  }

  // 提升物体阶段
  {
    // 创建相对移动阶段，用于提升物体
    auto stage =
        std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
    // 从父级继承组属性
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    // 设置最小和最大移动距离
    stage->setMinMaxDistance(0.1, 0.3);
    // 设置逆运动学参考帧
    stage->setIKFrame(hand_frame);
    // 设置标记命名空间，用于可视化
    stage->properties().set("marker_ns", "lift_object");

    // 设置向上移动方向
    geometry_msgs::msg::Vector3Stamped vec;
    // 设置方向参考坐标系为世界坐标系
    vec.header.frame_id = "world";
    // 设置Z轴方向（向上）
    vec.vector.z = 1.0;
    // 设置移动方向
    stage->setDirection(vec);
    // 将提升物体阶段添加到抓取容器中
    grasp->insert(std::move(stage));
  }

  // 将抓取容器添加到任务中
  task.add(std::move(grasp));
}

// 移动到放置位置阶段
{
  // 创建移动到放置位置的连接阶段
  auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
      "move to place", // 阶段名称
      // 为机械臂组配置采样规划器，为手爪组配置插值规划器
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                { hand_group_name, interpolation_planner } });
  // 设置连接阶段的超时时间
  stage_move_to_place->setTimeout(5.0);
  // 从父级继承属性
  stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  // 将移动到放置位置阶段添加到任务中
  task.add(std::move(stage_move_to_place));
}

// 放置容器（序列容器）
{
  // 创建放置序列容器
  auto place = std::make_unique<mtc::SerialContainer>("place object");
  // 将任务属性暴露给放置容器
  task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
  // 从父级继承属性
  place->properties().configureInitFrom(mtc::Stage::PARENT,
                                        { "eef", "group", "ik_frame" });

  // 生成放置姿态阶段
  {
    // 创建放置姿态生成阶段
    auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
    // 从父级继承属性
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    // 设置标记命名空间，用于可视化
    stage->properties().set("marker_ns", "place_pose");
    // 设置目标物体
    stage->setObject("object");

    // 创建目标姿态消息
    geometry_msgs::msg::PoseStamped target_pose_msg;
    // 设置姿态参考坐标系为物体坐标系
    target_pose_msg.header.frame_id = "object";
    // 设置Y方向偏移0.5米
    target_pose_msg.pose.position.y = 0.5;
    // 设置单位四元数（无旋转）
    target_pose_msg.pose.orientation.w = 1.0;
    // 设置目标姿态
    stage->setPose(target_pose_msg);
    // 设置监控阶段为附加物体阶段
    stage->setMonitoredStage(attach_object_stage);

    // 创建逆运动学计算阶段，包装放置姿态生成阶段
    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
    // 设置最大逆运动学解数量
    wrapper->setMaxIKSolutions(2);
    // 设置最小解距离
    wrapper->setMinSolutionDistance(1.0);
    // 设置逆运动学参考帧为物体
    wrapper->setIKFrame("object");
    // 从父级继承末端执行器和组属性
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    // 从接口继承目标姿态属性
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    // 将逆运动学计算阶段添加到放置容器中
    place->insert(std::move(wrapper));
  }

  // 打开手爪阶段
  {
    // 创建移动到目标姿态阶段
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    // 设置手爪运动组
    stage->setGroup(hand_group_name);
    // 设置目标为"open"姿态
    stage->setGoal("open");
    // 将打开手爪阶段添加到放置容器中
    place->insert(std::move(stage));
  }

  // 禁止碰撞阶段（手爪和物体）
  {
    // 创建规划场景修改阶段
    auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
    // 禁止手爪和物体之间的碰撞
    stage->allowCollisions("object", // 物体名称
                          // 获取手爪组的所有碰撞几何体链接名称
                          task.getRobotModel()
                              ->getJointModelGroup(hand_group_name)
                              ->getLinkModelNamesWithCollisionGeometry(),
                          false); // 禁止碰撞
    // 将禁止碰撞阶段添加到放置容器中
    place->insert(std::move(stage));
  }

  // 分离物体阶段
  {
    // 创建规划场景修改阶段
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    // 将物体从手爪上分离
    stage->detachObject("object", hand_frame);
    // 将分离物体阶段添加到放置容器中
    place->insert(std::move(stage));
  }

  //  retreat阶段(后退)
  {
    // 创建相对移动阶段，用于 retreat
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
    // 从父级继承组属性
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    // 设置最小和最大移动距离
    stage->setMinMaxDistance(0.1, 0.3);
    // 设置逆运动学参考帧
    stage->setIKFrame(hand_frame);
    // 设置标记命名空间，用于可视化
    stage->properties().set("marker_ns", "retreat");

    // 设置 retreat 方向
    geometry_msgs::msg::Vector3Stamped vec;
    // 设置方向参考坐标系为世界坐标系
    vec.header.frame_id = "world";
    // 设置X轴负方向（后退）
    vec.vector.x = -0.5;
    // 设置移动方向
    stage->setDirection(vec);
    // 将 retreat 阶段添加到放置容器中
    place->insert(std::move(stage));
  }

  // 将放置容器添加到任务中
  task.add(std::move(place));
}

// 返回Home阶段
{
  // 创建移动到目标姿态阶段
  auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
  // 从父级继承组属性
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  // 设置目标为"ready"姿态
  stage->setGoal("ready");
  // 将返回Home阶段添加到任务中
  task.add(std::move(stage));
}

// 返回构建完成的任务对象
return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}