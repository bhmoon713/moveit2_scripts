#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>

#include <custom_msgs/msg/inference_result.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <gazebo_msgs/msg/model_states.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class PickAndPlaceTrajectory {
public:
  // --- type aliases (must be visible before constructor) ---
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;
  using RobotTrajectory = moveit_msgs::msg::RobotTrajectory;

  explicit PickAndPlaceTrajectory(rclcpp::Node::SharedPtr base_node)
      : base_node_(std::move(base_node)) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Pick And Place Trajectory...");

    // Configure node options
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // Dedicated node and executor thread for MoveIt + subs
    move_group_node_ =
        rclcpp::Node::make_shared("move_group_node", node_options);
    executor_.add_node(move_group_node_);
    std::thread([this]() { this->executor_.spin(); }).detach();

    // Subscribe to circle inference
    circle_sub_ = move_group_node_
                      ->create_subscription<custom_msgs::msg::InferenceResult>(
                          "/circle_inference_result",
                          rclcpp::QoS(rclcpp::KeepLast(20)).best_effort(),
                          std::bind(&PickAndPlaceTrajectory::onCircleInference,
                                    this, std::placeholders::_1));

    // MoveGroup interfaces
    move_group_robot_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_ROBOT);
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_GRIPPER);
    move_group_gripper_->setMaxVelocityScalingFactor(0.2);
    move_group_gripper_->setMaxAccelerationScalingFactor(0.2);

    // Joint model groups
    joint_model_group_robot_ =
        move_group_robot_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_ROBOT);
    joint_model_group_gripper_ =
        move_group_gripper_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_GRIPPER);

    // Info
    RCLCPP_INFO(LOGGER, "Planning Frame: %s",
                move_group_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Link: %s",
                move_group_robot_->getEndEffectorLink().c_str());
    std::vector<std::string> group_names =
        move_group_robot_->getJointModelGroupNames();
    for (size_t i = 0; i < group_names.size(); ++i)
      RCLCPP_INFO(LOGGER, "Group %zu: %s", i, group_names[i].c_str());

    // Seed current joint states (also sizes the vectors)
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);
    current_state_gripper_ = move_group_gripper_->getCurrentState(10);
    current_state_gripper_->copyJointGroupPositions(
        joint_model_group_gripper_, joint_group_positions_gripper_);

    move_group_robot_->setStartStateToCurrentState();
    move_group_gripper_->setStartStateToCurrentState();

    RCLCPP_INFO(LOGGER, "Class Initialized: Pick And Place Trajectory");
  }

  ~PickAndPlaceTrajectory() {
    RCLCPP_INFO(LOGGER, "Class Terminated: Pick And Place Trajectory");
  }

  void execute_trajectory_plan() {
    RCLCPP_INFO(LOGGER,
                "Planning and Executing Draw X Cartesian Trajectory...");

    // Home
    RCLCPP_INFO(LOGGER, "Going to Home Position...");
    setup_joint_value_target(+0.0000, -1.5708, +1.5708, -1.5708, -1.5708,
                             +0.0000);
    RCLCPP_INFO(LOGGER, "Planning Joint Value Trajectory...");
    plan_trajectory_kinematics();
    RCLCPP_INFO(LOGGER, "Executing Joint Value Trajectory...");
    execute_trajectory_kinematics();

    // Open gripper
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    RCLCPP_INFO(LOGGER, "Opening Gripper...");
    setup_named_pose_gripper("open");
    plan_trajectory_gripper();
    execute_trajectory_gripper();

    // Pregrasp
    RCLCPP_INFO(LOGGER, "Going to Pregrasp Position...");
    setup_goal_pose_target(+0.300, +0.330, +0.320, -1.000, +0.000, +0.000,
                           +0.000);
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Down (cartesian)
    RCLCPP_INFO(LOGGER, "Doing X Cartesian Trajectory...");
    setup_waypoints_target(0.000, 0.000, -0.040);
    plan_trajectory_cartesian();
    execute_trajectory_cartesian();

    // Close gripper
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    RCLCPP_INFO(LOGGER, "Closing Gripper...");
    setup_named_pose_gripper("close_0");
    plan_trajectory_gripper();
    execute_trajectory_gripper();

    // Up
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    setup_waypoints_target(0.000, 0.000, +0.040);
    plan_trajectory_cartesian();
    execute_trajectory_cartesian();

    // Move toward camera center (fixed delta first)
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    setup_waypoints_target(-0.550, -0.130, 0.000);
    plan_trajectory_cartesian();
    execute_trajectory_cartesian();

    // Then refine using latest circle offsets (wait up to ~5s for data)
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    double dx = 0.0, dy = 0.0;
    if (wait_for_circle_offsets(std::chrono::seconds(5), dx, dy)) {
      RCLCPP_INFO(
          LOGGER,
          "Preparing Cartesian Trajectory to target base (dx=%.4f, dy=%.4f)...",
          dx, dy);
      setup_waypoints_target(dx, dy, 0.000);
      plan_trajectory_cartesian();
      execute_trajectory_cartesian();
    } else {
      RCLCPP_WARN(
          LOGGER,
          "Timed out waiting for circle offsets; skipping fine alignment.");
    }

    // Place down
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    RCLCPP_INFO(LOGGER, "Preparing Cartesian Trajectory (place down)...");
    setup_waypoints_target(0.0, 0.0, -0.610);
    plan_trajectory_cartesian();
    execute_trajectory_cartesian();

    // Open
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    RCLCPP_INFO(LOGGER, "Opening Gripper...");
    setup_named_pose_gripper("open");
    plan_trajectory_gripper();
    execute_trajectory_gripper();

    // Lift up
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    RCLCPP_INFO(LOGGER, "Preparing Cartesian Trajectory (lift up)...");
    setup_waypoints_target(0.000, 0.000, +0.610);
    plan_trajectory_cartesian();
    execute_trajectory_cartesian();

    // Close gripper (post-place)
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    RCLCPP_INFO(LOGGER, "Closing Gripper...");
    setup_named_pose_gripper("close_3");
    plan_trajectory_gripper();
    execute_trajectory_gripper();

    // Home
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    RCLCPP_INFO(LOGGER, "Going to Home Position...");
    setup_joint_value_target(+0.0000, -1.5708, +1.5708, -1.5708, -1.5708,
                             +0.0000);
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();

    RCLCPP_INFO(LOGGER, "Wait 5 seconds to reach final state.");
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    RCLCPP_INFO(LOGGER, "One cycle complete.");
  }

private:
  // --- nodes / executor ---
  rclcpp::Node::SharedPtr base_node_;
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  // --- move groups ---
  std::shared_ptr<MoveGroupInterface> move_group_robot_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
  const JointModelGroup *joint_model_group_robot_{nullptr};
  const JointModelGroup *joint_model_group_gripper_{nullptr};

  // --- kinematics state/plans ---
  std::vector<double> joint_group_positions_robot_;
  RobotStatePtr current_state_robot_;
  Plan kinematics_trajectory_plan_;
  Pose target_pose_robot_;
  bool plan_success_robot_{false};

  std::vector<double> joint_group_positions_gripper_;
  RobotStatePtr current_state_gripper_;
  Plan gripper_trajectory_plan_;
  bool plan_success_gripper_{false};

  // --- (optional) gazebo sub (kept for future) ---
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr
      model_states_sub_;

  // --- cartesian planning ---
  std::vector<Pose> cartesian_waypoints_;
  RobotTrajectory cartesian_trajectory_plan_;
  const double jump_threshold_ = 0.0;
  const double end_effector_step_ = 0.01;
  double plan_fraction_robot_ = 0.0;

  // --- circle inference subscription + storage ---
  rclcpp::Subscription<custom_msgs::msg::InferenceResult>::SharedPtr
      circle_sub_;
  std::mutex small_circle_mutex_;
  double small_circle_x_px_ = std::numeric_limits<double>::quiet_NaN();
  double small_circle_y_px_ = std::numeric_limits<double>::quiet_NaN();
  bool have_small_circle_ = false;

  // stored metric deltas derived from circle center (updated in callback)
  double dx_to_target_{0.0};
  double dy_to_target_{0.0};

  // --- helpers ---
  void setup_joint_value_target(float a0, float a1, float a2, float a3,
                                float a4, float a5) {
    joint_group_positions_robot_[0] = a0;
    joint_group_positions_robot_[1] = a1;
    joint_group_positions_robot_[2] = a2;
    joint_group_positions_robot_[3] = a3;
    joint_group_positions_robot_[4] = a4;
    joint_group_positions_robot_[5] = a5;
    move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
  }

  void setup_goal_pose_target(float x, float y, float z, float qx, float qy,
                              float qz, float qw) {
    target_pose_robot_.position.x = x;
    target_pose_robot_.position.y = y;
    target_pose_robot_.position.z = z;
    target_pose_robot_.orientation.x = qx;
    target_pose_robot_.orientation.y = qy;
    target_pose_robot_.orientation.z = qz;
    target_pose_robot_.orientation.w = qw;
    move_group_robot_->setPoseTarget(target_pose_robot_);
  }

  void plan_trajectory_kinematics() {
    plan_success_robot_ =
        (move_group_robot_->plan(kinematics_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_kinematics() {
    if (plan_success_robot_) {
      move_group_robot_->execute(kinematics_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Success!");
    } else {
      RCLCPP_ERROR(LOGGER, "Robot Kinematics Trajectory Failed!");
    }
  }

  void setup_waypoints_target(float dx, float dy, float dz) {
    target_pose_robot_ = move_group_robot_->getCurrentPose().pose;
    cartesian_waypoints_.push_back(target_pose_robot_);
    target_pose_robot_.position.x += dx;
    target_pose_robot_.position.y += dy;
    target_pose_robot_.position.z += dz;
    cartesian_waypoints_.push_back(target_pose_robot_);
  }

  void plan_trajectory_cartesian() {
    plan_fraction_robot_ = move_group_robot_->computeCartesianPath(
        cartesian_waypoints_, end_effector_step_, jump_threshold_,
        cartesian_trajectory_plan_);
  }

  void execute_trajectory_cartesian() {
    if (plan_fraction_robot_ >= 0.0) {
      move_group_robot_->execute(cartesian_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Success (fraction=%.3f)!",
                  plan_fraction_robot_);
    } else {
      RCLCPP_ERROR(LOGGER, "Robot Cartesian Trajectory Failed!");
    }
    cartesian_waypoints_.clear();
  }

  void setup_named_pose_gripper(const std::string &pose_name) {
    move_group_gripper_->setNamedTarget(pose_name);
  }

  void plan_trajectory_gripper() {
    plan_success_gripper_ =
        (move_group_gripper_->plan(gripper_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_gripper() {
    if (plan_success_gripper_) {
      move_group_gripper_->execute(gripper_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Gripper Action Command Success!");
    } else {
      RCLCPP_ERROR(LOGGER, "Gripper Action Command Failed!");
    }
  }

  // circle inference callback
  void
  onCircleInference(const custom_msgs::msg::InferenceResult::SharedPtr msg) {
    // Estimate radius from bbox (works for both "circle_small" and "circle")
    const double r_px = 0.25 * (static_cast<double>(msg->box_width) +
                                static_cast<double>(msg->box_height));
    const int r_px_rounded = static_cast<int>(std::lround(r_px));

    // NEW: filter out false perception of 19-pixel-radius circles
    if (r_px_rounded == 19) {
      RCLCPP_WARN_THROTTLE(
          move_group_node_->get_logger(), *move_group_node_->get_clock(),
          std::chrono::milliseconds(2000).count(),
          "Ignoring circle with ~19 px radius (r=%.2f).", r_px);
      return;
    }

    // Prefer class_name == "circle_small"
    bool is_small = (msg->class_name == std::string("circle_small"));

    // Fallback if publisher uses "circle": treat 10..20 px as "small"
    if (!is_small && msg->class_name == std::string("circle")) {
      if (r_px >= 10.0 && r_px <= 20.0)
        is_small = true;
    }

    if (!is_small)
      return;

    // Store latest (pixel) center
    {
      std::lock_guard<std::mutex> lk(small_circle_mutex_);
      small_circle_x_px_ = static_cast<double>(msg->x);
      small_circle_y_px_ = static_cast<double>(msg->y);
      have_small_circle_ = true;

      // Convert pixel error to meters (2 px ≈ 1 mm → 0.002 m per px)
      dx_to_target_ = (small_circle_x_px_ - 373.0) * 0.002;
      dy_to_target_ = (small_circle_y_px_ - 290.0) * 0.002;
    }

    RCLCPP_INFO_THROTTLE(
        move_group_node_->get_logger(), *move_group_node_->get_clock(),
        std::chrono::milliseconds(2000).count(),
        "distance to target (m): (%.4f, %.4f), r_px=%.2f (~%d px)",
        dx_to_target_, dy_to_target_, r_px, r_px_rounded);
  }

  // wait helper for offsets
  template <class Rep, class Period>
  bool wait_for_circle_offsets(std::chrono::duration<Rep, Period> timeout,
                               double &dx, double &dy) {
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
      {
        std::lock_guard<std::mutex> lk(small_circle_mutex_);
        if (have_small_circle_) {
          dx = dx_to_target_;
          dy = dy_to_target_;
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
  }
}; // class PickAndPlaceTrajectory

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto base_node = std::make_shared<rclcpp::Node>("pick_and_place_trajectory");
  PickAndPlaceTrajectory node(base_node);
  node.execute_trajectory_plan();
  rclcpp::shutdown();
  return 0;
}
