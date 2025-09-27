#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

// program variables
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";

class DoubleMotionTrajectory {
public:
  DoubleMotionTrajectory(rclcpp::Node::SharedPtr base_node_)
      : base_node_(base_node_) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Double Motion Trajectory...");

    // configure node options
    rclcpp::NodeOptions node_options;
    // auto-declare node_options parameters from overrides
    node_options.automatically_declare_parameters_from_overrides(true);

    // initialize move_group node
    move_group_node_ =
        rclcpp::Node::make_shared("move_group_node", node_options);
    // start move_group node in a new executor thread and spin it
    executor_.add_node(move_group_node_);
    std::thread([this]() { this->executor_.spin(); }).detach();

    // initialize move_group interfaces
    move_group_robot_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_ROBOT);

    // get initial state of robot
    joint_model_group_robot_ =
        move_group_robot_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_ROBOT);

    // print out basic system information
    RCLCPP_INFO(LOGGER, "Planning Frame: %s",
                move_group_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Link: %s",
                move_group_robot_->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::vector<std::string> group_names =
        move_group_robot_->getJointModelGroupNames();
    // more efficient method than std::copy() method used in the docs
    for (long unsigned int i = 0; i < group_names.size(); i++) {
      RCLCPP_INFO(LOGGER, "Group %ld: %s", i, group_names[i].c_str());
    }

    // get current state of robot
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);

    // set start state of robot to current state
    move_group_robot_->setStartStateToCurrentState();

    // indicate initialization
    RCLCPP_INFO(LOGGER, "Class Initialized: Double Motion Trajectory");
  }

  ~DoubleMotionTrajectory() {
    // indicate termination
    RCLCPP_INFO(LOGGER, "Class Terminated: Double Motion Trajectory");
  }

  void execute_trajectory_plan() {
    RCLCPP_INFO(LOGGER, "Planning and Executing Double Motion Trajectory...");

    // setup the joint value target
    RCLCPP_INFO(LOGGER, "Preparing Joint Value Trajectory 1...");
    setup_joint_value_target(+0.0000, -1.5708, +0.0000, +0.0000, +0.0000,
                             -1.5708);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Joint Value Trajectory 1...");
    plan_trajectory_kinematics();
    RCLCPP_INFO(LOGGER, "Executing Joint Value Trajectory 1...");
    execute_trajectory_kinematics();

    // setup the joint value target
    RCLCPP_INFO(LOGGER, "Preparing Joint Value Trajectory 2...");
    // setup_joint_value_target(-1.5708, -1.5708, +1.5708, +0.0000, +0.0000,
    //                          +1.5708);
    setup_goal_pose_target(+0.343, +0.3, +0.284, -1.000, +0.000, +0.000,
                           +0.000);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Joint Value Trajectory 2...");
    plan_trajectory_kinematics();
    RCLCPP_INFO(LOGGER, "Executing Joint Value Trajectory 2...");
    execute_trajectory_kinematics();

    // // setup the poset value target
    // RCLCPP_INFO(LOGGER, "Preparing Joint Value Trajectory 3...");
    // // setup_goal_pose_target(0.04, 0.05, 0.04, -1.0000, +0.0000, +0.0000,
    // //                          +0.0000);
    // // setup_goal_pose_target(+0.3, +0.4, +0.284, -1.000, +0.000, +0.000,
    // //                        +0.000);
    // setup_goal_pose_target(+0.343, +0.3, +0.284, -1.000, +0.000, +0.000,
    //                        +0.000);
    // // plan and execute the trajectory
    // RCLCPP_INFO(LOGGER, "Planning Joint Value Trajectory 3...");
    // plan_trajectory_kinematics();
    // RCLCPP_INFO(LOGGER, "Executing Joint Value Trajectory 3...");
    // execute_trajectory_kinematics();

    RCLCPP_INFO(LOGGER, "Double Motion Trajectory Execution Complete");
  }

private:
  // using shorthand for lengthy class references
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;

  // declare rclcpp base node class
  rclcpp::Node::SharedPtr base_node_;

  // declare move_group node
  rclcpp::Node::SharedPtr move_group_node_;

  // declare single threaded executor for move_group node
  rclcpp::executors::SingleThreadedExecutor executor_;

  // declare move_group_interface variables for robot
  std::shared_ptr<MoveGroupInterface> move_group_robot_;

  // declare joint_model_group for robot
  const JointModelGroup *joint_model_group_robot_;

  // declare trajectory planning variables for robot
  std::vector<double> joint_group_positions_robot_;
  RobotStatePtr current_state_robot_;
  Plan kinematics_trajectory_plan_;
  Pose target_pose_robot_;
  bool plan_success_robot_ = false;

  void setup_joint_value_target(float angle0, float angle1, float angle2,
                                float angle3, float angle4, float angle5) {
    // set the joint values for each joint of robot arm
    joint_group_positions_robot_[0] = angle0; // Shoulder Pan
    joint_group_positions_robot_[1] = angle1; // Shoulder Lift
    joint_group_positions_robot_[2] = angle2; // Elbow
    joint_group_positions_robot_[3] = angle3; // Wrist 1
    joint_group_positions_robot_[4] = angle4; // Wrist 2
    joint_group_positions_robot_[5] = angle5; // Wrist 3
    move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
  }

  void setup_goal_pose_target(float pos_x, float pos_y, float pos_z,
                              float quat_x, float quat_y, float quat_z,
                              float quat_w) {
    // set the pose values for end effector of robot arm
    target_pose_robot_.position.x = pos_x;
    target_pose_robot_.position.y = pos_y;
    target_pose_robot_.position.z = pos_z;
    target_pose_robot_.orientation.x = quat_x;
    target_pose_robot_.orientation.y = quat_y;
    target_pose_robot_.orientation.z = quat_z;
    target_pose_robot_.orientation.w = quat_w;
    move_group_robot_->setPoseTarget(target_pose_robot_);
  }

  void plan_trajectory_kinematics() {
    // plan the trajectory to target using kinematics
    plan_success_robot_ =
        (move_group_robot_->plan(kinematics_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_kinematics() {
    // execute the planned trajectory to target using kinematics
    if (plan_success_robot_) {
      move_group_robot_->execute(kinematics_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Failed !");
    }
  }

}; // class DoubleMotionTrajectory

int main(int argc, char **argv) {

  // initialize program node
  rclcpp::init(argc, argv);

  // initialize base_node as shared pointer
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("double_motion_trajectory");

  // instantiate class
  DoubleMotionTrajectory double_motion_trajectory_node(base_node);

  // execute trajectory plan
  double_motion_trajectory_node.execute_trajectory_plan();

  // shutdown ros2 node
  rclcpp::shutdown();

  return 0;
}

// End of Code