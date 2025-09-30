cmake_minimum_required(VERSION 3.8)
project(moveit2_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Generate the executable
add_executable(joint_space_trajectory
    src/joint_space_trajectory.cpp)
target_include_directories(joint_space_trajectory
    PUBLIC include)
ament_target_dependencies(joint_space_trajectory
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)


add_executable(goal_pose_trajectory
    src/goal_pose_trajectory.cpp)
target_include_directories(goal_pose_trajectory
    PUBLIC include)
ament_target_dependencies(goal_pose_trajectory
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)


add_executable(double_motion_trajectory
    src/double_motion_trajectory.cpp)
target_include_directories(double_motion_trajectory
    PUBLIC include)
ament_target_dependencies(double_motion_trajectory
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)

# Generate the executable
add_executable(cartesian_trajectory
    src/cartesian_trajectory.cpp)
target_include_directories(cartesian_trajectory
    PUBLIC include)
ament_target_dependencies(cartesian_trajectory
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)


# Generate the executable
add_executable(kinematics_plugin_trajectory
    src/kinematics_plugin_trajectory.cpp)
target_include_directories(kinematics_plugin_trajectory
    PUBLIC include)
ament_target_dependencies(kinematics_plugin_trajectory
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)

# Generate the executable
add_executable(pick_and_place_trajectory
    src/pick_and_place_trajectory.cpp)
target_include_directories(pick_and_place_trajectory
    PUBLIC include)
ament_target_dependencies(pick_and_place_trajectory
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)

add_executable(pick_and_place 
	src/pick_and_place.cpp)
target_include_directories(pick_and_place
    PUBLIC include)
ament_target_dependencies(pick_and_place
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs
    gazebo_msgs)

add_executable(pick_and_place_real
    src/pick_and_place_real.cpp)
target_include_directories(pick_and_place_real
    PUBLIC include)
ament_target_dependencies(pick_and_place_real
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)

add_executable(pick_and_place_perception
    src/pick_and_place_perception.cpp)
target_include_directories(pick_and_place_perception
    PUBLIC include)
ament_target_dependencies(pick_and_place_perception
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs
	custom_msgs)

add_executable(pick_and_place_perception_real
    src/pick_and_place_perception_real.cpp)
target_include_directories(pick_and_place_perception_real
    PUBLIC include)
ament_target_dependencies(pick_and_place_perception_real
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs
	custom_msgs)

add_executable(draw_x
    src/draw_x.cpp)
target_include_directories(draw_x
    PUBLIC include)
ament_target_dependencies(draw_x
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs)

add_executable(draw_x_cartesian_trajectory
    src/draw_x_cartesian_trajectory.cpp)
target_include_directories(draw_x_cartesian_trajectory
    PUBLIC include)
ament_target_dependencies(draw_x_cartesian_trajectory
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)

add_executable(motion1
    src/motion1.cpp)
target_include_directories(motion1
    PUBLIC include)
ament_target_dependencies(motion1
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)
	
# Install the executable
install(TARGETS 
    joint_space_trajectory
    goal_pose_trajectory
	double_motion_trajectory
    cartesian_trajectory
    kinematics_plugin_trajectory
	pick_and_place_trajectory
	pick_and_place
	pick_and_place_real
	pick_and_place_perception
	pick_and_place_perception_real
	draw_x_cartesian_trajectory
	draw_x
	motion1
    DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(DIRECTORY 
    launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
