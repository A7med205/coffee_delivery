#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>


// Custom message header (replace with your actual path/package name)
#include <my_robot_msgs/msg/pick_place_poses.hpp>

static const rclcpp::Logger LOGGER =
    rclcpp::get_logger("pick_place_subscriber");

// Hard-coded planning group names
static const std::string PLANNING_GROUP = "manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

/**
 * @class PickPlaceNode
 * @brief Subscribes to PickPlacePoses and performs pick-and-place using MoveIt
 */
class PickPlaceNode : public rclcpp::Node {
public:
  PickPlaceNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("pick_place_subscriber_node", options) {
    // 1. Create MoveGroupInterfaces for arm and gripper
    move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), PLANNING_GROUP);
    move_group_gripper_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), PLANNING_GROUP_GRIPPER);

    // 2. Create a subscription to receive pick & place poses
    pick_place_sub_ =
        this->create_subscription<my_robot_msgs::msg::PickPlacePoses>(
            "pick_place_topic", // Topic name
            10,                 // Queue size
            std::bind(&PickPlaceNode::pickPlaceCallback, this,
                      std::placeholders::_1));

    RCLCPP_INFO(LOGGER, "PickPlaceNode is ready and waiting for messages on "
                        "'pick_place_topic'.");
  }

private:
  /**
   * @brief Callback that runs each time we receive pick/place poses.
   */
  void
  pickPlaceCallback(const my_robot_msgs::msg::PickPlacePoses::SharedPtr msg) {
    RCLCPP_INFO(LOGGER, "Received a new pick-and-place request!");

    // Extract pick and place poses from the message
    geometry_msgs::msg::Pose pick_pose = msg->pick_pose;
    geometry_msgs::msg::Pose place_pose = msg->place_pose;

    // Ensure the current state is our start state
    move_group_gripper_->setStartStateToCurrentState();
    move_group_->setStartStateToCurrentState();

    // ---------------------------------------------------
    // 1) Move Arm to the Pick Pose
    // ---------------------------------------------------
    move_group_->setPoseTarget(pick_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success =
        (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      move_group_->execute(plan);
      RCLCPP_INFO(LOGGER, "Arm moved to pick pose.");
    } else {
      RCLCPP_WARN(LOGGER, "Failed to plan to pick pose.");
      return;
    }

    // ---------------------------------------------------
    // 2) Cartesian Approach (optional example)
    // ---------------------------------------------------
    std::vector<geometry_msgs::msg::Pose> approach_waypoints;
    // Move down 5 cm
    pick_pose.position.z -= 0.05;
    approach_waypoints.push_back(pick_pose);
    // Move down another 5 cm
    pick_pose.position.z -= 0.05;
    approach_waypoints.push_back(pick_pose);

    moveit_msgs::msg::RobotTrajectory approach_trajectory;
    const double eef_step = 0.01;      // 1 cm resolution
    const double jump_threshold = 0.0; // Disable jump threshold
    double fraction = move_group_->computeCartesianPath(
        approach_waypoints, eef_step, jump_threshold, approach_trajectory);

    if (fraction > 0.0) {
      move_group_->execute(approach_trajectory);
      RCLCPP_INFO(LOGGER, "Approach successful.");
    } else {
      RCLCPP_WARN(LOGGER, "Approach path failed or partial. Fraction = %.2f",
                  fraction);
    }

    // ---------------------------------------------------
    // 3) Close the Gripper
    // ---------------------------------------------------
    move_group_gripper_->setNamedTarget("0_60");
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    success = (move_group_gripper_->plan(gripper_plan) ==
               moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      move_group_gripper_->execute(gripper_plan);
      RCLCPP_INFO(LOGGER, "Gripper closed.");
    } else {
      RCLCPP_WARN(LOGGER, "Failed to close gripper.");
    }

    // ---------------------------------------------------
    // 4) Retreat
    // ---------------------------------------------------
    std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
    pick_pose.position.z += 0.05;
    retreat_waypoints.push_back(pick_pose);
    pick_pose.position.z += 0.05;
    retreat_waypoints.push_back(pick_pose);

    moveit_msgs::msg::RobotTrajectory retreat_trajectory;
    fraction = move_group_->computeCartesianPath(
        retreat_waypoints, eef_step, jump_threshold, retreat_trajectory);

    if (fraction > 0.0) {
      move_group_->execute(retreat_trajectory);
      RCLCPP_INFO(LOGGER, "Retreat successful.");
    } else {
      RCLCPP_WARN(LOGGER, "Retreat path failed or partial. Fraction = %.2f",
                  fraction);
    }

    // ---------------------------------------------------
    // 5) Move Arm to the Place Pose
    // ---------------------------------------------------
    move_group_->setPoseTarget(place_pose);
    success =
        (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
      move_group_->execute(plan);
      RCLCPP_INFO(LOGGER, "Arm moved to place pose.");
    } else {
      RCLCPP_WARN(LOGGER, "Failed to plan to place pose.");
      return;
    }

    // ---------------------------------------------------
    // 6) Open the Gripper
    // ---------------------------------------------------
    move_group_gripper_->setNamedTarget("open_gripper");
    success = (move_group_gripper_->plan(gripper_plan) ==
               moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      move_group_gripper_->execute(gripper_plan);
      RCLCPP_INFO(LOGGER, "Gripper opened.");
    } else {
      RCLCPP_WARN(LOGGER, "Failed to open gripper.");
    }

    RCLCPP_INFO(LOGGER, "Pick-and-place sequence completed!");
  }

  // Subscriptions
  rclcpp::Subscription<my_robot_msgs::msg::PickPlacePoses>::SharedPtr
      pick_place_sub_;

  // MoveIt interfaces
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      move_group_gripper_;
};

int main(int argc, char **argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create node (and spin to receive messages)
  auto node = std::make_shared<PickPlaceNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
