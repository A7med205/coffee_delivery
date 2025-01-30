#include "rclcpp/publisher.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

// Custom messages header
#include <moveit2_scripts/msg/int_command.hpp>
#include <moveit2_scripts/msg/int_state.hpp>
#include <moveit2_scripts/msg/place_pos.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_node");

// Planning group names
static const std::string PLANNING_GROUP = "manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class PickPlaceNode : public rclcpp::Node {
public:
  PickPlaceNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("pick_place_node", options) {

    // Subscribers
    pick_place_sub_ = this->create_subscription<moveit2_scripts::msg::PlacePos>(
        "/pos_topic", 10,
        std::bind(&PickPlaceNode::pick_place_callback, this,
                  std::placeholders::_1));

    command_sub_ = this->create_subscription<moveit2_scripts::msg::IntCommand>(
        "command_topic", 10,
        std::bind(&PickPlaceNode::command_callback, this,
                  std::placeholders::_1));
    // Publisher
    state_pub = this->create_publisher<moveit2_scripts::msg::IntState>(
        "/current_state", 10);

    // Initialize the PlanningSceneInterface here or wherever is convenient
    planning_scene_interface_ =
        std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // Add the table box to the planning scene
    add_table_collision();

    // Startup values
    cmd = Command::None;
    x_coord = y_coord = 0;

    RCLCPP_INFO(LOGGER, "PickPlaceNode is ready and waiting for messages on "
                        "'pick_place_topic'.");
  }

  void initialize_move_groups() {
    // Initialize MoveGroupInterfaces
    move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), PLANNING_GROUP);
    move_group_gripper_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), PLANNING_GROUP_GRIPPER);
  }

private:
  void add_table_collision() {
    // Creating collision objects
    moveit_msgs::msg::CollisionObject table_object, coffee_machine;
    table_object.id = "table";
    table_object.header.frame_id = "base_link";
    coffee_machine.id = "machine";
    coffee_machine.header.frame_id = "base_link";

    // Defining primitives
    shape_msgs::msg::SolidPrimitive table_primitive, machine_primitive;
    table_primitive.type = table_primitive.BOX;
    table_primitive.dimensions.resize(3);
    table_primitive.dimensions[0] = 0.85; // X dimension
    table_primitive.dimensions[1] = 1.81; // Y dimension
    table_primitive.dimensions[2] = 0.05; // Z dimension
    machine_primitive.type = machine_primitive.BOX;
    machine_primitive.dimensions.resize(3);
    machine_primitive.dimensions[0] = 0.65; // X dimension
    machine_primitive.dimensions[1] = 0.20; // Y dimension
    machine_primitive.dimensions[2] = 0.55; // Z dimension

    // Defining the pose of the table
    geometry_msgs::msg::Pose table_pose, machine_pose;
    table_pose.orientation.w = 1.0; // No rotation
    table_pose.position.x = 0.3;
    table_pose.position.y = 0.36;
    table_pose.position.z = -0.05 / 2.0 - 0.005;
    machine_pose.orientation.w = 1.0;
    machine_pose.position.x = 0.3;
    machine_pose.position.y = 0.828;
    machine_pose.position.z = 0.225;

    table_object.primitives.push_back(table_primitive);
    table_object.primitive_poses.push_back(table_pose);
    table_object.operation = table_object.ADD;
    coffee_machine.primitives.push_back(machine_primitive);
    coffee_machine.primitive_poses.push_back(machine_pose);
    coffee_machine.operation = coffee_machine.ADD;

    // Adding objects to the planning scene
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(table_object);
    collision_objects.push_back(coffee_machine);
    planning_scene_interface_->addCollisionObjects(collision_objects);

    RCLCPP_INFO(LOGGER, "Added 'table' collision object to the world.");
  }

  void command_callback(const moveit2_scripts::msg::IntCommand::SharedPtr msg) {
    RCLCPP_INFO(LOGGER, "Received new command");
    // Automatic mode
    if (msg->data == 1) {
      cmd = Command::Pick_Pose;
      incremental_ = false;
      // Incremental mode
    } else if (msg->data == 2) {
      if (cmd == Command::None) {
        cmd = Command::Pick_Pose;
      }
      incremental_ = true;
      // Home
    } else if (msg->data == 4) {
      cmd = Command::Home;
      incremental_ = false;
    }

    // Checking if coordinates are avilable
    if (x_coord != 0 && y_coord != 0) {
      arm_sequence();
    } else {
      RCLCPP_INFO(LOGGER, "No coordinates avilable");
    }
  }

  void
  pick_place_callback(const moveit2_scripts::msg::PlacePos::SharedPtr msg) {
    RCLCPP_INFO(LOGGER, "Received new pick-and-place coordinates");
    x_coord = msg->data[0];
    y_coord = msg->data[1];
  }

  void arm_sequence() {
    // Ensuring the current state is our start state
    move_group_gripper_->setStartStateToCurrentState();
    move_group_->setStartStateToCurrentState();

    // Movegroup interface plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    bool success;

    // Setting pick and itermediatery poses
    geometry_msgs::msg::Pose pick_pose;
    pick_pose.position.x = 0.212;
    pick_pose.position.y = 0.346;
    pick_pose.position.z = 0.352;
    pick_pose.orientation.x = 1.0;
    pick_pose.orientation.y = 0.0;
    pick_pose.orientation.z = 0.0;
    pick_pose.orientation.w = 0.0;

    geometry_msgs::msg::Pose intermediatery_pose;
    intermediatery_pose.position.x = -0.334;
    intermediatery_pose.position.y = 0.057;
    intermediatery_pose.position.z = 0.403;
    intermediatery_pose.orientation.x = 1.0;
    intermediatery_pose.orientation.y = 0.0;
    intermediatery_pose.orientation.z = 0.0;
    intermediatery_pose.orientation.w = 0.0;

    // Cartesian approach/retreat parameters
    const double eef_step = 0.01;      // 1 cm resolution
    const double jump_threshold = 0.0; // Disable jump threshold

    // Publisher message
    moveit2_scripts::msg::IntState state_msg;

    while (cmd != Command::None) {
      switch (cmd) {
      case Command::Pick_Pose: {
        // Moving Arm to the Pick Pose
        move_group_->setPoseTarget(pick_pose);
        success =
            (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
          move_group_->execute(plan);
          RCLCPP_INFO(LOGGER, "Arm moved to pick pose.");
          state_msg.data = 2;
          cmd = Command::Pre_Approach;
        } else {
          state_msg.data = 0;
          cmd = Command::None;
          RCLCPP_WARN(LOGGER, "Failed to plan to pick pose.");
        }
        break;
      }

      case Command::Pre_Approach: {
        // Opening the Gripper
        move_group_gripper_->setNamedTarget("open_gripper");

        success = (move_group_gripper_->plan(gripper_plan) ==
                   moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
          move_group_gripper_->execute(gripper_plan);
          RCLCPP_INFO(LOGGER, "Gripper opened.");
          state_msg.data = 3;
          cmd = Command::Pre_Grasp;
        } else {
          state_msg.data = 0;
          cmd = Command::None;
          RCLCPP_WARN(LOGGER, "Failed to open gripper.");
        }
        break;
      }

      case Command::Pre_Grasp: {
        // Cartesian approach
        std::vector<geometry_msgs::msg::Pose> approach_waypoints;
        // Moving down 3.5 cm
        pick_pose.position.z -= 0.035;
        approach_waypoints.push_back(pick_pose);
        // Moving down another 3.5 cm
        pick_pose.position.z -= 0.035;
        approach_waypoints.push_back(pick_pose);

        moveit_msgs::msg::RobotTrajectory approach_trajectory;
        double fraction = move_group_->computeCartesianPath(
            approach_waypoints, eef_step, jump_threshold, approach_trajectory);

        if (fraction > 0.0) {
          move_group_->execute(approach_trajectory);
          RCLCPP_INFO(LOGGER, "Approach successful.");
          state_msg.data = 4;
          cmd = Command::Cup_Grasped;
        } else {
          state_msg.data = 0;
          cmd = Command::None;
          RCLCPP_WARN(LOGGER,
                      "Approach path failed or partial. Fraction = %.2f",
                      fraction);
        }
        break;
      }

      case Command::Cup_Grasped: {
        // Closing the gripper
        move_group_gripper_->setNamedTarget("close_gripper -0.2");
        success = (move_group_gripper_->plan(gripper_plan) ==
                   moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
          move_group_gripper_->execute(gripper_plan);
          RCLCPP_INFO(LOGGER, "Gripper closed.");
          state_msg.data = 5;
          cmd = Command::Pick_Pose_2;
        } else {
          state_msg.data = 0;
          cmd = Command::None;
          RCLCPP_WARN(LOGGER, "Failed to close gripper.");
        }
        break;
      }

      case Command::Pick_Pose_2: {
        // Retreating
        std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
        pick_pose.position.z += 0.035;
        retreat_waypoints.push_back(pick_pose);
        pick_pose.position.z += 0.035;
        retreat_waypoints.push_back(pick_pose);

        moveit_msgs::msg::RobotTrajectory retreat_trajectory;
        double fraction = move_group_->computeCartesianPath(
            retreat_waypoints, eef_step, jump_threshold, retreat_trajectory);

        if (fraction > 0.0) {
          move_group_->execute(retreat_trajectory);
          RCLCPP_INFO(LOGGER, "Retreat successful.");
          state_msg.data = 6;
          cmd = Command::Intermediate_Pose;
        } else {
          state_msg.data = 0;
          cmd = Command::None;
          RCLCPP_WARN(LOGGER, "Retreat path failed or partial. Fraction = %.2f",
                      fraction);
        }
        break;
      }

      case Command::Intermediate_Pose: {
        // Moving arm to intermediatery pose
        // Set the intermediatery pose as the target
        move_group_->setPoseTarget(intermediatery_pose);
        success =
            (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
          move_group_->execute(plan);
          RCLCPP_INFO(LOGGER, "Arm moved to intermediatery pose.");
          state_msg.data = 7;
          cmd = Command::Place_Pose;
        } else {
          state_msg.data = 0;
          cmd = Command::None;
          RCLCPP_WARN(LOGGER, "Failed to plan to intermediatery pose.");
        }
        break;
      }

      case Command::Place_Pose: {
        // Moving arm to the place pose
        geometry_msgs::msg::Pose place_pose;
        place_pose.position.x = x_coord;
        place_pose.position.y = y_coord;
        place_pose.position.z = -0.110;
        place_pose.orientation.x = 1.0;
        place_pose.orientation.y = 0.0;
        place_pose.orientation.z = 0.0;
        place_pose.orientation.w = 0.0;
        move_group_->setPoseTarget(place_pose);
        success =
            (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
          move_group_->execute(plan);
          RCLCPP_INFO(LOGGER, "Arm moved to place pose.");
          state_msg.data = 8;
          cmd = Command::Cup_Placed;
        } else {
          state_msg.data = 0;
          cmd = Command::None;
          RCLCPP_WARN(LOGGER, "Failed to plan to place pose.");
        }
        break;
      }

      case Command::Cup_Placed: {
        // Opening the gripper
        move_group_gripper_->setNamedTarget("open_gripper");
        success = (move_group_gripper_->plan(gripper_plan) ==
                   moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
          move_group_gripper_->execute(gripper_plan);
          RCLCPP_INFO(LOGGER, "Pick-and-place sequence completed.");
          state_msg.data = 9;
          cmd = Command::Home;
        } else {
          state_msg.data = 0;
          cmd = Command::None;
          RCLCPP_WARN(LOGGER, "Failed to open gripper.");
        }
        break;
      }

      case Command::Home: {
        // Returning to home
        move_group_->setNamedTarget("home");
        success =
            (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
          move_group_->execute(plan);
          RCLCPP_INFO(LOGGER, "Returned to home.");
          state_msg.data = 1;
          cmd = Command::None;
        } else {
          state_msg.data = 0;
          cmd = Command::None;
          RCLCPP_WARN(LOGGER, "Failed to return to home.");
        }
        break;
      }

      default:
        break;
      }

      state_pub->publish(state_msg); // Publishing

      // Interrupting if in incremental mode
      if (incremental_) {
        break;
      }
    }

    if (cmd == Command::None) {
      x_coord = y_coord = 0; // Resetting coordinates
    }
  }

  rclcpp::Subscription<moveit2_scripts::msg::PlacePos>::SharedPtr
      pick_place_sub_;
  rclcpp::Subscription<moveit2_scripts::msg::IntCommand>::SharedPtr
      command_sub_;
  rclcpp::Publisher<moveit2_scripts::msg::IntState>::SharedPtr state_pub;

  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>
      planning_scene_interface_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      move_group_gripper_;

  enum class Command {
    None,
    Home,
    Pick_Pose,
    Pre_Approach,
    Pre_Grasp,
    Cup_Grasped,
    Pick_Pose_2,
    Intermediate_Pose,
    Place_Pose,
    Cup_Placed
  };
  Command cmd;
  double x_coord, y_coord;
  bool incremental_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceNode>();
  node->initialize_move_groups(); // Initializing move groups
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
