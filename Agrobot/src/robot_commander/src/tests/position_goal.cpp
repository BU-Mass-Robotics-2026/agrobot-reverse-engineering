// This template demonstrates how to use the MoveGroupInterface to plan and execute a motion to a position goal
// It initializes a ROS 2 node, creates a MoveGroupInterface for the "arm" group, and plans/executes to a position target

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char** argv)
{
    // --- Setup ---

    rclcpp::init(argc, argv); // Initialize ROS 2

    auto node = std::make_shared<rclcpp::Node>("test_position_goal_node"); // Create a node
    rclcpp::executors::SingleThreadedExecutor executor;                 // Create an executor
    executor.add_node(node);                                            // Add the node to the executor
    auto spinner = std::thread([&executor]() { executor.spin(); });     // Spin the executor in a separate thread

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm"); // Create a MoveGroupInterface for the "arm" group
    arm.setMaxVelocityScalingFactor(1.0);                                   // Set the maximum velocity scaling factor
    arm.setMaxAccelerationScalingFactor(1.0);                               // Set the maximum acceleration scaling factor

    // --- Plan and execute to position goal ---

    tf2::Quaternion q1;          // Create a quaternion to represent the orientation
    q1.setRPY(3.14, 0.0, 0.0);   // Set the roll, pitch, and yaw of the quaternion
    q1 = q1.normalize();         // Normalize the quaternion

    geometry_msgs::msg::PoseStamped target_pose1; // Create a Pose message to hold the target pose
    target_pose1.header.frame_id = "world_frame"; // Set the frame of reference for the target pose
    target_pose1.pose.position.x = 1.0;           // Set the x position of the target pose
    target_pose1.pose.position.y = 0.5;           // Set the y position of the target pose
    target_pose1.pose.position.z = 1.0;           // Set the z position of the target pose
    target_pose1.pose.orientation.w = 1.0;        // Set the x orientation of the target pose
    target_pose1.pose.orientation.x = q1.getX();  // Set the x orientation of the target pose
    target_pose1.pose.orientation.y = q1.getY();  // Set the y orientation of the target pose
    target_pose1.pose.orientation.z = q1.getZ();  // Set the z orientation of the target pose
    target_pose1.pose.orientation.w = q1.getW();  // Set the w orientation of the target pose

    arm.setStartStateToCurrentState();  // Set the start state to the current state
    arm.setPoseTarget(target_pose1);    // Set the pose target for the arm

    moveit::planning_interface::MoveGroupInterface::Plan plan1;                 // Create a plan object
    bool success = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS); // Plan to the named target and check if it was successful

    if (success)
    {
        arm.execute(plan1); // Execute the plan if it was successful
    }

    // --- Shutdown ---

    // Stop the background executor before spinner goes out of scope.
    // A joinable std::thread destructor calls std::terminate(), which
    // caused the post-success "terminate called without an active exception" abort.
    executor.cancel();
    if (spinner.joinable()) {
        spinner.join();
    }
    rclcpp::shutdown(); // Shutdown ROS 2
    return 0; // Exit the program
}