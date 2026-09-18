// This template demonstrates how to use the MoveGroupInterface to plan and execute a motion to a joint goal
// It initializes a ROS 2 node, creates a MoveGroupInterface for the "arm" group, and plans/executes to a joint target

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char** argv)
{
    // --- Setup ---

    rclcpp::init(argc, argv); // Initialize ROS 2

    auto node = std::make_shared<rclcpp::Node>("test_joint_goal_node"); // Create a node
    rclcpp::executors::SingleThreadedExecutor executor;                 // Create an executor
    executor.add_node(node);                                            // Add the node to the executor
    auto spinner = std::thread([&executor]() { executor.spin(); });     // Spin the executor in a separate thread

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm"); // Create a MoveGroupInterface for the "arm" group
    arm.setMaxVelocityScalingFactor(1.0);                                   // Set the maximum velocity scaling factor
    arm.setMaxAccelerationScalingFactor(1.0);                               // Set the maximum acceleration scaling factor


    // --- Plan and execute to joint goal ---

    std::vector<double> joints = { 1.5, 0.5, 0.0, 1.5, 0.0, -0.7, 0.0 }; // Create a vector to hold the joint group positions

    arm.setStartStateToCurrentState(); // Set the start state to the current state
    arm.setJointValueTarget(joints);   // Set the joint target using the vector of joint values

    moveit::planning_interface::MoveGroupInterface::Plan plan1;                 // Create a plan object
    bool success = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS); // Plan to the joint target and check if it was successful

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