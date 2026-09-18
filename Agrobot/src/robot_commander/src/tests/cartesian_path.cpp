// This template demonstrates how to use the MoveGroupInterface to plan and execute a motion along a Cartesian path
// It initializes a ROS 2 node, creates a MoveGroupInterface for the "arm" group, and plans/executes along a Cartesian path

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char** argv)
{
    // --- Setup ---

    rclcpp::init(argc, argv); // Initialize ROS 2

    auto node = std::make_shared<rclcpp::Node>("test_cartesian_path_node"); // Create a node
    rclcpp::executors::SingleThreadedExecutor executor; // Create an executor
    executor.add_node(node); // Add the node to the executor
    auto spinner = std::thread([&executor]() { executor.spin(); }); // Spin the executor in a separate thread

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm"); // Create a MoveGroupInterface for the "arm" group
    arm.setMaxVelocityScalingFactor(1.0); // Set the maximum velocity scaling factor
    arm.setMaxAccelerationScalingFactor(1.0); // Set the maximum acceleration scaling factor

    // --- Plan and execute along a Cartesian path ---

    std::vector<geometry_msgs::msg::Pose> waypoints; // Create a vector to hold the waypoints

    geometry_msgs::msg::Pose pose1 = arm.getCurrentPose().pose; // Get the current pose of the end effector and use it as the starting point for the Cartesian path
    pose1.position.z += -0.2;                                   // Move the end effector down by 20 cm
    waypoints.push_back(pose1);                                 // Add the first waypoint to the vector

    geometry_msgs::msg::Pose pose2 = pose1; // Create a second pose based on the first waypoint
    pose2.position.y += 0.2;                // Move the end effector to the left by 20 cm
    waypoints.push_back(pose2);             // Add the second waypoint to the vector

    geometry_msgs::msg::Pose pose3 = pose2; // Create a third pose based on the second waypoint
    pose3.position.y += -0.2;               // Move the end effector back to the right by 20 cm
    pose3.position.z += 0.2;                // Move the end effector back up by 20 cm
    waypoints.push_back(pose3);             // Add the third waypoint to the vector

    moveit_msgs::msg::RobotTrajectory trajectory; // Create a RobotTrajectory message to hold the planned trajectory

    double fraction = arm.computeCartesianPath(waypoints, 0.01, trajectory); // Try to plan a Cartesian path through the waypoints with a step size of 1 cm, returning the fraction of the path that was successfully planned

    if (fraction == 1) // Check if the entire path was planned successfully
    {
        arm.execute(trajectory); // Execute the planned trajectory if it was successful
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