#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <robot_interfaces/msg/joint_command.hpp>
#include <robot_interfaces/msg/pose_command.hpp>
#include <robot_interfaces/msg/position_command.hpp>
#include <robot_interfaces/action/pick_sequence.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using PoseCommand = robot_interfaces::msg::PoseCommand;
using JointCommand = robot_interfaces::msg::JointCommand;
using PositionCommand = robot_interfaces::msg::PositionCommand;
using PickSequence = robot_interfaces::action::PickSequence;
using GoalHandlePickSeq = rclcpp_action::ServerGoalHandle<PickSequence>;
using PoseArray = geometry_msgs::msg::PoseArray;
using Pose = geometry_msgs::msg::Pose;
using Bool = std_msgs::msg::Bool;
using namespace std::placeholders;

static constexpr int POSES_PER_TOMATO = 3; // approach, grasp, retract

class Commander
{
    public:

        // -------------------------------------------------------------------------------------------------
        // Constructor
        // -------------------------------------------------------------------------------------------------

        // Constructor for the Commander class, which takes a shared pointer to a ROS 2 node as an argument
        Commander(std::shared_ptr<rclcpp::Node> node)
        {
            node_ = node; // Store the node in a member variable

            arm_ = std::make_shared<MoveGroupInterface>(node_, "arm"); // Create a MoveGroupInterface for the "arm" group
            arm_->setMaxVelocityScalingFactor(1.0);                    // Set the maximum velocity scaling factor
            arm_->setMaxAccelerationScalingFactor(1.0);                // Set the maximum acceleration scaling factor
            arm_->setEndEffectorLink("link6");                         // Set the end effector link

            // Create subscriptions for receiving motion command messages and bind them to their respective callback functions
            pose_cmd_sub_ = node_->create_subscription<PoseCommand>("/agrobot/pose_cmd", 10, std::bind(&Commander::poseCmdCallback, this, _1));
            joint_cmd_sub_ = node_->create_subscription<JointCommand>("/agrobot/joint_cmd", 10, std::bind(&Commander::jointCmdCallback, this, _1));
            position_cmd_sub_ = node_->create_subscription<PositionCommand>("/agrobot/position_cmd", 10, std::bind(&Commander::positionCmdCallback, this, _1));

            // Create a subscription for receiving proceed signals and bind it to the proceed callback function
            proceed_sub_ = node_->create_subscription<Bool>("/agrobot/proceed", 10, std::bind(&Commander::proceedCallback, this, _1));

            // Create an action server for handling pick sequence goals and bind it to the goal, cancel, and accepted callback functions
            action_server_ = rclcpp_action::create_server<PickSequence>(
                node_, 
                "/agrobot/pick_sequence", 
                std::bind(&Commander::handleGoal, this, _1, _2), 
                std::bind(&Commander::handleCancel, this, _1), 
                std::bind(&Commander::handleAccepted, this, _1)
            );

            RCLCPP_INFO(node_->get_logger(), "Commander node initialized and ready to receive commands."); // Log that the commander node has been initialized
        }

        // -------------------------------------------------------------------------------------------------
        // Motion helpers
        // -------------------------------------------------------------------------------------------------

        // Method to move the arm to a named pose target
        void goToPoseTarget(const std::string &name)
        {
            arm_->setStartStateToCurrentState(); // Set the start state to the current state
            arm_->setNamedTarget(name);          // Set the named target
            planAndExecute(arm_);                // Plan and execute the motion to the named target
        }

        // Method to move the arm to a joint target specified by a vector of joint values
        void goToJointTarget(const std::vector<double> &joints)
        {
            arm_->setStartStateToCurrentState(); // Set the start state to the current state
            arm_->setJointValueTarget(joints);   // Set the joint target
            planAndExecute(arm_);                // Plan and execute the motion to the joint target
        }

        // Method to move the arm to a position target specified by position (x, y, z) and orientation (roll, pitch, yaw) values, with an option to use Cartesian path planning
        void goToPositionTarget(double x, double y, double z, double roll, double pitch, double yaw, bool cartesian_path = false)
        {
            tf2::Quaternion q;          // Create a quaternion to represent the orientation
            q.setRPY(roll, pitch, yaw); // Set the roll, pitch, and yaw of the quaternion
            q = q.normalize();          // Normalize the quaternion

            geometry_msgs::msg::PoseStamped target_pose; // Create a Pose message to hold the target pose
            target_pose.header.frame_id = "world_frame"; // Set the frame of reference for the target pose
            target_pose.pose.position.x = x;             // Set the x position of the target pose
            target_pose.pose.position.y = y;             // Set the y position of the target pose
            target_pose.pose.position.z = z;             // Set the z position of the target pose
            target_pose.pose.orientation.x = q.getX();   // Set the x orientation of the target pose
            target_pose.pose.orientation.y = q.getY();   // Set the y orientation of the target pose
            target_pose.pose.orientation.z = q.getZ();   // Set the z orientation of the target pose
            target_pose.pose.orientation.w = q.getW();   // Set the w orientation of the target pose

            // If cartesian_path is false, plan and execute a motion to the pose target
            if (!cartesian_path)
            {
                arm_->setPoseTarget(target_pose);    // Set the pose target
                planAndExecute(arm_);                // Plan and execute the motion to the pose target
            }
            // If cartesian_path is true, plan and execute a Cartesian path to the pose target
            else
            {
                std::vector<geometry_msgs::msg::Pose> waypoints; // Create a vector to hold the waypoints for the Cartesian path
                waypoints.push_back(target_pose.pose);           // Add the target pose to the waypoints
                moveit_msgs::msg::RobotTrajectory trajectory;    // Create a RobotTrajectory message to hold the trajectory of the Cartesian path

                double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory); // Try to plan a Cartesian path through the waypoints with a step size of 1 cm, returning the fraction of the path that was successfully planned
                
                if (fraction == 1)  // Check if the entire path was planned successfully
                {
                    arm_->execute(trajectory); // Execute the planned trajectory if it was successful
                }
            }
        }

        void goToPoseTarget(const Pose &pose)
        {
            geometry_msgs::msg::PoseStamped stamped;
            stamped.header.frame_id = "linear_rail_link";
            stamped.header.stamp = node_->get_clock()->now();
            stamped.pose = pose;
            arm_->setStartStateToCurrentState();
            arm_->setPoseTarget(stamped);
            planAndExecute(arm_);
        }

    private:

        // -------------------------------------------------------------------------------------------------
        // Private members
        // -------------------------------------------------------------------------------------------------

        std::shared_ptr<rclcpp::Node> node_;      // Member variable to hold the shared pointer to the ROS 2 node
        std::shared_ptr<MoveGroupInterface> arm_; // Member variable to hold the MoveGroupInterface for controlling the robot's arm

        rclcpp::Subscription<PoseCommand>::SharedPtr pose_cmd_sub_;         // Subscription for receiving named target command messages
        rclcpp::Subscription<JointCommand>::SharedPtr joint_cmd_sub_;       // Subscription for receiving joint command messages
        rclcpp::Subscription<PositionCommand>::SharedPtr position_cmd_sub_; // Subscription for receiving position command messages
        rclcpp::Subscription<Bool>::SharedPtr proceed_sub_;                 // Subscription for receiving proceed signals

        rclcpp_action::Server<PickSequence>::SharedPtr action_server_; // Action server for handling pick sequence goals

        std::mutex proceed_mutex_;           // Mutex for synchronizing access to the proceed flag
        std::condition_variable proceed_cv_; // Condition variable for waiting on proceed signals
        bool proceed_flag_ = false;          // Flag to indicate whether a proceed signal has been received
        
        // -------------------------------------------------------------------------------------------------
        // Topic callbacks
        // -------------------------------------------------------------------------------------------------

        // Callback function to handle incoming named pose command messages
        void poseCmdCallback(const PoseCommand::SharedPtr msg)
        {
            std::string target_name(msg->pose_name); // Get the target name from the message

            if (target_name == "crouch" || target_name == "attention" || target_name == "vertical" || target_name == "bin") // Check if the target name is one of the valid named targets
            {
                goToPoseTarget(target_name); // Plan and execute a motion to the named target
            }
        }

        // Callback function to handle incoming joint command messages
        void jointCmdCallback(const JointCommand::SharedPtr msg)
        {
            std::vector<double> joints = {msg->j0, msg->j1, msg->j2, msg->j3, msg->j4, msg->j5, msg->j6};
            goToJointTarget(joints);
        }

        // Callback function to handle incoming position command messages
        void positionCmdCallback(const PositionCommand::SharedPtr msg)
        {
            goToPositionTarget(msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw, msg->cartesian_path); // Plan and execute a motion to the position target specified in the message
        }

        void proceedCallback(const Bool::SharedPtr msg)
        {
            if (!msg->data)                                 // If the proceed signal is false, ignore it
                return;

            std::lock_guard<std::mutex> lk(proceed_mutex_); // Lock the mutex to safely update the proceed flag
            proceed_flag_ = true;                           // Set the proceed flag to true to indicate that a proceed signal has been received
            proceed_cv_.notify_all();                       // Notify any waiting threads that a proceed signal has been received
        }

        // -------------------------------------------------------------------------------------------------
        // Action server callbacks
        // -------------------------------------------------------------------------------------------------

        // Callback function to handle incoming pick sequence goals
        rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const PickSequence::Goal> goal)
        {
            (void)uuid;
            if (goal->targets.poses.size() % POSES_PER_TOMATO != 0) // Check if the number of poses in the goal is a multiple of the number of poses per tomato
            {
                RCLCPP_ERROR(node_->get_logger(), "Rejecting goal: pose count %zu not a multiple of %d", goal->targets.poses.size(), POSES_PER_TOMATO);
                return rclcpp_action::GoalResponse::REJECT; // Reject the goal if the pose count is not valid
            }

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // Accept and execute the goal if the pose count is valid
        }

        // Callback function to handle cancel requests for pick sequence goals
        rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandlePickSeq> goal_handle)
        {
            (void)goal_handle;
            RCLCPP_INFO(node_->get_logger(), "Cancel requested"); // Log that a cancel request has been received
            proceed_cv_.notify_all();                             // Notify any waiting threads to unblock them, allowing the canceling thread to proceed
            return rclcpp_action::CancelResponse::ACCEPT;         // Accept the cancel request
        }

        // Callback function to handle accepted pick sequence goals, which starts the execution of the pick sequence in a detached thread
        void handleAccepted(const std::shared_ptr<GoalHandlePickSeq> goal_handle)
        {
            std::thread{[this, goal_handle]() { executeSequence(goal_handle); }}.detach();
        }

        // -------------------------------------------------------------------------------------------------
        // Pick sequence helper functions
        // -------------------------------------------------------------------------------------------------

        // Helper function to publish feedback about the current step of the pick sequence to the action client
        void publishStep(const std::shared_ptr<GoalHandlePickSeq> &goal_handle, std::shared_ptr<PickSequence::Feedback> &feedback, const std::string &step, bool awaiting)
        {
            feedback->step = step;                   // Update the feedback message with the current step
            feedback->awaiting_confirm = awaiting;   // Update the feedback message to indicate whether we are awaiting confirmation to proceed
            goal_handle->publish_feedback(feedback); // Publish the feedback message to the action client
        }

        // Helper function to wait for a proceed signal from the action client before continuing to the next step of the pick sequence, while also checking for cancel requests
        void waitForProceed(const std::shared_ptr<GoalHandlePickSeq> &goal_handle, std::shared_ptr<PickSequence::Feedback> &feedback)
        {
            {
                std::lock_guard<std::mutex> lk(proceed_mutex_); // Lock the mutex to safely update the proceed flag
                proceed_flag_ = false;                          // Reset the proceed flag to false before waiting for the next proceed signal
            }

            publishStep(goal_handle, feedback, feedback->step, true);                                            // Publish feedback indicating that we are awaiting confirmation to proceed
            std::unique_lock<std::mutex> lk(proceed_mutex_);                                                     // Lock the mutex to wait for a proceed signal or a cancel request
            proceed_cv_.wait(lk, [this, &goal_handle] { return proceed_flag_ || goal_handle->is_canceling(); }); // Wait until either a proceed signal is received or a cancel request is made
        }

        // Helper function to execute a step using an explicit Pose object (approach, grasp, retract)
        bool executeStep(const std::shared_ptr<GoalHandlePickSeq> &goal_handle, std::shared_ptr<PickSequence::Feedback> &feedback, const geometry_msgs::msg::Pose &pose_target, const std::string &step, const size_t log_idx, const size_t n)
        {
            if (goal_handle->is_canceling()) return false;

            RCLCPP_INFO(node_->get_logger(), "Tomato %zu/%zu: %s", log_idx, n, step.c_str()); // Log the current step of the pick sequence for the current tomato
            publishStep(goal_handle, feedback, step, false);                                  // Publish feedback about the current step to the action client
            goToPoseTarget(pose_target);                                                      // Execute the motion for the current step

            return !goal_handle->is_canceling();
        }

        // Helper function to execute a step using a Named String target (binning)
        bool executeStep(const std::shared_ptr<GoalHandlePickSeq> &goal_handle, std::shared_ptr<PickSequence::Feedback> &feedback, const std::string &named_target, const std::string &step, const size_t log_idx, const size_t n)
        {
            if (goal_handle->is_canceling()) return false;

            RCLCPP_INFO(node_->get_logger(), "Tomato %zu/%zu: %s", log_idx, n, step.c_str()); // Log the current step of the pick sequence for the current tomato
            publishStep(goal_handle, feedback, step, false);                                  // Publish feedback about the current step to the action client
            goToPoseTarget(named_target);                                                     // Execute the motion for the current step

            return !goal_handle->is_canceling();
        }

        // -------------------------------------------------------------------------------------------------
        // Pick sequence execution (runs in detached thread)
        // -------------------------------------------------------------------------------------------------

        void executeSequence(const std::shared_ptr<GoalHandlePickSeq> goal_handle)
        {
            auto feedback = std::make_shared<PickSequence::Feedback>(); // Create a shared pointer to a feedback message that will be sent to the action client during execution
            auto result = std::make_shared<PickSequence::Result>();     // Create a shared pointer to a result message that will be sent to the action client when execution is complete

            const auto &poses = goal_handle->get_goal()->targets.poses; // Get the target poses from the goal message
            const size_t n = poses.size() / POSES_PER_TOMATO;           // Calculate the number of tomatoes to pick based on the number of poses and the number of poses per tomato
            feedback->total_tomatoes = static_cast<uint32_t>(n);        // Update the feedback message with the total number of tomatoes to pick

            RCLCPP_INFO(node_->get_logger(), "Starting pick sequence for %zu tomatoes", n);

            // Main loop to iterate through the poses for each tomato and execute the pick sequence steps (approach, grasp, retract, bin)
            for (size_t i = 0; i < poses.size(); i += POSES_PER_TOMATO)
            {
                if (goal_handle->is_canceling()) break; // Check if a cancel request has been made before starting the next tomato's pick sequence

                // Fixed: Changed 'current_tomato' to 'tomato_index' to match your PickSequence.action definitions exactly
                feedback->tomato_index = static_cast<uint32_t>(i / POSES_PER_TOMATO) + 1; // Update the feedback message with the index of the current tomato being processed (1-based index for user-friendly display)
                const size_t log_idx = feedback->tomato_index;                            // Calculate the index for logging purposes (1-based index)

                if (!executeStep(goal_handle, feedback, poses[i], "approaching", log_idx, n)) break;
                if (!executeStep(goal_handle, feedback, poses[i+1], "grasping", log_idx, n)) break;
                if (!executeStep(goal_handle, feedback, poses[i+2], "retracting", log_idx, n)) break;
                if (!executeStep(goal_handle, feedback, "bin", "binning", log_idx, n)) break;
            }

            // Centralized cancellation/success handling
            if (goal_handle->is_canceling())
            {
                result->success = false;
                result->message = "Sequence was canceled.";
                goal_handle->canceled(result);
                RCLCPP_INFO(node_->get_logger(), "Sequence processing canceled.");
            }
            else
            {
                result->success = true;
                result->message = "All tomatoes successfully processed!";
                goal_handle->succeed(result);
                RCLCPP_INFO(node_->get_logger(), "Sequence processing completed successfully.");
            }
        }

        // -------------------------------------------------------------------------------------------------
        // Plan and execute helper function
        // -------------------------------------------------------------------------------------------------

        void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
        {
            MoveGroupInterface::Plan plan; // Create a Plan object to hold the planned trajectory
            bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS); // Attempt to plan a motion using the provided MoveGroupInterface and check if it was successful

            if (success)
            {
                interface->execute(plan); // If planning was successful, execute the planned trajectory
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Planning failed"); // Log an error message if planning failed
            }
        }
};


int main(int argc, char** argv)
{
    // --- Setup ---

    rclcpp::init(argc, argv); // Initialize ROS 2

    auto node = std::make_shared<rclcpp::Node>("commander"); // Create a commander node
    auto commander = std::make_shared<Commander>(node);      // Create an instance of the Commander class, passing the node as an argument

    rclcpp::spin(node); // Spin the node to keep it alive and responsive to callbacks

    // --- Shutdown ---

    rclcpp::shutdown(); // Shutdown ROS 2
    return 0; // Exit the program
}