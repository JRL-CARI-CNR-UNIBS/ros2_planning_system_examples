// Copyright 2024 National Council of Research of Italy (CNR) - Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "plansys2_actions_cost/move_action_cost_base.hpp"

namespace plansys2_actions_cost
{

void MoveActionCostBase::initialize(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
    std::cerr << "Initialize action cost base" << std::endl;
    node_ = node;

    compute_path_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
        node_->shared_from_this(),
        "/compute_path_to_pose");

    RCLCPP_INFO(node_->get_logger(), "Waiting for compute path action server...");

    bool is_action_server_ready =
        compute_path_action_client_->wait_for_action_server(std::chrono::seconds(5));

    if (is_action_server_ready) {
        RCLCPP_INFO(node_->get_logger(), "Compute path action server ready");
    } else {
        RCLCPP_DEBUG(
        node_->get_logger(), "Failed to connect to compute path action server");
    }

    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/computed_path", 10);
    pos_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        10,
        std::bind(&MoveActionCostBase::current_pos_callback, this, std::placeholders::_1));
}

void MoveActionCostBase::compute_action_cost(const geometry_msgs::msg::PoseStamped & goal, 
                                             const plansys2_msgs::msg::ActionExecution::SharedPtr msg)
{
    std::cerr << "Compute action cost" << std::endl;
    auto send_goal_options = 
        rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();

    // Feedback callbkack
    send_goal_options.feedback_callback = [this](
        ComputePathGoalHandle::SharedPtr goal_handle,
        ComputePathFeedback feedback)
        {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "Feedback received from action cost computation (I am computing)");
        };
    // Result callbkack -> Path
    send_goal_options.result_callback = [this, msg](ComputePathResult & result)
    {
        std::cerr << "Result callback" << std::endl;
        ActionCostPtr action_cost = std::make_shared<plansys2_msgs::msg::ActionCost>();

        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_DEBUG(
            node_->get_logger(), "Compute path to cost. Goal failed with error code %d",
            static_cast<int>(result.code));

            action_cost->nominal_cost = std::numeric_limits<double>::infinity();
            action_cost->std_dev_cost = 0.0;
            // action_executor_client_->set_action_cost(action_cost);
            // action_executor_client_->send_response(msg);
            return;
        }

        auto path = result.result->path;
        path_ptr_ = std::make_shared<nav_msgs::msg::Path>(path);
        action_cost = this->compute_cost_function();
        RCLCPP_DEBUG(
            node_->get_logger(), "Computed path cost: %f", action_cost->nominal_cost);
        std::cerr << "Computed path cost: " << action_cost->nominal_cost << std::endl;
        // action_executor_client_->set_action_cost(action_cost);
        // action_executor_client_->send_response(msg);

        path_pub_->publish(path);
    };

    send_goal_options.goal_response_callback =
        [this, msg](const ComputePathGoalHandle::SharedPtr & goal_handle)
        {
            if (!goal_handle) {
                RCLCPP_DEBUG(node_->get_logger(), "Goal was rejected by server");

                ActionCostPtr action_cost = std::make_shared<plansys2_msgs::msg::ActionCost>();
                action_cost->nominal_cost = std::numeric_limits<double>::infinity();
                action_cost->std_dev_cost = 0.0;
                // action_executor_client_->set_action_cost(action_cost);
                // action_executor_client_->send_response(msg);
            } else {
                RCLCPP_DEBUG(
                node_->get_logger(), "Goal accepted by server, waiting for result");
            }

        };
    nav2_msgs::action::ComputePathToPose::Goal path_to_pose_goal = nav2_msgs::action::ComputePathToPose::Goal();
    // TODO(samuele): Retrieve this as a parameter in initialize method
    path_to_pose_goal.planner_id = "GridBased";
    path_to_pose_goal.start.header.frame_id = "map";
    path_to_pose_goal.start = current_pos_;
    std::cerr << "Start pose: " << current_pos_.pose.position.x << " " << current_pos_.pose.position.y << std::endl;
    path_to_pose_goal.goal = goal;
    std::cerr << "Goal pose: " << goal.pose.position.x << " " << goal.pose.position.y << std::endl;
    std::cerr << "pre send" << std::endl;

    auto future_goal_handle = compute_path_action_client_->async_send_goal(path_to_pose_goal, send_goal_options);
    std::cerr << "post send" << std::endl;
    // future_goal_handle->wait();
}

void MoveActionCostBase::current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    current_pos_.header = msg->header;
    current_pos_.pose = msg->pose.pose;
}




}  // namespace plansys2_actions_cost