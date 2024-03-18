// Copyright 2019 Intelligent Robotics Lab
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

#include <math.h>

#include <memory>
#include <string>
#include <map>
#include <algorithm>
#include <functional>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2_ros/static_transform_broadcaster.h"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction()
  : plansys2::ActionExecutorClient("move", 500ms)
  {
    RCLCPP_INFO(get_logger(), "MoveAction created");

    tf_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    RCLCPP_INFO(get_logger(), "MoveAction created");

    declare_parameter<std::vector<std::string>>("waypoints", std::vector<std::string>());
    auto waypoint_names = get_parameter("waypoints").as_string_array();
    for (const auto& name : waypoint_names) {
      geometry_msgs::msg::PoseStamped wp;
      wp.header.frame_id = "map";
      wp.header.stamp = now();
      declare_parameter<double>(name + ".x", 0.0);
      declare_parameter<double>(name + ".y", 0.0);

      wp.pose.position.x = get_parameter(name + ".x").as_double();
      wp.pose.position.y = get_parameter(name + ".y").as_double();
      wp.pose.position.z = 0.0;
      wp.pose.orientation.x = 0.0;
      wp.pose.orientation.y = 0.0;
      wp.pose.orientation.z = 0.0;
      wp.pose.orientation.w = 1.0;

      waypoints_[name] = wp;
      RCLCPP_INFO(get_logger(), "Waypoint: %s, x: %f, y: %f", name.c_str(), wp.pose.position.x, wp.pose.position.y);

      // /* Send waypoints to tf broadcaster */
      // geometry_msgs::msg::TransformStamped wp_t;
      // wp_t.header = wp.header;
      // wp_t.child_frame_id = std::string(get_name()) + "_" + name;
      // wp_t.transform.translation.x = wp.pose.position.x;
      // wp_t.transform.translation.y = wp.pose.position.y;
      // wp_t.transform.translation.z = wp.pose.position.z;
      // wp_t.transform.rotation = wp.pose.orientation;
      // tf_broadcaster_->sendTransform(wp_t);

    }

    using namespace std::placeholders;
    pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose",
      10,
      std::bind(&MoveAction::current_pos_callback, this, _1));
    path_pub_ = create_publisher<nav_msgs::msg::Path>("path_topic", 10);
  }

  void current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    current_pos_.header = msg->header;
    current_pos_.pose = msg->pose.pose;
  }

  CallbackReturnT
  on_configure(const rclcpp_lifecycle::State & state)
  {
    CallbackReturnT result = plansys2::ActionExecutorClient::on_configure(state);
    if(result != CallbackReturnT::SUCCESS)
    {
      return result;
    }
    compute_path_action_client_ =
      rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
      shared_from_this(),
      "compute_path_to_pose");

    bool is_action_server_ready;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for compute path action server...");
      is_action_server_ready =
        compute_path_action_client_->wait_for_action_server(std::chrono::seconds(5));
    } while (!is_action_server_ready);
    RCLCPP_INFO(get_logger(), "Compute path action server ready");

    costmap_subscriber_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
      shared_from_this(), 
      "/global_costmap/costmap_raw");

    RCLCPP_INFO(get_logger(), "COSTMAP created");

    // path_pub_->on_activate();
    
    return CallbackReturnT::SUCCESS; 
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    send_feedback(0.0, "Move starting");

    navigation_action_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      shared_from_this(),
      "navigate_to_pose");

    bool is_action_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");

      is_action_server_ready =
        navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
    } while (!is_action_server_ready);

    RCLCPP_INFO(get_logger(), "Navigation action server ready");

    auto wp_to_navigate = get_arguments()[2];  // The goal is in the 3rd argument of the action
    RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());

    goal_pos_ = waypoints_[wp_to_navigate];
    navigation_goal_.pose = goal_pos_;

    dist_to_move = getDistance(goal_pos_.pose, current_pos_.pose);

    auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.feedback_callback = [this](
      NavigationGoalHandle::SharedPtr,
      NavigationFeedback feedback) {
        set_action_cost(feedback->distance_remaining, 0.0);
        send_feedback(
          std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),
          "Move running");
      };

    send_goal_options.result_callback = [this](auto) {
        finish(true, 1.0, "Move completed");
      };

    future_navigation_goal_handle_ =
      navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);

    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  using ComputePathGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>;
  using ComputePathFeedback =
    const std::shared_ptr<const nav2_msgs::action::ComputePathToPose::Feedback>;
  using ComputePathResult =
    const ComputePathGoalHandle::WrappedResult;
    // const std::shared_ptr<const nav2_msgs::action::ComputePathToPose::Result>;

  double getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2)
  {
    return sqrt(
      (pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
      (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
  }

  void do_work()
  {
  }
  double calculate_path_cost(const nav_msgs::msg::Path& path)
  {
    if(path.poses.size() == 0)
    {
      RCLCPP_INFO(get_logger(), "Empty path. Returning infinity.");
      return std::numeric_limits<double>::infinity();
    }

    unsigned int mx = 0;
    unsigned int my = 0;
    unsigned int cost = 0.0;

    std::vector<double> costs;
    costs.reserve(path.poses.size());

    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap = nullptr;
    bool costmap_ready = false;

    while (!costmap_ready && rclcpp::ok()) {
        try {
            costmap = costmap_subscriber_->getCostmap();
            if (costmap) {
                costmap_ready = true;
            } else {
                return std::numeric_limits<double>::infinity();
            }
        } catch (const std::exception& e) {
            RCLCPP_INFO(get_logger(), "Waiting costmap");
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    for (const auto& pose : path.poses)
    {
      costmap->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my);
      cost = costmap->getCost(mx, my);
      
      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        return std::numeric_limits<double>::infinity();
      }
      costs.push_back(static_cast<double>(cost));
    }
    double std_cost = 0.0;
    double cumulative_cost = std::accumulate(costs.begin(), costs.end(), 0.0);
    double lambda = 0.98;
    
    double weighted_sum = 0.0;

    for (size_t i = 0; i < costs.size(); ++i) {
        double weight = std::pow(lambda, static_cast<double>(i)); // Peso
        weighted_sum += costs[i] * weight; // Aggiorna l'accumulatore con il contributo pesato di questo elemento
    }

    RCLCPP_INFO(get_logger(), "L'accumulo pesato degli elementi è: %f", weighted_sum);

    double mean_cost = cumulative_cost/costs.size();

    for(const auto& cost: costs)
    {
      std_cost += std::pow(cost - mean_cost, 2);
    }
    std_cost = std::sqrt(std_cost/costs.size());
    RCLCPP_INFO(get_logger(), "-------------------------------");
    RCLCPP_INFO(get_logger(), "Path mean cost: %f", mean_cost);
    RCLCPP_INFO(get_logger(), "Path cumulative cost: %f", cumulative_cost);
    RCLCPP_INFO(get_logger(), "Path std cost: %f", std_cost);
    RCLCPP_INFO(get_logger(), "Path weighted cost: %f", weighted_sum);

    return cumulative_cost;

  }

  void compute_action_cost(const plansys2_msgs::msg::ActionExecution::SharedPtr msg)
  {
    
    RCLCPP_INFO(get_logger(), "Computing action cost");
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();

    send_goal_options.feedback_callback = [this](
      ComputePathGoalHandle::SharedPtr goal_handle,
      ComputePathFeedback feedback)
    {
      RCLCPP_INFO(get_logger(), "Feedback received from action cost computation (I am computing)");
    };

    send_goal_options.result_callback = [this, msg](ComputePathResult &result)
    {
      ActionCostPtr action_cost = std::make_shared<plansys2_msgs::msg::ActionCost>();

      if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
      {
          RCLCPP_INFO(get_logger(), "Compute path to cost. Goal failed with error code %d", static_cast<int>(result.code));
  
          action_cost->nominal_cost = std::numeric_limits<double>::infinity();
          action_cost->std_dev_cost = 0.0;
          this->set_action_cost(action_cost);
          this->send_response(msg);
          return;
      }

      auto path = result.result->path;
      double path_length = nav2_util::geometry_utils::calculate_path_length(path);

      auto test_cost = calculate_path_cost(path);
      RCLCPP_INFO(get_logger(), "Length of path: %f", path_length);

      action_cost->nominal_cost = path_length;
      action_cost->std_dev_cost = 0.0;
      this->set_action_cost(action_cost);
      this->send_response(msg);      

      
      path_pub_->publish(path);


    };

    send_goal_options.goal_response_callback = [this, msg](const ComputePathGoalHandle::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        ActionCostPtr action_cost = std::make_shared<plansys2_msgs::msg::ActionCost>();
        action_cost->nominal_cost = std::numeric_limits<double>::infinity();
        action_cost->std_dev_cost = 0.0;
        this->set_action_cost(action_cost);
        this->send_response(msg); 
      }
      else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }

    };

    nav2_msgs::action::ComputePathToPose::Goal goal = nav2_msgs::action::ComputePathToPose::Goal();
    
    auto wp_to_navigate = get_arguments()[2];  // The goal is in the 3rd argument of the action
    
    RCLCPP_INFO(get_logger(), "Computing cost to [%s]", wp_to_navigate.c_str()); //.c_str());
    if(waypoints_.find(wp_to_navigate) == waypoints_.end())
    {
      RCLCPP_ERROR(get_logger(), "Waypoint not found, has to be managed by specialized arguments");
      set_action_cost(std::numeric_limits<double>::infinity(), 0.0);
    }
    goal.start = current_pos_;
    goal.goal = waypoints_[wp_to_navigate];

    /* Send waypoints to tf broadcaster */
    geometry_msgs::msg::TransformStamped wp_t;
    wp_t.header = goal.goal.header;
    wp_t.child_frame_id = std::string(get_name()) + "_" + wp_to_navigate;
    wp_t.transform.translation.x = goal.goal.pose.position.x;
    wp_t.transform.translation.y = goal.goal.pose.position.y;
    wp_t.transform.translation.z = goal.goal.pose.position.z;
    wp_t.transform.rotation = goal.goal.pose.orientation;
    tf_broadcaster_->sendTransform(wp_t);

    goal.planner_id = "GridBased";
    RCLCPP_INFO(get_logger(), "Pre send goal");

    auto future_goal_handle = compute_path_action_client_->async_send_goal(goal, send_goal_options);
    RCLCPP_INFO(get_logger(), "Post send goal");

  } 

  std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber_;

  using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using NavigationFeedback =
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;


  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;

  rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr compute_path_action_client_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  geometry_msgs::msg::PoseStamped current_pos_;
  geometry_msgs::msg::PoseStamped goal_pos_;
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;

  double dist_to_move;

  bool test_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
