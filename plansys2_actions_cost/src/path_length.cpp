#include "plansys2_actions_cost/path_length.hpp"

namespace plansys2_actions_cost
{
ActionCostPtr compute_path_length(const nav_msgs::msg::Path::SharedPtr path_ptr)
{
  ActionCostPtr action_cost = std::make_shared<plansys2_msgs::msg::ActionCost>();

  action_cost->nominal_cost = nav2_util::geometry_utils::calculate_path_length(*path_ptr);
  action_cost->std_dev_cost = 0.0;

  return action_cost;
}
}   // namespace plansys2_actions_cost


// namespace plansys2_actions_cost
// {

//     plansys2_msgs::msg::ActionCost PathLength::cost_function()
//     {
//         plansys2_msgs::msg::ActionCost action_cost;

//         if (path_->poses.size() < 2)
//         {
//             return action_cost;
//         }

//         action_cost.nominal_cost = nav2_util::geometry_utils::calculate_path_length(*path_);
//         action_cost.std_dev_cost = 0.0;

//         return action_cost;
//     }

// } // namespace plansys2_actions_cost
