/**
 * VoronoiPlanner node action callbacks.
 *
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * January 13, 2024
 */

/**
 * This is free software.
 * You can redistribute it and/or modify this file under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 3 of the License, or (at your option) any later
 * version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this file; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <voronoi_planner/voronoi_planner.hpp>

namespace VoronoiPlanner
{

/**
 * @brief Handles a new FindPath goal request.
 *
 * @param uuid ID of the new request.
 * @param goal Pointer to goal object to parse.
 * @return Handling result code.
 */
rclcpp_action::GoalResponse VoronoiPlannerNode::find_path_handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  FindPathGoalSharedPtr goal)
{
  UNUSED(uuid);
  UNUSED(goal);

  RCLCPP_INFO(this->get_logger(), "Received path request");

  if (false)        // var_.load(std::memory_order_acquire)
  {
    RCLCPP_ERROR(this->get_logger(), "Rejecting path request: planner is busy");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief Handles a new FindPath cancellation request.
 *
 * @param goal_handle Handle to the goal object.
 * @return Cancellation operation status.
 */
rclcpp_action::CancelResponse VoronoiPlannerNode::find_path_handle_cancel(
  const FindPathGoalHandleSharedPtr goal_handle)
{
  UNUSED(goal_handle);

  RCLCPP_INFO(this->get_logger(), "Received path cancellation request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief Starts execution of FindPath.
 *
 * @param goal_handle Handle to the goal object.
 */
void VoronoiPlannerNode::find_path_handle_accepted(const FindPathGoalHandleSharedPtr goal_handle)
{
  std::thread{
    std::bind(
      &VoronoiPlannerNode::compute_path,
      this,
      std::placeholders::_1),
    goal_handle}.detach();
}

/**
 * @brief Find Path successfully completed.
 *
 * @param find_path_goal_handle Handle to the goal object.
 */
void VoronoiPlannerNode::find_path_succeeded(FindPathGoalHandleSharedPtr find_path_goal_handle)
{
  RCLCPP_WARN(get_logger(), "Find Path SUCCEEDED");
  auto result = std::make_shared<FindPath::Result>();
  result->result.header.set__stamp(get_clock()->now());
  result->result.set__result(CommandResultStamped::SUCCESS);
  result->result.set__error_msg("");
  find_path_goal_handle->succeed(result);
}

/**
 * @brief Find Path failed.
 *
 * @param find_path_goal_handle Handle to the goal object.
 */
void VoronoiPlannerNode::find_path_failed(
  FindPathGoalHandleSharedPtr find_path_goal_handle,
  std::string error_msg)
{
  RCLCPP_ERROR(get_logger(), "Find Path FAILED: %s", error_msg.c_str());
  auto result = std::make_shared<FindPath::Result>();
  result->result.header.set__stamp(get_clock()->now());
  result->result.set__result(CommandResultStamped::FAILED);
  result->result.set__error_msg(error_msg);
  find_path_goal_handle->abort(result);
}

/**
 * @brief Find Path terminated with error.
 *
 * @param find_path_goal_handle Handle to the goal object.
 */
void VoronoiPlannerNode::find_path_error(
  FindPathGoalHandleSharedPtr find_path_goal_handle,
  std::string error_msg)
{
  RCLCPP_ERROR(get_logger(), "Find Path ERROR: %s", error_msg.c_str());
  auto result = std::make_shared<FindPath::Result>();
  result->result.header.set__stamp(get_clock()->now());
  result->result.set__result(CommandResultStamped::ERROR);
  result->result.set__error_msg(error_msg);
  find_path_goal_handle->abort(result);
}

/**
 * @brief Find Path canceled.
 *
 * @param find_path_goal_handle Handle to the goal object.
 */
void VoronoiPlannerNode::find_path_canceled(FindPathGoalHandleSharedPtr find_path_goal_handle)
{
  RCLCPP_ERROR(get_logger(), "Find Path CANCELED");
  auto result = std::make_shared<FindPath::Result>();
  result->result.header.set__stamp(get_clock()->now());
  result->result.set__result(CommandResultStamped::FAILED);
  result->result.set__error_msg("Find Path CANCELED");
  find_path_goal_handle->canceled(result);
}

} // namespace FlightControl
