// SPDX-License-Identifier: Apache-2.0
#include "planning/freespace_planner_core.hpp"

#include "../behavior_path_planner/behavior_freespace_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace planning {
namespace {

double yawOf(const TrajectoryPoint &point) {
  return std::atan2(
      2.0 * (point.orientation_w * point.orientation_z +
             point.orientation_x * point.orientation_y),
      1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                   point.orientation_z * point.orientation_z));
}

double distanceToSegment(const TrajectoryPoint &point,
                         const TrajectoryPoint &a,
                         const TrajectoryPoint &b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length_squared = dx * dx + dy * dy;
  const double t = length_squared > 0.0 ? std::clamp(
      ((point.x - a.x) * dx + (point.y - a.y) * dy) / length_squared,
      0.0, 1.0) : 0.0;
  return std::hypot(point.x - a.x - t * dx,
                    point.y - a.y - t * dy);
}

Trajectory stoppedTrajectory(Trajectory path) {
  for (auto &point : path) point.longitudinal_velocity_mps = 0.0F;
  return path;
}

}  // namespace

FreespacePlannerCore::FreespacePlannerCore(
    FreespacePlannerCoreParameters parameters, Search search)
    : parameters_(std::move(parameters)), search_(std::move(search)) {
  if (!search_) search_ = generateFreespacePullOverCandidate;
}

void FreespacePlannerCore::onRoute(BehaviorGoalPose goal) {
  goal_ = goal;
  trajectory_.clear();
  partial_trajectory_.clear();
  reversing_indices_.clear();
  previous_target_index_ = target_index_ = 0;
  obstacle_found_at_s_.reset();
  reset_in_progress_ = false;
  completed_ = false;
  new_parking_cycle_ = true;
  pending_reset_status_ = true;
}

void FreespacePlannerCore::reset(FreespacePlannerCoreOutput &output) {
  trajectory_.clear();
  partial_trajectory_.clear();
  reversing_indices_.clear();
  previous_target_index_ = target_index_ = 0;
  obstacle_found_at_s_.reset();
  completed_ = false;
  output.completed_status = false;
}

bool FreespacePlannerCore::stopped() const {
  return !odometry_.empty() && std::all_of(
      odometry_.begin(), odometry_.end(), [&](const auto &sample) {
        return std::abs(sample.second) <= parameters_.stopped_velocity_mps;
      });
}

bool FreespacePlannerCore::obstacleOnPartial(
    const CrosswalkOccupancyGrid &grid,
    const TrajectoryPoint &ego) const {
  if (partial_trajectory_.empty() || grid.width == 0 || grid.height == 0 ||
      grid.values.size() != grid.width * grid.height) return false;
  std::size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < partial_trajectory_.size(); ++i) {
    const auto &point = partial_trajectory_[i];
    const double distance = std::hypot(point.x - ego.x, point.y - ego.y);
    if (distance < best) {
      best = distance;
      nearest = i;
    }
  }
  for (std::size_t i = nearest; i < partial_trajectory_.size(); ++i) {
    const auto &point = partial_trajectory_[i];
    if (!detail::footprintFree(grid, parameters_.search,
                               point.x, point.y, yawOf(point))) return true;
  }
  return false;
}

bool FreespacePlannerCore::courseOut(const TrajectoryPoint &ego) const {
  if (trajectory_.empty()) return false;
  double distance = std::hypot(ego.x - trajectory_.front().x,
                               ego.y - trajectory_.front().y);
  for (std::size_t i = 1; i < trajectory_.size(); ++i)
    distance = std::min(distance, distanceToSegment(
        ego, trajectory_[i - 1], trajectory_[i]));
  return distance > parameters_.course_out_distance_m;
}

bool FreespacePlannerCore::nearTarget(const TrajectoryPoint &ego) const {
  if (trajectory_.empty()) return false;
  const auto &target = trajectory_[target_index_];
  const double yaw = yawOf(target);
  const double dx = ego.x - target.x;
  const double dy = ego.y - target.y;
  const double longitudinal = std::cos(yaw) * dx + std::sin(yaw) * dy;
  const double lateral = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  const double yaw_error = std::remainder(yawOf(ego) - yaw,
                                          2.0 * std::acos(-1.0));
  return std::abs(yaw_error) < std::acos(-1.0) / 2.0 &&
         std::abs(longitudinal) < parameters_.arrived_distance_m &&
         std::abs(lateral) < parameters_.arrived_distance_m;
}

Trajectory FreespacePlannerCore::partial() const {
  if (trajectory_.empty() || previous_target_index_ > target_index_)
    return {};
  Trajectory path(trajectory_.begin() + previous_target_index_,
                  trajectory_.begin() + target_index_ + 1);
  if (path.size() >= 2)
    path.front().longitudinal_velocity_mps =
        path[1].longitudinal_velocity_mps;
  path.back().longitudinal_velocity_mps = 0.0F;
  return path;
}

FreespacePlannerCoreOutput FreespacePlannerCore::tick(
    const FreespacePlannerCoreInput &input) {
  FreespacePlannerCoreOutput output;
  if (pending_reset_status_) {
    output.completed_status = false;
    pending_reset_status_ = false;
  }
  if (!input.parking_active) {
    reset(output);
    reset_in_progress_ = false;
    output.target_index = target_index_;
    return output;
  }
  if (input.ego && std::isfinite(input.now_s) &&
      std::isfinite(input.ego->longitudinal_velocity_mps)) {
    odometry_.emplace_back(input.now_s,
                           input.ego->longitudinal_velocity_mps);
    while (odometry_.size() > 1 &&
           input.now_s - odometry_.front().first >=
               parameters_.stopped_time_s)
      odometry_.pop_front();
  }
  if (!goal_ || !input.occupancy_grid || !input.ego)
    return output;

  if (completed_) {
    output.trajectory = stoppedTrajectory(partial_trajectory_);
    output.completed = true;
    output.target_index = target_index_;
    return output;
  }

  bool plan_required = trajectory_.empty();
  if (!plan_required && parameters_.replan_when_obstacle_found) {
    if (obstacleOnPartial(*input.occupancy_grid, *input.ego)) {
      if (!obstacle_found_at_s_) obstacle_found_at_s_ = input.now_s;
      plan_required = input.now_s - *obstacle_found_at_s_ >
          parameters_.obstacle_time_s;
    } else {
      obstacle_found_at_s_.reset();
    }
  }
  if (!plan_required && parameters_.replan_when_course_out)
    plan_required = courseOut(*input.ego);

  if (!reset_in_progress_ && plan_required) {
    if (!new_parking_cycle_) {
      output.trajectory = stoppedTrajectory(
          partial_trajectory_.empty() ? Trajectory{*input.ego} :
                                        partial_trajectory_);
    }
    reset(output);
    reset_in_progress_ = true;
  }

  if (reset_in_progress_) {
    if (stopped()) {
      auto candidate = search_(*input.ego, *goal_,
                               *input.occupancy_grid, parameters_.search);
      reset_in_progress_ = false;
      if (candidate && candidate->path.size() > 1) {
        trajectory_ = std::move(candidate->path);
        for (std::size_t i = 0; i + 1 < trajectory_.size(); ++i) {
          if (trajectory_[i].longitudinal_velocity_mps *
              trajectory_[i + 1].longitudinal_velocity_mps < 0.0F)
            reversing_indices_.push_back(i);
        }
        for (const auto index : candidate->phase_end_indices) {
          if (index < trajectory_.size() - 1 &&
              std::find(reversing_indices_.begin(),
                        reversing_indices_.end(), index) ==
                  reversing_indices_.end())
            reversing_indices_.push_back(index);
        }
        std::sort(reversing_indices_.begin(), reversing_indices_.end());
        output.full_trajectory = trajectory_;
        previous_target_index_ = 0;
        target_index_ = reversing_indices_.empty() ?
            trajectory_.size() - 1 : reversing_indices_.front();
        output.planned = true;
      }
    } else {
      output.waiting_for_stop = true;
    }
  }

  if (trajectory_.size() <= 1) {
    new_parking_cycle_ = false;
    return output;
  }
  if (stopped() && nearTarget(*input.ego)) {
    const auto next = std::find_if(
        reversing_indices_.begin(), reversing_indices_.end(),
        [&](std::size_t index) { return index > target_index_; });
    const std::size_t next_index = next == reversing_indices_.end() ?
        trajectory_.size() - 1 : *next;
    if (next_index == target_index_) {
      completed_ = true;
      output.completed_status = true;
    } else {
      previous_target_index_ = target_index_;
      target_index_ = next_index;
    }
  }
  partial_trajectory_ = partial();
  output.trajectory = partial_trajectory_;
  output.completed = completed_;
  output.target_index = target_index_;
  new_parking_cycle_ = false;
  return output;
}

}  // namespace planning
