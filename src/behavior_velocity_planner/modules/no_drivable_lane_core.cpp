// 원본 NoDrivableLane 모듈의 접근 상태와 정지 경로를 이관합니다.

#include "planning/no_drivable_lane_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace planning {
namespace {

bool within(SpeedBumpPoint point, const std::vector<SpeedBumpPoint> &polygon) {
  bool inside = false;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto a = polygon[i];
    const auto b = polygon[(i + 1) % polygon.size()];
    if ((a.y > point.y) != (b.y > point.y) &&
        point.x < a.x + (point.y - a.y) * (b.x - a.x) / (b.y - a.y)) {
      inside = !inside;
    }
  }
  return inside;
}

double closestArc(const Trajectory &path, double x, double y) {
  double best = std::numeric_limits<double>::infinity();
  double selected = 0.0;
  double arc = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i + 1].x - path[i].x;
    const double dy = path[i + 1].y - path[i].y;
    const double length2 = dx * dx + dy * dy;
    if (length2 < 1.0e-12) continue;
    const double ratio = std::clamp(
        ((x - path[i].x) * dx + (y - path[i].y) * dy) / length2, 0.0, 1.0);
    const double distance = std::hypot(
        x - path[i].x - ratio * dx, y - path[i].y - ratio * dy);
    if (distance < best) {
      best = distance;
      selected = arc + ratio * std::sqrt(length2);
    }
    arc += std::sqrt(length2);
  }
  return selected;
}

}  // 익명 네임스페이스

NoDrivableLaneCore::NoDrivableLaneCore(std::vector<SpeedBumpPoint> polygon)
    : polygon_(std::move(polygon)) {}

NoDrivableLaneResult NoDrivableLaneCore::apply(
    const Trajectory &path, double ego_x, double ego_y,
    bool vehicle_stopped) {
  NoDrivableLaneResult result{path, false, state_};
  if (path.size() < 2 || polygon_.size() < 3) return result;
  SpeedBumpFeature feature;
  feature.polygon = polygon_;
  feature.speed_kph = 0.0;
  SpeedBumpParameters parameters;
  parameters.front_offset_m = 3.79;
  parameters.slow_start_margin_m = 1.5;
  parameters.rear_overhang_m = 100000.0;
  parameters.slow_end_margin_m = 0.0;
  const auto planned = planSpeedBump(path, feature, parameters);
  const double ego_arc = closestArc(path, ego_x, ego_y);
  double stop_arc = std::numeric_limits<double>::infinity();
  if (planned.slowed) {
    double arc = 0.0;
    for (std::size_t i = 0; i < planned.trajectory.size(); ++i) {
      if (i) arc += std::hypot(
          planned.trajectory[i].x - planned.trajectory[i - 1].x,
          planned.trajectory[i].y - planned.trajectory[i - 1].y);
      if (planned.trajectory[i].longitudinal_velocity_mps < 1.0e-3F) {
        stop_arc = arc;
        break;
      }
    }
  }
  if (state_ == NoDrivableLaneState::Init) {
    if (within({path.front().x, path.front().y}, polygon_) ||
        (planned.slowed && ego_arc >= stop_arc)) {
      state_ = NoDrivableLaneState::Inside;
    } else if (planned.slowed) {
      state_ = NoDrivableLaneState::Approaching;
    }
  } else if (state_ == NoDrivableLaneState::Approaching) {
    if (planned.slowed) {
      result.trajectory = planned.trajectory;
      result.stopped = true;
      if (vehicle_stopped && ego_arc >= stop_arc) {
        state_ = NoDrivableLaneState::Stopped;
      }
    }
  } else if (state_ == NoDrivableLaneState::Inside) {
    double arc = 0.0;
    for (std::size_t i = 0; i < result.trajectory.size(); ++i) {
      if (i) arc += std::hypot(
          path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
      if (arc >= ego_arc) result.trajectory[i].longitudinal_velocity_mps = 0.0F;
    }
    result.stopped = true;
    if (vehicle_stopped) state_ = NoDrivableLaneState::Stopped;
  } else {
    double arc = 0.0;
    for (std::size_t i = 0; i < result.trajectory.size(); ++i) {
      if (i) arc += std::hypot(
          path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
      if (arc >= ego_arc) result.trajectory[i].longitudinal_velocity_mps = 0.0F;
    }
    result.stopped = true;
  }
  result.state = state_;
  return result;
}

}  // planning 네임스페이스
