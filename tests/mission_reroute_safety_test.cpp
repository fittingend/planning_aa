#include "planning/mission_reroute_safety.hpp"

#include <cassert>

using namespace planning;

int main() {
  const MissionRerouteGeometry geometry{
      [](std::int64_t id) {
        const double start = static_cast<double>(id - 1) * 20.0;
        return std::vector<CrosswalkPoint>{{start, 0.0},
                                           {start + 20.0, 0.0}};
      },
      [](std::int64_t id, RouteCheckpoint pose) {
        const double start = static_cast<double>(id - 1) * 20.0;
        return pose.x >= start && pose.x <= start + 20.0 &&
               pose.y >= -2.0 && pose.y <= 2.0;
      }};
  RouteSelectorRoute original;
  original.route.sections = {{1, {1}}, {2, {2}}, {3, {3}}};
  RouteSelectorRoute target;
  target.route.sections = {{1, {1}}, {2, {2}}, {4, {4}}};
  target.start = {1.0, 0.0};
  target.goal = {39.0, 0.0};
  MissionPlannerEgo ego;
  ego.pose = target.start;
  ego.speed_mps = 2.0;
  assert(checkMissionRerouteSafety(original, target, ego, geometry));
  target.goal.x = 25.0;
  assert(!checkMissionRerouteSafety(original, target, ego, geometry));
  target.goal.x = 39.0;
  ego.speed_mps = 4.0;
  assert(!checkMissionRerouteSafety(original, target, ego, geometry));
  ego.speed_mps = 0.0;
  assert(checkMissionRerouteSafety(original, target, ego, geometry));
  ego.speed_mps = 2.0;
  target.start.x = 25.0;
  assert(!checkMissionRerouteSafety(original, target, ego, geometry));
  target.start.x = 1.0;
  target.route.sections = {{4, {4}}};
  assert(!checkMissionRerouteSafety(original, target, ego, geometry));
}
