// SPDX-License-Identifier: Apache-2.0
#include "route_visualization_output.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace adaptive {

RouteVisualizationOutput::RouteVisualizationOutput(const std::string &target) {
  in_addr parsed{};
  if (inet_pton(AF_INET, target.c_str(), &parsed) != 1) return;
  socket_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  address_ = parsed.s_addr;
}

RouteVisualizationOutput::~RouteVisualizationOutput() {
  if (socket_ >= 0) ::close(socket_);
}

bool RouteVisualizationOutput::send(const planning::Trajectory &path) const {
  if (socket_ < 0 || path.size() < 2) return false;
  constexpr std::size_t max_points = 500;
  const std::size_t stride = std::max<std::size_t>(
      1, (path.size() - 1 + max_points - 2) / (max_points - 1));
  std::ostringstream packet;
  packet << "PLAA_ROUTE1 " << std::fixed << std::setprecision(3);
  const auto append = [&](const planning::TrajectoryPoint &point) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) return false;
    packet << point.x << ',' << point.y << ',' << point.z << ';';
    return true;
  };
  for (std::size_t index = 0; index + 1 < path.size(); index += stride)
    if (!append(path[index])) return false;
  if (!append(path.back())) return false;
  const auto data = packet.str();
  if (data.size() > 60000) return false;
  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port = htons(12508);
  destination.sin_addr.s_addr = address_;
  return ::sendto(socket_, data.data(), data.size(), MSG_DONTWAIT,
                  reinterpret_cast<sockaddr *>(&destination),
                  sizeof(destination)) == static_cast<ssize_t>(data.size());
}

}  // namespace adaptive
