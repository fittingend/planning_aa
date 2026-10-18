// SPDX-License-Identifier: Apache-2.0
#include "diagnostic_output.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

namespace adaptive {
namespace {
constexpr std::uint16_t kPort = 12507;
constexpr std::size_t kCheckCount = 14;
}  // namespace

DiagnosticOutput::DiagnosticOutput(const std::string &target) {
  if (target.empty()) return;
  in_addr parsed{};
  if (inet_pton(AF_INET, target.c_str(), &parsed) != 1) return;
  const int descriptor = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (descriptor < 0) return;
  address_ = parsed.s_addr;
  socket_ = descriptor;
}

DiagnosticOutput::~DiagnosticOutput() {
  if (socket_ >= 0) ::close(socket_);
}

bool DiagnosticOutput::send(
    const planning::ValidationOutcome &outcome, double stamp_s) const {
  if (socket_ < 0 || !outcome.data_ready ||
      outcome.diagnostics.size() != kCheckCount ||
      !std::isfinite(stamp_s) || stamp_s < 0.0 ||
      stamp_s > static_cast<double>(std::numeric_limits<std::int64_t>::max()))
    return false;
  std::array<char, kCheckCount + 1> levels{};
  for (std::size_t i = 0; i < kCheckCount; ++i) {
    levels[i] = static_cast<char>('0' +
        static_cast<int>(outcome.diagnostics[i].level));
  }
  std::array<char, 96> packet{};
  const auto length = std::snprintf(packet.data(), packet.size(),
      "PLAA1 %lld %s", static_cast<long long>(stamp_s), levels.data());
  if (length <= 0 || static_cast<std::size_t>(length) >= packet.size())
    return false;
  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port = htons(kPort);
  destination.sin_addr.s_addr = address_;
  return ::sendto(socket_, packet.data(), static_cast<std::size_t>(length),
                  MSG_DONTWAIT, reinterpret_cast<sockaddr *>(&destination),
                  sizeof(destination)) == length;
}

}  // namespace adaptive
