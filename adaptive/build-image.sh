#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
sdk_dir="${PARA_SDK:-${repo_dir}/adaptive/vendor/para-sdk/x86_64}"
sdk_dir="$(cd "${sdk_dir}" && pwd -P)"
oss_ap_default="${repo_dir}/adaptive/vendor/oss-ap"
if test -f "${repo_dir}/../oss-ap-planning-bridge/src/Ecu_Machine/Exe_missionplanner/include/oss/srv/MP2RS/rs_si_missionplanner_skeleton.h"; then
  oss_ap_default="${repo_dir}/../oss-ap-planning-bridge"
fi
oss_ap_dir="${OSS_AP_SOURCE:-${oss_ap_default}}"
oss_ap_dir="$(cd "${oss_ap_dir}" && pwd -P)"
test -f "${oss_ap_dir}/src/Ecu_Machine/Exe_missionplanner/include/oss/srv/MP2RS/rs_si_missionplanner_skeleton.h" || {
  echo "Service 1814 artifacts missing in ${oss_ap_dir}; set OSS_AP_SOURCE to oss-ap-planning-bridge" >&2
  exit 1
}
test -f "${sdk_dir}/VERSION" || { echo "PARA SDK VERSION file missing: ${sdk_dir}" >&2; exit 1; }
test -f "${sdk_dir}/share/cmake/Modules/PARA/Para.cmake" || {
  echo "PARA CMake module missing: ${sdk_dir}" >&2; exit 1;
}
test "$(uname -m)" = x86_64 || { echo "This bootstrap targets x86_64" >&2; exit 1; }

sdk_version="$(cat "${sdk_dir}/VERSION")"
case "${sdk_version}" in
  R20-11_*_native-linux-gcc) ;;
  *) echo "Unsupported PARA SDK version: ${sdk_version}" >&2; exit 1 ;;
esac
image_tag="${IMAGE_TAG:-planning-aa-adaptive:local}"
boost_library="${BOOST_SERIALIZATION_LIBRARY:-/usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.74.0}"
test -f "${boost_library}" || {
  echo "Boost serialization runtime library missing: ${boost_library}" >&2
  exit 1
}
boost_context="$(mktemp -d)"
trap 'rm -rf "${boost_context}"' EXIT
cp -L "${boost_library}" "${boost_context}/libboost_serialization.so.1.74.0"
docker buildx build --load --platform linux/amd64 \
  --build-context "para_sdk=${sdk_dir}" \
  --build-context "oss_ap=${oss_ap_dir}/src/Ecu_Machine" \
  --build-context "boost_serialization=${boost_context}" \
  --build-arg "PARA_SDK_VERSION=${sdk_version}" \
  -f "${repo_dir}/adaptive/Dockerfile" \
  -t "${image_tag}" "${repo_dir}"
echo "Built ${image_tag} with ${sdk_version}"
