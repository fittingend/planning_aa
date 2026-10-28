#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
arch="${ARCH:-$(uname -m)}"
case "${arch}" in
  x86_64) platform=linux/amd64; suffix="" ;;
  aarch64) platform=linux/arm64; suffix=-aarch64 ;;
  *) echo "Unsupported architecture: ${arch}" >&2; exit 1 ;;
esac
sdk_dir="${PARA_SDK:-${repo_dir}/adaptive/vendor/para-sdk/${arch}}"
bridge_dir="${OSS_AP_SOURCE:-${repo_dir}/adaptive/vendor/oss-ap}"
test -d "${sdk_dir}" || { echo "PARA SDK directory missing: ${sdk_dir}" >&2; exit 1; }
test -d "${bridge_dir}" || { echo "Bridge source directory missing: ${bridge_dir}" >&2; exit 1; }
sdk_dir="$(cd "${sdk_dir}" && pwd -P)"
bridge_dir="$(cd "${bridge_dir}" && pwd -P)"
test -f "${sdk_dir}/VERSION" || { echo "PARA SDK missing: ${sdk_dir}" >&2; exit 1; }
test -f "${sdk_dir}/share/cmake/Modules/PARA/Para.cmake" || {
  echo "PARA CMake module missing: ${sdk_dir}" >&2; exit 1;
}
test -f "${bridge_dir}/src/Ecu_Machine/package.xml" || {
  echo "Local Service Creator source missing: ${bridge_dir}" >&2; exit 1;
}
test -f "${bridge_dir}/src/msgs/autoware_msgs/autoware_planning_msgs/package.xml" || {
  echo "Local ROS message definitions missing: ${bridge_dir}/src/msgs" >&2; exit 1;
}
test -f "${bridge_dir}/docker/entrypoint/ap_entrypoint.sh" || {
  echo "Local Service Creator entrypoint missing: ${bridge_dir}" >&2; exit 1;
}
image_tag="${BRIDGE_IMAGE:-oss-ap-planning-bridge:local${suffix}}"
sdk_version="$(cat "${sdk_dir}/VERSION")"
case "${sdk_version}" in
  R20-11_*_native-linux-gcc) ;;
  *) echo "Unsupported PARA SDK version: ${sdk_version}" >&2; exit 1 ;;
esac
build_mode="${BRIDGE_BUILD_MODE:-auto}"
case "${build_mode}" in
  auto)
    if [[ "$(uname -m)" == x86_64 && "${arch}" == aarch64 ]]; then
      build_mode=cross
    else
      build_mode=native
    fi
    ;;
  cross|native) ;;
  *) echo "BRIDGE_BUILD_MODE must be auto, cross, or native" >&2; exit 1 ;;
esac
dockerfile="${repo_dir}/adaptive/Dockerfile.bridge-local"
extra_context=()
if [[ "${build_mode}" == cross ]]; then
  [[ "$(uname -m)" == x86_64 && "${arch}" == aarch64 ]] || {
    echo "Bridge cross build requires an x86_64 host and ARCH=aarch64" >&2
    exit 1
  }
  host_sdk_dir="${HOST_PARA_SDK:-${repo_dir}/adaptive/vendor/para-sdk/x86_64}"
  test -d "${host_sdk_dir}" || { echo "Host PARA SDK directory missing: ${host_sdk_dir}" >&2; exit 1; }
  host_sdk_dir="$(cd "${host_sdk_dir}" && pwd -P)"
  test -x "${host_sdk_dir}/share/cyclonedds/bin/idlc" || {
    echo "Host PARA idlc missing: ${host_sdk_dir}" >&2; exit 1;
  }
  test "$(cat "${host_sdk_dir}/VERSION")" = "${sdk_version}" || {
    echo "Host and target PARA SDK versions must match" >&2; exit 1;
  }
  dockerfile="${repo_dir}/adaptive/Dockerfile.bridge-cross"
  extra_context=(--build-context "host_sdk=${host_sdk_dir}")
fi
echo "Building ${image_tag} (${build_mode}, ${sdk_version})"
docker buildx build --load --platform "${platform}" \
  --build-context "para_sdk=${sdk_dir}" \
  --build-context "bridge_vendor=${bridge_dir}" \
  "${extra_context[@]}" \
  --build-arg "PARA_SDK_VERSION=${sdk_version}" \
  -f "${dockerfile}" \
  -t "${image_tag}" "${repo_dir}"
echo "Built ${image_tag}"
