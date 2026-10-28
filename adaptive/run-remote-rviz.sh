#!/usr/bin/env bash
set -euo pipefail

# The board runs PlanningAA and Autoware. This PC runs only the RViz client.
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
domain="${ROS_DOMAIN_ID:-42}"
image="${KATECH_RVIZ_IMAGE:-katech-autoware-system-control:adsw-perception}"
interface="${KATECH_NETWORK_INTERFACE:-}"
name="planning-aa-remote-rviz-${domain}"

[[ "$(uname -m)" == x86_64 ]] || { echo "Run remote RViz on an x86_64 PC." >&2; exit 1; }
[[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]] || {
  echo "Run from a desktop X11 session (DISPLAY and /tmp/.X11-unix required)." >&2
  exit 1
}
if [[ -n "${interface}" ]] && ! ip link show "${interface}" >/dev/null 2>&1; then
  echo "Network interface does not exist: ${interface}" >&2
  exit 1
fi
if ! docker info >/dev/null 2>&1; then
  echo "Cannot access the Docker daemon." >&2
  exit 1
fi
if ! docker image inspect "${image}" >/dev/null 2>&1; then
  rviz_archive="${repo_dir}/deployment/images/pc-rviz-amd64.tar.zst"
  if [[ -f "${rviz_archive}" ]]; then
    command -v zstd >/dev/null || { echo "zstd is required to load ${rviz_archive}" >&2; exit 1; }
    echo "Loading RViz image from ${rviz_archive}"
    zstd -dc "${rviz_archive}" | docker load
  fi
fi
image_arch="$(docker image inspect --format '{{.Architecture}}' "${image}")" || {
  echo "RViz image missing: ${image}" >&2
  exit 1
}
[[ "${image_arch}" == amd64 ]] || { echo "RViz image must be amd64: ${image}" >&2; exit 1; }
if docker container inspect "${name}" >/dev/null 2>&1; then
  echo "Container already exists: ${name}" >&2
  exit 1
fi
dds_args=()
if [[ -n "${interface}" ]]; then
  dds_uri="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${interface}\" multicast=\"true\"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>"
  dds_args+=(-e "CYCLONEDDS_URI=${dds_uri}")
fi
x_args=(-e "DISPLAY=${DISPLAY}" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix)
xauthority_file="${XAUTHORITY:-${HOME}/.Xauthority}"
if [[ -f "${xauthority_file}" ]]; then
  x_args+=(-e "XAUTHORITY=${xauthority_file}" -v "${xauthority_file}:${xauthority_file}:ro")
fi
if [[ -n "${XDG_RUNTIME_DIR:-}" && -d "${XDG_RUNTIME_DIR}" ]]; then
  x_args+=(-e "XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR}" -v "${XDG_RUNTIME_DIR}:${XDG_RUNTIME_DIR}")
fi
if [[ -d /dev/dri ]]; then
  x_args+=(--device /dev/dri:/dev/dri)
fi
if command -v xhost >/dev/null 2>&1; then
  xhost "+si:localuser:$(id -un)" >/dev/null || true
fi
echo "Remote RViz: image=${image} domain=${domain} interface=${interface:-auto}"
docker run --rm --init --platform linux/amd64 --name "${name}" \
  --network host --ipc host \
  -e "ROS_DOMAIN_ID=${domain}" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e ROS_LOCALHOST_ONLY=0 -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e "LOCAL_UID=$(id -u)" -e "LOCAL_GID=$(id -g)" \
  -e "LOCAL_USER=$(id -un)" -e "LOCAL_GROUP=$(id -gn)" \
  "${dds_args[@]}" "${x_args[@]}" \
  "${image}" rviz2 -d /opt/autoware/share/obigo_launch/rviz/autoware.rviz
