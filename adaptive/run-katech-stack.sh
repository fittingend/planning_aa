#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
network="${AP_NET:-planning-aa-net}"
domain="${ROS_DOMAIN_ID:-42}"
map_dir="${KATECH_MAP_PATH:-${HOME}/autoware_map/sample-map-planning}"
image_repo="${KATECH_IMAGE_REPO:-ghcr.io/oss-test-group/autoware-partition}"
perception_image="${KATECH_PERCEPTION_IMAGE:-${image_repo}:adsw-perception}"
control_image="${KATECH_CONTROL_IMAGE:-${image_repo}:adsw-control}"
perception_name="planning-aa-perception"
control_name="planning-aa-control"
rviz_mode="${KATECH_RVIZ:-auto}"
log_dir="${KATECH_LOG_DIR:-${repo_dir}/output/katech-logs/$(date +%Y%m%d_%H%M%S)_$$}"

case "${rviz_mode}" in
  auto)
    if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
      rviz_mode=true
    else
      rviz_mode=false
    fi
    ;;
  1|true|yes) rviz_mode=true ;;
  0|false|no) rviz_mode=false ;;
  *) echo "KATECH_RVIZ must be auto, true, or false" >&2; exit 1 ;;
esac
if [[ "${rviz_mode}" == true && ( -z "${DISPLAY:-}" || ! -d /tmp/.X11-unix ) ]]; then
  echo "RViz requested, but DISPLAY or the X11 socket is unavailable." >&2
  exit 1
fi

test -f "${map_dir}/lanelet2_map.osm" || {
  echo "Lanelet map missing: ${map_dir}/lanelet2_map.osm" >&2
  exit 1
}
if ! docker info >/dev/null 2>&1; then
  echo "Cannot access the Docker daemon. Check Docker service and user permissions." >&2
  exit 1
fi
missing_images=()
for image in "${perception_image}" "${control_image}"; do
  if ! docker image inspect "${image}" >/dev/null 2>&1; then
    missing_images+=("${image}")
  fi
done
if ((${#missing_images[@]})); then
  echo "Required KATECH images are unavailable locally:" >&2
  for image in "${missing_images[@]}"; do
    echo "  ${image}" >&2
    echo "  docker pull '${image}'" >&2
  done
  exit 1
fi
for name in "${perception_name}" "${control_name}" planning-aa-bridge planning-aa; do
  if docker container inspect "${name}" >/dev/null 2>&1; then
    echo "Container already exists: ${name}" >&2
    exit 1
  fi
done
if ! docker network inspect "${network}" >/dev/null 2>&1; then
  docker network create --subnet 192.168.100.0/24 "${network}" >/dev/null
fi

mkdir -p "${log_dir}"
log_dir="$(cd "${log_dir}" && pwd -P)"
collector_pids=()
stack_pid=""

cleanup() {
  trap - EXIT INT TERM
  docker stop planning-aa planning-aa-bridge "${control_name}" "${perception_name}" >/dev/null 2>&1 || true
  if [[ -n "${stack_pid}" ]]; then
    wait "${stack_pid}" >/dev/null 2>&1 || true
  fi
  for pid in "${collector_pids[@]}"; do
    kill "${pid}" >/dev/null 2>&1 || true
  done
  echo "Logs saved in: ${log_dir}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

collect_logs() {
  local name="$1" file="$2"
  docker logs --timestamps --follow "${name}" >"${file}" 2>&1 &
  collector_pids+=("$!")
}

open_log_terminal() {
  local title="$1" file="$2"
  if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    return 1
  fi
  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="${title}" -- bash -c 'tail -n +1 -F -- "$1"' _ "${file}" && return 0
  fi
  if command -v konsole >/dev/null 2>&1; then
    konsole --new-window -p "tabtitle=${title}" -e tail -n +1 -F -- "${file}" >/dev/null 2>&1 &
    return 0
  fi
  if command -v xterm >/dev/null 2>&1; then
    xterm -T "${title}" -e tail -n +1 -F -- "${file}" >/dev/null 2>&1 &
    return 0
  fi
  return 1
}

common_args=(
  --rm --init --network "${network}" --ipc=host --pid=host
  -e "ROS_DOMAIN_ID=${domain}"
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  -e ROS_LOCALHOST_ONLY=0
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  -e "LOCAL_UID=$(id -u)" -e "LOCAL_GID=$(id -g)"
  -e "LOCAL_USER=$(id -un)" -e "LOCAL_GROUP=$(id -gn)"
  -v "${map_dir}:/autoware_map:ro"
  -v /etc/localtime:/etc/localtime:ro
)
if [[ "${rviz_mode}" == true ]]; then
  common_args+=(
    -e "DISPLAY=${DISPLAY}"
    -e QT_X11_NO_MITSHM=1
    -v /tmp/.X11-unix:/tmp/.X11-unix
  )
  xauthority_file="${XAUTHORITY:-${HOME}/.Xauthority}"
  if [[ -f "${xauthority_file}" ]]; then
    common_args+=(-e "XAUTHORITY=${xauthority_file}" -v "${xauthority_file}:${xauthority_file}:ro")
  fi
  if [[ -d /dev/dri ]]; then
    common_args+=(--device /dev/dri:/dev/dri)
  fi
  if command -v xhost >/dev/null 2>&1; then
    xhost "+si:localuser:$(id -un)" >/dev/null || true
  fi
fi

docker run -d --name "${perception_name}" "${common_args[@]}" \
  "${perception_image}" bash -lc \
  "ros2 launch obigo_launch sample_adsw_perception_run.launch.xml map_path:=/autoware_map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit rviz:=${rviz_mode}" >/dev/null
sleep "${KATECH_START_DELAY:-10}"
if [[ "$(docker inspect -f '{{.State.Running}}' "${perception_name}")" != true ]]; then
  echo "Perception container stopped during startup; inspect its Docker logs." >&2
  exit 1
fi
docker run -d --name "${control_name}" "${common_args[@]}" \
  "${control_image}" bash -lc \
  'ros2 launch obigo_launch sample_adsw_control_run.launch.xml map_path:=/autoware_map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit' >/dev/null
if [[ "$(docker inspect -f '{{.State.Running}}' "${control_name}")" != true ]]; then
  echo "Control container stopped during startup; inspect its Docker logs." >&2
  exit 1
fi

for name in "${perception_name}" "${control_name}" planning-aa-bridge planning-aa; do
  : >"${log_dir}/${name}.log"
done
collect_logs "${perception_name}" "${log_dir}/${perception_name}.log"
collect_logs "${control_name}" "${log_dir}/${control_name}.log"

ROS_DOMAIN_ID="${domain}" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  AP_NET="${network}" bash "${repo_dir}/adaptive/run-stack.sh" \
  >"${log_dir}/planning-aa.log" 2>&1 &
stack_pid="$!"

for attempt in {1..50}; do
  if docker container inspect planning-aa-bridge >/dev/null 2>&1; then
    break
  fi
  sleep 0.2
done
if ! docker container inspect planning-aa-bridge >/dev/null 2>&1; then
  echo "Bridge did not start; inspect ${log_dir}/planning-aa.log" >&2
  exit 1
fi
collect_logs planning-aa-bridge "${log_dir}/planning-aa-bridge.log"

opened=0
for name in "${perception_name}" "${control_name}" planning-aa-bridge planning-aa; do
  if open_log_terminal "${name}" "${log_dir}/${name}.log"; then
    ((opened += 1))
  fi
done
if ((opened < 4)); then
  echo "Opened ${opened}/4 log windows; all logs are saved in ${log_dir}." >&2
fi

echo "Perception and control share ${network} and ROS_DOMAIN_ID=${domain} with the bridge."
echo "RViz visualization: ${rviz_mode} (set KATECH_RVIZ=true or false to override)."
echo "Logs: ${log_dir}"
wait "${stack_pid}"
