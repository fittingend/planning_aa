#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
network="${AP_NET:-planning-aa-net}"
domain="${ROS_DOMAIN_ID:-42}"
map_dir="${KATECH_MAP_PATH:-${HOME}/autoware_map/sample-map-planning}"
image_repo="${KATECH_IMAGE_REPO:-katech-autoware-system-control}"
perception_image="${KATECH_PERCEPTION_IMAGE:-${image_repo}:adsw-perception}"
control_image="${KATECH_CONTROL_IMAGE:-${image_repo}:adsw-control}"
perception_name="planning-aa-perception"
control_name="planning-aa-control"
rviz_mode="${KATECH_RVIZ:-auto}"
log_dir="${KATECH_LOG_DIR:-${repo_dir}/output/katech-logs/$(date +%Y%m%d_%H%M%S)_$$}"
integrated="${PLANNING_AA_INTEGRATED:-0}"
case "${integrated}" in
  0)
    planning_containers=(planning-aa-bridge planning-aa)
    planning_source_ip="${PLANNING_AA_IP:-192.168.100.93}"
    planning_script=run-stack.sh
    ;;
  1)
    planning_containers=(planning-aa-integrated)
    planning_source_ip=192.168.100.92
    planning_script=run-integrated.sh
    ;;
  *) echo "PLANNING_AA_INTEGRATED must be 0 or 1" >&2; exit 1 ;;
esac

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
    echo "  Build or provide this image locally before starting the stack." >&2
  done
  exit 1
fi
echo "Perception image: ${perception_image}"
echo "Control image: ${control_image}"
for name in "${perception_name}" "${control_name}" planning-aa-bridge planning-aa planning-aa-integrated; do
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
log_window_pidfiles=()
diagnostic_pid=""
validator_diagnostic_pid=""
route_visualization_pid=""
stack_pid=""

cleanup() {
  trap - EXIT INT TERM
  local pid_file pid
  for pid_file in "${log_window_pidfiles[@]}"; do
    if [[ -f "${pid_file}" ]]; then
      pid=""
      read -r pid <"${pid_file}" || true
      if [[ "${pid:-}" =~ ^[0-9]+$ ]] &&
         [[ -r "/proc/${pid}/cmdline" ]] &&
         tr '\0' '\n' <"/proc/${pid}/cmdline" | grep -Fxq -- "${pid_file}"; then
        kill "${pid}" >/dev/null 2>&1 || true
      fi
    fi
  done
  docker stop "${planning_containers[@]}" "${control_name}" "${perception_name}" >/dev/null 2>&1 || true
  if [[ -n "${stack_pid}" ]]; then
    wait "${stack_pid}" >/dev/null 2>&1 || true
  fi
  for pid in "${collector_pids[@]}"; do
    kill "${pid}" >/dev/null 2>&1 || true
  done
  if [[ -n "${diagnostic_pid}" ]]; then
    kill "${diagnostic_pid}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${validator_diagnostic_pid}" ]]; then
    kill "${validator_diagnostic_pid}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${route_visualization_pid}" ]]; then
    kill "${route_visualization_pid}" >/dev/null 2>&1 || true
  fi
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
  local pid_file="${log_dir}/.${title}.window.pid"
  local log_window_script='
    pid_file="$1"
    log_file="$2"
    printf "%s\n" "$BASHPID" >"$pid_file"
    tail -n +1 -F -- "$log_file" &
    tail_pid=$!
    close_log_tail() {
      kill "$tail_pid" >/dev/null 2>&1 || true
      wait "$tail_pid" >/dev/null 2>&1 || true
      rm -f -- "$pid_file"
    }
    trap close_log_tail EXIT
    trap "exit 0" TERM
    wait "$tail_pid"
  '
  if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    return 1
  fi
  rm -f -- "${pid_file}"
  if command -v gnome-terminal >/dev/null 2>&1; then
    if gnome-terminal --title="${title}" -- bash -c "${log_window_script}" _ "${pid_file}" "${file}"; then
      log_window_pidfiles+=("${pid_file}")
      return 0
    fi
  fi
  if command -v konsole >/dev/null 2>&1; then
    konsole --new-window -p "tabtitle=${title}" -e bash -c "${log_window_script}" _ "${pid_file}" "${file}" >/dev/null 2>&1 &
    log_window_pidfiles+=("${pid_file}")
    return 0
  fi
  if command -v xterm >/dev/null 2>&1; then
    xterm -T "${title}" -e bash -c "${log_window_script}" _ "${pid_file}" "${file}" >/dev/null 2>&1 &
    log_window_pidfiles+=("${pid_file}")
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
perception_ip="$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "${perception_name}")"
if [[ -z "${perception_ip}" ]]; then
  echo "Cannot determine Perception container IP for route visualization." >&2
  exit 1
fi

for name in "${perception_name}" "${control_name}" "${planning_containers[@]}"; do
  : >"${log_dir}/${name}.log"
done
collect_logs "${perception_name}" "${log_dir}/${perception_name}.log"
collect_logs "${control_name}" "${log_dir}/${control_name}.log"

# Run a read-only ROS subscriber in control's own environment. Keep its concise
# publisher/sample report separate from the control launch log.
docker exec -i "${control_name}" bash -c \
  'source /opt/ros/humble/setup.bash && source /opt/autoware/setup.bash && python3 -u -' \
  <"${repo_dir}/adaptive/trajectory-diagnostic.py" \
  >"${log_dir}/trajectory-diagnostic.log" 2>&1 &
diagnostic_pid="$!"

ROS_DOMAIN_ID="${domain}" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  SERVICECREATOR_TRACE_VALUES="${SERVICECREATOR_TRACE_VALUES:-1}" \
  SERVICECREATOR_METHOD_TIMEOUT_MS="${SERVICECREATOR_METHOD_TIMEOUT_MS:-2000}" \
  PLANNING_AA_DIAGNOSTIC_TARGET=192.168.100.92 \
  PLANNING_AA_ROUTE_VISUALIZATION_TARGET="${perception_ip}" \
  PLANNING_AA_TIMING="${PLANNING_AA_TIMING:-1}" \
  PLANNING_AA_SIMULATION="${PLANNING_AA_SIMULATION:-1}" \
  AP_NET="${network}" bash "${repo_dir}/adaptive/${planning_script}" \
  >"${log_dir}/planning-aa.log" 2>&1 &
stack_pid="$!"

for attempt in {1..50}; do
  if docker container inspect "${planning_containers[0]}" >/dev/null 2>&1; then
    break
  fi
  sleep 0.2
done
if ! docker container inspect "${planning_containers[0]}" >/dev/null 2>&1; then
  echo "Planning container did not start; inspect ${log_dir}/planning-aa.log" >&2
  exit 1
fi
collect_logs "${planning_containers[0]}" "${log_dir}/${planning_containers[0]}.log"

# The AA owns validation. This adapter only publishes its measured result
# under the diagnostic names required by Autoware's system graph.
docker exec -i -e "PLANNING_AA_SOURCE_IP=${planning_source_ip}" \
  "${planning_containers[0]}" bash -c \
  'source /opt/ros/humble/setup.bash && python3 -u -' \
  <"${repo_dir}/adaptive/validator-diagnostic-bridge.py" \
  >"${log_dir}/validator-diagnostic-bridge.log" 2>&1 &
validator_diagnostic_pid="$!"
docker exec -i "${perception_name}" bash -c \
  'source /opt/ros/humble/setup.bash && source /opt/autoware/setup.bash && python3 -u -' \
  <"${repo_dir}/adaptive/route-visualization-bridge.py" \
  >"${log_dir}/route-visualization-bridge.log" 2>&1 &
route_visualization_pid="$!"
validator_ready=false
for attempt in {1..30}; do
  if grep -q 'PlanningAA validator diagnostics listening' \
      "${log_dir}/validator-diagnostic-bridge.log"; then
    validator_ready=true
    break
  fi
  if ! kill -0 "${validator_diagnostic_pid}" 2>/dev/null; then
    break
  fi
  sleep 0.2
done
if [[ "${validator_ready}" != true ]] ||
   ! kill -0 "${validator_diagnostic_pid}" 2>/dev/null; then
  echo "AA validator diagnostic adapter failed; inspect ${log_dir}/validator-diagnostic-bridge.log" >&2
  exit 1
fi
if ! grep -q 'PlanningAA route visualization listening' \
    "${log_dir}/route-visualization-bridge.log"; then
  for attempt in {1..30}; do
    if grep -q 'PlanningAA route visualization listening' \
        "${log_dir}/route-visualization-bridge.log"; then
      break
    fi
    if ! kill -0 "${route_visualization_pid}" 2>/dev/null; then
      break
    fi
    sleep 0.2
  done
fi
if ! grep -q 'PlanningAA route visualization listening' \
    "${log_dir}/route-visualization-bridge.log" ||
   ! kill -0 "${route_visualization_pid}" 2>/dev/null; then
  echo "Route visualization adapter failed; inspect ${log_dir}/route-visualization-bridge.log" >&2
  exit 1
fi

opened=0
for name in "${perception_name}" "${control_name}" "${planning_containers[@]}"; do
  if open_log_terminal "${name}" "${log_dir}/${name}.log"; then
    ((opened += 1))
  fi
done
if ((opened < 2 + ${#planning_containers[@]})); then
  echo "Opened ${opened}/$((2 + ${#planning_containers[@]})) log windows; all logs are saved in ${log_dir}." >&2
fi

echo "Perception and control share ${network} and ROS_DOMAIN_ID=${domain} with the bridge."
echo "RViz visualization: ${rviz_mode} (set KATECH_RVIZ=true or false to override)."
echo "Planning simulation mode: ${PLANNING_AA_SIMULATION:-1} (set PLANNING_AA_SIMULATION=0 for real signals)."
echo "Logs: ${log_dir}"
echo "Trajectory publisher and sample report: ${log_dir}/trajectory-diagnostic.log"
echo "AA validator diagnostic adapter: ${log_dir}/validator-diagnostic-bridge.log"
echo "Full route RViz marker: ${log_dir}/route-visualization-bridge.log"
wait "${stack_pid}"
