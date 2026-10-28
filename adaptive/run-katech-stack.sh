#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
arch="${ARCH:-$(uname -m)}"
case "${arch}" in
  x86_64) docker_arch=amd64; image_suffix=""; default_katech_repo=katech-autoware-system-control ;;
  aarch64) docker_arch=arm64; image_suffix=-aarch64; default_katech_repo=katech-autoware-system-control-arm64 ;;
  *) echo "Unsupported architecture: ${arch}" >&2; exit 1 ;;
esac
network="${AP_NET:-planning-aa-net}"
subnet="${AP_SUBNET:-10.88.0.0/24}"
bridge_ip="${AP_BRIDGE_IP:-10.88.0.92}"
planning_ip="${PLANNING_AA_IP:-10.88.0.93}"
domain="${ROS_DOMAIN_ID:-42}"
map_dir="${KATECH_MAP_PATH:-${repo_dir}/deployment/map/sample-map-planning}"
image_repo="${KATECH_IMAGE_REPO:-${default_katech_repo}}"
perception_image="${KATECH_PERCEPTION_IMAGE:-${image_repo}:adsw-perception}"
control_image="${KATECH_CONTROL_IMAGE:-${image_repo}:adsw-control}"
perception_name="planning-aa-perception"
control_name="planning-aa-control"
rviz_mode="${KATECH_RVIZ:-auto}"
log_windows="${KATECH_LOG_WINDOWS:-auto}"
log_dir="${KATECH_LOG_DIR:-${repo_dir}/output/katech-logs/$(date +%Y%m%d_%H%M%S)_$$}"
integrated="${PLANNING_AA_INTEGRATED:-0}"
network_mode="${KATECH_NETWORK_MODE:-bridge}"
case "${network_mode}" in
  bridge) ;;
  host)
    if [[ "${integrated}" != 1 ]]; then
      echo "KATECH_NETWORK_MODE=host requires PLANNING_AA_INTEGRATED=1." >&2
      exit 1
    fi
    network_interface="${KATECH_NETWORK_INTERFACE:-}"
    if [[ -z "${network_interface}" ]] || ! ip link show "${network_interface}" >/dev/null 2>&1; then
      echo "Set KATECH_NETWORK_INTERFACE to the board LAN interface (see: ip -br address)." >&2
      exit 1
    fi
    bridge_ip="$(ip -o -4 addr show dev "${network_interface}" scope global | awk 'NR==1 {split($4, address, "/"); print address[1]}')"
    if [[ -z "${bridge_ip}" ]]; then
      echo "No IPv4 address on ${network_interface}." >&2
      exit 1
    fi
    network=host
    dds_uri="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${network_interface}\" multicast=\"true\"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>"
    ;;
  *) echo "KATECH_NETWORK_MODE must be bridge or host." >&2; exit 1 ;;
esac
case "${integrated}" in
  0)
    planning_containers=(planning-aa-bridge planning-aa)
    planning_images=("${BRIDGE_IMAGE:-oss-ap-planning-bridge:local${image_suffix}}"
                     "${IMAGE_TAG:-planning-aa-adaptive:local${image_suffix}}")
    planning_source_ip="${planning_ip}"
    planning_script=run-stack.sh
    ;;
  1)
    planning_containers=(planning-aa-integrated)
    planning_images=("${INTEGRATED_IMAGE:-planning-aa-integrated:local${image_suffix}}")
    planning_source_ip="${bridge_ip}"
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
case "${log_windows}" in
  auto)
    if [[ "${network_mode}" == host ]]; then
      log_windows=false
    else
      log_windows=true
    fi
    ;;
  1|true|yes) log_windows=true ;;
  0|false|no) log_windows=false ;;
  *) echo "KATECH_LOG_WINDOWS must be auto, true, or false" >&2; exit 1 ;;
esac
if [[ "${rviz_mode}" == true && ( -z "${DISPLAY:-}" || ! -d /tmp/.X11-unix ) ]]; then
  echo "RViz requested, but DISPLAY or the X11 socket is unavailable." >&2
  exit 1
fi
if [[ "${network_mode}" == host && "${rviz_mode}" == true ]]; then
  echo "Host network mode runs RViz on the external PC; set KATECH_RVIZ=false." >&2
  exit 1
fi
rviz_sidecar=false
rviz_name=planning-aa-rviz-x86
if [[ "${rviz_mode}" == true && "${docker_arch}" == arm64 && "$(uname -m)" == x86_64 ]]; then
  rviz_sidecar=true
  rviz_mode=false
  rviz_image="${KATECH_RVIZ_IMAGE:-katech-autoware-system-control:adsw-perception}"
fi

test -f "${map_dir}/lanelet2_map.osm" || {
  echo "Lanelet map missing: ${map_dir}/lanelet2_map.osm" >&2
  exit 1
}
if ! docker info >/dev/null 2>&1; then
  echo "Cannot access the Docker daemon. Check Docker service and user permissions." >&2
  exit 1
fi
if [[ "${docker_arch}" == arm64 ]]; then
  board_archive="${repo_dir}/deployment/images/board-arm64.tar.zst"
  if [[ -f "${board_archive}" ]]; then
    need_board_images=false
    for image in "${perception_image}" "${control_image}" "${planning_images[@]}"; do
      if ! docker image inspect "${image}" >/dev/null 2>&1; then
        need_board_images=true
        break
      fi
    done
    if [[ "${need_board_images}" == true ]]; then
      command -v zstd >/dev/null || { echo "zstd is required to load ${board_archive}" >&2; exit 1; }
      echo "Loading ARM64 images from ${board_archive}"
      zstd -dc "${board_archive}" | docker load
    fi
  fi
fi
missing_images=()
for image in "${perception_image}" "${control_image}" "${planning_images[@]}"; do
  if ! docker image inspect "${image}" >/dev/null 2>&1; then
    missing_images+=("${image}")
  fi
done
if [[ "${rviz_sidecar}" == true ]]; then
  if ! docker image inspect "${rviz_image}" >/dev/null 2>&1; then
    echo "x86_64 RViz image is missing: ${rviz_image}" >&2
    exit 1
  fi
  rviz_arch="$(docker image inspect --format '{{.Architecture}}' "${rviz_image}")"
  if [[ "${rviz_arch}" != amd64 ]]; then
    echo "RViz image architecture mismatch: ${rviz_image} is ${rviz_arch}, expected amd64" >&2
    exit 1
  fi
fi
if ((${#missing_images[@]})); then
  echo "Required KATECH images are unavailable locally:" >&2
  for image in "${missing_images[@]}"; do
    echo "  ${image}" >&2
    echo "  Build or provide this image locally before starting the stack." >&2
  done
  exit 1
fi
for image in "${perception_image}" "${control_image}" "${planning_images[@]}"; do
  image_arch="$(docker image inspect --format '{{.Architecture}}' "${image}")"
  if [[ "${image_arch}" != "${docker_arch}" ]]; then
    echo "KATECH image architecture mismatch: ${image} is ${image_arch}, expected ${docker_arch}" >&2
    exit 1
  fi
done
if [[ "${docker_arch}" == arm64 ]]; then
  if ! docker run --rm --platform linux/arm64 --network none \
      --entrypoint /bin/bash "${control_image}" -lc \
      'source /opt/ros/humble/setup.bash && source /opt/autoware/setup.bash && source /opt/autoware-system/setup.bash && ros2 pkg prefix tier4_system_launch >/dev/null && ros2 pkg prefix autoware_mrm_handler >/dev/null && grep -q '\''name="launch_system" value="true"'\'' /opt/autoware-system/share/obigo_launch/launch/sample_adsw_control_run.launch.xml'; then
    echo "ARM64 Control image is missing the local System-in-Control build: ${control_image}" >&2
    echo "Run ./adaptive/build-katech-control-system-image.sh first." >&2
    exit 1
  fi
fi
echo "Perception image: ${perception_image}"
echo "Control image: ${control_image}"
for name in "${perception_name}" "${control_name}" "${rviz_name}" planning-aa-bridge planning-aa planning-aa-integrated; do
  if docker container inspect "${name}" >/dev/null 2>&1; then
    if [[ "$(docker container inspect -f '{{.State.Running}}' "${name}")" == true ]]; then
      echo "Container already running: ${name}; stop the previous stack before restarting." >&2
      exit 1
    fi
    docker rm "${name}" >/dev/null
    echo "Removed stopped container from previous run: ${name}"
  fi
done
if [[ "${network_mode}" == bridge ]]; then
  if ! docker network inspect "${network}" >/dev/null 2>&1; then
    docker network create --subnet "${subnet}" "${network}" >/dev/null
  elif [[ "$(docker network inspect --format '{{(index .IPAM.Config 0).Subnet}}' "${network}")" != "${subnet}" ]]; then
    echo "Docker network ${network} has a different subnet; set AP_NET to a new name or recreate it with ${subnet}." >&2
    exit 1
  fi
fi

mkdir -p "${log_dir}"
log_dir="$(cd "${log_dir}" && pwd -P)"
qemu_diagnostics=false
if [[ "${docker_arch}" == arm64 && "$(uname -m)" == x86_64 &&
      "${PLANNING_AA_SIMULATION:-1}" == 1 && "${KATECH_QEMU_DIAGNOSTICS:-1}" == 1 ]]; then
  qemu_diagnostics=true
  config_source="${log_dir}/qemu-config-source"
  config_overlay="${log_dir}/qemu-config"
  mkdir -p "${config_source}"
  for config in \
    "topics.yaml:/opt/autoware-system/share/obigo_launch/config/system/component_state_monitor/topics.yaml" \
    "planning.yaml:/opt/autoware-system/share/obigo_launch/config/system/diagnostics/planning.yaml"; do
    filename="${config%%:*}"
    image_path="${config#*:}"
    docker run --rm --platform linux/arm64 --network none --entrypoint /bin/cat \
      "${control_image}" "${image_path}" >"${config_source}/${filename}"
  done
  python3 "${repo_dir}/adaptive/qemu-diagnostic-config.py" "${config_source}" "${config_overlay}"
fi
collector_pids=()
log_window_pidfiles=()
diagnostic_pid=""
validator_diagnostic_pid=""
route_visualization_pid=""
stack_pid=""

cleanup() {
  trap - EXIT
  trap '' INT TERM
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
  if [[ "${rviz_sidecar}" == true ]]; then
    docker stop "${rviz_name}" >/dev/null 2>&1 || true
  fi
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
  docker rm -f "${rviz_name}" "${planning_containers[@]}" "${control_name}" "${perception_name}" >/dev/null 2>&1 || true
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

check_container_started() {
  local name="$1" label="$2" state
  state="$(docker container inspect -f '{{.State.Running}}' "${name}" 2>/dev/null || true)"
  if [[ "${state}" == true ]]; then
    return 0
  fi
  if docker container inspect "${name}" >/dev/null 2>&1; then
    docker logs --timestamps "${name}" >"${log_dir}/${name}.log" 2>&1 || true
    echo "${label} container exited during startup: $(docker inspect -f 'exit={{.State.ExitCode}} oom={{.State.OOMKilled}} error={{.State.Error}}' "${name}")" >&2
    echo "Last log lines:" >&2
    tail -n 60 "${log_dir}/${name}.log" >&2
    echo "Full log: ${log_dir}/${name}.log" >&2
  else
    echo "${label} container disappeared during startup. Check Docker daemon logs." >&2
  fi
  return 1
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
  --init --platform "linux/${docker_arch}" --network "${network}" --ipc=host --pid=host
  -e "ROS_DOMAIN_ID=${domain}"
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  -e ROS_LOCALHOST_ONLY=0
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  -e "LOCAL_UID=$(id -u)" -e "LOCAL_GID=$(id -g)"
  -e "LOCAL_USER=$(id -un)" -e "LOCAL_GROUP=$(id -gn)"
  -v "${map_dir}:/autoware_map:ro"
  -v /etc/localtime:/etc/localtime:ro
)
if [[ "${network_mode}" == host ]]; then
  common_args+=(-e "CYCLONEDDS_URI=${dds_uri}")
fi
if [[ "${rviz_mode}" == true || "${rviz_sidecar}" == true ]]; then
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
if [[ "${rviz_sidecar}" == true ]]; then
  rviz_args=(
    --init --platform linux/amd64 --network "${network}" --ipc=host --pid=host
    -e "DISPLAY=${DISPLAY}" -e "ROS_DOMAIN_ID=${domain}"
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e QT_X11_NO_MITSHM=1
    -v /tmp/.X11-unix:/tmp/.X11-unix
  )
  xauthority_file="${XAUTHORITY:-${HOME}/.Xauthority}"
  if [[ -f "${xauthority_file}" ]]; then
    rviz_args+=(-e "XAUTHORITY=${xauthority_file}" -v "${xauthority_file}:${xauthority_file}:ro")
  fi
  if [[ -d /dev/dri ]]; then
    rviz_args+=(--device /dev/dri:/dev/dri)
  fi
  docker run -d --name "${rviz_name}" "${rviz_args[@]}" \
    "${rviz_image}" bash -lc \
    'source /opt/ros/humble/setup.bash && source /opt/autoware/setup.bash && exec rviz2 -d /opt/autoware/share/obigo_launch/rviz/autoware.rviz' >/dev/null
fi
sleep "${KATECH_START_DELAY:-10}"
check_container_started "${perception_name}" Perception
if [[ "${rviz_sidecar}" == true ]]; then
  check_container_started "${rviz_name}" RViz
fi
control_args=("${common_args[@]}")
if [[ "${qemu_diagnostics}" == true ]]; then
  control_args+=(
    -v "${config_overlay}/topics.yaml:/opt/autoware-system/share/obigo_launch/config/system/component_state_monitor/topics.yaml:ro"
    -v "${config_overlay}/planning.yaml:/opt/autoware-system/share/obigo_launch/config/system/diagnostics/planning.yaml:ro"
  )
fi
docker run -d --name "${control_name}" "${control_args[@]}" \
  "${control_image}" bash -lc \
  'ros2 launch obigo_launch sample_adsw_control_run.launch.xml map_path:=/autoware_map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit' >/dev/null
check_container_started "${control_name}" Control
if [[ "${network_mode}" == host ]]; then
  perception_ip="${bridge_ip}"
else
  perception_ip="$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "${perception_name}")"
fi
if [[ -z "${perception_ip}" ]]; then
  echo "Cannot determine Perception container IP for route visualization." >&2
  exit 1
fi

for name in "${perception_name}" "${control_name}" "${planning_containers[@]}"; do
  : >"${log_dir}/${name}.log"
done
if [[ "${rviz_sidecar}" == true ]]; then
  : >"${log_dir}/${rviz_name}.log"
  collect_logs "${rviz_name}" "${log_dir}/${rviz_name}.log"
fi
collect_logs "${perception_name}" "${log_dir}/${perception_name}.log"
collect_logs "${control_name}" "${log_dir}/${control_name}.log"

# Run a read-only ROS subscriber in control's own environment. Keep its concise
# publisher/sample report separate from the control launch log.
docker exec -i "${control_name}" bash -c \
  'source /opt/ros/humble/setup.bash && source /opt/autoware/setup.bash && python3 -u -' \
  <"${repo_dir}/adaptive/trajectory-diagnostic.py" \
  >"${log_dir}/trajectory-diagnostic.log" 2>&1 &
diagnostic_pid="$!"

planning_env=(
  "ROS_DOMAIN_ID=${domain}"
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  "AP_BRIDGE_IP=${bridge_ip}"
)
if [[ "${network_mode}" == host ]]; then
  planning_env+=("CYCLONEDDS_URI=${dds_uri}")
fi
env "${planning_env[@]}" \
  SERVICECREATOR_TRACE_VALUES="${SERVICECREATOR_TRACE_VALUES:-1}" \
  SERVICECREATOR_METHOD_TIMEOUT_MS="${SERVICECREATOR_METHOD_TIMEOUT_MS:-2000}" \
  PLANNING_AA_DIAGNOSTIC_TARGET="${bridge_ip}" \
  PLANNING_AA_ROUTE_VISUALIZATION_TARGET="${perception_ip}" \
  PLANNING_AA_TIMING="${PLANNING_AA_TIMING:-1}" \
  PLANNING_AA_SIMULATION="${PLANNING_AA_SIMULATION:-1}" \
  PLANNING_AA_STATIC_MAP="${PLANNING_AA_STATIC_MAP:-1}" \
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

if [[ "${log_windows}" == true ]]; then
  opened=0
  for name in "${perception_name}" "${control_name}" "${planning_containers[@]}"; do
    if open_log_terminal "${name}" "${log_dir}/${name}.log"; then
      ((opened += 1))
    fi
  done
  if ((opened < 2 + ${#planning_containers[@]})); then
    echo "Opened ${opened}/$((2 + ${#planning_containers[@]})) log windows; all logs are saved in ${log_dir}." >&2
  fi
else
  echo "Log windows disabled; follow logs with: tail -F ${log_dir}/*.log"
fi

echo "Perception and control share ${network} and ROS_DOMAIN_ID=${domain} with the bridge."
if [[ "${network_mode}" == host ]]; then
  echo "Board LAN: ${network_interface} (${bridge_ip}); start RViz on the external PC with ./adaptive/run-remote-rviz.sh."
fi
if [[ "${rviz_sidecar}" == true ]]; then
  echo "RViz visualization: x86_64 sidecar (${rviz_image})."
else
  echo "RViz visualization: ${rviz_mode} (set KATECH_RVIZ=true or false to override)."
fi
echo "Planning simulation mode: ${PLANNING_AA_SIMULATION:-1} (set PLANNING_AA_SIMULATION=0 for real signals)."
if [[ "${qemu_diagnostics}" == true ]]; then
  echo "ARM64 QEMU trajectory diagnostic thresholds: 0.25 Hz error, 4.0 s timeout (simulation only)."
fi
echo "Logs: ${log_dir}"
echo "Trajectory publisher and sample report: ${log_dir}/trajectory-diagnostic.log"
echo "AA validator diagnostic adapter: ${log_dir}/validator-diagnostic-bridge.log"
echo "Full route RViz marker: ${log_dir}/route-visualization-bridge.log"
wait "${stack_pid}"
