#!/usr/bin/env bash
set -euo pipefail

image="${INTEGRATED_IMAGE:-planning-aa-integrated:local}"
network="${AP_NET:-planning-aa-net}"
address=192.168.100.92

docker image inspect "${image}" >/dev/null || {
  echo "Integrated image is missing: ${image}; run ./adaptive/build-integrated-image.sh" >&2
  exit 1
}
if ! docker network inspect "${network}" >/dev/null 2>&1; then
  docker network create --subnet 192.168.100.0/24 "${network}" >/dev/null
fi

extra_env=()
for key in RMW_IMPLEMENTATION PARA_LOG_LEVEL SERVICECREATOR_INPUT_MODE \
           SERVICECREATOR_POLL_PERIOD_MS SERVICECREATOR_MAX_PAYLOAD_BYTES \
           SERVICECREATOR_TRACE_VALUES SERVICECREATOR_METHOD_TIMEOUT_MS \
           PLANNING_AA_DIAGNOSTIC_TARGET PLANNING_AA_ROUTE_VISUALIZATION_TARGET \
           PLANNING_AA_TIMING PLANNING_AA_SIMULATION PLANNING_AA_AVOIDANCE_DEBUG; do
  if [[ -v ${key} ]]; then
    extra_env+=(-e "${key}=${!key}")
  fi
done

docker run --rm --init --stop-timeout 10 \
  --name planning-aa-integrated \
  --network "${network}" --ip "${address}" \
  --shm-size="${PLANNING_AA_SHM_SIZE:-2g}" \
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}" \
  -e ROS_LOCALHOST_ONLY=0 \
  -e "MMF_DDS_DOMAIN_ID=${MMF_DDS_DOMAIN_ID:-42}" \
  "${extra_env[@]}" \
  "${image}" "$@"
