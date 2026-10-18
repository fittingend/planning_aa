#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
network="${AP_NET:-planning-aa-net}"

if ! docker network inspect "${network}" >/dev/null 2>&1; then
  docker network create --subnet 192.168.100.0/24 "${network}" >/dev/null
fi

bridge_env=()
for key in RMW_IMPLEMENTATION PARA_LOG_LEVEL SERVICECREATOR_INPUT_MODE \
           SERVICECREATOR_POLL_PERIOD_MS SERVICECREATOR_MAX_PAYLOAD_BYTES \
           SERVICECREATOR_TRACE_VALUES; do
  if [[ -v ${key} ]]; then
    bridge_env+=(-e "${key}=${!key}")
  fi
done
docker run -d --name planning-aa-bridge --rm \
  --network "${network}" --ip 192.168.100.92 --shm-size=2g \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e ROS_LOCALHOST_ONLY=0 \
  -e MMF_DDS_DOMAIN_ID="${MMF_DDS_DOMAIN_ID:-42}" \
  "${bridge_env[@]}" \
  "${BRIDGE_IMAGE:-oss-ap-planning-bridge:local}" >/dev/null
trap 'docker stop planning-aa-bridge >/dev/null 2>&1 || true' EXIT

AP_NET="${network}" bash "${repo_dir}/adaptive/run-image.sh"
