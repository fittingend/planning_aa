#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
arch="${ARCH:-$(uname -m)}"
case "${arch}" in
  x86_64) suffix=""; docker_arch=amd64 ;;
  aarch64) suffix=-aarch64; docker_arch=arm64 ;;
  *) echo "Unsupported architecture: ${arch}" >&2; exit 1 ;;
esac
network="${AP_NET:-planning-aa-net}"
subnet="${AP_SUBNET:-10.88.0.0/24}"
bridge_ip="${AP_BRIDGE_IP:-10.88.0.92}"
bridge_image="${BRIDGE_IMAGE:-oss-ap-planning-bridge:local${suffix}}"
docker image inspect "${bridge_image}" >/dev/null || {
  echo "Bridge image is missing: ${bridge_image}; run ./adaptive/build-local-bridge-image.sh" >&2
  exit 1
}
bridge_arch="$(docker image inspect --format '{{.Architecture}}' "${bridge_image}")"
if [[ "${bridge_arch}" != "${docker_arch}" ]]; then
  echo "Bridge image architecture mismatch: ${bridge_image} is ${bridge_arch}, expected ${docker_arch}" >&2
  exit 1
fi

if ! docker network inspect "${network}" >/dev/null 2>&1; then
  docker network create --subnet "${subnet}" "${network}" >/dev/null
elif [[ "$(docker network inspect --format '{{(index .IPAM.Config 0).Subnet}}' "${network}")" != "${subnet}" ]]; then
  echo "Docker network ${network} has a different subnet; set AP_NET to a new name or recreate it with ${subnet}." >&2
  exit 1
fi

bridge_env=()
for key in RMW_IMPLEMENTATION PARA_LOG_LEVEL SERVICECREATOR_INPUT_MODE \
           SERVICECREATOR_POLL_PERIOD_MS SERVICECREATOR_MAX_PAYLOAD_BYTES \
           SERVICECREATOR_TRACE_VALUES SERVICECREATOR_METHOD_TIMEOUT_MS; do
  if [[ -v ${key} ]]; then
    bridge_env+=(-e "${key}=${!key}")
  fi
done
docker run -d --name planning-aa-bridge --rm --platform "linux/${docker_arch}" \
  --network "${network}" --ip "${bridge_ip}" --shm-size=2g \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e ROS_LOCALHOST_ONLY=0 \
  -e MMF_DDS_DOMAIN_ID="${MMF_DDS_DOMAIN_ID:-42}" \
  -e "AP_SOMEIP_ADDRESS=${bridge_ip}" \
  "${bridge_env[@]}" \
  "${bridge_image}" >/dev/null
trap 'docker stop planning-aa-bridge >/dev/null 2>&1 || true' EXIT

AP_NET="${network}" bash "${repo_dir}/adaptive/run-image.sh"
