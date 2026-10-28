#!/usr/bin/env bash
set -euo pipefail

arch="${ARCH:-$(uname -m)}"
case "${arch}" in
  x86_64) suffix=""; docker_arch=amd64 ;;
  aarch64) suffix=-aarch64; docker_arch=arm64 ;;
  *) echo "Unsupported architecture: ${arch}" >&2; exit 1 ;;
esac
image="${INTEGRATED_IMAGE:-planning-aa-integrated:local${suffix}}"
docker image inspect "${image}" >/dev/null || {
  echo "Integrated image is missing: ${image}; run ./adaptive/build-integrated-image.sh" >&2
  exit 1
}
image_arch="$(docker image inspect --format '{{.Architecture}}' "${image}")"
if [[ "${image_arch}" != "${docker_arch}" ]]; then
  echo "Integrated image architecture mismatch: ${image} is ${image_arch}, expected ${docker_arch}" >&2
  exit 1
fi
network="${AP_NET:-planning-aa-net}"
subnet="${AP_SUBNET:-10.88.0.0/24}"
address="${AP_BRIDGE_IP:-10.88.0.92}"

network_args=(--network "${network}")
if [[ "${network}" != host ]]; then
  if ! docker network inspect "${network}" >/dev/null 2>&1; then
    docker network create --subnet "${subnet}" "${network}" >/dev/null
  elif [[ "$(docker network inspect --format '{{(index .IPAM.Config 0).Subnet}}' "${network}")" != "${subnet}" ]]; then
    echo "Docker network ${network} has a different subnet; set AP_NET to a new name or recreate it with ${subnet}." >&2
    exit 1
  fi
  network_args+=(--ip "${address}")
fi

extra_env=()
for key in RMW_IMPLEMENTATION CYCLONEDDS_URI PARA_LOG_LEVEL SERVICECREATOR_INPUT_MODE \
           SERVICECREATOR_POLL_PERIOD_MS SERVICECREATOR_MAX_PAYLOAD_BYTES \
           SERVICECREATOR_TRACE_VALUES SERVICECREATOR_METHOD_TIMEOUT_MS \
           PLANNING_AA_DIAGNOSTIC_TARGET PLANNING_AA_ROUTE_VISUALIZATION_TARGET \
           PLANNING_AA_TIMING PLANNING_AA_SIMULATION PLANNING_AA_AVOIDANCE_DEBUG \
           PLANNING_AA_STATIC_MAP; do
  if [[ -v ${key} ]]; then
    extra_env+=(-e "${key}=${!key}")
  fi
done

docker run --rm --init --stop-timeout 10 --platform "linux/${docker_arch}" \
  --name planning-aa-integrated \
  "${network_args[@]}" \
  --shm-size="${PLANNING_AA_SHM_SIZE:-2g}" \
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}" \
  -e ROS_LOCALHOST_ONLY=0 \
  -e "MMF_DDS_DOMAIN_ID=${MMF_DDS_DOMAIN_ID:-42}" \
  -e "AP_SOMEIP_ADDRESS=${address}" \
  "${extra_env[@]}" \
  "${image}" "$@"
