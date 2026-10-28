#!/usr/bin/env bash
set -euo pipefail
arch="${ARCH:-$(uname -m)}"
case "${arch}" in
  x86_64) suffix=""; docker_arch=amd64 ;;
  aarch64) suffix=-aarch64; docker_arch=arm64 ;;
  *) echo "Unsupported architecture: ${arch}" >&2; exit 1 ;;
esac
image="${IMAGE_TAG:-decision-ap-planning:local${suffix}}"
docker image inspect "${image}" >/dev/null || {
  echo "DecisionAP image is missing: ${image}; run ./adaptive/build-image.sh" >&2
  exit 1
}
image_arch="$(docker image inspect --format '{{.Architecture}}' "${image}")"
if [[ "${image_arch}" != "${docker_arch}" ]]; then
  echo "DecisionAP image architecture mismatch: ${image} is ${image_arch}, expected ${docker_arch}" >&2
  exit 1
fi
manifest=/opt/planning-ap/opt/DecisionAP/etc/exec/DecisionAP.json
if ! manifest_content="$(docker run --rm --platform "linux/${docker_arch}" --network none --entrypoint /bin/cat "${image}" "${manifest}")"; then
  echo "Cannot read DecisionAP manifest from ${image}." >&2
  exit 1
fi
if ! grep -Eq '"dependent-process"[[:space:]]*:[[:space:]]*"CM"' <<< "${manifest_content}"; then
  echo "${image} does not contain the DecisionAP CM dependency. Rebuild with ./adaptive/build-image.sh (check IMAGE_TAG if overridden)." >&2
  exit 1
fi
network="${AP_NET:-decision-ap-net}"
subnet="${AP_SUBNET:-10.88.0.0/24}"
planning_ip="${PLANNING_AA_IP:-10.88.0.93}"
bridge_ip="${AP_BRIDGE_IP:-10.88.0.92}"
if ! docker network inspect "${network}" >/dev/null 2>&1; then
  docker network create --subnet "${subnet}" "${network}" >/dev/null
elif [[ "$(docker network inspect --format '{{(index .IPAM.Config 0).Subnet}}' "${network}")" != "${subnet}" ]]; then
  echo "Docker network ${network} has a different subnet; set AP_NET to a new name or recreate it with ${subnet}." >&2
  exit 1
fi
docker run --rm --init --stop-timeout 10 --platform "linux/${docker_arch}" \
  --shm-size="${PLANNING_AA_SHM_SIZE:-2g}" \
  --network "${network}" --ip "${planning_ip}" \
  -e "PLANNING_AA_SOMEIP_ADDRESS=${planning_ip}" \
  -e "AP_SOMEIP_ADDRESS=${bridge_ip}" \
  -e "PLANNING_AA_DIAGNOSTIC_TARGET=${PLANNING_AA_DIAGNOSTIC_TARGET:-}" \
  -e "PLANNING_AA_ROUTE_VISUALIZATION_TARGET=${PLANNING_AA_ROUTE_VISUALIZATION_TARGET:-}" \
  -e "PLANNING_AA_TIMING=${PLANNING_AA_TIMING:-0}" \
  -e "PLANNING_AA_SIMULATION=${PLANNING_AA_SIMULATION:-0}" \
  -e "PLANNING_AA_STATIC_MAP=${PLANNING_AA_STATIC_MAP:-0}" \
  -e "PLANNING_AA_AVOIDANCE_DEBUG=${PLANNING_AA_AVOIDANCE_DEBUG:-}" \
  --name decision-ap-planning \
  "${image}" "$@"
