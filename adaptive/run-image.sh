#!/usr/bin/env bash
set -euo pipefail
image="${IMAGE_TAG:-planning-aa-adaptive:local}"
manifest=/opt/planning-ap/opt/PlanningAA/etc/exec/PlanningAA.json
if ! manifest_content="$(docker run --rm --network none --entrypoint /bin/cat "${image}" "${manifest}")"; then
  echo "Cannot read PlanningAA manifest from ${image}." >&2
  exit 1
fi
if ! grep -Eq '"dependent-process"[[:space:]]*:[[:space:]]*"CM"' <<< "${manifest_content}"; then
  echo "${image} does not contain the PlanningAA CM dependency. Rebuild with ./adaptive/build-image.sh (check IMAGE_TAG if overridden)." >&2
  exit 1
fi
network="${AP_NET:-planning-aa-net}"
if ! docker network inspect "${network}" >/dev/null 2>&1; then
  docker network create --subnet 192.168.100.0/24 "${network}" >/dev/null
fi
docker run --rm --init --stop-timeout 10 \
  --shm-size="${PLANNING_AA_SHM_SIZE:-2g}" \
  --network "${network}" --ip "${PLANNING_AA_IP:-192.168.100.93}" \
  -e "PLANNING_AA_DIAGNOSTIC_TARGET=${PLANNING_AA_DIAGNOSTIC_TARGET:-}" \
  -e "PLANNING_AA_ROUTE_VISUALIZATION_TARGET=${PLANNING_AA_ROUTE_VISUALIZATION_TARGET:-}" \
  -e "PLANNING_AA_TIMING=${PLANNING_AA_TIMING:-0}" \
  -e "PLANNING_AA_SIMULATION=${PLANNING_AA_SIMULATION:-0}" \
  -e "PLANNING_AA_AVOIDANCE_DEBUG=${PLANNING_AA_AVOIDANCE_DEBUG:-}" \
  --name planning-aa \
  "${image}" "$@"
